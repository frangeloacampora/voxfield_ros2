#!/usr/bin/env python3
"""Dataset-free multi-sensor smoke test (MULTI_SENSOR_PLAN.md §8 Phase 8
step 2).

Same dataset-free shape as test_smoke.launch.py (Phase 12 step 1 of the
ROS 2 port plan), but starts scripts/fake_sensor_publisher.py in
multi-sensor mode (`num_sensors:=2`, yaw offsets 0/180deg -- two
limited-FOV sensors mounted back to back on a spinning/orbiting rig, each
only seeing its own +-90deg forward half) and the server with
`sensor_names: [s0, s1]` bound to the two `pointcloud_0`/`pointcloud_1`
topics the publisher creates. `method` is read from sys.argv exactly like
test_smoke.launch.py; `voxfield_ros/CMakeLists.txt` registers this once
each for `voxfield` (the projective/NP path) and `voxblox` (the
non-projective/ray-casting path), covering both integration paths.

Asserts, within the plan's 40s budget:
  - `~/mesh` eventually has vertices past x=+4 *and* past x=-4 -- i.e.
    material from *both* opposite walls made it into the one shared map,
    not just whichever side happened to be in view first. (The rig keeps
    spinning throughout the run, so this isn't just "sensor 0 always faces
    +x": over the run both sensors sweep past both walls; the assertion is
    that the *shared* map ends up with material on both sides, which is
    the thing multi-sensor fusion is actually for.) A quick *manual*
    single-sensor sanity run (not part of this automated test -- see
    docs/MULTI_SENSOR_NOTES.md) confirms that with only one sensor
    connected, early in the run only one wall has mesh yet.
  - `~/esdf_slice` receives a non-empty cloud.
  - `~/save_map` -> `~/load_map` round-trips, exactly like the existing
    single-sensor smoke test.
  - No `ERROR` line appears in the server's captured output.
"""
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

import launch
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from sensor_msgs.msg import PointCloud2
from voxfield_msgs.msg import Mesh
from voxfield_msgs.srv import FilePath


def _method_from_argv(default="voxfield"):
    for arg in sys.argv:
        if arg.startswith("method:="):
            return arg.split(":=", 1)[1]
    return default


METHOD = _method_from_argv()
NODE_NAME = "voxfield_node"

# Absolute paths only: see test_smoke.launch.py's identical note about
# /opt/ros/jazzy's python3 C extensions and a non-system python3 earlier on
# PATH.
FAKE_SENSOR_PUBLISHER = str(
    Path(__file__).resolve().parents[2] / "scripts" / "fake_sensor_publisher.py"
)


def _mesh_vertex_world_x(block_edge_length, block_index_x, x_uint16):
    """Inverts mesh_vis.h's generateVoxbloxMeshMsg() vertex quantization:
    normalized = 0.5 * (vertex / block_size - block_index), then
    x_uint16 = 65535 * normalized.x. Solving for the world-frame x:
    vertex_x = block_size * (block_index_x + 2 * (x_uint16 / 65535))."""
    normalized_x = x_uint16 / 65535.0
    return block_edge_length * (block_index_x + 2.0 * normalized_x)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    pkg_share = get_package_share_directory("voxfield_ros")
    param_file = os.path.join(pkg_share, "cfg", "param", "kitti_param.yaml")
    calib_file = os.path.join(pkg_share, "cfg", "calib", "kitti_calib.yaml")

    fake_publisher = ExecuteProcess(
        cmd=[
            "/usr/bin/python3", FAKE_SENSOR_PUBLISHER,
            "--ros-args",
            "-p", "num_sensors:=2",
            "-p", "sensor_yaw_offsets_deg:=0,180",
            # Coarser than kitti_calib.yaml's 1024x64 default -- this is a
            # synthetic smoke test, not a fidelity check, and a smaller
            # per-tick ray count keeps the launch test fast.
            "-p", "width:=256",
            "-p", "height:=16",
        ],
        output="screen",
    )

    server = Node(
        package="voxfield_ros",
        executable=f"{METHOD}_server",
        name=NODE_NAME,
        output="screen",
        parameters=[
            param_file,
            calib_file,
            {
                "use_sim_time": False,
                # See test_smoke.launch.py's comment: must stay an int, not
                # a float, or rcl_yaml_param_parser silently drops this
                # override (kitti_param.yaml's own value is a YAML int).
                "update_esdf_every_n_sec": 1,
                # Match the fake publisher's coarser resolution (per-sensor
                # override not needed: both s0/s1 inherit these top-level
                # values, M2).
                "width": 256,
                "height": 16,
                "sensor_names": ["s0", "s1"],
                "sensors.s0.topic": "pointcloud_0",
                "sensors.s1.topic": "pointcloud_1",
            },
        ],
    )

    return (
        launch.LaunchDescription(
            [
                fake_publisher,
                server,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"fake_publisher": fake_publisher, "server": server},
    )


class TestMultiSensorSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_multi_sensor_smoke_checker")

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for(self, msg_type, topic, predicate, timeout_sec=30.0):
        """Spin, accumulating every message that arrives (not just the
        first), until `predicate` is satisfied by the accumulated state or
        time runs out. `predicate` takes the list of all messages received
        so far and returns True once satisfied."""
        received = []

        def cb(msg):
            received.append(msg)

        sub = self.node.create_subscription(msg_type, topic, cb, 10)
        end_time = time.time() + timeout_sec
        try:
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                if predicate(received):
                    return received
        finally:
            self.node.destroy_subscription(sub)
        return received if predicate(received) else None

    def test_mesh_has_both_walls(self):
        def has_both_walls(messages):
            saw_plus_x = False
            saw_minus_x = False
            for msg in messages:
                block_edge_length = msg.block_edge_length
                for block in msg.mesh_blocks:
                    block_index_x = block.index[0]
                    for x_uint16 in block.x:
                        world_x = _mesh_vertex_world_x(
                            block_edge_length, block_index_x, x_uint16)
                        if world_x > 4.0:
                            saw_plus_x = True
                        elif world_x < -4.0:
                            saw_minus_x = True
                    if saw_plus_x and saw_minus_x:
                        return True
            return saw_plus_x and saw_minus_x

        messages = self._wait_for(
            Mesh, f"/{NODE_NAME}/mesh", has_both_walls, timeout_sec=40.0)
        self.assertIsNotNone(
            messages,
            f"[{METHOD}] /{NODE_NAME}/mesh never accumulated vertices past "
            "both x=+4 and x=-4 within 40s (both sensors' walls should "
            "eventually appear in the one shared map)",
        )

    def test_esdf_slice_nonempty(self):
        def nonempty_cloud(messages):
            return any(len(msg.data) > 0 for msg in messages)

        messages = self._wait_for(
            PointCloud2, f"/{NODE_NAME}/esdf_slice", nonempty_cloud,
            timeout_sec=40.0)
        self.assertIsNotNone(
            messages,
            f"[{METHOD}] /{NODE_NAME}/esdf_slice never produced a "
            "non-empty cloud within 40s",
        )

    def test_save_load_roundtrip(self, proc_output, server):
        # Block until there's actually a map to save.
        def nonempty_cloud(messages):
            return any(len(msg.data) > 0 for msg in messages)

        esdf_slice = self._wait_for(
            PointCloud2, f"/{NODE_NAME}/esdf_slice", nonempty_cloud,
            timeout_sec=40.0)
        self.assertIsNotNone(
            esdf_slice,
            f"[{METHOD}] /{NODE_NAME}/esdf_slice never produced a "
            "non-empty cloud within 40s (needed before save_map/load_map)",
        )

        fd, map_path = tempfile.mkstemp(
            suffix=".tsdf", prefix="voxfield_multi_sensor_smoke_")
        os.close(fd)
        os.remove(map_path)  # save_map must create it, not just write into it
        try:
            save_client = self.node.create_client(
                FilePath, f"/{NODE_NAME}/save_map")
            self.assertTrue(
                save_client.wait_for_service(timeout_sec=10.0),
                f"/{NODE_NAME}/save_map service not available",
            )
            future = save_client.call_async(FilePath.Request(file_path=map_path))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=15.0)
            self.assertTrue(future.done(), "save_map call did not complete")
            self.assertTrue(
                os.path.isfile(map_path) and os.path.getsize(map_path) > 0,
                f"save_map did not create a non-empty file at {map_path}",
            )

            load_client = self.node.create_client(
                FilePath, f"/{NODE_NAME}/load_map")
            self.assertTrue(
                load_client.wait_for_service(timeout_sec=10.0),
                f"/{NODE_NAME}/load_map service not available",
            )
            future = load_client.call_async(FilePath.Request(file_path=map_path))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=15.0)
            self.assertTrue(future.done(), "load_map call did not complete")
            time.sleep(1.0)
            all_output = "".join(
                event.text.decode(errors="replace") for event in proc_output
            )
            self.assertNotIn(
                "Failed to load map",
                all_output,
                "load_map logged a failure for the file save_map just wrote",
            )
        finally:
            if os.path.isfile(map_path):
                os.remove(map_path)

    def test_no_error_lines(self, proc_output, server):
        # Give the server a moment to have logged everything from the
        # other tests' activity before checking.
        time.sleep(1.0)
        # proc_output[server] (IoHandler.__getitem__) returns only this
        # process's captured events, not the fake publisher's.
        all_output = "".join(
            event.text.decode(errors="replace") for event in proc_output[server]
        )
        error_lines = [
            line for line in all_output.splitlines() if "ERROR" in line
        ]
        self.assertEqual(
            error_lines, [],
            f"[{METHOD}] server output contained ERROR line(s): {error_lines}",
        )
