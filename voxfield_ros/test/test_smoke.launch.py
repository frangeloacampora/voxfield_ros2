#!/usr/bin/env python3
"""Dataset-free smoke test (ROS2_PORT_PLAN.md §6 Phase 12 step 1).

Starts scripts/fake_sensor_publisher.py (a synthetic closed-room point
cloud + moving world->velodyne TF, needing no external dataset) alongside
one `<method>_server` against kitti_param.yaml/kitti_calib.yaml with
`use_sim_time:=false` and `update_esdf_every_n_sec` overridden to `1.0`
(kitti_param.yaml ships with ESDF integration off, `0`, since it's meant
for the flagship VoxfieldServer that runs it on demand via a separate
mechanism -- overridden here so every ESDF-capable method actually
exercises that path too; the override must stay an int, see the inline
comment where it's set -- rcl_yaml_param_parser silently drops a later
--params-file's override when its YAML-inferred type differs from an
earlier file's for the same key).

For `method=voxfield` this checks the full set the plan calls for:
`~/mesh` receiving a message with at least one non-empty mesh block,
`~/tsdf_slice`/`~/esdf_slice` receiving a non-empty cloud, and a
`~/save_map` -> `~/load_map` round trip through a temp file, asserting the
load actually succeeds (the server logs "Successfully loaded TSDF layer."
and no "Failed to load map" error). This round trip failed before the
`VoxfieldServer::saveMap()` fix (docs/ROS2_PORT_NOTES.md "Known upstream
issues" #11). Every other method
(`np_tsdf`, `voxblox`, `fiesta`, `voxedt`) only checks `~/mesh`, per the
plan's "repeat quickly ... (mesh only)".

`method` is read from sys.argv (`method:=<name>`, default `voxfield`);
`voxfield_ros/CMakeLists.txt` registers this file once per method via
`add_launch_test(... ARGS "method:=<name>")`.
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

# Absolute paths only: /opt/ros/jazzy's python3 C extensions don't load
# under a non-system python3 (e.g. a miniconda install earlier on PATH --
# see docs/ROS2_PORT_NOTES.md's Phase 3/4 "miniconda's python3" note).
FAKE_SENSOR_PUBLISHER = str(
    Path(__file__).resolve().parents[2] / "scripts" / "fake_sensor_publisher.py"
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    pkg_share = get_package_share_directory("voxfield_ros")
    param_file = os.path.join(pkg_share, "cfg", "param", "kitti_param.yaml")
    calib_file = os.path.join(pkg_share, "cfg", "calib", "kitti_calib.yaml")

    fake_publisher = ExecuteProcess(
        cmd=["/usr/bin/python3", FAKE_SENSOR_PUBLISHER],
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
            # `update_esdf_every_n_sec` must stay an int (`1`, not `1.0`):
            # kitti_param.yaml declares it as `0` (YAML-inferred int), and
            # rcl_yaml_param_parser silently drops a later --params-file's
            # override for the same key when its inferred type differs
            # from an earlier file's -- confirmed empirically while
            # writing this test, not documented anywhere obvious. A
            # same-typed override merges fine.
            {"use_sim_time": False, "update_esdf_every_n_sec": 1},
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


class TestSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_smoke_checker")

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for(self, msg_type, topic, predicate, timeout_sec=30.0):
        """Spin until a message on `topic` satisfies `predicate`, or time out."""
        received = []

        def cb(msg):
            if predicate(msg):
                received.append(msg)

        sub = self.node.create_subscription(msg_type, topic, cb, 10)
        end_time = time.time() + timeout_sec
        try:
            while time.time() < end_time and not received:
                rclpy.spin_once(self.node, timeout_sec=0.5)
        finally:
            self.node.destroy_subscription(sub)
        return received[0] if received else None

    def test_mesh_received(self):
        def has_nonempty_block(msg):
            return any(len(block.x) > 0 for block in msg.mesh_blocks)

        msg = self._wait_for(
            Mesh, f"/{NODE_NAME}/mesh", has_nonempty_block, timeout_sec=30.0
        )
        self.assertIsNotNone(
            msg,
            f"[{METHOD}] no /{NODE_NAME}/mesh message with a non-empty "
            "mesh_block received within 30s",
        )

    def test_slice_and_map_roundtrip(self, proc_output, server):
        if METHOD != "voxfield":
            self.skipTest("full mesh/slice/save/load check only runs for voxfield")

        def nonempty_cloud(msg):
            return len(msg.data) > 0

        tsdf_slice = self._wait_for(
            PointCloud2, f"/{NODE_NAME}/tsdf_slice", nonempty_cloud, timeout_sec=15.0
        )
        esdf_slice = self._wait_for(
            PointCloud2, f"/{NODE_NAME}/esdf_slice", nonempty_cloud, timeout_sec=15.0
        )
        self.assertTrue(
            tsdf_slice is not None or esdf_slice is not None,
            f"[{METHOD}] neither /{NODE_NAME}/tsdf_slice nor "
            f"/{NODE_NAME}/esdf_slice produced a non-empty cloud within 30s",
        )
        # VoxfieldServer::saveMap()/loadMap() are multi-layer: they save (and
        # expect to load) both the TSDF and ESDF layers from one file, so
        # the round trip below needs the ESDF layer to actually have data
        # -- not just the TSDF one -- or loadMap() correctly, and loudly,
        # rejects the file. Block on esdf_slice specifically before saving.
        self.assertIsNotNone(
            esdf_slice,
            f"[{METHOD}] /{NODE_NAME}/esdf_slice never produced a non-empty "
            "cloud within 30s (needed before save_map/load_map, which are "
            "multi-layer for voxfield)",
        )

        fd, map_path = tempfile.mkstemp(suffix=".tsdf", prefix="voxfield_smoke_")
        os.close(fd)
        os.remove(map_path)  # save_map must create it, not just write into it
        try:
            save_client = self.node.create_client(
                FilePath, f"/{NODE_NAME}/save_map"
            )
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
                FilePath, f"/{NODE_NAME}/load_map"
            )
            self.assertTrue(
                load_client.wait_for_service(timeout_sec=10.0),
                f"/{NODE_NAME}/load_map service not available",
            )
            future = load_client.call_async(FilePath.Request(file_path=map_path))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=15.0)
            self.assertTrue(future.done(), "load_map call did not complete")
            # FilePath.srv has no success field (D13), so check the outcome
            # in the server's log. NpTsdfServer::loadMap() logs
            # "Successfully loaded TSDF layer." once the TSDF half is read,
            # and loadMapCallback() logs "Failed to load map from '...'"
            # if either half (TSDF or ESDF) fails. Before the fix for
            # docs/ROS2_PORT_NOTES.md "Known upstream issues" #11,
            # saveMap() wrote only the ESDF layer, so this load always
            # failed on the TSDF half.
            proc_output.assertWaitFor(
                "Successfully loaded TSDF layer.", process=server, timeout=10
            )
            # The callback has returned (the response arrived), so any
            # failure line was already written; give the output capture a
            # moment to catch up before checking that it's absent.
            time.sleep(1.0)
            # Only the server logs this string, so all captured output
            # can be searched.
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
