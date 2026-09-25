#!/usr/bin/env python3
"""Synthetic sensor publisher for the dataset-free smoke test
(ROS2_PORT_PLAN.md §6 Phase 12 step 1) and the multi-sensor smoke test
(MULTI_SENSOR_PLAN.md §8 Phase 8 step 1).

Publishes an organized `sensor_msgs/PointCloud2` of a closed, axis-aligned
box room (walls + floor + ceiling; default 10x10x3 m) as seen by a
spinning-lidar-style sensor circling near the room's center, using the
same `width`/`height`/`fov_up`/`fov_down` organized-cloud convention as
`kitti_calib.yaml` by default -- every ray from the sensor's origin is
guaranteed to hit a wall, floor, or ceiling (the room is airtight and the
sensor never leaves it), so the cloud is fully dense (`is_dense: true`,
no NaNs) every frame.

By default also broadcasts a moving `world -> <sensor_frame>` TF at the
same rate, so both the projective (NpTsdfServer/VoxfieldServer) and
non-projective (TsdfServer/VoxbloxServer/FiestaServer/VoxedtServer)
integration paths have real geometry and real motion to integrate,
without needing an external dataset.

With `publish_tf:=false` (the `cow`/`vicon` dataset shape, i.e.
`use_tf_transforms: false`), publishes a
`geometry_msgs/TransformStamped` on `transform_topic` instead of
broadcasting TF -- exercising `Transformer`'s queue/interpolation path
(ROS2_PORT_PLAN.md §6 Phase 12 step 5) rather than its `tf2_ros::Buffer`
path. In that mode the transform is published twice as often as the
point cloud (point clouds on even ticks only), so every cloud's stamp
falls strictly between two queued transforms and
`Transformer::lookupTransformQueue`'s interpolation actually runs instead
of exact-match lookup.

With `num_sensors` > 1 (MULTI_SENSOR_PLAN.md Phase 8 step 1), the single
spinning sensor becomes a rigid "rig" that orbits/spins exactly as before,
carrying `num_sensors` individual sensors mounted around its perimeter,
each a fixed `sensor_rig_offset_m` (default 0.5 m) out from the rig
center and yaw-rotated by its own `sensor_yaw_offsets_deg` entry (default:
evenly spaced around 360deg) -- so each sensor faces radially outward.
Each sensor only emits points within +-90deg of its own forward axis (a
limited-FOV sensor, not a second full spherical scan), publishes on
`<pointcloud_topic>_<i>` with `header.frame_id = <sensor_frame>_<i>`, and
(when `publish_tf` is true) is reachable via a static
`<sensor_frame>_rig -> <sensor_frame>_<i>` TF hanging off the moving
`world -> <sensor_frame>_rig` TF. `num_sensors: 1` (the default) reuses
the exact single-sensor code path unchanged: same topic name, same frame
name, no rig, full spherical FOV.

ROS parameters (all optional, defaults match kitti_calib.yaml):
    world_frame (string, "world")
    sensor_frame (string, "velodyne")
    pointcloud_topic (string, "pointcloud")
    publish_tf (bool, true): broadcast TF vs. publish a transform topic.
    transform_topic (string, "transform"): used when publish_tf is false.
    width (int, 1024), height (int, 64)
    fov_up_deg (double, 3.0), fov_down_deg (double, -25.0)
    room_half_x (double, 5.0), room_half_y (double, 5.0), room_height (double, 3.0)
    orbit_radius (double, 1.5), orbit_period_sec (double, 20.0)
    spin_period_sec (double, 30.0)
    rate_hz (double, 10.0)
    num_sensors (int, 1): 1 = unchanged single-sensor behavior.
    sensor_yaw_offsets_deg (string, ""): comma-separated list of
        `num_sensors` yaw offsets in degrees, e.g. "0,180". Empty: evenly
        spaced (a plain ROS 2 parameter can't default to an empty array
        and still be told apart from "explicitly set to empty", the same
        issue MULTI_SENSOR_PLAN.md M1 hit for `sensor_names` -- a
        comma-separated string sidesteps it here).
    sensor_rig_offset_m (double, 0.5): each sensor's radial mount offset
        from the rig center.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def _yaw_quaternion(yaw):
    return float(np.sin(yaw / 2.0)), float(np.cos(yaw / 2.0))


def _yaw_rotation_matrix(yaw):
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
    return np.array(
        [[cos_yaw, -sin_yaw, 0.0], [sin_yaw, cos_yaw, 0.0], [0.0, 0.0, 1.0]]
    )


class FakeSensorPublisher(Node):
    def __init__(self):
        super().__init__("fake_sensor_publisher")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("sensor_frame", "velodyne")
        self.declare_parameter("pointcloud_topic", "pointcloud")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("transform_topic", "transform")
        self.declare_parameter("width", 1024)
        self.declare_parameter("height", 64)
        self.declare_parameter("fov_up_deg", 3.0)
        self.declare_parameter("fov_down_deg", -25.0)
        self.declare_parameter("room_half_x", 5.0)
        self.declare_parameter("room_half_y", 5.0)
        self.declare_parameter("room_height", 3.0)
        self.declare_parameter("orbit_radius", 1.5)
        self.declare_parameter("orbit_period_sec", 20.0)
        self.declare_parameter("spin_period_sec", 30.0)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("num_sensors", 1)
        self.declare_parameter("sensor_yaw_offsets_deg", "")
        self.declare_parameter("sensor_rig_offset_m", 0.5)

        self.world_frame = self.get_parameter("world_frame").value
        self.sensor_frame = self.get_parameter("sensor_frame").value
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        fov_up = np.deg2rad(self.get_parameter("fov_up_deg").value)
        fov_down = np.deg2rad(self.get_parameter("fov_down_deg").value)
        self.room_half_x = self.get_parameter("room_half_x").value
        self.room_half_y = self.get_parameter("room_half_y").value
        self.room_height = self.get_parameter("room_height").value
        self.orbit_radius = self.get_parameter("orbit_radius").value
        self.orbit_period = self.get_parameter("orbit_period_sec").value
        self.spin_period = self.get_parameter("spin_period_sec").value
        rate_hz = self.get_parameter("rate_hz").value

        # Precompute the sensor-local unit ray direction for every
        # (row, col) of the organized cloud once; only the sensor pose
        # changes per frame.
        elevations = np.linspace(fov_up, fov_down, height)
        azimuths = np.linspace(0.0, 2.0 * np.pi, width, endpoint=False)
        el, az = np.meshgrid(elevations, azimuths, indexing="ij")
        cos_el = np.cos(el)
        self.dirs_local = np.stack(
            [cos_el * np.cos(az), cos_el * np.sin(az), np.sin(el)], axis=-1
        ).reshape(-1, 3)  # (height*width, 3)
        # +-90deg of the forward (local +x, azimuth 0) axis: cos(azimuth) >= 0.
        forward_mask = (np.cos(az) >= 0.0).reshape(-1)
        self.dirs_local_forward = self.dirs_local[forward_mask]

        self.width = width
        self.height = height
        self.start_time = self.get_clock().now()

        self.num_sensors = self.get_parameter("num_sensors").value
        if self.num_sensors < 1:
            raise ValueError("num_sensors must be >= 1")
        yaw_offsets_str = self.get_parameter("sensor_yaw_offsets_deg").value
        if yaw_offsets_str:
            self.sensor_yaw_offsets_deg = [
                float(tok) for tok in yaw_offsets_str.split(",")
            ]
            if len(self.sensor_yaw_offsets_deg) != self.num_sensors:
                raise ValueError(
                    "sensor_yaw_offsets_deg has "
                    f"{len(self.sensor_yaw_offsets_deg)} entries, expected "
                    f"num_sensors={self.num_sensors}"
                )
        else:
            self.sensor_yaw_offsets_deg = [
                i * 360.0 / self.num_sensors for i in range(self.num_sensors)
            ]
        self.sensor_rig_offset_m = self.get_parameter("sensor_rig_offset_m").value

        pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.publish_tf = self.get_parameter("publish_tf").value

        if self.num_sensors == 1:
            # Single-sensor mode: identical to the pre-Phase-8 behavior.
            self.cloud_pub = self.create_publisher(PointCloud2, pointcloud_topic, 10)
            if self.publish_tf:
                self.tf_broadcaster = TransformBroadcaster(self)
            else:
                transform_topic = self.get_parameter("transform_topic").value
                self.transform_pub = self.create_publisher(
                    TransformStamped, transform_topic, 40
                )
        else:
            self.rig_frame = f"{self.sensor_frame}_rig"
            self.cloud_pubs = [
                self.create_publisher(PointCloud2, f"{pointcloud_topic}_{i}", 10)
                for i in range(self.num_sensors)
            ]
            if self.publish_tf:
                self.tf_broadcaster = TransformBroadcaster(self)
                self._publish_static_sensor_transforms()
            else:
                transform_topic = self.get_parameter("transform_topic").value
                self.transform_pub = self.create_publisher(
                    TransformStamped, transform_topic, 40
                )

        self.tick_count = 0
        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self.tick)

    def sensor_pose(self, t_sec):
        """Position (3,) and yaw (rad) of the sensor (or, with multiple
        sensors, the rig) at time t_sec, always inside the room."""
        omega_orbit = 2.0 * np.pi / self.orbit_period
        x = self.orbit_radius * np.cos(omega_orbit * t_sec)
        y = self.orbit_radius * np.sin(omega_orbit * t_sec)
        z = self.room_height / 2.0
        yaw = 2.0 * np.pi * t_sec / self.spin_period
        return np.array([x, y, z]), yaw

    def raycast(self, position, yaw, dirs_local=None):
        """Ray/box exit distances for every `dirs_local` direction (the
        full spherical scan by default), given the sensor is at `position`
        with yaw `yaw` (radians) inside the axis-aligned room. Returns
        local-frame hit points (N, 3)."""
        if dirs_local is None:
            dirs_local = self.dirs_local
        rot = _yaw_rotation_matrix(yaw)
        dirs_world = dirs_local @ rot.T  # (N, 3)

        box_min = np.array([-self.room_half_x, -self.room_half_y, 0.0])
        box_max = np.array([self.room_half_x, self.room_half_y, self.room_height])

        with np.errstate(divide="ignore", invalid="ignore"):
            t_pos = np.where(
                dirs_world > 0, (box_max - position) / dirs_world, np.inf
            )
            t_neg = np.where(
                dirs_world < 0, (box_min - position) / dirs_world, np.inf
            )
        t_exit = np.minimum(t_pos, t_neg).min(axis=1)  # (N,)

        hit_world = position + t_exit[:, None] * dirs_world
        # Back into the sensor-local frame for the PointCloud2 payload.
        hit_local = (hit_world - position) @ rot
        return hit_local.astype(np.float32)

    def _sensor_world_pose(self, rig_position, rig_yaw, sensor_index):
        """World position (3,) and yaw (rad) of sensor `sensor_index`,
        mounted `sensor_rig_offset_m` out from the rig center in the
        direction of its own yaw offset and rotated by that same offset,
        composed onto the rig's own (moving) world pose."""
        yaw_i = np.deg2rad(self.sensor_yaw_offsets_deg[sensor_index])
        offset_rig = self.sensor_rig_offset_m * np.array(
            [np.cos(yaw_i), np.sin(yaw_i), 0.0]
        )
        rig_rot = _yaw_rotation_matrix(rig_yaw)
        world_position = rig_position + rig_rot @ offset_rig
        world_yaw = rig_yaw + yaw_i
        return world_position, world_yaw

    def _publish_static_sensor_transforms(self):
        broadcaster = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        transforms = []
        for i in range(self.num_sensors):
            yaw_i = np.deg2rad(self.sensor_yaw_offsets_deg[i])
            offset_rig = self.sensor_rig_offset_m * np.array(
                [np.cos(yaw_i), np.sin(yaw_i), 0.0]
            )
            sin_half, cos_half = _yaw_quaternion(yaw_i)
            msg = TransformStamped()
            msg.header.stamp = now
            msg.header.frame_id = self.rig_frame
            msg.child_frame_id = f"{self.sensor_frame}_{i}"
            msg.transform.translation.x = float(offset_rig[0])
            msg.transform.translation.y = float(offset_rig[1])
            msg.transform.translation.z = float(offset_rig[2])
            msg.transform.rotation.x = 0.0
            msg.transform.rotation.y = 0.0
            msg.transform.rotation.z = sin_half
            msg.transform.rotation.w = cos_half
            transforms.append(msg)
        broadcaster.sendTransform(transforms)
        # Keep the broadcaster alive: tf2's static broadcaster relies on a
        # transient-local publisher, which needs the publisher object (and
        # therefore this reference) to outlive node startup.
        self._static_tf_broadcaster = broadcaster

    def _publish_transform(self, publisher_or_broadcaster, use_tf, stamp,
                            parent_frame, child_frame, position, yaw):
        sin_half, cos_half = _yaw_quaternion(yaw)
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame
        msg.transform.translation.x = float(position[0])
        msg.transform.translation.y = float(position[1])
        msg.transform.translation.z = float(position[2])
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = sin_half
        msg.transform.rotation.w = cos_half
        if use_tf:
            publisher_or_broadcaster.sendTransform(msg)
        else:
            publisher_or_broadcaster.publish(msg)

    def _make_cloud_msg(self, points_local, stamp, frame_id, width, height):
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = height
        cloud.width = width
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * width
        cloud.is_dense = True
        cloud.data = points_local.tobytes()
        return cloud

    def tick(self):
        if self.num_sensors == 1:
            self._tick_single_sensor()
        else:
            self._tick_multi_sensor()

    def _tick_single_sensor(self):
        now = self.get_clock().now()
        t_sec = (now - self.start_time).nanoseconds * 1e-9
        position, yaw = self.sensor_pose(t_sec)
        self.tick_count += 1

        # In transform-topic mode, publish the cloud at half the transform
        # rate so every cloud stamp is bracketed by two queued transforms,
        # exercising Transformer::lookupTransformQueue's interpolation
        # instead of the tf2_ros::Buffer path's own interpolation.
        publish_cloud = self.publish_tf or (self.tick_count % 2 == 0)

        if publish_cloud:
            points_local = self.raycast(position, yaw)
            cloud = self._make_cloud_msg(
                points_local, now.to_msg(), self.sensor_frame, self.width,
                self.height)
            self.cloud_pub.publish(cloud)

        self._publish_transform(
            self.tf_broadcaster if self.publish_tf else self.transform_pub,
            self.publish_tf, now.to_msg(), self.world_frame, self.sensor_frame,
            position, yaw)

    def _tick_multi_sensor(self):
        now = self.get_clock().now()
        t_sec = (now - self.start_time).nanoseconds * 1e-9
        rig_position, rig_yaw = self.sensor_pose(t_sec)
        self.tick_count += 1

        publish_cloud = self.publish_tf or (self.tick_count % 2 == 0)

        if publish_cloud:
            for i in range(self.num_sensors):
                sensor_position, sensor_yaw = self._sensor_world_pose(
                    rig_position, rig_yaw, i)
                points_local = self.raycast(
                    sensor_position, sensor_yaw, self.dirs_local_forward)
                cloud = self._make_cloud_msg(
                    points_local, now.to_msg(), f"{self.sensor_frame}_{i}",
                    len(points_local), 1)
                self.cloud_pubs[i].publish(cloud)

        if self.publish_tf:
            # Only the moving rig pose needs republishing every tick; each
            # sensor's own offset from the rig is a one-time static TF.
            self._publish_transform(
                self.tf_broadcaster, True, now.to_msg(), self.world_frame,
                self.rig_frame, rig_position, rig_yaw)
        else:
            # Queue mode: publish the rig's own pose (T_G_D) on
            # transform_topic; each sensor's fixed rig-relative extrinsic
            # (translation sensor_rig_offset_m in the direction of its own
            # yaw offset, rotation = that yaw offset) is a server-side
            # sensors.<name>.T_B_C parameter, not something this publisher
            # sends.
            self._publish_transform(
                self.transform_pub, False, now.to_msg(), self.world_frame,
                self.rig_frame, rig_position, rig_yaw)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
