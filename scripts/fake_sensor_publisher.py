#!/usr/bin/env python3
"""Synthetic sensor publisher for the dataset-free smoke test
(ROS2_PORT_PLAN.md §6 Phase 12 step 1).

Publishes an organized `sensor_msgs/PointCloud2` of a closed, axis-aligned
box room (walls + floor + ceiling; default 10x10x3 m) as seen by a
spinning-lidar-style sensor circling near the room's center, using the
same `width`/`height`/`fov_up`/`fov_down` organized-cloud convention as
`kitti_calib.yaml` by default -- every ray from the sensor's origin is
guaranteed to hit a wall, floor, or ceiling (the room is airtight and the
sensor never leaves it), so the cloud is fully dense (`is_dense: true`,
no NaNs) every frame.

Also broadcasts a moving `world -> <sensor_frame>` TF at the same rate,
so both the projective (NpTsdfServer/VoxfieldServer) and non-projective
(TsdfServer/VoxbloxServer/FiestaServer/VoxedtServer) integration paths
have real geometry and real motion to integrate, without needing an
external dataset.

ROS parameters (all optional, defaults match kitti_calib.yaml):
    world_frame (string, "world")
    sensor_frame (string, "velodyne")
    pointcloud_topic (string, "pointcloud")
    width (int, 1024), height (int, 64)
    fov_up_deg (double, 3.0), fov_down_deg (double, -25.0)
    room_half_x (double, 5.0), room_half_y (double, 5.0), room_height (double, 3.0)
    orbit_radius (double, 1.5), orbit_period_sec (double, 20.0)
    spin_period_sec (double, 30.0)
    rate_hz (double, 10.0)
"""
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


class FakeSensorPublisher(Node):
    def __init__(self):
        super().__init__("fake_sensor_publisher")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("sensor_frame", "velodyne")
        self.declare_parameter("pointcloud_topic", "pointcloud")
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

        self.width = width
        self.height = height
        self.start_time = self.get_clock().now()

        pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.cloud_pub = self.create_publisher(PointCloud2, pointcloud_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self.tick)

    def sensor_pose(self, t_sec):
        """Position (3,) and yaw (rad) of the sensor at time t_sec, always
        inside the room."""
        omega_orbit = 2.0 * np.pi / self.orbit_period
        x = self.orbit_radius * np.cos(omega_orbit * t_sec)
        y = self.orbit_radius * np.sin(omega_orbit * t_sec)
        z = self.room_height / 2.0
        yaw = 2.0 * np.pi * t_sec / self.spin_period
        return np.array([x, y, z]), yaw

    def raycast(self, position, yaw):
        """Ray/box exit distances for every precomputed local direction,
        given the sensor is at `position` with yaw `yaw` (radians) inside
        the axis-aligned room. Returns local-frame hit points (N, 3)."""
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        # Rotation from sensor-local to world (yaw about Z only).
        rot = np.array(
            [[cos_yaw, -sin_yaw, 0.0], [sin_yaw, cos_yaw, 0.0], [0.0, 0.0, 1.0]]
        )
        dirs_world = self.dirs_local @ rot.T  # (N, 3)

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

    def tick(self):
        now = self.get_clock().now()
        t_sec = (now - self.start_time).nanoseconds * 1e-9
        position, yaw = self.sensor_pose(t_sec)

        points_local = self.raycast(position, yaw)

        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = self.sensor_frame

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = self.height
        cloud.width = self.width
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * self.width
        cloud.is_dense = True
        cloud.data = points_local.tobytes()
        self.cloud_pub.publish(cloud)

        cos_yaw2, sin_yaw2 = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = self.sensor_frame
        tf_msg.transform.translation.x = float(position[0])
        tf_msg.transform.translation.y = float(position[1])
        tf_msg.transform.translation.z = float(position[2])
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = float(sin_yaw2)
        tf_msg.transform.rotation.w = float(cos_yaw2)
        self.tf_broadcaster.sendTransform(tf_msg)


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
