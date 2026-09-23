"""Ported from ROS 1 `launch/voxblox_launch/kitti_voxblox.launch`.

`ros2 launch voxfield_ros kitti_voxblox.launch.py bag_file:=<path/to/ros2_bag>`

Note: the ROS 1 original used /velodyne_points_filtered and speed 0.25
here (unlike kitti_voxfield.launch's /velodyne_points and speed 0.5) --
preserved as this wrapper's defaults.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("voxfield_ros")
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag_file", default_value=""),
            DeclareLaunchArgument("play_bag", default_value="true"),
            DeclareLaunchArgument("speed", default_value="0.25"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=""),
            DeclareLaunchArgument("robot_model_file", default_value=""),
            DeclareLaunchArgument("pointcloud_topic", default_value="/velodyne_points_filtered"),
            DeclareLaunchArgument("transform_topic", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "mapping.launch.py")
                ),
                launch_arguments={
                    "method": "voxblox",
                    "dataset": "kitti",
                    "bag_file": LaunchConfiguration("bag_file"),
                    "play_bag": LaunchConfiguration("play_bag"),
                    "speed": LaunchConfiguration("speed"),
                    "rviz": LaunchConfiguration("rviz"),
                    "rviz_config": LaunchConfiguration("rviz_config"),
                    "robot_model_file": LaunchConfiguration("robot_model_file"),
                    "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                    "transform_topic": LaunchConfiguration("transform_topic"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ]
    )
