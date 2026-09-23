"""Ported from ROS 1 `launch/voxedt_launch/cow_voxedt.launch`.

`ros2 launch voxfield_ros cow_voxedt.launch.py bag_file:=<path/to/ros2_bag>`
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
            DeclareLaunchArgument("speed", default_value=""),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=""),
            DeclareLaunchArgument("robot_model_file", default_value=""),
            DeclareLaunchArgument("pointcloud_topic", default_value=""),
            DeclareLaunchArgument("transform_topic", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "mapping.launch.py")
                ),
                launch_arguments={
                    "method": "voxedt",
                    "dataset": "cow",
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
