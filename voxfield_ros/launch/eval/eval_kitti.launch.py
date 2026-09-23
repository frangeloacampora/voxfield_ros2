"""Ported from ROS 1 `launch/eval/eval_kitti.launch`.

`voxblox_file_path` / `gt_file_path` were hard-coded to the original
author's machine (`/Users/helen/data/...`); per ROS2_PORT_PLAN.md §6
Phase 10 step 3, they now default to empty and must be passed explicitly:

    ros2 launch voxfield_ros eval_kitti.launch.py \\
        voxblox_file_path:=<path/to/map.vxblx> gt_file_path:=<path/to/gt.ply>

(The ROS 1 original's `<rosparam file=".../cfg/euroc_dataset.yaml"/>` line
was already commented out there, so there is nothing to carry over.)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("voxblox_file_path", default_value=""),
            DeclareLaunchArgument("gt_file_path", default_value=""),
            Node(
                package="voxfield_ros",
                executable="voxblox_eval",
                name="voxblox_eval",
                output="screen",
                parameters=[
                    {
                        "color_mode": "colors",
                        "frame_id": "world",
                        "verbose": True,
                        "visualize": False,
                        "recolor_by_error": False,
                        "voxblox_file_path": LaunchConfiguration("voxblox_file_path"),
                        "gt_file_path": LaunchConfiguration("gt_file_path"),
                    }
                ],
            ),
        ]
    )
