"""Ported from ROS 1 `launch/eval/eval_cow_and_lady.launch`.

Runs `voxblox_eval` against pre-computed Cow & Lady TSDF/ESDF/occupancy
map files and opens RViz2 with `cfg/rviz/eval.rviz`. All defaults below
(including the map file paths) match the ROS 1 original except `gt_file_path`
/ `voxblox_file_path` / `voxblox_esdf_file_path` / `voxblox_occ_file_path`,
whose ROS 1 defaults were hard-coded to the original author's machine
(`/media/yuepan/...`) -- per ROS2_PORT_PLAN.md §6 Phase 10 step 3, those
now default to empty and must be passed explicitly:

    ros2 launch voxfield_ros eval_cow_and_lady.launch.py \\
        gt_file_path:=<path/to/cow_and_lady_gt.ply> \\
        voxblox_file_path:=<path/to/map.tsdf> \\
        voxblox_esdf_file_path:=<path/to/map.esdf> \\
        voxblox_occ_file_path:=<path/to/map.occ>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("voxfield_ros")
    rviz_config = os.path.join(pkg_share, "cfg", "rviz", "eval.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gt_file_path", default_value=""),
            DeclareLaunchArgument("voxblox_file_path", default_value=""),
            DeclareLaunchArgument("voxblox_esdf_file_path", default_value=""),
            DeclareLaunchArgument("voxblox_occ_file_path", default_value=""),
            Node(
                package="voxfield_ros",
                executable="voxblox_eval",
                name="voxblox_eval",
                output="screen",
                parameters=[
                    {
                        "color_mode": "normals",
                        "frame_id": "world",
                        "verbose": True,
                        "visualize": True,
                        "recolor_by_error": False,
                        "eval_esdf": True,
                        "use_occ_ref": True,
                        "slice_level": 0.8,
                        "error_limit_m": 0.2,
                        "eval_only_positive": False,
                        "gt_file_path": LaunchConfiguration("gt_file_path"),
                        "voxblox_file_path": LaunchConfiguration("voxblox_file_path"),
                        "voxblox_esdf_file_path": LaunchConfiguration(
                            "voxblox_esdf_file_path"
                        ),
                        "voxblox_occ_file_path": LaunchConfiguration(
                            "voxblox_occ_file_path"
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rvizvisualisation",
                output="log",
                arguments=["-d", rviz_config],
            ),
        ]
    )
