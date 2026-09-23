"""Ported from ROS 1 `launch/eval/eval_euroc.launch`.

`voxblox_file_path` / `gt_file_path` were hard-coded to the original
author's machine (`/Users/helen/data/...`); per ROS2_PORT_PLAN.md §6
Phase 10 step 3, they now default to empty and must be passed explicitly:

    ros2 launch voxfield_ros eval_euroc.launch.py \\
        voxblox_file_path:=<path/to/map.vxblx> gt_file_path:=<path/to/gt.ply>

Known upstream issue (not fixed, see docs/ROS2_PORT_NOTES.md): the ROS 1
original also loaded `<rosparam file="$(find voxfield_ros)/cfg/
euroc_dataset.yaml"/>`, but `cfg/euroc_dataset.yaml` does not exist
anywhere in this repository (in ROS 1 or here) -- that line was already
dead on arrival. It is omitted here rather than ported byte-for-byte,
since in ROS 2 a `parameters=[<missing file>]` entry aborts the whole
launch instead of just failing to set a few params.
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
                        "color_mode": "normals",
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
