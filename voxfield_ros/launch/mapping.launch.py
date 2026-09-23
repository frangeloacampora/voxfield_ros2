"""Generic mapping launch file (ROS2_PORT_PLAN.md §6 Phase 10).

Starts one `<method>_server` executable with a dataset's parameter/
calibration YAML, optional rosbag playback, and optional RViz2 -- the
common shape of every `voxfield_ros/launch/*_launch/*.launch` file from
ROS 1. Per-dataset defaults below (param/calib file, robot model, point
cloud/transform topics, RViz config, bag playback speed) are taken from
that dataset's most common ROS 1 launch file; the small per-dataset-work
wrappers in this directory (e.g. `kitti_voxfield.launch.py`) fix `method`/
`dataset` and pass any per-file deviation the original ROS 1 file had
(different topic, different speed, ...) as an explicit override.

`robot_model_file` accepts the sentinel value `__none__` to mean "do not
set this parameter at all" (as opposed to `''`, which means "use this
dataset's default"), matching the two ROS 1 files
(`kitti_fiesta.launch`/`kitti_voxedt.launch`) that never set
`robot_model_file` and so left it at the server's own default (an empty
string -- see `TsdfServer::robot_model_file_` / `NpTsdfServer::
robot_model_file_`).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Per-dataset defaults, taken from that dataset's ROS 1 launch files (see
# docs/ROS2_PORT_NOTES.md, Phase 10, for the file-by-file source table).
_DATASET_DEFAULTS = {
    "cow": {
        "param_file": "cow_param.yaml",
        "calib_file": "cow_calib.yaml",
        "robot_model_file": "camera.dae",
        "pointcloud_topic": "/camera/depth_registered/points",
        "transform_topic": "/kinect/vrpn_client/estimated_transform",
        "rviz_config": "cow_5cm.rviz",
        "speed": "0.25",
    },
    "kitti": {
        "param_file": "kitti_param.yaml",
        "calib_file": "kitti_calib.yaml",
        "robot_model_file": "car.dae",
        "pointcloud_topic": "/velodyne_points",
        "transform_topic": "",
        "rviz_config": "kitti_25cm.rviz",
        "speed": "0.5",
    },
    "mai": {
        "param_file": "mai_param.yaml",
        "calib_file": "mai_calib.yaml",
        "robot_model_file": "car.dae",
        "pointcloud_topic": "/velodyne_points",
        "transform_topic": "",
        "rviz_config": "mai_25cm.rviz",
        "speed": "0.25",
    },
    "basement": {
        "param_file": "basement_param.yaml",
        "calib_file": "basement_calib.yaml",
        "robot_model_file": "hummingbird.mesh",
        "pointcloud_topic": "/velodyne_points",
        "transform_topic": "",
        "rviz_config": "basement_10cm.rviz",
        "speed": "0.5",
    },
    "vicon": {
        "param_file": "vicon_param.yaml",
        "calib_file": "vicon_calib.yaml",
        "robot_model_file": "drone_x500.dae",
        "pointcloud_topic": "/camera/depth_registered/points",
        "transform_topic": "/mavros/setpoint_position/local",
        "rviz_config": "vicon_10cm.rviz",
        "speed": "1.0",
    },
}

_NONE_SENTINEL = "__none__"


def _resolve(value, default):
    return value if value else default


def _to_bool(value):
    return value.lower() in ("true", "1", "yes")


def launch_setup(context, *args, **kwargs):
    method = LaunchConfiguration("method").perform(context)
    dataset = LaunchConfiguration("dataset").perform(context)
    if dataset not in _DATASET_DEFAULTS:
        raise RuntimeError(
            f"Unknown dataset '{dataset}'; expected one of "
            f"{sorted(_DATASET_DEFAULTS)}")
    defaults = _DATASET_DEFAULTS[dataset]

    pkg_share = get_package_share_directory("voxfield_ros")

    bag_file = LaunchConfiguration("bag_file").perform(context)
    play_bag = _to_bool(LaunchConfiguration("play_bag").perform(context))
    speed = _resolve(LaunchConfiguration("speed").perform(context), defaults["speed"])
    do_rviz = _to_bool(LaunchConfiguration("rviz").perform(context))
    rviz_config = _resolve(
        LaunchConfiguration("rviz_config").perform(context),
        os.path.join(pkg_share, "cfg", "rviz", defaults["rviz_config"]),
    )
    robot_model_file_arg = LaunchConfiguration("robot_model_file").perform(context)
    if robot_model_file_arg == _NONE_SENTINEL:
        robot_model_file = ""
    else:
        robot_model_file = _resolve(
            robot_model_file_arg,
            os.path.join(pkg_share, "cfg", "model", defaults["robot_model_file"]),
        )
    pointcloud_topic = _resolve(
        LaunchConfiguration("pointcloud_topic").perform(context),
        defaults["pointcloud_topic"],
    )
    transform_topic = _resolve(
        LaunchConfiguration("transform_topic").perform(context),
        defaults["transform_topic"],
    )
    use_sim_time = _to_bool(LaunchConfiguration("use_sim_time").perform(context))

    param_file = os.path.join(pkg_share, "cfg", "param", defaults["param_file"])
    calib_file = os.path.join(pkg_share, "cfg", "calib", defaults["calib_file"])

    remappings = [("pointcloud", pointcloud_topic)]
    if transform_topic:
        remappings.append(("transform", transform_topic))

    actions = []

    if bag_file and play_bag:
        actions.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_file, "--clock", "-r", speed],
                output="screen",
            )
        )

    node_parameters = [param_file, calib_file, {"use_sim_time": use_sim_time}]
    if robot_model_file:
        node_parameters.append({"robot_model_file": robot_model_file})

    actions.append(
        Node(
            package="voxfield_ros",
            executable=f"{method}_server",
            name="voxfield_node",
            output="screen",
            parameters=node_parameters,
            remappings=remappings,
        )
    )

    if do_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rvizvisualisation",
                output="log",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "method",
                description=(
                    "Mapping method / executable: voxfield, voxblox, fiesta, "
                    "voxedt, np_tsdf, or tsdf (selects '<method>_server')."
                ),
            ),
            DeclareLaunchArgument(
                "dataset",
                description=(
                    "Dataset preset providing defaults: one of "
                    f"{sorted(_DATASET_DEFAULTS)}."
                ),
            ),
            DeclareLaunchArgument(
                "bag_file",
                default_value="",
                description="ROS 2 bag directory to play. Empty: do not play a bag.",
            ),
            DeclareLaunchArgument("play_bag", default_value="true"),
            DeclareLaunchArgument(
                "speed",
                default_value="",
                description="Bag playback rate. Empty: per-dataset default.",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="",
                description="Empty: per-dataset default under cfg/rviz/.",
            ),
            DeclareLaunchArgument(
                "robot_model_file",
                default_value="",
                description=(
                    "Empty: per-dataset default under cfg/model/. "
                    f"'{_NONE_SENTINEL}': don't set this parameter at all."
                ),
            ),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="",
                description="Empty: per-dataset default.",
            ),
            DeclareLaunchArgument(
                "transform_topic",
                default_value="",
                description=(
                    "Empty: per-dataset default (itself often empty, for "
                    "datasets that use TF instead of a transform topic)."
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
