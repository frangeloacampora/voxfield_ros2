"""Multi-sensor mapping launch file (MULTI_SENSOR_PLAN.md §8 Phase 8 step 3).

The multi-sensor counterpart to `mapping.launch.py`: starts one
`<method>_server` executable with a map-global parameter YAML
(`param_file`) plus a `sensor_names`/`sensors.*` YAML (`sensors_file`,
MULTI_SENSOR_PLAN.md §7.2), optional rosbag playback, and optional RViz2.
Built for (and, for now, only shipped with configs for) the Athena robot's
dual-Livox (+ optional dual-RGB-D) rig -- see `cfg/multi_sensor/athena_*.yaml`.

Unlike `mapping.launch.py`'s per-dataset topic remap (one `pointcloud`
subscription), each sensor's real topic is already set by `sensors_file`
itself (`sensors.<name>.topic`), so no point cloud remapping happens here.
The only remap this file adds is TF, and only when `tf_remap_prefix` is
set: Athena's bag publishes a *namespaced* TF tree (`/athena/tf`,
`/athena/tf_static`) rather than the global `/tf`/`/tf_static`
`tf2_ros::TransformListener` (used by both the server and RViz2) expects,
so both the server and RViz2 need `/tf:=<prefix>/tf` and
`/tf_static:=<prefix>/tf_static` remaps -- confirmed necessary for RViz2 too,
not just the server (MULTI_SENSOR_PLAN.md §9 pitfall 9).

With `rgbd:=true`, also starts one `ComposableNodeContainer` per Athena
camera (`front`, `back`), each holding an `image_transport::Republisher`
(decodes that camera's `.../compressedDepth` topic to a raw depth image;
Athena's bag has no plain point cloud for its RGB-D cameras, only
compressed depth + camera_info -- see docs/MULTI_SENSOR_NOTES.md §3) feeding
a `depth_image_proc::PointCloudXyzNode` (turns the raw depth image +
camera_info into the point cloud `sensors_file`'s corresponding
`sensors.<camera>_rgbd.topic` should be set to). Component plugin names
and remap keys (`in_transport`/`out_transport` parameters + `in/<transport>`/
`out` remaps for `image_transport::Republisher`; `image_rect`/`camera_info`/
`points` remaps for `depth_image_proc::PointCloudXyzNode`) were confirmed
against this machine's installed `image_transport`/`depth_image_proc`
(`ros2 component types`, `ros2 param list` on each node standalone) while
writing this file, not assumed from memory -- see docs/MULTI_SENSOR_NOTES.md
for how.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Athena's RGB-D topic layout (docs/ROS2_PORT_NOTES.md §3 / this plan's §3):
# only compressed depth + camera_info are recorded, no plain point cloud.
_ATHENA_RGBD_CAMERAS = ["front", "back"]


def _to_bool(value):
    return value.lower() in ("true", "1", "yes")


def _rgbd_container_actions(camera):
    """One ComposableNodeContainer for `camera` ("front" or "back"):
    compressedDepth -> raw (image_transport::Republisher) -> point cloud
    (depth_image_proc::PointCloudXyzNode), remapped onto Athena's real
    topics. The resulting point cloud is published on
    `/athena/<camera>_rgbd/points`; `sensors_file` must set
    `sensors.<camera>_rgbd.topic` to that same topic to actually use it.
    """
    depth_base = f"/athena/{camera}_rgbd/depth/image_raw"
    camera_info_topic = f"/athena/{camera}_rgbd/depth/camera_info"
    raw_depth_topic = f"/athena/{camera}_rgbd/depth/image_raw_decoded"
    points_topic = f"/athena/{camera}_rgbd/points"

    republish_node = ComposableNode(
        package="image_transport",
        plugin="image_transport::Republisher",
        name=f"{camera}_rgbd_republish",
        parameters=[{"in_transport": "compressedDepth", "out_transport": "raw"}],
        remappings=[
            ("in/compressedDepth", f"{depth_base}/compressedDepth"),
            ("out", raw_depth_topic),
        ],
    )
    point_cloud_node = ComposableNode(
        package="depth_image_proc",
        plugin="depth_image_proc::PointCloudXyzNode",
        name=f"{camera}_rgbd_point_cloud_xyz",
        remappings=[
            ("image_rect", raw_depth_topic),
            ("camera_info", camera_info_topic),
            ("points", points_topic),
        ],
    )
    return [
        ComposableNodeContainer(
            name=f"{camera}_rgbd_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[republish_node, point_cloud_node],
            output="screen",
        )
    ]


def launch_setup(context, *args, **kwargs):
    method = LaunchConfiguration("method").perform(context)
    param_file = LaunchConfiguration("param_file").perform(context)
    sensors_file = LaunchConfiguration("sensors_file").perform(context)
    bag_file = LaunchConfiguration("bag_file").perform(context)
    play_bag = _to_bool(LaunchConfiguration("play_bag").perform(context))
    speed = LaunchConfiguration("speed").perform(context)
    start_offset = LaunchConfiguration("start_offset").perform(context)
    do_rviz = _to_bool(LaunchConfiguration("rviz").perform(context))
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    use_sim_time = _to_bool(LaunchConfiguration("use_sim_time").perform(context))
    tf_remap_prefix = LaunchConfiguration("tf_remap_prefix").perform(context)
    rgbd = _to_bool(LaunchConfiguration("rgbd").perform(context))

    pkg_share = get_package_share_directory("voxfield_ros")
    if not rviz_config:
        rviz_config = os.path.join(
            pkg_share, "cfg", "rviz", "multi_sensor.rviz")

    actions = []

    if bag_file and play_bag:
        qos_override_file = os.path.join(
            pkg_share, "cfg", "multi_sensor", "tf_static_qos_override.yaml")
        bag_cmd = [
            "ros2", "bag", "play", bag_file, "--clock", "-r", speed,
            "--qos-profile-overrides-path", qos_override_file,
        ]
        if start_offset:
            bag_cmd += ["--start-offset", start_offset]
        actions.append(ExecuteProcess(cmd=bag_cmd, output="screen"))

    if rgbd:
        for camera in _ATHENA_RGBD_CAMERAS:
            actions.extend(_rgbd_container_actions(camera))

    server_remappings = []
    if tf_remap_prefix:
        server_remappings.append(("/tf", f"{tf_remap_prefix}/tf"))
        server_remappings.append(("/tf_static", f"{tf_remap_prefix}/tf_static"))

    actions.append(
        Node(
            package="voxfield_ros",
            executable=f"{method}_server",
            name="voxfield_node",
            output="screen",
            parameters=[
                param_file,
                sensors_file,
                {"use_sim_time": use_sim_time},
            ],
            remappings=server_remappings,
        )
    )

    if do_rviz:
        rviz_remappings = list(server_remappings)  # RViz2 needs it too (§9 pitfall 9).
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rvizvisualisation",
                output="log",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=rviz_remappings,
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
                "param_file",
                description="Map-global parameter YAML (MULTI_SENSOR_PLAN.md §7.5).",
            ),
            DeclareLaunchArgument(
                "sensors_file",
                description=(
                    "sensor_names + sensors.*.* YAML (MULTI_SENSOR_PLAN.md §7.2)."
                ),
            ),
            DeclareLaunchArgument(
                "bag_file",
                default_value="",
                description="ROS 2 bag directory to play. Empty: do not play a bag.",
            ),
            DeclareLaunchArgument("play_bag", default_value="true"),
            DeclareLaunchArgument("speed", default_value="1.0"),
            DeclareLaunchArgument(
                "start_offset",
                default_value="",
                description="Seconds into the bag to start playback. Empty: from the start.",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="",
                description="Empty: cfg/rviz/multi_sensor.rviz.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "tf_remap_prefix",
                default_value="",
                description=(
                    "If set (e.g. '/athena'), remaps /tf and /tf_static to "
                    "<prefix>/tf and <prefix>/tf_static on both the server "
                    "and RViz2, for a bag with a namespaced TF tree."
                ),
            ),
            DeclareLaunchArgument(
                "rgbd",
                default_value="false",
                description=(
                    "Start the compressedDepth -> raw -> point cloud "
                    "pipeline for Athena's front/back RGB-D cameras."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
