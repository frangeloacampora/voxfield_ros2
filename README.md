This repository represents the official implementation of the paper:

## Voxfield: Non-Projective Signed Distance Fields for Online Planning and 3D Reconstruction

*Yue Pan*, *Yves Kompis*, *Luca Bartolomei*, *Ruben Mascaro*, *Cyrill Stachniss*, *Margarita Chli*

[ETH Zurich](https://asl.ethz.ch/v4rl.html) | [University of Bonn](https://www.ipb.uni-bonn.de/)

[**1-min demo video**](https://youtu.be/sPNzTOLqb2I) | [**7-min presentation**](https://www.youtube.com/watch?v=4HB4RXChrbg&t=3s) | [**paper**](https://www.research-collection.ethz.ch/handle/20.500.11850/560719)

![Pipeline](./docs/assets/pipeline-voxfield.png)

---

Voxfield is an improved version of the volumetric mapping framework [**Voxblox**](https://github.com/ethz-asl/voxblox) on both the mapping accuracy and efficiency. 
Voxfield is heavily based on the original [**Voxblox**](https://github.com/ethz-asl/voxblox) implementation, with the additional capacity of a non-projective TSDF integration and an efficient ESDF integration based on TSDF map. The constructed TSDF and ESDF map can be used for 3D reconstruction and robot path planning in real-time.

This repository also provides the implementation of other state-of-the-art methods ([Voxblox](https://arxiv.org/abs/1611.03631), [FIESTA](https://arxiv.org/abs/1903.02144), [EDT](https://arxiv.org/abs/2105.04419)) on this task. A comparison of these methods' workflow are shown below.

![Pipeline](./docs/assets/comparison.png)

Voxfield can be seamlessly integrated into those projects that originally use Voxblox as their volumetric mapping backbone (Cblox, Voxgraph, Kimera, etc). In addition, we provide an example on a multi-resolution panoptic mapping framework [**Panmap**](https://github.com/VIS4ROB-lab/voxfield-panmap) for high-fidelity large-scale semantic reconstruction.

**This is a ROS 2 (Jazzy) port.** The packages, C++ namespace, include paths, protobuf package, message package, and RViz plugin are all renamed from `voxblox*` to `voxfield*` (see `docs/ROS2_PORT_NOTES.md` for the full rename log). This means Voxfield no longer masquerades as Voxblox: it can be installed and used *alongside* an unrelated `voxblox` ROS 2 install in the same workspace without symbol, protobuf, message, or RViz-plugin clashes. The trade-off is that projects built against upstream Voxblox (Cblox, Voxgraph, Kimera, Panmap) need their includes (`voxblox/...` → `voxfield/...`), namespace (`voxblox::` → `voxfield::`), and message package (`voxblox_msgs` → `voxfield_msgs`) updated to depend on Voxfield instead. Mapping algorithms, default parameter values, and all ROS topic/service/parameter names are otherwise unchanged from the original ROS 1 Voxfield, and the protobuf message format is wire-compatible, so `.tsdf`/`.esdf`/`.vxblx` map files saved by ROS 1 Voxfield or Voxblox still load here.

ROS 1 support lives on in this repository's git history (tag/branch predating the ROS 2 port); this branch is ROS 2-only going forward.

## Installation

Prerequisites: Ubuntu 24.04 with ROS 2 Jazzy installed (`/opt/ros/jazzy`).

```
mkdir -p ~/voxfield_ws/src
cd ~/voxfield_ws/src
git clone <this-repo-url> voxfield_ros2
cd ~/voxfield_ws
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
```

If a separate `voxblox` ROS 2 install is also sourced in your shell (e.g. from an unrelated project), Voxfield builds and runs alongside it without conflict — see `docs/ROS2_PORT_NOTES.md` §2.1 for how that's verified.

## Instructions

- To run the non-projective TSDF mapping and ESDF mapping of the proposed Voxfield, use the executables: ```np_tsdf_server``` and ```voxfield_server```. 
- To run the original TSDF mapping and ESDF mapping of Voxblox, use the executables: ```tsdf_server``` and ```voxblox_server```. 
- To run the ESDF mapping of FIESTA, use the executables: ```fiesta_server```.
- To run the ESDF mapping of EDT, use the executables: ```voxedt_server```.
- List of the ros services can be found [here](https://voxblox.readthedocs.io/en/latest/pages/The-Voxblox-Node.html), which should be the same as Voxblox.

## ROS interface

Topic/service names below are relative to the node (`~` = node-private); this repo's launch files all name the node `voxfield_node`, so e.g. `~/mesh` is `/voxfield_node/mesh`. Not every server exposes every topic below — the "Servers" column says which do. All servers accept the same `pointcloud`/`transform` input shape; what differs is which outputs they produce (ESDF-capable servers add `~/esdf_*`, FIESTA/EDT add occupancy-map saving, etc).

### Input

| Topic/service | Type | Servers | Notes |
|---|---|---|---|
| `pointcloud` | `sensor_msgs/msg/PointCloud2` | all | The sensor data to integrate. Remap this to your sensor's topic. Legacy (single-sensor) mode only: with `sensor_names` set, each sensor subscribes to its own `sensors.<name>.topic` instead (see [Multiple sensors](#multiple-sensors-one-map)). |
| `freespace_pointcloud` | `sensor_msgs/msg/PointCloud2` | all | Optional; only subscribed if `use_freespace_pointcloud: true`. |
| `transform` | `geometry_msgs/msg/TransformStamped` | all | Only subscribed if `use_tf_transforms: false` (e.g. the `cow`/`vicon` presets); otherwise pose comes from TF. Queued and interpolated between samples, not just exact-matched. |
| `~/intensity_image` | `sensor_msgs/msg/Image` | `intensity_server` | Via `cv_bridge`. |
| `~/tsdf_map_in` | `voxfield_msgs/msg/Layer` | all | Merge in an externally-published TSDF layer. |
| `~/esdf_map_in` | `voxfield_msgs/msg/Layer` | ESDF-capable | Merge in an externally-published ESDF layer. |

**Pose:** by default (`use_tf_transforms: true`), looked up via `tf2_ros::Buffer` as `world_frame ← sensor_frame` (both ROS params; an empty `sensor_frame` uses the cloud's own `header.frame_id`) at each point cloud's timestamp — so `/tf`/`/tf_static` (or a bag played with `--clock`) must actually carry that transform. With `use_tf_transforms: false`, poses instead come from the `transform` topic above.

### Output

| Topic | Type | Servers | Contents |
|---|---|---|---|
| `~/mesh` | `voxfield_msgs/msg/Mesh` (latched) | all | The incremental reconstructed surface mesh. View live in RViz2 via `voxfield_rviz_plugin/VoxfieldMesh` (see `cfg/rviz/*.rviz`). |
| `~/surface_pointcloud` | `sensor_msgs/msg/PointCloud2` | all | Points on the reconstructed surface. |
| `~/tsdf_pointcloud` | `sensor_msgs/msg/PointCloud2` (`pcl::PointXYZI`) | all | Full TSDF; `intensity` = signed distance at that voxel. |
| `~/gsdf_pointcloud` | `sensor_msgs/msg/PointCloud2` (`pcl::PointXYZI`) | `np_tsdf_server`, `voxfield_server` | Signed distance gradient field. |
| `~/esdf_pointcloud` | `sensor_msgs/msg/PointCloud2` (`pcl::PointXYZI`) | ESDF-capable | Full ESDF; `intensity` = Euclidean signed distance. |
| `~/tsdf_slice` / `~/gsdf_slice` / `~/esdf_slice` / `~/esdf_error_slice` | `sensor_msgs/msg/PointCloud2` (`pcl::PointXYZI`) | as above | Same `PointXYZI` shape as their full-volume counterparts, but just one horizontal slice at `slice_level` (cheap top-down view). |
| `~/occupied_nodes` | `visualization_msgs/msg/MarkerArray` | all | Occupied voxels as cube markers. |
| `~/traversable` | `sensor_msgs/msg/PointCloud2` | ESDF-capable | Voxels with ESDF distance ≥ `traversability_radius` (only published if `publish_traversable: true`). |
| `~/Robot_model` | `visualization_msgs/msg/Marker` | all | `MESH_RESOURCE` marker at the current sensor pose, pointing at `robot_model_file`. |
| `~/tsdf_map_out` | `voxfield_msgs/msg/Layer` | all | Raw TSDF voxel layer (for another node to consume, not for viewing — see below). |
| `~/esdf_map_out` | `voxfield_msgs/msg/Layer` | ESDF-capable | Raw ESDF voxel layer. |
| `~/icp_transform` | `geometry_msgs/msg/TransformStamped` (latched) | all | ICP correction; only published if `enable_icp: true`. Also broadcast on `/tf`. |
| `~/intensity_pointcloud` / `~/intensity_mesh` | `sensor_msgs/msg/PointCloud2` / `voxfield_msgs/msg/Mesh` | `intensity_server` | Colored by intensity instead of geometry. |

`~/tsdf_map_out`/`~/esdf_map_out` and their `_in` counterparts carry the **raw voxel grid** (`Block[]`, each voxel packed as `uint32[]`), not a point cloud — that's how two nodes share a live map (e.g. a planner subscribing to the mapper's output). The `~/*_pointcloud`/`~/*_slice` topics are the human/RViz-facing view of the same data, resampled into `PointXYZI`.

### Services

| Service | Type | Servers | Effect |
|---|---|---|---|
| `~/generate_mesh` | `std_srvs/srv/Empty` | all | Force a full mesh regeneration. If the `mesh_filename` param is set, also writes an ASCII PLY to that path. |
| `~/clear_map` | `std_srvs/srv/Empty` | all | Clear the whole map. |
| `~/save_map` / `~/load_map` | `voxfield_msgs/srv/FilePath` | all | Save/load the TSDF layer as a binary file. `voxfield_server`, `voxblox_server`, `fiesta_server` and `voxedt_server` also save/load their ESDF layer in the same file (TSDF first, then ESDF). |
| `~/save_esdf_map` | `voxfield_msgs/srv/FilePath` | `voxfield_server`, `voxblox_server`, `fiesta_server`, `voxedt_server` | Save just the ESDF layer, replacing the file (the format `voxblox_eval`'s `voxblox_esdf_file_path` expects). |
| `~/save_occ_map` / `~/save_all_map` | `voxfield_msgs/srv/FilePath` | `fiesta_server`, `voxedt_server` | Save the occupancy layer (replacing the file) / everything (`<path>.tsdf`, `<path>.esdf`, `<path>.occ`). |
| `~/publish_pointclouds` | `std_srvs/srv/Empty` | all | Force-publish every pointcloud output once, bypassing the `publish_pointclouds`/`publish_slices` param gating. |
| `~/publish_map` | `std_srvs/srv/Empty` | all | Force-publish `~/tsdf_map_out`/`~/esdf_map_out` once. |

`FilePath`'s request is just `string file_path`; the response is empty (ROS 2 has no service-call failure channel, so a failed save/load logs `RCLCPP_ERROR` rather than returning an error to the caller — check the node's log, not the response).

### On-disk map file format

`~/save_map`/`~/save_esdf_map`/`~/save_occ_map`/`~/save_all_map` all write the same binary protobuf layer format (`voxfield::io::SaveLayer`) — the on-disk equivalent of a `voxfield_msgs/msg/Layer`, not a point cloud or mesh. No extension is enforced by the code; this repo's launch/eval files use `.tsdf`/`.esdf`/`.occ`/`.vxblx` by convention. These files are wire-compatible with maps saved by the original ROS 1 Voxblox/Voxfield (see `docs/ROS2_PORT_NOTES.md` for the compatibility check). Note that ROS 1 Voxfield's `voxfield_server`/`voxblox_server` `~/save_map` wrote only the ESDF layer (a since-fixed upstream bug, "Known upstream issues" #11 in `docs/ROS2_PORT_NOTES.md`), so `~/load_map` rejects such ROS 1 files. They are still readable as a plain ESDF layer (e.g. by `voxblox_eval`).

`~/generate_mesh` (with `mesh_filename` set) instead writes an **ASCII PLY** (`element vertex`/`element face`, `x y z normal_x normal_y normal_z red green blue alpha` per vertex) — a normal, tool-readable mesh file, unlike the protobuf map files above.

## Example Usage

The datasets below were recorded as ROS 1 bags. Convert one to a ROS 2 bag first with [`rosbags`](https://gitlab.com/ternaris/rosbags) (`pip install --user rosbags`):

```
rosbags-convert --src <dataset>.bag --dst <dataset>_ros2
```

### Run on the Cow & Lady real-world RGB-D dataset

1. Download the dataset [here](https://projects.asl.ethz.ch/datasets/doku.php?id=iros2017) or use the following command in a target folder:

```
wget http://robotics.ethz.ch/~asl-datasets/iros_2017_voxblox/data.bag
wget http://robotics.ethz.ch/~asl-datasets/iros_2017_voxblox/voxblox_cow_extras.zip
```

2. Convert the bag as above, then run Voxfield mapping on the Cow & Lady dataset:

```
ros2 launch voxfield_ros cow_voxfield.launch.py bag_file:=<path/to/data_ros2>
```

### Run on the KITTI real-world LiDAR or stereo dataset

1. Download the full dataset [here](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) or a rosbag of sequence07 [here](https://drive.google.com/file/d/1_qUfwUw88rEKitUpt1kjswv7Cv4GPs0b/view).
   Then use the [kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag) package to convert the full dataset to rosbags.
2. Convert the bag as above, then run Voxfield mapping on the KITTI dataset:

```
ros2 launch voxfield_ros kitti_voxfield.launch.py bag_file:=<path/to/kitti_ros2>
```

### Run on the MaiCity synthetic LiDAR dataset

1. Download the dataset [here](https://www.ipb.uni-bonn.de/data/mai-city-dataset/) or use the following command in a target folder:

```
wget https://www.ipb.uni-bonn.de/html/projects/mai_city/mai_city.tar.gz
tar -xvf mai_city.tar.gz
```

2. Convert the bag as above, then run Voxfield mapping on the MaiCity dataset:

```
ros2 launch voxfield_ros mai_voxfield.launch.py bag_file:=<path/to/mai_city_ros2>
```

### Run on your own data

Use the generic launch file directly, picking whichever dataset preset (`cow`/`kitti`/`mai`/`basement`/`vicon`) is closest to your sensor setup for its default topics/robot model/RViz config, then override what differs:

```
ros2 launch voxfield_ros mapping.launch.py \
    method:=voxfield dataset:=kitti \
    bag_file:=<path/to/your_bag_ros2> \
    pointcloud_topic:=<your/pointcloud/topic> \
    transform_topic:=<your/transform/topic>   # only if you're not using TF
```

`ros2 launch voxfield_ros mapping.launch.py --show-args` lists every override (`speed`, `rviz`, `rviz_config`, `robot_model_file`, `use_sim_time`, ...).

### Multiple sensors (one map)

Every server can fuse several depth sensors (e.g. two LiDARs, or LiDARs plus RGB-D cameras) into **one** TSDF, ESDF and mesh. Each sensor gets its own subscription, pose lookup, throttle and integrator settings. For the projective `np_tsdf`/`voxfield` servers, each sensor also gets its own range-image model. All of them write into the same map.

**Legacy vs multi-sensor mode.** If the `sensor_names` parameter is not set, a server behaves exactly as before: one sensor on the `pointcloud` topic, configured by the top-level parameters. Setting `sensor_names` switches to multi-sensor mode, and each named sensor is configured under `sensors.<name>.*`. Any key a sensor doesn't set is inherited from the top-level parameter of the same name, so shared settings are written once. To use legacy mode, omit `sensor_names` entirely: an empty list (`[]`) doesn't parse in ROS 2 YAML.

```yaml
/**:
  ros__parameters:
    sensor_names: [front_lidar, back_lidar]
    world_frame: map
    body_frame: base_link        # pose used for block removal / clear sphere
    sensor_is_lidar: true        # inherited by both sensors
    fov_up: 53.5
    fov_down: -8.0
    width: 360
    height: 32
    max_ray_length_m: 12.0
    sensors:
      front_lidar:
        topic: /athena/front_lidar/points_raw_livox
      back_lidar:
        topic: /athena/back_lidar/points_raw_livox
        # any per-sensor key can be overridden here, e.g. max_ray_length_m: 8.0
```

The complete example, validated on a robot with two Livox LiDARs and two RGB-D cameras, is `voxfield_ros/cfg/multi_sensor/athena_param.yaml` (map-global) plus `athena_dual_lidar.yaml` or `athena_lidar_rgbd.yaml` (sensors).

**Per-sensor keys** (`sensors.<name>.<key>`). Any other key is rejected at start-up, with one error that lists every problem found:

| Keys | Notes |
|---|---|
| `topic` (required), `freespace_topic` | Input topics. |
| `frame` | Overrides the cloud's frame for the pose lookup. Default: the cloud's `header.frame_id`. It is **not** inherited from `sensor_frame`. |
| `pointcloud_queue_size`, `input_qos_best_effort`, `min_time_between_msgs_sec` | Subscription depth, QoS and per-sensor throttle. |
| `T_B_C`, `invert_T_B_C` | Extrinsic, only used when `use_tf_transforms: false`. |
| `method`, `min_ray_length_m`, `max_ray_length_m`, `voxel_carving_enabled`, `allow_clear`, `use_const_weight`, `use_weight_dropoff`, `use_sparsity_compensation_factor`, `sparsity_compensation_factor`, `anti_grazing`, `merge_with_clear`, `start_voxel_subsampling_factor`, `max_consecutive_ray_collisions`, `clear_checks_every_n_frames`, `max_integration_time_s` | TSDF integrator. |
| `weight_reduction_exp`, `normal_available`, `reliable_band_ratio`, `curve_assumption`, `reliable_normal_ratio_thre` | Non-projective integrator (`np_tsdf`, `voxfield`). |
| `sensor_is_lidar`, `width`, `height`, `fov_up`, `fov_down`, `vx`, `vy`, `fx`, `fy`, `smooth_thre_ratio`, `min_z`, `min_dist` | Range-image model (`np_tsdf`, `voxfield`): LiDAR field of view, or camera intrinsics. |

Map-global keys can't be set per sensor. They include the voxel size, truncation, threads, `world_frame`, `use_tf_transforms`, `enable_icp`, `body_frame`, and the `esdf_*`/`occ_*`/`mesh_*` and `publish_*`/`update_*` keys. Multi-sensor mode adds three global keys:
- `sensor_names`: the sensors to configure.
- `body_frame`: the pose used for block removal and the clear sphere. The default `""` uses each sensor's own pose, as in legacy mode.
- `transform_queue_retention_sec` (default 1.0): how much transform-queue history is kept for out-of-order lookups when `use_tf_transforms: false`.

Things to know:
- **Frame counting:** every integrated cloud from every sensor counts as one "frame". A negative `update_mesh_every_n_sec`/`update_esdf_every_n_sec` ("every N frames") therefore fires N_sensors times as often. Use positive values (seconds) with several sensors.
- **ICP** (`enable_icp`) is single-sensor only. With more than one sensor, the server logs an error and disables it.
- The robot-model marker follows the first sensor in `sensor_names`.

**Launch.** `multi_sensor_mapping.launch.py` starts a server with a map-global `param_file` and a `sensors_file`. It can also play a bag (`bag_file`, `speed`, `start_offset`) and start RViz2 with `cfg/rviz/multi_sensor.rviz`:

```
ros2 launch voxfield_ros multi_sensor_mapping.launch.py method:=voxfield \
    param_file:=<share>/cfg/multi_sensor/athena_param.yaml \
    sensors_file:=<share>/cfg/multi_sensor/athena_dual_lidar.yaml \
    bag_file:=<path/to/bag> tf_remap_prefix:=/athena
```

- `tf_remap_prefix:=/athena` remaps `/tf` and `/tf_static` to `/athena/tf`/`/athena/tf_static` for both the server and RViz2. Use it for robots that publish namespaced TF.
- Bag playback always applies `cfg/multi_sensor/tf_static_qos_override.yaml`, so late-joining nodes still receive `/tf_static`.

**RGB-D cameras via `depth_image_proc`.** A depth camera is just another sensor. Set `sensor_is_lidar: false`, its `width`/`height`, and the intrinsics `fx`/`fy`/`vx` (= cx)/`vy` (= cy) from its `camera_info`. The server needs a `PointCloud2`, so `rgbd:=true` loads one `ComposableNodeContainer` per camera. Each container runs `image_transport::Republisher` (`in_transport: compressedDepth`, `out_transport: raw`) and then `depth_image_proc::PointCloudXyzNode`, which publishes `/athena/<camera>_rgbd/points`. Use `athena_lidar_rgbd.yaml` as the sensors file. It also shows typical camera settings: `max_ray_length_m: 4.0`, and `min_time_between_msgs_sec: 0.4` to keep dense depth clouds within the integration budget.

`docs/MULTI_SENSOR_NOTES.md` (Phase 9) has the measurements, tuning decisions and known issues from validating this on a real robot bag.

### Customizing, comparison and evaluation

To change the mapping and visualization parameters such as voxel size and truncation distance, please configure the `.yaml` files under `./voxfield_ros/cfg/param/` folder (ROS 2 parameter-file format — see `scripts/convert_ros1_params.py` if you're porting parameters from a ROS 1 Voxblox/Voxfield setup).

For the comparison with other state-of-the-art methods (Voxblox, FIESTA, EDT), set `bag_file` on the corresponding launch file `[dataset]_[method].launch.py` and launch it.

To evaluate the TSDF, mesh and ESDF mapping quality, one first need to use the ros service to save the corresponding map. You can configure the data path and evaluation setup [here](https://github.com/VIS4ROB-lab/voxfield-panmap/blob/master/panoptic_mapping_utils/config/evaluate_config.yaml) and conduct the evaluation by launching [here](https://github.com/VIS4ROB-lab/voxfield-panmap/blob/master/panoptic_mapping_utils/launch/evaluate_panmap.launch). You may also check the evaluation metrics [here](https://github.com/VIS4ROB-lab/voxfield-panmap/blob/master/panoptic_mapping_utils/src/evaluation/map_evaluator.cpp).

### Used for online path planning

Please check these [instructions](https://voxblox.readthedocs.io/en/latest/pages/Using-Voxblox-for-Planning.html) and the repository [mav_voxblox_planning](https://github.com/ethz-asl/mav_voxblox_planning).

### Replace Voxblox in your high-level volumetric mapping project

- [Panmap](https://github.com/VIS4ROB-lab/voxfield-panmap) (Multi-resolution panoptic mapping)
- Kimera (Semantic-metric mapping)
- Voxgraph (Global consistent mapping)
- ... ...

## Citation

If you find this code useful for your work or use it in your project, please consider citing the paper:

```
@inproceedings{pan2022iros,
  title={Voxfield: Non-Projective Signed Distance Fields for Online Planning and 3D Reconstruction},
  author={Yue Pan and Yves Kompis and Luca Bartolomei and Ruben Mascaro and Cyrill Stachniss and Margarita Chli},
  booktitle={Proceedings of the IEEE/RSJ Int. Conf. on Intelligent Robots and Systems (IROS)},
  year={2022}
}
```

## Acknowledgments

We thanks greatly for the authors of the following opensource projects: 

- [Voxblox](https://github.com/ethz-asl/voxblox) (underlying data structure, mesh reconstruction, visualization, comparison baseline)
- [FIESTA](https://github.com/HKUST-Aerial-Robotics/FIESTA) (comparison baseline)
- [VDB-EDT](https://github.com/zhudelong/VDB-EDT) (comparison baseline)
