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

**This is a ROS 2 (Jazzy) port.** The packages, C++ namespace, include paths, protobuf package, message package, and RViz plugin are all renamed from `voxblox*` to `voxfield*` (see `docs/ROS2_PORT_NOTES.md` for the full rename log). This means Voxfield no longer masquerades as Voxblox: it can be installed and used *alongside* an unrelated `voxblox` ROS 2 install in the same workspace without symbol, protobuf, message, or RViz-plugin clashes. The trade-off is that projects built against upstream Voxblox (Cblox, Voxgraph, Kimera, Panmap) need their includes (`voxblox/...` → `voxfield/...`), namespace (`voxblox::` → `voxfield::`), and message package (`voxblox_msgs` → `voxfield_msgs`) updated to depend on Voxfield instead. All ROS topic, service and parameter names are unchanged from the original ROS 1 Voxfield, and the protobuf map format is wire-compatible, so `.tsdf`/`.esdf`/`.vxblx` files saved by ROS 1 Voxfield or Voxblox still load here. The mapping algorithms are the originals, with the upstream bugs listed in `docs/ROS2_PORT_NOTES.md` ("Known upstream issues") fixed. The one fix that changes results on purpose: LiDAR points are now weighted by range instead of by their height above the sensor (set `lidar_z_weighting: true` to get the old behavior).

The original ROS 1 code is kept under the git tag `ros1`. `main` is ROS 2-only.

## Project status

- **ROS 2 Jazzy port: complete.** All seven servers (`tsdf`, `np_tsdf`, `voxblox`, `voxfield`, `fiesta`, `voxedt`, `intensity`), the dataset launch files, RViz2 configs and the `voxfield_rviz_plugin` mesh display are ported.
- **Multiple sensors in one map: complete.** Any number of LiDARs and depth cameras can feed one TSDF, ESDF and mesh ([details](#multiple-sensors-one-map)).
- **Validated on real robot data:** bags from the Athena robot (two Livox LiDARs, two RGB-D cameras). Every server was checked on one bag, with map-quality measurements; `voxfield_server` also ran end to end in RViz2 on four bags. Synthetic smoke tests cover all servers, and the full test suite (188 tests) passes.
- **Known limitations:**
  - With both LiDARs at full 30 m range, mapping doesn't keep up at 1× playback; the shipped Athena config uses 12 m rays to stay real-time.
  - On the Athena bags, the back LiDAR's recorded mounting angle is off by about 1.4°. Distant walls (beyond ~6 m) can look slightly doubled when both LiDARs are fused.
  - The Livox `intensity` field isn't used (you'll see a harmless `Failed to find match for field 'intensity'` message).

  Details, measurements and follow-ups are in `docs/MULTI_SENSOR_NOTES.md`.

## Quick start

### 1. Install

You need Ubuntu 24.04 with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) installed in `/opt/ros/jazzy`.

```
mkdir -p ~/voxfield_ws/src && cd ~/voxfield_ws/src
git clone https://github.com/frangeloacampora/voxfield_ros2.git
cd ~/voxfield_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

The build is Release by default and takes a few minutes. Add `source ~/voxfield_ws/install/setup.bash` to every new terminal you use (or to your `~/.bashrc`).

If you use **conda** (or another Python that comes first on your `PATH`), ROS 2's Python tools break with errors like `No module named 'rclpy._rclpy_pybind11'`. Run commands through the included clean-environment wrapper, which sources only ROS 2 and this workspace:

```
~/voxfield_ws/src/voxfield_ros2/scripts/clean_env.sh colcon build --symlink-install
~/voxfield_ws/src/voxfield_ros2/scripts/clean_env.sh ros2 launch voxfield_ros multi_sensor_mapping.launch.py ...
```

### 2. Run it on an Athena robot bag

With a bag recorded on the Athena robot, one command plays the bag, runs the mapper on both Livox LiDARs, and opens RViz2:

```
C=$(ros2 pkg prefix voxfield_ros)/share/voxfield_ros/cfg/multi_sensor
ros2 launch voxfield_ros multi_sensor_mapping.launch.py \
    method:=voxfield \
    param_file:=$C/athena_param.yaml \
    sensors_file:=$C/athena_dual_lidar.yaml \
    bag_file:=/path/to/your/athena_bag \
    tf_remap_prefix:=/athena
```

- `tf_remap_prefix:=/athena` is needed because Athena publishes its transforms on `/athena/tf` and `/athena/tf_static` instead of `/tf` and `/tf_static`.
- In RViz2 you'll see the **mesh** (coloured by surface direction) and the **ESDF slice**, a horizontal cut through the distance field at `slice_level` (0.5 m in `map`), coloured by distance to the nearest obstacle. The raw LiDAR clouds are there too, switched off; tick them in the Displays panel.
- To also use the two RGB-D cameras, use `sensors_file:=$C/athena_lidar_rgbd.yaml` and add `rgbd:=true`. This starts the decoding chain that turns Athena's compressed depth images into point clouds.
- `speed:=0.5` plays the bag at half speed. `start_offset:=<seconds>` skips the start of the bag, but only if the bag publishes `/tf_static` again later; most bags record it only at the beginning, and without it no mesh appears.
- To use another mapping method, change `method:=` (see [Choosing a server](#choosing-a-server)).

### 3. Run it on your own robot or bags

You need two YAML files, a **parameter file** (map settings) and a **sensors file** (which topics to use), plus a TF tree that connects your map frame to each sensor.

**a) Find your topics and frames.** For a bag:

```
ros2 bag info /path/to/bag                      # lists topics and message types
ros2 bag play /path/to/bag                      # in a second terminal:
ros2 topic echo --once --field header.frame_id /your/points_topic
ros2 run tf2_ros tf2_echo map <that_frame_id>   # must print a transform
```

You need:
- one `sensor_msgs/msg/PointCloud2` topic per sensor;
- a TF path from your world frame (usually `map`, or `odom` if you have no localization) to each cloud's `frame_id`, available at the clouds' timestamps.

A depth camera that only publishes images can be converted to point clouds with `depth_image_proc::PointCloudXyzNode` (see how `multi_sensor_mapping.launch.py` does it for Athena).

**b) Write the sensors file.** Copy `cfg/multi_sensor/athena_dual_lidar.yaml` and adapt it:

```yaml
/**:
  ros__parameters:
    sensor_names: [front_lidar, rear_lidar]   # any names you like
    # Settings shared by all sensors (each sensor can override any of them):
    sensor_is_lidar: true
    fov_up: 15.0          # LiDAR vertical field of view in degrees...
    fov_down: -15.0       # ...(only needed for the voxfield / np_tsdf servers)
    width: 360            # range-image size used by voxfield / np_tsdf
    height: 32
    sensors:
      front_lidar:
        topic: /front/points
      rear_lidar:
        topic: /rear/points
        max_ray_length_m: 20.0      # example of a per-sensor override
      # A depth camera sensor:
      # front_camera:
      #   topic: /camera/depth/points
      #   sensor_is_lidar: false
      #   width: 640
      #   height: 480
      #   fx: 525.0          # intrinsics from the camera's camera_info
      #   fy: 525.0
      #   vx: 319.5          # cx
      #   vy: 239.5          # cy
      #   max_ray_length_m: 4.0
```

- For **one sensor**, list just one name.
- A sensor's frame comes from its cloud's `header.frame_id`; set `frame:` under the sensor only to override it.
- For the voxfield/np_tsdf servers, `fov_up`/`fov_down` must cover your LiDAR's vertical range, and `width` × `height` controls how finely each scan is sampled. 360 × 32 worked well for the Livox Mid-360.
- Any mistake (unknown key, missing topic, duplicate topic) stops the server at start-up with a message listing every problem.

**c) Write the parameter file.** Copy `cfg/multi_sensor/athena_param.yaml` and change at least:
- `world_frame`: your map frame (`map` or `odom`).
- `body_frame`: your robot's base frame (e.g. `base_link`). Map areas far from the robot are dropped relative to this frame.
- `tsdf_voxel_size` (and `esdf_voxel_size`, `occ_voxel_size`): 0.1 m suits a ground robot; smaller is more detailed but slower.
- `max_ray_length_m`: how far each LiDAR point is used. Longer covers more but costs time.
- `slice_level`: the height of the ESDF slice shown in RViz2.
- Optionally `mesh_filename: /path/to/mesh.ply`, so `~/generate_mesh` saves the mesh to disk.

**d) Run it:**

```
ros2 launch voxfield_ros multi_sensor_mapping.launch.py \
    method:=voxfield \
    param_file:=/path/to/my_param.yaml \
    sensors_file:=/path/to/my_sensors.yaml \
    bag_file:=/path/to/bag
```

- Add `tf_remap_prefix:=/ns` if your robot publishes TF on `/ns/tf` and `/ns/tf_static`.
- For a **live robot** instead of a bag, leave out `bag_file` and add `use_sim_time:=false`.

### 4. Save the map and mesh

While the mapper runs:

```
ros2 service call /voxfield_node/save_map voxfield_msgs/srv/FilePath "{file_path: /path/to/map.tsdf}"
ros2 service call /voxfield_node/generate_mesh std_srvs/srv/Empty "{}"   # writes mesh_filename
```

A saved map can be loaded back with `~/load_map`. The mesh is a standard PLY file (MeshLab, CloudCompare, Blender).

### 5. If it runs too slowly or looks wrong

- **The view lags behind the bag, or the log says `Input pointcloud queue getting too long` repeatedly:** the mapper can't keep up. In the parameter file, lower `max_ray_length_m`, raise `update_esdf_every_n_sec` (e.g. 5), or throttle sensors with `min_time_between_msgs_sec`. Or play the bag slower with `speed:=0.5`. One such message in the first seconds is normal while TF starts up.
- **No mesh at all:** usually TF. Check that `ros2 run tf2_ros tf2_echo <world_frame> <cloud frame_id>` works, that `tf_remap_prefix` matches your TF topics, and that `/tf_static` isn't skipped by `start_offset`.
- **Large empty areas in the mesh:** points beyond `max_ray_length_m` aren't used, and sparse or far-away surfaces need several hits before they appear. Increasing `max_ray_length_m` helps (at the cost of speed); smaller voxels make holes worse, not better.
- **Doubled walls:** two sensors disagree about their mounting. Check the sensors' transforms in your robot description.

## Choosing a server

`method:=` in the launch files selects the executable `<method>_server`:

| `method` | TSDF (surface) | ESDF (distance field) | Notes |
|---|---|---|---|
| `voxfield` | non-projective (Voxfield) | Voxfield | The method of the paper; most accurate surfaces. |
| `np_tsdf` | non-projective (Voxfield) | none | Surface only. |
| `voxblox` | ray casting (Voxblox) | Voxblox | Uses every LiDAR point, no range image. |
| `tsdf` | ray casting (Voxblox) | none | Surface only. |
| `fiesta` | ray casting | FIESTA | ESDF from an occupancy map. |
| `voxedt` | ray casting | EDT | ESDF from an occupancy map. |

All servers take the same inputs and parameters. The full list of services is in the [Services](#services) table below and in the [Voxblox documentation](https://voxblox.readthedocs.io/en/latest/pages/The-Voxblox-Node.html).

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

## Multiple sensors (one map)

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
| `weight_reduction_exp`, `normal_available`, `reliable_band_ratio`, `curve_assumption`, `reliable_normal_ratio_thre` | Non-projective integrator (`np_tsdf`, `voxfield`). `weight_reduction_exp` is also the TSDF integrator's LiDAR range-weight exponent. |
| `sensor_is_lidar`, `width`, `height`, `fov_up`, `fov_down`, `vx`, `vy`, `fx`, `fy`, `smooth_thre_ratio`, `min_z`, `min_dist` | Range-image model (`np_tsdf`, `voxfield`): LiDAR field of view, or camera intrinsics. `sensor_is_lidar` also selects every server's point weight model: 1 / z² for a depth camera, 1 / range^`weight_reduction_exp` for a LiDAR. |

Map-global keys can't be set per sensor. They include the voxel size, truncation, threads, `world_frame`, `use_tf_transforms`, `enable_icp`, `body_frame`, `lidar_z_weighting`, and the `esdf_*`/`occ_*`/`mesh_*` and `publish_*`/`update_*` keys. Multi-sensor mode adds three global keys:
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

## Further topics

### Customizing, comparison and evaluation

To change the mapping and visualization parameters such as voxel size and truncation distance, please configure the `.yaml` files under `./voxfield_ros/cfg/param/` folder (ROS 2 parameter-file format — see `scripts/convert_ros1_params.py` if you're porting parameters from a ROS 1 Voxblox/Voxfield setup).

To compare with the other methods (Voxblox, FIESTA, EDT), run the same bag and configuration with a different `method:=` (see [Choosing a server](#choosing-a-server)).

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
