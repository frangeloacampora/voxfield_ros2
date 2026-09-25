# Voxfield ROS 2: Multi-Sensor Fusion into One Map: Specification & Implementation Plan

> **Audience:** an autonomous coding agent implementing the change end to end.
> **Repo:** `~/src/voxfield_ros2`, branch `ros2-port`. `ROS2_PORT_PLAN.md` Phases 0–12 are done (last commit `335553d`).
> **Goal in one sentence:** one mapping server node takes point clouds from **N sensors** (e.g. front + back LiDAR, plus optional RGB-D) and builds **one** TSDF → **one** ESDF / mesh / occupancy map, instead of one map per sensor.
> **Workspace:** `~/voxfield_ws` (the repo is symlinked into `src/`). Build and test commands are the same as in the port plan.

---

## 0. How to use this document

1. Read §1–§6 fully before touching code. They explain what exists today, why the chosen architecture is the right one, and the binding design decisions.
2. Execute §8 phases **in order**. Each phase has explicit **acceptance criteria**. Don't start the next phase until the current one passes.
3. Work on a new branch off `ros2-port`: `git checkout -b multi-sensor`. Make **one commit per phase**. Phase 1 (golden regression tests) and Phase 2 (pure refactor) must each be their own commits, with no behavior change, so the later diffs show only the feature.
4. **Backward-compatibility rule (the most important rule here):** with no `sensor_names` parameter set, every server must behave **exactly** as it does today: same subscriptions (`pointcloud`, `freespace_pointcloud`, `transform`), same parameters, same outputs, and the same TSDF voxel values for the same input. Every shipped YAML, launch file, RViz config, and the existing smoke test must keep working unmodified. Phase 1 builds the golden tests that enforce this.
5. Record every decision not covered here, plus any deviation, in a new `docs/MULTI_SENSOR_NOTES.md` (same style as `docs/ROS2_PORT_NOTES.md`).
6. §9 lists pitfalls that **will** bite if ignored. §7 is the parameter reference.
7. **Memory safety when running on real bags** (from `docs/BUG_voxfield_server_camera_mode_memory.md`): always run servers under a cgroup cap, e.g. `systemd-run --user --scope -p MemoryMax=3G -p MemorySwapMax=0 <full path to binary> --ros-args ...`. Monitor the real binary's PID (`pgrep -f install/voxfield_ros/lib/voxfield_ros/<name>`), not the `ros2 run` wrapper's.

---

## 1. Goal and scope

### 1.1 Goal
Every mapping server built on `TsdfServer` or `NpTsdfServer` (`tsdf_server`, `np_tsdf_server`, `voxblox_server`, `voxfield_server`, `fiesta_server`, `voxedt_server`) can be configured with a list of named sensors. Each sensor has its own input topic, frame, extrinsics, ray-length limits, integrator settings, and (for the non-projective servers) its own range-image projection model. All sensors integrate into the **same** `TsdfMap`. The ESDF, occupancy, mesh, and every output topic and service are computed once, from that shared map. It is validated on the Athena bag `~/src/rosbag2_2026_09_23-14_32_47` with both Livox LiDARs, and optionally the RGB-D cameras, feeding one `voxfield_server` and one `voxblox_server`.

### 1.2 In scope
- Per-sensor input frontend inside `TsdfServer` and `NpTsdfServer`: subscription, queue, throttle, frame resolution, extrinsics, integrator instance, and projection model.
- Parameter schema for N sensors, with inheritance from the existing top-level parameters and strict validation.
- `Transformer` changes: per-sensor frames and extrinsics, and a transform queue that's safe with several sensors.
- Extracting `NpTsdfServer`'s range-image code into a reusable, per-sensor `RangeImageProjector`.
- Unit tests, golden regression tests for legacy mode, a multi-sensor launch smoke test, and a real-bag validation run.
- A multi-sensor launch file and an example config for the Athena robot (dual Livox + optional RGB-D through `depth_image_proc`).
- Docs: README section, `docs/MULTI_SENSOR_NOTES.md`, and an updated interface table.

### 1.3 Out of scope (non-goals)
- Changes to the core library's map or ESDF algorithms (`voxfield/`). The ESDF integrators already consume "updated" flags on the shared TSDF layer and don't care where the data came from (§2.3). **No `voxfield/src/integrator/*` changes are expected.** If one turns out to be necessary, stop and document why.
- Running sensors on parallel threads. Integration stays serialized on the single-threaded executor (§5 M7).
- Motion compensation (deskew) of Livox clouds using the per-point `t` field.
- Self-filtering (removing robot-body points seen by another sensor). Mention it in the docs as a preprocessing concern; `hector_depth_self_filter` exists on this machine, but don't depend on it.
- ICP in multi-sensor mode (§5 M11). It stays single-sensor only.
- Multi-sensor support in `IntensityServer` (intensity images) and `SimulationServer`. They must still compile and behave the same.
- Merging maps from several **robots** or separate server processes.
- Deduplicating `TsdfServer` and `NpTsdfServer`. Still out of scope, as in the port plan §3.2. Shared new code goes in new helper headers both servers use.

---

## 2. Current standing (what exists today)

### 2.1 Port status
`ROS2_PORT_PLAN.md` Phases 0–12 are complete (see `docs/ROS2_PORT_NOTES.md`). On top of that, commit `335553d` fixed several real upstream bugs: `computeNormalImage` out-of-bounds, camera projection returning a bool as depth, float intrinsics, and the ESDF `setLocalRange()` memory blow-up (#12). Real-bag validation so far:
- Athena bags (`rosbag2_2026_09_23-12_40_03`, `-13_40_18`, `-14_32_47`): `voxblox_server`, `fiesta_server`, and `voxedt_server` were run on **one** Livox LiDAR (front), using bag-side remaps (`/athena/front_lidar/points_raw_livox:=/pointcloud`, `/athena/tf:=/tf`, `/athena/tf_static:=/tf_static`) and a local, uncommitted param file (`world_frame: map`, `sensor_frame: front_lidar_laser_frame`, `voxel_size: 0.1`).
- `voxfield_server` / `np_tsdf_server` were not run on the Livox data. Notes line ~1345 assumed Livox clouds aren't usable for NP-TSDF, but that's only half true; see §3.3.

### 2.2 Today's data flow (per server process: exactly one sensor)
```
             "pointcloud" (1 topic)            TF (world_frame <- sensor_frame param)
                     |                              or "transform" topic + T_B_D/T_B_C params
                     v                                            |
  insertPointcloud() -- throttle (1 last_msg_time) -- 1 queue <---+ Transformer (1 extrinsic)
                     |
                     v
  processPointCloudMessageAndInsert(msg, T_G_C)
    [NP only: project to 1 range image (width/height/fov/fx.. = 1 sensor model) -> normals]
    [optional ICP]
    tsdf_integrator_ (1 instance, 1 config: min/max_ray_length_m, carving, weights...)
                     |
                     v
          TsdfMap (shared layer) --updated flags--> ESDF / occupancy / mesh  (sensor-agnostic)
                     |
     removeDistantBlocks(sensor position), publishRobotMesh(sensor pose), newPoseCallback(sensor pose)
```
Today's only way to use two sensors is to run **two server processes**, which gives two independent TSDFs and two ESDFs. That's the situation this plan fixes.

### 2.3 What is already sensor-agnostic (no change needed)
- `TsdfMap` / `Layer<TsdfVoxel>`: voxels hold weighted-average distance, color, and weight, plus per-block `Update` flags. Integrating clouds from any origin into one layer is the normal voxblox use case. Voxblox itself moves a single sensor through the map.
- ESDF: `VoxfieldServer::updateEsdf()` → `EsdfVoxfieldIntegrator::updateFromTsdfLayer()`, `VoxbloxServer` (`EsdfIntegrator`), `FiestaServer`/`VoxedtServer` (TSDF → `OccTsdfIntegrator` → occupancy → ESDF). All are driven by the TSDF layer's updated blocks, not by sensor data.
- Mesh (`MeshIntegrator` on the TSDF layer's updated blocks), all `~/…` publishers, services, map I/O, `~/tsdf_map_in` / `~/esdf_map_in`.

### 2.4 Single-sensor assumptions: complete inventory
Every item here must be addressed. Line numbers are as of `335553d`.

| # | Where | Assumption | Multi-sensor consequence | Fix (decision) |
|---|---|---|---|---|
| A1 | `tsdf_server.cc:70`, `np_tsdf_server.cc:74` | One `pointcloud_sub_` on topic `pointcloud` | Only one input stream | Per-sensor subscription (M2, M4) |
| A2 | `tsdf_server.cc:92`, `np_tsdf_server.cc:96` | One `freespace_pointcloud_sub_` | Same | Per-sensor optional `freespace_topic` (M4) |
| A3 | `*_server.h`: `pointcloud_queue_`, `freespace_pointcloud_queue_` | One queue, drained only by that topic's callback | Sensors would block each other's clouds that are waiting for TF | Per-sensor queues (M4) |
| A4 | `*_server.cc:455/507`: `last_msg_time_ptcloud_` | One throttle clock | Sensor B's cloud with a stamp ≤ sensor A's would be throttled away whenever `min_time_between_msgs_sec > 0` | Per-sensor throttle (M4) |
| A5 | `*_server.cc:432/484`: `lookupTransform(sensor_frame_, …)` | Frame comes from the **`sensor_frame` param**, not `header.frame_id` (a voxfield change; upstream voxblox used the header) | All sensors would be treated as one frame, so every cloud but one is placed wrongly | Per-sensor frame; default is the message header (M5) |
| A6 | `transformer.cc:16,88` | `Transformer` overrides `from_frame` with its own `sensor_frame_` | Same as A5, a second time | Remove the override from `Transformer`; resolve the frame in the sensor frontend (M5) |
| A7 | `transformer.cc:46-49` | One `T_B_C` (queue mode) | Every sensor would use one extrinsic | Per-sensor `T_B_C`; `T_B_D` stays global (M6) |
| A8 | `transformer.cc:195` `transform_queue_.erase(begin, it)` | Lookups arrive in monotonically increasing time | With 2 sensors, a cloud stamped slightly earlier than one already processed finds its bracketing poses erased and waits until it's dropped | Time-window retention (M6) |
| A9 | `tsdf_server.cc:110-122`, `np_tsdf_server.cc:114-126` | One `tsdf_integrator_` with one config (`min/max_ray_length_m`, carving, weighting, `method`) | LiDAR (45 m) and RGB-D (5 m) need different ray limits | One integrator per sensor on the shared layer (M3) |
| A10 | `np_tsdf_server.h` `width_ … fy_`, `np_tsdf_server.cc:211-230, 333, 835-1057` | One range-image model | Can't mix LiDAR and camera, or two different LiDARs | `RangeImageProjector` per sensor (M8) |
| A11 | `np_tsdf_server.cc:333` (`min_z_`, `min_dist_`) | One noise filter | Per-sensor mounting height and near-range differ | Per-sensor (M8) |
| A12 | `*_server.cc:98/102, 300/349` ICP | One correction, one TF broadcast | Corrections from different sensors would fight | Disabled when >1 sensor (M11) |
| A13 | `*_server.cc:383/434` `publishRobotMesh(T_G_C)` | Model marker follows the sensor | N sensors would make the marker jump between poses | Primary sensor only (M10) |
| A14 | `*_server.cc:377/428` + `{voxblox,voxfield,fiesta,voxedt}_server.cc` `removeDistantBlocks(T_G_C.getPosition())` | "Robot position" means the sensor position | Blocks near the radius boundary get dropped and re-added depending on which sensor came last | Optional `body_frame` (M9) |
| A15 | `voxblox_server.cc:274` `addNewRobotPosition` | Same | Clear sphere around each sensor instead of the robot | Optional `body_frame` (M9) |
| A16 | `*_server.cc:490/541` `frame_count_++` per callback | 1 frame = 1 sensor sweep | "Every N frames" triggers (negative `update_*_every_n_sec`) fire N× as often | Count integrated clouds; document it (M12) |
| A17 | `intensity_server.cc:81` | Relies on `Transformer`'s `sensor_frame_` override | Would change behavior once A6 is removed | Resolve the frame explicitly in `IntensityServer` (M5) |

---

## 3. Target sensor setup (verified facts from the Athena bag)

Read on this machine from `~/src/rosbag2_2026_09_23-14_32_47` (mcap, 16 files, 413.9 s):

| Topic | Type | Rate | Frame | Shape / notes |
|---|---|---|---|---|
| `/athena/front_lidar/points_raw_livox` | PointCloud2 | 10 Hz | `front_lidar_laser_frame` | **Unorganized** (`height=1`, ~20k pts). Fields `x,y,z` (FLOAT32), `intensity` (UINT8), `tag` (UINT8), `ring` (UINT8), `t` (UINT32) |
| `/athena/back_lidar/points_raw_livox` | PointCloud2 | 10 Hz | `back_lidar_laser_frame` | Same. Stamps within ~0.5 ms of the front cloud (the two sensors are synchronized) |
| `/athena/{front,back}_rgbd/depth/image_raw/compressedDepth` | CompressedImage | ~4.5 Hz | (see camera_info) | Depth only as `compressedDepth`. No point cloud is recorded |
| `/athena/{front,back}_rgbd/depth/camera_info` | CameraInfo | 30 Hz | `{front,back}_rgbd_color_optical_frame` | 640×480, fx=fy≈451.5/452.0, cx≈325.8/327.1, cy≈242.6/241.0 |
| `/athena/tf`, `/athena/tf_static` | TFMessage | 36.6 Hz / latched | `map → odom → … → base_footprint_link → base_link → {front,back}_sensor_mount_link → {front,back}_{lidar_link → lidar_laser_frame, rgbd_link → …}` | TF is **namespaced**. Either remap on the server node (`/tf:=/athena/tf`, `/tf_static:=/athena/tf_static`) or on bag play |

### 3.1 Consequences
- Two LiDARs with different frames and synchronized stamps is the primary use case. It exercises A3–A8 immediately: the two clouds arrive about 0.5 ms apart, in either order.
- RGB-D needs an external `image_transport republish` (compressedDepth → raw) followed by `depth_image_proc::PointCloudXyzNode`. Both are installed (`/opt/ros/jazzy/share/{compressed_depth_image_transport,depth_image_proc}`). The resulting cloud is organized (640×480, NaNs for invalid pixels), in `*_rgbd_color_optical_frame`. `convertPointcloud` already drops non-finite points (`conversions.h:116`).
- The RGB-D rate (4.5 Hz, 307k points per frame) dominates integration cost. The per-sensor `min_time_between_msgs_sec` and `max_ray_length_m` (M4) are the knobs for that.

### 3.2 Livox and the ray-casting servers
`voxblox_server`, `fiesta_server`, and `voxedt_server` already work on a single Livox (Phase 12). Nothing Livox-specific is needed there.

### 3.3 Livox and the non-projective servers (`voxfield_server`, `np_tsdf_server`)
`NpTsdfServer` doesn't need an organized input cloud: `projectPointCloudToImage()` projects **arbitrary** points into a spherical range image (`projectPointToImageLiDAR`, yaw → column, pitch → row). A Livox cloud *can* be used, but with care:
- Its FOV isn't KITTI's. Set per-sensor `fov_up`/`fov_down` to the real sensor (check the Livox model; a Mid-360 is about +52°/−7°). Measure it from the bag: compute min/max pitch over a few clouds (Phase 9 step 1).
- A non-repetitive scan fills a range image sparsely in 100 ms, so `computeNormalImage()` finds no valid neighbor for many pixels and returns a zero normal. Pick a **coarse** image per sensor (e.g. 360×32 instead of 1024×64) so neighboring pixels are both filled, **or** set `normal_available: false` for that sensor. Phase 9 measures the fraction of points with a valid normal per sensor. That's why `width/height/fov_*/normal_available` must be **per sensor** (M8).

---

## 4. Architecture options considered (decision: C)

**A. External cloud merger → unchanged server.** A separate node transforms every sensor's cloud into a common frame (e.g. `base_link`), concatenates them, and publishes one cloud. **Rejected.** TSDF integration raycasts from the cloud's origin (`T_G_C.getPosition()`), and free-space carving and projective distances are only correct from the **real** sensor origin. A merged cloud raycasts from `base_link`, so rays pass through the robot body and free space is cleared wrongly. The NP-TSDF range image and normals would be meaningless for a merged cloud. Merging also forces a time-synchronization policy (`message_filters`) that drops clouds whenever one sensor lags.

**B. One server per sensor + merge the maps.** `~/tsdf_map_out` → `~/tsdf_map_in` with `MapDerializationAction::kMerge`. **Rejected.** The publisher sends cumulative voxel state, so repeated merges double-count weights. Free-space clearing by one sensor can't be undone in the other map. Each process still computes its own ESDF, bandwidth grows with map size, and N extra processes each hold a full map.

**C. One server, N sensor frontends, one shared map. (Chosen.)** Each sensor keeps its own origin, extrinsics, ray limits, and projection model. Integration is serialized into one layer, and everything downstream (ESDF, mesh, outputs) is unchanged. The code change is contained in `TsdfServer`, `NpTsdfServer`, `Transformer`, `ros_params.h`, and new helper headers. The derived ESDF servers only need the `newPoseCallback` / body-position change (M9).

---

## 5. Binding design decisions

**M1. Two modes, selected by one parameter.**
- **Legacy mode:** the `sensor_names` parameter is **not set** (the default). The server builds exactly one sensor named `default` from the existing top-level parameters (§7.3) and subscribes to `pointcloud` / `freespace_pointcloud` exactly as today. This must reproduce today's behavior bit for bit, with one deliberate exception, fix F1 below.
- **Multi-sensor mode:** `sensor_names` is a non-empty string array, e.g. `[front_lidar, back_lidar]`. The server builds one sensor per name from `sensors.<name>.*` parameters and does **not** subscribe to `pointcloud` / `freespace_pointcloud`.
- `sensor_names: []` can't be written in YAML: `rcl_yaml_param_parser` rejects empty arrays (it can't infer the type). "Unset" therefore means legacy. Don't make an empty array the declared default; declare it as `PARAMETER_NOT_SET` and treat "not set" as legacy.
- **F1 (deliberate legacy fix):** today, with `sensor_frame` unset (`""`), the lookup uses frame `""` and fails forever. In legacy mode after the change, an empty `sensor_frame` falls back to `header.frame_id` (upstream voxblox behavior). This only changes a configuration that never worked. Record it in the notes.

**M2. Parameter schema: flat, dotted, inherited.** ROS 2 parameters can't be lists of dicts, so:
```yaml
/**:
  ros__parameters:
    sensor_names: [front_lidar, back_lidar]
    sensors:
      front_lidar:
        topic: /athena/front_lidar/points_raw_livox
        max_ray_length_m: 30.0
      back_lidar:
        topic: /athena/back_lidar/points_raw_livox
```
The YAML nesting flattens to parameter names `sensors.front_lidar.topic` and so on. **Every per-sensor key defaults to the value of the top-level parameter with the same name**, which is already read and defaulted today. So a sensor block only lists what differs, and a multi-sensor config is the old config plus `sensor_names` and `topic`s. The exact per-sensor key list, and which keys are forbidden per sensor, is in §7.

**M3. One integrator instance per sensor, all on the shared layer.** Each `SensorInput` owns a `std::unique_ptr<TsdfIntegratorBase>` (or `NpTsdfIntegratorBase`) constructed with `tsdf_map_->getTsdfLayerPtr()` and that sensor's config. Why not one integrator with swapped configs? `Config` is `const` inside the integrators, and `FastTsdfIntegrator` keeps per-stream state (`start_voxel_approx_set_`, `voxel_observed_approx_set_`, `clear_checks_every_n_frames` counter) that must not mix between sensors. Integrators are documented "NOT thread safe", but sequential calls from different integrator instances into one layer are safe: each call finishes with `updateLayerWithStoredBlocks()` before returning. The per-sensor config is built as **base config (top-level params, or the config passed to the programmatic constructor) + `sensors.<name>.*` overrides from the whitelist**.
- **Map-global keys** (voxel size, truncation distance, `max_weight`, `integrator_threads`) must be identical for all sensors. Mixing truncation distances or weight caps in one layer gives inconsistent voxels. They are **forbidden** per sensor (M13).
- The legacy `tsdf_integrator_` member is **removed**. Nothing outside the two base servers uses it (verified by grep; `SimulationServer` has its own). If a subclass needs it, add `TsdfIntegratorBase* primaryIntegrator()`.

**M4. `SensorInput`: the per-sensor frontend.** New header `voxfield_ros/include/voxfield_ros/sensor_input.h`:
```cpp
namespace voxfield {

struct SensorConfig {
  std::string name;                   // "default" in legacy mode
  std::string topic;                  // legacy: "pointcloud"
  std::string freespace_topic;        // "" = none; legacy: "freespace_pointcloud" iff use_freespace_pointcloud
  std::string frame;                  // "" = use header.frame_id (M5); legacy: sensor_frame param
  int queue_size = 1;                 // inherits pointcloud_queue_size
  bool input_qos_best_effort = false; // inherits
  double min_time_between_msgs_sec = 0.0;  // inherits
  bool has_T_B_C = false;             // queue mode only (M6)
  Transformation T_B_C;
};

template <typename IntegratorBaseT>
struct SensorInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SensorConfig config;
  std::unique_ptr<IntegratorBaseT> integrator;
  std::unique_ptr<RangeImageProjector> projector;  // NpTsdfServer only; nullptr in TsdfServer
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub, freespace_sub;
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> queue, freespace_queue;
  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_freespace_msg_time{0, 0, RCL_ROS_TIME};
  // Stats (logged when verbose_, and in the dropped-cloud warning).
  size_t num_received = 0, num_throttled = 0, num_dropped = 0, num_integrated = 0;
};
}  // namespace voxfield
```
- Servers hold `std::vector<std::unique_ptr<SensorInput<…>>> sensors_`. Keep them as `unique_ptr`: subscription lambdas capture the raw pointer, which must stay stable.
- Subscriptions: `node_->create_subscription<PointCloud2>(cfg.topic, qos, [this, s](PointCloud2::SharedPtr m) { insertPointcloud(m, s); })`. `topic` is used verbatim (absolute or relative), so it stays remappable.
- `sensors_[0]` is the **primary sensor** (the first entry in `sensor_names`, or `default`). It's used for M10, and by the kept public single-argument API (M14).
- The queue logic (`kMaxQueueSize = 10`, drop-oldest, the throttled error) moves unchanged into the per-sensor versions of `getNextPointcloudFromQueue`. The error message gains the sensor name, and `num_dropped` is counted.

**M5. Frame resolution lives in the sensor frontend, not the `Transformer`.**
- Frame used for a cloud: `cfg.frame.empty() ? msg->header.frame_id : cfg.frame`.
- In legacy mode, `cfg.frame` = the `sensor_frame` param. That preserves today's override semantics, plus F1.
- `Transformer` loses its `sensor_frame_` member and its override (`transformer.cc:16, 86-90`). `lookupTransform(from, to, stamp, T)` then means exactly what it says.
- `IntensityServer::intensityImageCallback` (`intensity_server.cc:81`) must pass `sensor_frame_.empty() ? image->header.frame_id : sensor_frame_` to keep its current behavior (A17).
- `TsdfServer::sensor_frame_` / `NpTsdfServer::sensor_frame_` stay as members (legacy value, used by `IntensityServer`).

**M6. `Transformer` in queue mode (`use_tf_transforms: false`).**
- **Split the extrinsics.** `T_B_D` (the dynamic pose's frame relative to the body) stays global. `T_B_C` becomes per sensor: `sensors.<name>.T_B_C` + `invert_T_B_C`, defaulting to the global `T_B_C`. New API:
  ```cpp
  // T_G_C = T_G_D(stamp) * T_B_D^-1 * T_B_C
  bool lookupTransformQueue(const rclcpp::Time& stamp, const Transformation& T_B_C, Transformation* T_G_C);
  bool lookupSensorTransform(const std::string& frame, const Transformation* T_B_C_or_null,
                             const rclcpp::Time& stamp, Transformation* T_G_C);  // TF or queue per use_tf_transforms_
  ```
  Keep the old `lookupTransform(from, to, stamp, T)` working. Its queue branch uses the global `T_D_C_`, so legacy output is identical.
- **Queue retention (fixes A8).** Replace `transform_queue_.erase(begin, it)` with "erase entries older than `stamp - transform_queue_retention_sec`", where `transform_queue_retention_sec` is a new global param, default `1.0`. Also cap the deque at 10 000 entries by dropping the oldest. Lookup scans from the oldest entry, and older retained entries are all before `stamp - timestamp_tolerance`, so they're skipped. Single-sensor output stays identical except when two consecutive clouds are closer together than `timestamp_tolerance_sec` (1 ms). Note that in the notes.
- **TF mode** needs no queue change. `tf2_ros::Buffer` keeps 10 s of history and handles out-of-order lookups.
- Add a public `std::shared_ptr<tf2_ros::Buffer> tfBuffer()` accessor. Tests use it to inject transforms with `setTransform()`. It's harmless in production.

**M7. Threading: unchanged (port plan D9).** Keep the single-threaded executor. All sensor callbacks, timers, and services run serially, so there are never two integrators writing the layer at once, and no mutex is needed. Don't create per-sensor callback groups. Document the throughput budget in the notes: measure integration ms per sensor in Phase 9. If the sum of (integration time × rate) over sensors approaches 1 s/s, the fix is per-sensor throttling or downsampling, **not** threads.

**M8. `RangeImageProjector` (NP servers).** New `voxfield_ros/include/voxfield_ros/range_image_projector.h` + `src/range_image_projector.cc`. Move, verbatim in behavior, `projectPointCloudToImage`, `projectPointToImageLiDAR`, `projectPointToImageCamera`, `computeNormalImage`, `extractPointCloud`, `extractNormals`, and `extractColors` out of `NpTsdfServer`:
```cpp
class RangeImageProjector {
 public:
  struct Config {
    bool sensor_is_lidar = false;
    int width = 0, height = 0;
    float fov_up = 0.f, fov_down = 0.f;        // degrees, LiDAR
    float vx = 0.f, vy = 0.f, fx = 0.f, fy = 0.f;  // pixels, camera
    float smooth_thre_ratio = 1.0f;
    float min_z = -1000.0f, min_dist = 0.1f;
    bool isValid(std::string* why) const;      // width/height > 0; camera: fx,fy > 0; LiDAR: fov_up != fov_down
  };
  explicit RangeImageProjector(const Config&);
  // Full preprocessing pipeline used by NpTsdfServer::processPointCloudMessageAndInsert:
  void process(const Pointcloud& points_in, const Colors& colors_in,
               Pointcloud* points_out, Pointcloud* normals_out, Colors* colors_out) const;
  // + the individual methods above, public (tests use them)
};
```
- `fov_down_rad_` / `fov_rad_` are computed in the constructor, as in `getServerConfigFromRosParam` today.
- Per-sensor projector config = top-level values (legacy read, unchanged) + `sensors.<name>.*` overrides.
- Legacy mode keeps today's "log error if width/height ≤ 0" behavior. Multi-sensor mode **fails fast**: any invalid projector config throws `std::invalid_argument` from the constructor, with the sensor name and reason (see M13).
- Keep thin `NpTsdfServer::projectPointToImageCamera(...)` etc. wrappers that delegate to `sensors_[0]->projector`, so `test/test_np_tsdf_server.cc` compiles and passes **unchanged** in Phase 2. New projector tests go into `test/test_range_image_projector.cc`.
- `normal_available`, `reliable_band_ratio`, `curve_assumption`, and `reliable_normal_ratio_thre` are integrator-config keys, so they're per sensor via M3 (see §3.3 for why that matters).

**M9. Body position for block removal and the clear sphere (optional `body_frame`).**
- New global param `body_frame` (default `""`).
- `""` (default): unchanged. `removeDistantBlocks`, `clearDistantMesh`, and `newPoseCallback` receive the sensor pose `T_G_C`.
- Set (e.g. `base_link`): per integrated cloud, look up `T_G_B` at the cloud's stamp (TF mode: `world_frame ← body_frame`; queue mode: `T_G_D * T_B_D⁻¹`). Pass `T_G_B` to `removeDistantBlocks`, `clearDistantMesh`, and `newPoseCallback`. If the lookup fails, fall back to `T_G_C` with an `RCLCPP_WARN_THROTTLE` (10 s).
- This fixes A14/A15 for all four ESDF servers without touching their `newPoseCallback` bodies, because they only use `.getPosition()`.

**M10. Robot model marker.** `publishRobotMesh` runs only for clouds from the **primary sensor** (with the global `T_C_CH`, unchanged). In legacy mode that's every cloud, as today.

**M11. ICP.** If `enable_icp` is true and `sensors_.size() > 1`, log `RCLCPP_ERROR("ICP is not supported with multiple sensors; disabling")` and set `enable_icp_ = false` before `icp_transform_pub_` would be created. Note multi-sensor ICP (one shared correction, one designated ICP sensor) as a follow-up. Legacy behavior is unchanged. `basement_param.yaml` enables ICP and must keep working.

**M12. Frame counting.** `frame_count_` counts **integrated clouds across all sensors**; that's what the unchanged code would naturally do. Negative `update_mesh_every_n_sec` / `update_esdf_every_n_sec` ("every N frames") therefore fire N_sensors× as often. Document this in README and notes, and use positive (seconds) values in multi-sensor example configs. The per-callback post-processing in `insertPointcloud` (`publish_pointclouds_on_update_`, timing print, memory log, `frame_count_++`) stays per callback that processed ≥1 cloud, as today.

**M13. Validation (multi-sensor mode only; fail fast).** In the server constructor, before creating any subscription:
1. Every name in `sensor_names` is non-empty, unique, and matches `[A-Za-z0-9_]+`, so it's a valid parameter-name segment.
2. Every sensor has a non-empty `topic`. Topics must be unique across sensors (two sensors on one topic would double-integrate).
3. **Unknown / typo keys:** iterate `node_->get_node_parameters_interface()->get_parameter_overrides()`. Every key with prefix `sensors.` must be `sensors.<name in sensor_names>.<key in the §7.1 whitelist>`. Otherwise throw. A typo like `max_ray_lenght_m` must not be silently ignored.
4. **Forbidden per-sensor keys** (§7.2) throw with a message naming the key and saying it's map-global.
5. NP servers: `RangeImageProjector::Config::isValid()` for each sensor.
6. Queue mode (`use_tf_transforms: false`): each sensor needs a `T_B_C`, per sensor or inherited from the global one. Warn if several sensors end up with the identical `T_B_C`; that's almost always a config mistake.
7. On any failure, throw `std::invalid_argument` with a message that lists **all** problems found, not just the first. `node_main.h` mains let it propagate; the process exits non-zero with the message on stderr.
8. On success, log at INFO one line per sensor: name, topic, frame (or `<header>`), and the effective min/max ray length. For NP servers also log the projection model.

**M14. API compatibility for library users.**
- Keep `void insertPointcloud(PointCloud2::SharedPtr)`, `void insertFreespacePointcloud(...)`, and `virtual void processPointCloudMessageAndInsert(msg, T_G_C, is_freespace)`. They route to the primary sensor.
- New overloads take `SensorInput<…>*`.
- `integratePointcloud(T_G_C, points, colors, is_freespace)` keeps its signature and uses the primary sensor's integrator. A new overload takes the sensor.
- All constructors keep their signatures. The programmatic constructor's `integrator_config` becomes the **base** config that per-sensor overrides apply to.

**M15. Output and interface.** No new output topics or services are required. Add to docs: in multi-sensor mode the `pointcloud`/`freespace_pointcloud` subscriptions are replaced by the per-sensor topics. Optional (Phase 7): publish per-sensor stats on `~/sensor_status` as `diagnostic_msgs/DiagnosticArray`. Skip it if time-boxed; the INFO logs are enough.

---

## 6. Target layout (new and changed files)

```
voxfield_ros2/
├── MULTI_SENSOR_PLAN.md                           (this file)
├── docs/MULTI_SENSOR_NOTES.md                     (new: decisions, measurements, deviations)
├── README.md                                      (new "Multiple sensors" section)
├── scripts/fake_sensor_publisher.py               (extended: N sensors on a rig, yaw-sector clipping)
└── voxfield_ros/
    ├── include/voxfield_ros/
    │   ├── sensor_input.h                         (new: SensorConfig, SensorInput<T>)
    │   ├── sensor_config_loader.h                 (new: load + validate sensors from params)
    │   ├── range_image_projector.h                (new, extracted from NpTsdfServer)
    │   ├── param_utils.h                          (+ std::vector<std::string> support, + override-listing helper)
    │   ├── ros_params.h                           (+ per-sensor integrator/projector config builders)
    │   ├── transformer.h                          (M5/M6)
    │   ├── tsdf_server.h, np_tsdf_server.h        (sensors_ vector, overloads)
    │   └── intensity_server.h                     (unchanged API)
    ├── src/
    │   ├── sensor_config_loader.cc                (new)
    │   ├── range_image_projector.cc               (new)
    │   ├── transformer.cc, tsdf_server.cc, np_tsdf_server.cc, intensity_server.cc
    │   └── voxblox_server.cc, voxfield_server.cc, fiesta_server.cc, voxedt_server.cc   (only if M9 needs it; ideally untouched)
    ├── cfg/multi_sensor/
    │   ├── athena_param.yaml                      (new: map params for the Athena robot)
    │   ├── athena_dual_lidar.yaml                 (new: sensor_names + sensors.* for the 2 Livox)
    │   └── athena_lidar_rgbd.yaml                 (new: 2 Livox + 2 RGB-D)
    ├── cfg/rviz/multi_sensor.rviz                 (new)
    ├── launch/multi_sensor_mapping.launch.py      (new)
    └── test/
        ├── test_legacy_golden.cc                  (new, Phase 1)
        ├── test_data/golden_*.tsdf                (new, Phase 1)
        ├── test_range_image_projector.cc          (new)
        ├── test_sensor_config.cc                  (new)
        ├── test_transformer_multi.cc              (new)
        ├── test_multi_sensor_server.cc            (new)
        └── test_multi_sensor_smoke.launch.py      (new)
```

---

## 7. Parameter reference

### 7.1 Per-sensor keys (`sensors.<name>.<key>`): the whitelist
Everything not listed here is rejected by M13 step 3. "Inherits" means the default is the top-level parameter's effective value.

| Key | Type | Default | Servers | Meaning |
|---|---|---|---|---|
| `topic` | string | **required** | all | PointCloud2 input topic |
| `freespace_topic` | string | `""` | all | Optional freespace cloud topic |
| `frame` | string | `""` = `header.frame_id` | all | Overrides the cloud's frame for the pose lookup. **Not** inherited from `sensor_frame` (that would force all sensors into one frame) |
| `pointcloud_queue_size` | int | inherits | all | Subscription depth |
| `input_qos_best_effort` | bool | inherits | all | QoS choice (port plan D7) |
| `min_time_between_msgs_sec` | double | inherits | all | Per-sensor throttle |
| `T_B_C` + `invert_T_B_C` | double[16] + bool | inherits | all, queue mode only | Sensor extrinsic w.r.t. body |
| `method` | string | inherits | all | `simple` / `merged` / `fast` integrator |
| `min_ray_length_m`, `max_ray_length_m` | double | inherits | all | Ray limits |
| `voxel_carving_enabled`, `allow_clear`, `use_const_weight`, `use_weight_dropoff`, `use_sparsity_compensation_factor`, `sparsity_compensation_factor`, `anti_grazing`, `merge_with_clear`, `start_voxel_subsampling_factor`, `max_consecutive_ray_collisions`, `clear_checks_every_n_frames`, `max_integration_time_s` | as today | inherits | all | Integrator behavior. Take the exact key names from `ros_params.h:64-180`; the parameter name, not the C++ field name, is authoritative (e.g. `anti_grazing` → `enable_anti_grazing`) |
| `weight_reduction_exp`, `normal_available`, `reliable_band_ratio`, `curve_assumption`, `reliable_normal_ratio_thre` | as today | inherits | NP | NP-integrator behavior |
| `sensor_is_lidar`, `width`, `height`, `fov_up`, `fov_down`, `vx`, `vy`, `fx`, `fy`, `smooth_thre_ratio`, `min_z`, `min_dist` | as today | inherits | NP | Range-image model and noise filter (M8) |

Before writing the whitelist, `grep -n 'param(node' voxfield_ros/include/voxfield_ros/ros_params.h` and re-derive the integrator keys from the two integrator getters. If a key is read there but isn't listed above, decide whether it's map-global or per sensor and note the decision.

### 7.2 Forbidden per-sensor keys (map-global)
`tsdf_voxel_size`, `tsdf_voxels_per_side`, `voxel_size`, `voxels_per_side_in_block`, `truncation_distance`, `max_weight`, `integrator_threads`, `integration_order_mode`, `weight_dropoff_epsilon` (it depends on the voxel size), every `esdf_*` / `occ_*` / `mesh_*` key, `world_frame`, `use_tf_transforms`, `T_B_D`, `T_C_CH`, `enable_icp`, `body_frame`, and every `publish_*` / `update_*` key.

### 7.3 New global keys
| Key | Type | Default | Meaning |
|---|---|---|---|
| `sensor_names` | string[] | not set (legacy mode) | M1 |
| `body_frame` | string | `""` | M9 |
| `transform_queue_retention_sec` | double | `1.0` | M6 |

### 7.4 Legacy mode mapping (how `default` is built)
`topic = "pointcloud"`, `freespace_topic = use_freespace_pointcloud ? "freespace_pointcloud" : ""`, `frame = sensor_frame` (F1: empty → header), everything else from the top-level keys exactly as read today. No `sensors.*` key is read in legacy mode. If any `sensors.*` override exists while `sensor_names` is unset, log `RCLCPP_WARN` that it's ignored.

### 7.5 Example (Athena, dual Livox, `voxfield_server`)
```yaml
/**:
  ros__parameters:
    sensor_names: [front_lidar, back_lidar]
    world_frame: map
    body_frame: base_link
    use_tf_transforms: true
    # map-global (athena_param.yaml): tsdf_voxel_size 0.1, truncation_distance -3.0, ...
    update_mesh_every_n_sec: 1.0
    update_esdf_every_n_sec: 1.0
    sensor_is_lidar: true          # inherited by both
    fov_up: 52.0                   # measure in Phase 9 step 1; placeholder
    fov_down: -7.0
    width: 360
    height: 32
    min_ray_length_m: 0.3
    max_ray_length_m: 30.0
    sensors:
      front_lidar:
        topic: /athena/front_lidar/points_raw_livox
      back_lidar:
        topic: /athena/back_lidar/points_raw_livox
```
RGB-D sensors are added in `athena_lidar_rgbd.yaml` with `sensor_is_lidar: false`, `width: 640`, `height: 480`, `fx/fy/vx/vy` from `camera_info` (§3), `max_ray_length_m: 4.0`, and `min_time_between_msgs_sec: 0.4`.

---

## 8. Phased implementation

### Phase 0: Branch and baseline
1. `git checkout ros2-port && git checkout -b multi-sensor`.
2. Clean build + full test run: `colcon build && colcon test && colcon test-result --verbose`. Record the pass count in `docs/MULTI_SENSOR_NOTES.md`. It's the baseline every later phase is compared against.
3. Create `docs/MULTI_SENSOR_NOTES.md` with headings per phase.

**Accept:** baseline green and recorded.

### Phase 1: Golden regression tests for legacy mode (before any refactor)
These tests pin today's behavior so Phases 2–6 can prove "no change in legacy mode".
1. `test/test_legacy_golden.cc`, two cases: `TsdfServer` (`method: merged`) and `NpTsdfServer` (LiDAR model: `width 256, height 16, fov_up 3, fov_down -25`).
   - Node parameters: `integrator_threads: 1` (removes thread-order nondeterminism), `use_tf_transforms: false` (queue mode: poses can be injected without TF), `T_B_C`/`T_B_D` identity, `publish_*` false, `update_*_every_n_sec: 0`, `verbose: false`.
   - Access the protected `transformer_` via a test subclass (`class TestServer : public TsdfServer { public: using TsdfServer::transformer_; ... }`).
   - Feed 20 deterministic synthetic clouds of a box room (reuse the geometry logic of `scripts/fake_sensor_publisher.py`, ported to C++; fixed seed, no randomness). Before each cloud, call `transformer_.transformCallback()` with the pose at t−5 ms and t+5 ms, then `insertPointcloud(msg)`.
   - Save with `saveMap()` to a temp file.
2. Golden generation: when env `VOXFIELD_WRITE_GOLDEN=1` is set, the test writes `test/test_data/golden_{tsdf,np_tsdf}.tsdf` into the **source** tree (path via a CMake compile definition) instead of comparing. Generate once on the Phase 0 code and commit the files.
3. Comparison mode (default): load the golden and the fresh layer with `io::LoadLayer<TsdfVoxel>` and compare with `voxfield/include/voxfield/test/layer_test_utils.h` `CompareLayers`. Tolerance must be **exact** (0): the refactor may not change a single float. If `CompareLayers` has a hard-coded tolerance, add an exact variant in the test file instead of changing the shared util.
4. Register it with `ament_add_gtest`, linking like `test_server_map_io`.

**Accept:** the test passes on unmodified server code, and the golden files are committed. Sanity check that the test really compares: flip one voxel weight in a copy of the golden file and see it fail. Commit: `Multi-sensor Phase 1: legacy golden regression tests`.

### Phase 2: Pure refactor: extract `RangeImageProjector`, add string-array params (no behavior change)
1. Create `range_image_projector.{h,cc}` per M8. Move the code; don't rewrite it. Keep all the `ROS2_PORT deviation` comments with the code they describe.
2. `NpTsdfServer` owns one `RangeImageProjector` built from today's params (still a single member at this stage), and `processPointCloudMessageAndInsert` calls `projector->process(...)`. Keep the public wrapper methods (M8) so `test_np_tsdf_server.cc` is untouched.
3. `param_utils.h`: add `std::vector<std::string>` to `getParam` (accept `PARAMETER_STRING_ARRAY`; `PARAMETER_NOT_SET` → default). Add `std::vector<std::string> listParameterOverrides(rclcpp::Node&, const std::string& prefix)`, which returns the override names starting with the prefix. Extend `test_param_utils.cc`.
4. `test_range_image_projector.cc`: port the `test_np_tsdf_server.cc` cases to use the projector directly, and add a LiDAR round-trip case (points on a cylinder → image → extracted points equal to input within 1e-5).

**Accept:** all tests pass, including `test_legacy_golden` (exact) and the unchanged `test_np_tsdf_server`. `git diff --stat` shows no change to any `.yaml`, launch file, or core-library file. Commit.

### Phase 3: `Transformer` changes (M5, M6)
1. Remove `sensor_frame_` from `Transformer`. In `TsdfServer`/`NpTsdfServer::getNextPointcloudFromQueue`, resolve the frame as in M5 (legacy: `sensor_frame_` param, F1 fallback to header). Fix `IntensityServer` (A17).
2. Add per-sensor `T_B_C` support and `lookupSensorTransform` (M6). Keep the old signatures working.
3. Replace the erase policy with the retention window + size cap (M6). Add the `transform_queue_retention_sec` param (read in the `Transformer` constructor).
4. Add the `tfBuffer()` accessor.
5. `test_transformer_multi.cc`:
   - Queue mode, two sensors with different `T_B_C`: inject poses at 10 Hz. Look up sensor A at t=1.000, then sensor B at t=0.9995, which is **earlier** than A. Both succeed. `T_G_C_B == T_G_D(0.9995) * T_B_D⁻¹ * T_B_C_B` within 1e-6. Before the retention fix, the second lookup fails; run the test once against the Phase 2 code to prove that, and record it in the notes.
   - Retention: entries older than `stamp - retention` are gone after a lookup, and the deque size stays ≤ the cap.
   - TF mode via `tfBuffer()->setTransform(...)`: two child frames of `base_link`. `lookupSensorTransform` with `frame=""` uses the header frame; with `frame="x"` it uses `x`.
   - F1: legacy with `sensor_frame` unset resolves to `header.frame_id`.

**Accept:** new tests pass; `test_legacy_golden` exact; the existing smoke tests (5 launch tests) still pass. Commit.

### Phase 4: Sensor config loading and validation (M2, M13)
1. `sensor_input.h` (M4 structs) and `sensor_config_loader.{h,cc}`:
   ```cpp
   struct LoadedSensor { SensorConfig input; TsdfIntegratorBase::Config tsdf; NpTsdfIntegratorBase::Config np_tsdf;
                         RangeImageProjector::Config projector; std::string method; };
   // Legacy mode -> exactly one "default" sensor. Throws std::invalid_argument on validation errors (M13).
   std::vector<LoadedSensor> loadSensors(rclcpp::Node& node,
       const TsdfIntegratorBase::Config* tsdf_base, const NpTsdfIntegratorBase::Config* np_base,
       const RangeImageProjector::Config* projector_base, const std::string& legacy_method,
       const SensorConfig& legacy_input);
   ```
   The server fills `legacy_input` and `projector_base` from the top-level parameters it already reads, so each top-level key is still read once, in the same order. Parameter declaration order is visible in `ros2 param list`; keep it stable.
2. Per-sensor overrides: for each whitelisted key, `param(node, "sensors." + name + "." + key, field)`, where `field` was pre-filled from the base config. That gives "inherits" for free. **Read only whitelisted keys that are actually present in the overrides** (check with `listParameterOverrides`), so `ros2 param list` doesn't grow ~40 declared parameters per sensor. Record this in the notes.
   - Unit subtlety: the top-level `truncation_distance` may be negative (a multiple of the voxel size) and is converted in `ros_params.h`. Truncation is forbidden per sensor anyway, but check that every whitelisted key maps to a config field with the **same units** as the top-level key.
3. `test_sensor_config.cc`:
   - Legacy: no `sensor_names` → 1 sensor, `topic == "pointcloud"`, and the config equals `get*IntegratorConfigFromRosParam()` field by field.
   - Inheritance: top-level `max_ray_length_m: 45` with sensor B overriding 5 → A=45, B=5.
   - Errors, each producing one clear exception naming the key: unknown key `sensors.a.max_ray_lenght_m`; unknown sensor `sensors.c.topic` with `c` not in `sensor_names`; forbidden `sensors.a.truncation_distance`; missing topic; duplicate topic; duplicate name; invalid projector config (NP).
   - Several errors at once → the message lists all of them.
   - A `sensors.*` override in legacy mode → a warning, and it's ignored.

**Accept:** tests pass; golden exact. Commit.

### Phase 5: `TsdfServer` multi-sensor frontend
1. Replace `pointcloud_sub_`, `freespace_pointcloud_sub_`, `pointcloud_queue_`, `freespace_pointcloud_queue_`, `last_msg_time_*`, and `tsdf_integrator_` with `sensors_` (M4), built from `loadSensors()` in the constructor, at the point where the subscription and integrator are created today. Keep the construction order: config → publishers → subscriptions → integrators → services → timers.
   - Integrators need `tsdf_map_`, which is created after the subscriptions today. Create the integrators after `tsdf_map_.reset(...)`, and create **subscriptions last** among these, so no callback can fire before the integrators exist. With a single-threaded executor nothing fires during construction anyway, but make it correct by construction.
2. `insertPointcloud(msg, sensor)` / `insertFreespacePointcloud(msg, sensor)`: today's bodies, using `sensor->queue`, `sensor->last_msg_time`, and the per-sensor `getNextPointcloudFromQueue(sensor, …)` (frame resolution M5, `lookupSensorTransform` M6). Update the stats counters.
3. `processPointCloudMessageAndInsert(msg, T_G_C, is_freespace, sensor)`: integrate with `sensor->integrator`. ICP guard (M11). Body pose (M9; implement the lookup here). Robot marker only for the primary sensor (M10). `newPoseCallback(T_pose)` with the body or sensor pose.
4. Verbose logs gain the sensor name: `"[%s] Integrating a pointcloud with %lu points."`.
5. Keep the M14 compatibility overloads.
6. `test_multi_sensor_server.cc` (queue mode, `integrator_threads: 1`, injected poses as in Phase 1):
   - **Equivalence:** a multi-sensor server with sensors A and B, **identical** integrator settings and **identical** `T_B_C`, fed clouds alternately on A's and B's topics, must produce a TSDF **exactly** equal to a legacy server fed the same clouds in the same order on `pointcloud`. Call the callbacks directly: `insertPointcloud(msg, sensors_[i])` through the test subclass.
   - **Different extrinsics:** A looks at +x, B at −x (`T_B_C` yaw 0 vs 180°), and each sees only its half of the room. After integration, voxels on both the +x and −x walls are observed (weight > 0) with near-zero distance.
   - **Different ray limits:** B has `max_ray_length_m: 2`, so no voxel farther than 2 m + truncation from B's origin is updated by B's clouds. Check on a layer where only B integrated.
   - **Throttle is per sensor:** `min_time_between_msgs_sec: 0.5`, A and B at the same stamps → both integrate.
   - **ICP guard:** `enable_icp: true` with 2 sensors → ICP is off, and the node didn't create `~/icp_transform`.

**Accept:** all tests pass; golden exact for `TsdfServer`; the 5 existing smoke launch tests pass; `voxblox_server`, `fiesta_server`, and `voxedt_server` start in legacy mode and in multi-sensor mode (`ros2 node info` shows the per-sensor topics and no `pointcloud`). Commit.

### Phase 6: `NpTsdfServer` multi-sensor frontend
Same as Phase 5, plus: each `SensorInput` owns a `RangeImageProjector` built from its projector config (M8), and `processPointCloudMessageAndInsert` uses `sensor->projector->process(...)`. The wrapper methods delegate to `sensors_[0]->projector`.
- Tests (extend `test_multi_sensor_server.cc` for `NpTsdfServer`): the equivalence test (exact); a mixed test with sensor A LiDAR (`width 256, height 16`) and sensor B camera (`640×480, fx 450`) that constructs, integrates both, and observes voxels from both; and an invalid B projector (`width: 0`), which must throw at construction with B's name in the message.

**Accept:** all tests pass; golden exact for `NpTsdfServer`; `voxfield_server` and `np_tsdf_server` start in both modes. Commit.

### Phase 7: Body frame for derived ESDF servers (M9) + optional status topic
1. Verify that `VoxbloxServer`, `VoxfieldServer`, `FiestaServer`, and `VoxedtServer::newPoseCallback` now receive the body pose when `body_frame` is set. No code change should be needed in them. If one reads anything but `.getPosition()`, adapt it and note it.
2. Test: `body_frame: base_link` with two sensors 1 m apart, `max_block_distance_from_body: 5`. Integrate from both. Blocks are removed relative to `base_link`, not relative to either sensor. Check a block 5.3 m from the body but 4.8 m from sensor A: it's removed with `body_frame` set, and kept without.
3. Optional: the `~/sensor_status` DiagnosticArray (M15), published by the existing `publish_map` timer, with one status per sensor (received / throttled / dropped / integrated, last stamp, last integration ms). Skip it if time-boxed; say so in the notes.

**Accept:** tests pass; golden exact. Commit.

### Phase 8: Launch, configs, fake publisher, smoke test
1. `scripts/fake_sensor_publisher.py`: add `num_sensors` (default 1 = unchanged behavior) and `sensor_yaw_offsets_deg` (default evenly spaced). Sensor *i* has frame `<sensor_frame>_<i>`, is mounted on a rig frame `<sensor_frame>_rig` at 0.5 m offset and yaw offset *i*, and publishes on `<pointcloud_topic>_<i>`. Each sensor only emits points within ±90° of its forward axis. Add `publish_tf` support for the rig (moving `world → rig`, static `rig → sensor_i`).
2. `test/test_multi_sensor_smoke.launch.py` (registered with `add_launch_test`, `ARGS method:=voxfield` and `method:=voxblox`): fake publisher with `num_sensors:=2`, yaw offsets 0/180. Server with `sensor_names: [s0, s1]` and topics `pointcloud_0/1`, `update_esdf_every_n_sec: 1.0`. Assert within 40 s:
   - `~/mesh` has non-empty blocks on **both** the +x wall (x > 4) and the −x wall (x < −4). With a single sensor (a sanity run in the notes), only one wall appears.
   - `~/esdf_slice` is non-empty.
   - `~/save_map` + `~/load_map` round-trip, as in the existing smoke test.
   - No `ERROR` lines in the server output.
3. `launch/multi_sensor_mapping.launch.py`. Arguments: `method` (as in `mapping.launch.py`), `param_file` (map-global YAML), `sensors_file` (the `sensor_names` + `sensors.*` YAML), `bag_file`, `play_bag`, `speed`, `start_offset`, `rviz`, `rviz_config`, `use_sim_time` (default true), `tf_remap_prefix` (default `""`; if set, e.g. `/athena`, adds remappings `/tf → <prefix>/tf`, `/tf_static → <prefix>/tf_static` **to the server node**), and `rgbd` (default false). When `rgbd:=true`, the launch starts a `ComposableNodeContainer` per camera with `image_transport` republish (`compressedDepth` → raw) + `depth_image_proc::PointCloudXyzNode`, and remaps its `image_rect`/`camera_info`/`points` to the Athena topics. **Check the actual component plugin names and remap keys** with `ros2 component types`. Bag play uses `--clock`, plus the `/tf_static` QoS override file from port plan pitfall §8.7 if static TFs go missing.
4. Configs `cfg/multi_sensor/athena_*.yaml` per §7.5. Start from the local Phase 12 param file described in `docs/ROS2_PORT_NOTES.md` Step 4 (`world_frame: map`, `voxel_size: 0.1`, ICP off), converted to the full key set.
5. `cfg/rviz/multi_sensor.rviz`: fixed frame `map`, `VoxfieldMesh` on `/voxfield_node/mesh` (Transient Local), `esdf_slice`, one `PointCloud2` display per raw sensor topic in a distinct color, and TF.

**Accept:** `colcon test` passes, including the new launch tests and the unchanged old ones. `ros2 launch voxfield_ros multi_sensor_mapping.launch.py method:=voxfield param_file:=… sensors_file:=… play_bag:=false` starts cleanly, and M13 prints one INFO line per sensor. Commit.

### Phase 9: Real-bag validation (Athena bag `rosbag2_2026_09_23-14_32_47`)
Run everything under a memory cap (§0.7), use `--start-offset` if parts of the bag are bad, and time-box each run to about 60 s of bag time unless stated otherwise.
1. **Measure the Livox geometry:** a small `rosbag2_py` script in the scratchpad reads 50 clouds per LiDAR and prints the min/max pitch (deg), min/max range, and point count. Put the real `fov_up`/`fov_down` into `athena_dual_lidar.yaml` and record the numbers in the notes.
2. **Ray-casting fusion (`voxblox_server`):** run three times: (a) front only (legacy mode, as in Phase 12), (b) back only, (c) both (multi-sensor). With `verbose: true`, record the final allocated block count for each. Expect `max(a,b) < c ≤ a + b` (overlap makes it less than the sum). Save each map (`~/save_map`) and PLY (`~/generate_mesh` + `mesh_filename`). Screenshot (c) in RViz2 with both raw clouds overlaid: `docs/assets/multi_sensor_voxblox.png`.
3. **Consistency check:** in (c), static structure seen by both LiDARs must line up. No doubled walls. A doubled wall means an extrinsic/TF problem: check the `/athena/tf_static` chain and whether `frame` is wrong. Inspect a wall slice with `~/tsdf_slice`, and note the result.
4. **Per-sensor stats:** both sensors integrate at ≈10 Hz (`num_integrated`/elapsed), with `num_dropped == 0` after the first seconds of TF warm-up. Record the per-sensor integration time (the `timing: true` output). Check M7's budget.
5. **`voxfield_server` with both Livox:** tune `width/height` per §3.3. Add a temporary verbose log of the fraction of points with a non-zero normal per sensor. Try (i) `normal_available: true` with a coarse image, and (ii) `normal_available: false`. Record the valid-normal fraction, the mesh quality (screenshot), and the ESDF slice for both. Pick the default for `athena_dual_lidar.yaml` and justify it in the notes. Watch RSS; the #12 fix should keep it bounded.
6. **Optional RGB-D:** `rgbd:=true` with `athena_lidar_rgbd.yaml`. Check that the depth clouds appear in RViz2 aligned with the LiDAR geometry, that integration keeps up (per-sensor stats), and that the near-range detail improves. Also check the NP camera model with fx/fy/cx/cy from `camera_info`. If `depth_image_proc` or the republish chain doesn't work on this bag, document why and stop there. It's optional.
7. `fiesta_server` / `voxedt_server`: one multi-sensor run each, checking that the mesh and ESDF build and nothing errors.

**Accept:** steps 1–5 and 7 done, with numbers and screenshots in `docs/MULTI_SENSOR_NOTES.md`; step 6 done or explicitly reported. If the bag isn't readable in this environment, stop and give the user exact commands. Commit (docs + configs).

### Phase 10: Docs and cleanup
1. README: add a "Multiple sensors (one map)" section covering the concept, legacy vs multi-sensor mode, the §7.5 example, the per-sensor key table (short form, linking here), the frame-counting note (M12), ICP limitation (M11), the RGB-D via `depth_image_proc` recipe, and the `tf_remap_prefix` launch arg. Update the interface table: the `pointcloud` row gets "legacy mode only; multi-sensor mode subscribes to `sensors.<name>.topic`".
2. `docs/MULTI_SENSOR_NOTES.md`: decisions, the F1 fix, the M6 queue-policy change and its test evidence, measurements, and follow-ups (multi-sensor ICP, deskew, self-filter, per-sensor voxel downsampling, CameraInfo-driven intrinsics).
3. `clang-format` on changed C++ files only (port plan Phase 11 rule).
4. Final `colcon build` in both the clean env (`scripts/clean_env.sh`) and the user's normal shell with the Hector underlay; full `colcon test`.

**Accept:** Definition of Done (§10). Commit. Don't push or open a PR without the user's approval.

---

## 9. Pitfalls (read before coding)

1. **Silent legacy drift.** Moving parameter reads around can change **declaration order** and, worse, **defaults**. For example, reading `pointcloud_queue_size` before `getServerConfigFromRosParam()` changes nothing, but reading a sensor key with a different default does. The Phase 1 golden test catches voxel changes, not parameter-list changes. Also diff `ros2 param dump` of a legacy `voxfield_server` (with `kitti_param.yaml` + `kitti_calib.yaml`) before Phase 2 and after Phase 6. The only allowed differences are the new globals `body_frame` and `transform_queue_retention_sec`. Record the diff.
2. **Subscription callbacks capturing `this` plus a raw sensor pointer:** `sensors_` must hold `unique_ptr`s and must never be resized after subscriptions exist.
3. **The queue-mode erase policy (A8)** is invisible with one sensor and fatal with two. The Phase 3 test must be proven to fail on the old code.
4. **`frame = ""` in legacy mode used to mean "lookup fails forever".** F1 changes that. Make sure no shipped config relied on it: `grep -rn sensor_frame voxfield_ros/cfg`. KITTI/MaiCity set `velodyne`; check the others and note them.
5. **Empty YAML arrays** (`sensor_names: []`) fail to parse (§5 M1). Document "omit the key" instead.
6. **Per-sensor parameter explosion:** declaring every whitelisted key for every sensor makes `ros2 param list` huge and slows startup. Read only the keys present in the overrides (Phase 4 step 2).
7. **YAML int vs double** for per-sensor keys (`max_ray_length_m: 5`) is handled by `param_utils` coercion (port plan D6), but only through `param()`. Never call raw `declare_parameter` for sensor keys.
8. **Timestamps across sensors:** clouds from different sensors arrive in any order. Never compare one sensor's stamp against another sensor's throttle clock (A4). Never assume the TF or transform queue only moves forward (A8).
9. **Namespaced TF (`/athena/tf`):** `tf2_ros::TransformListener` subscribes to absolute `/tf` and `/tf_static`. Node-level remaps `/tf:=/athena/tf` work for it. Relative remaps like `tf:=…` don't. Check that no other node in the launch (RViz2) needs the same remap. RViz2 does, so set it on the rviz node too.
10. **`/tf_static` from `ros2 bag play`** may be volatile (port plan pitfall 7): use the QoS override file.
11. **Self-observation:** the back LiDAR may see the front sensor mount and vice versa. In the fused map these show up as permanent obstacles next to the robot, and the ESDF around the robot shrinks. Use per-sensor `min_ray_length_m` as the first mitigation. Document anything left over. Don't implement a self-filter (out of scope).
12. **Integration budget:** two Livox at 10 Hz is light. Adding 640×480 RGB-D at 4.5 Hz on a 0.1 m grid can dominate. Measure it (Phase 9). Use per-sensor `min_time_between_msgs_sec` / `max_ray_length_m`, not threads (M7).
13. **Memory:** the ESDF local-range fix (#12) bounds allocation to TSDF-backed blocks, but two sensors cover more space, so the TSDF itself is bigger. Keep the cgroup cap on every real-bag run.
14. **`kDefaultMaxIntensity` double definition** (port plan pitfall 13): the new shared headers (`sensor_input.h`, `range_image_projector.h`) must not include both `tsdf_server.h` and `np_tsdf_server.h`.
15. **Eigen alignment:** `SensorInput` holds `Transformation` inside `SensorConfig`, so keep `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` and allocate with `std::make_unique`.

---

## 10. Definition of Done

- [ ] Legacy mode (no `sensor_names`): `test_legacy_golden` passes with exact equality for `TsdfServer` and `NpTsdfServer`. All pre-existing unit tests and the 5 existing smoke launch tests pass unmodified. The `ros2 param dump` diff contains only the documented new globals.
- [ ] Multi-sensor mode works for `tsdf_server`, `np_tsdf_server`, `voxblox_server`, `voxfield_server`, `fiesta_server`, and `voxedt_server`: one TSDF, one ESDF, one mesh.
- [ ] Per-sensor: topic, frame, extrinsic (queue mode), throttle, queue, integrator config, and (NP) projection model. Inheritance from top-level keys, strict validation, and a clear error listing all problems.
- [ ] Transform queue is safe with out-of-order multi-sensor lookups (a test that fails on the old code).
- [ ] ICP disabled with an error in multi-sensor mode. Robot marker from the primary sensor only. Optional `body_frame` works.
- [ ] New unit tests (`test_sensor_config`, `test_transformer_multi`, `test_range_image_projector`, `test_multi_sensor_server`) and `test_multi_sensor_smoke.launch.py` (voxfield + voxblox) pass.
- [ ] `multi_sensor_mapping.launch.py` + `cfg/multi_sensor/athena_*.yaml` + `multi_sensor.rviz` exist and run on the Athena bag.
- [ ] Phase 9 measurements and screenshots recorded, or explicitly reported as pending with exact commands.
- [ ] README section, `docs/MULTI_SENSOR_NOTES.md`, and the updated interface table.
- [ ] Clean-env and Hector-underlay builds both succeed. Branch `multi-sensor`, one commit per phase, nothing pushed without approval.

---

## 11. Open decisions for the user (proceed with the default unless told otherwise)

1. **Athena configs in the repo.** Default: commit `cfg/multi_sensor/athena_*.yaml` as the worked example, since this is the target robot. Alternative: keep them local and ship only a generic `example_dual_lidar.yaml`.
2. **Behavior with `body_frame` unset in multi-sensor mode.** Default: use each sensor's own position (M9), same as legacy. Alternative: require `body_frame` whenever there's more than one sensor.
3. **Multi-sensor ICP.** Default: disabled (M11). Follow-up design: one designated `icp_sensor` whose correction applies to all sensors.
4. **RGB-D intrinsics.** Default: static per-sensor `fx/fy/vx/vy` in YAML, copied from `camera_info`. Follow-up: a per-sensor `camera_info_topic` that fills the projector config from the first `CameraInfo` message.
5. **Frame counting (M12).** Default: count every integrated cloud. Alternative: count only primary-sensor clouds, which keeps "every N frames" rates independent of the number of sensors.
