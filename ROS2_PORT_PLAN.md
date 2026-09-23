# Voxfield → ROS 2 Port: Specification & Implementation Plan

> **Audience:** an autonomous coding agent doing the port end to end.
> **Repo:** `~/src/voxfield_ros2` (fork of `VIS4ROB-lab/voxfield`, currently ROS 1 / catkin, packages named `voxblox*`).
> **Target:** ROS 2 **Jazzy**, Ubuntu **24.04**, `ament_cmake`, C++17.
> **Also in scope:** a full rename from `voxblox` to **`voxfield`** (packages, C++ namespace, include paths, protobuf package, messages, RViz plugin), so voxfield and voxblox can be installed and used side by side.

---

## 0. How to use this document

1. Read §1–§5 fully before touching code. They define *what* "done" means and the design decisions you must follow.
2. Execute §6 phases **in order**. Each phase has explicit **acceptance criteria**. Do not start the next phase until the current one passes.
3. Work on a branch: `git checkout -b ros2-port`. Make **one commit per phase** (or per sub-step within big phases) so each is reviewable and bisectable. The rename (Phase 1) **must** be its own commit(s), with no other changes, so that later diffs show only the port.
4. §7 is an API translation cheat sheet; §8 lists pitfalls that *will* bite you if ignored. Consult them constantly.
5. **Behavior preservation rule:** this is a port plus a rename, not a rewrite. Keep mapping algorithms, default values, **ROS parameter names, topic names, and service names** exactly as they are. Only the identifiers listed in §4 D1 get renamed. If you find a real bug in the original, do **not** fix it silently: add it to `docs/ROS2_PORT_NOTES.md` under "Known upstream issues" (§8.14 lists ones already found).
6. When you hit a decision not covered here, choose the option that keeps ROS 1 behavior identical, write down the choice in `docs/ROS2_PORT_NOTES.md`, and continue.

---

## 1. Goal and scope

### 1.1 Goal
A colcon workspace containing this repo builds cleanly on ROS 2 Jazzy and all unit tests pass. Every mapping server (`np_tsdf_server`, `voxfield_server`, `tsdf_server`, `voxblox_server`, `fiesta_server`, `voxedt_server`, `intensity_server`) runs on ROS 2 bag data with the same parameters, topics, services, and output quality as the ROS 1 version. Meshes display in RViz2 through the ported `voxfield_rviz_plugin`. All of this also works when some other `voxblox` ROS 2 install is present in the same environment.

### 1.2 In scope
- Rename all 4 packages, and everything that would clash with a voxblox install, to `voxfield*` (§4 D1).
- Port all 4 packages to ROS 2: `voxfield` (core lib), `voxfield_msgs`, `voxfield_ros`, `voxfield_rviz_plugin`.
- All executables listed in §3.3.
- All `.launch` files → Python launch files (`.launch.py`).
- The repo's shipped parameter/calibration YAML files (`voxfield_ros/cfg/param/*.yaml`, `cfg/calib/*.yaml`, etc., which come from upstream voxfield for its example datasets) → ROS 2 parameter-file format.
- All `.rviz` configs → RViz2 format.
- Unit tests (gtest) for the core library.
- A launch-based smoke test that runs without any dataset.
- README / install docs updated for ROS 2.

### 1.3 Out of scope (non-goals)
- ROS 1 compatibility. The repo becomes ROS 2-only; ROS 1 lives in git history.
- Algorithm changes, performance tuning, new features.
- Drop-in source compatibility with downstream voxblox users (Cblox, Voxgraph, Kimera, Panmap). After the rename, those projects need their includes and namespace adapted to use voxfield. Note this in the README. Do **not** add a `namespace voxfield = voxfield;` alias: it would reintroduce the clash the rename exists to avoid.
- Python protobuf generation (the old `PROTOBUF_COMPILE_PYTHON` flag); nothing in this repo uses it.
- Composable nodes/components. This is an optional stretch goal (§10), only after everything else is done.
- The `stereo_image_proc` / `image_undistort` / `dense_stereo` pipelines referenced in commented-out or `bak/` launch files. Port only what's needed for LiDAR/RGB-D point cloud input.
- Displays from third-party RViz plugins in old `.rviz` files (`mav_planning_rviz/PlanningPanel`, `rviz_plugin_tutorials/Imu`). Drop them.

---

## 2. Environment facts (verified on the target machine)

| Item | Value |
|---|---|
| OS | Ubuntu 24.04.4 |
| ROS | `/opt/ros/jazzy` |
| colcon | `/usr/bin/colcon` |
| Eigen | 3.4.0 |
| PCL | 1.14.0 (`libpcl-dev`) |
| OpenCV | 4.6.0 |
| protobuf | 3.21.12 (`libprotobuf-dev`, `/usr/bin/protoc`) |
| glog / gflags | 0.6.0 / 2.2.2 |
| ROS pkgs present | `cv_bridge`, `pcl_conversions`, `pcl_ros`, `tf2*`, `interactive_markers`, `rviz_common`, `rviz_default_plugins`, `rviz_ogre_vendor`, `rviz_rendering` |

### 2.1 A separate voxblox install is on this machine (why the rename matters)
The user's `~/.zshrc` sources `/opt/ros/jazzy` **and** `~/src/hector/install/setup.zsh`, which chains `/opt/hector/jazzy`. That underlay comes from apt packages `hector-jazzy-*` (`deb.teamhector.de`), installed as dependencies of other Hector work. The user does **not** use its voxblox packages, but because it's sourced, they are visible in every shell. It provides:

`minkindr`, `minkindr_conversions`, `voxblox`, `voxblox_proto`, `voxfield_msgs`, `voxfield_ros`, `voxfield_rviz_plugin`

(They are built from `https://github.com/tu-darmstadt-ros-pkg/voxblox_ros2`, branch `ros2`: a ROS 2 port of *upstream voxblox*, not voxfield.)

With the voxfield rename (§4 D1), none of this repo's packages, headers, namespaces, protobuf types, message types, plugin names, or Ogre material names collide with that install. The only shared thing left is the header-only `kindr/minimal/*` code (see D3), which is identical upstream code and harmless.

**Rules:**
- No package in this repo may be named `voxblox*`, `minkindr`, or `minkindr_conversions`.
- The build must succeed **both** in a clean shell (only `/opt/ros/jazzy` sourced) **and** in the user's normal shell with the Hector underlay sourced. Check both at the end of Phases 3, 7, and 11. Create `scripts/clean_env.sh` (sources only `/opt/ros/jazzy` + the workspace install) for the clean variant:
  ```bash
  env -i HOME="$HOME" USER="$USER" TERM="$TERM" DISPLAY="$DISPLAY" \
    bash --noprofile --norc -c 'source /opt/ros/jazzy/setup.bash && cd ~/voxfield_ws && colcon build ...'
  ```
- Never add a dependency on any `/opt/hector` package.

The Hector port is still a **useful reference implementation** for the mechanical ROS 1 → ROS 2 API translation. Its installed headers are readable under `/opt/hector/jazzy/include/voxfield_ros/` (e.g. `ros_parameters.hpp`, `transformer.h`, `tsdf_server.h`, `node_helper.h`). Read it for patterns; **do not depend on it or copy it blindly**. It lacks voxfield's NP-TSDF/Voxfield/FIESTA/EDT servers, `MultiMesh.msg` and `LayerWithTrajectory.msg`, and it uses a strict `declare_parameter<T>` style that breaks on this repo's YAML (see D6).

---

## 3. Current state inventory (ROS 1, before rename)

### 3.1 Packages
| Current package | New name | Build | ROS 1 deps | Notes |
|---|---|---|---|---|
| `voxblox` | `voxfield` | catkin_simple | `eigen_catkin`, `eigen_checks`, `gflags_catkin`, `glog_catkin`, `minkindr`, `protobuf_catkin` | Pure C++ core lib. The ROS 1 build is only in CMake/package.xml. Protobuf generated via `PROTOBUF_CATKIN_GENERATE_CPP2`. 10 gtests + 2 tool binaries. |
| `voxfield_msgs` | `voxfield_msgs` | catkin_simple | `std_msgs`, `nav_msgs` | 7 msgs (`Block`, `Layer`, `LayerWithTrajectory`, `Mesh`, `MeshBlock`, `MultiMesh`, `VoxelEvaluationDetails`), 1 srv (`FilePath`). |
| `voxfield_ros` | `voxfield_ros` | catkin_simple | `cv_bridge`, `gflags_catkin`, `interactive_markers`, `minkindr_conversions`, `pcl_conversions`, `pcl_ros`, `sensor_msgs`, `tf`, `voxblox`, `voxfield_msgs`, `voxfield_rviz_plugin` | ~9.7k LOC, all ROS-coupled. |
| `voxfield_rviz_plugin` | `voxfield_rviz_plugin` | catkin_simple | `rviz` (Qt5/Ogre1) | `VoxfieldMesh` and `VoxfieldMultiMesh` displays. |

External source deps pulled via `voxfield_*.rosinstall`: `catkin_simple`, `eigen_catkin`, `eigen_checks`, `gflags_catkin`, `glog_catkin`, `minkindr`, `minkindr_ros`, `protobuf_catkin`. **All of these go away** (see D3/D4).

### 3.2 `voxfield_ros` files and their ROS coupling
Headers (`include/voxfield_ros/`): `conversions.h`, `conversions_inl.h`, `fiesta_server.h`, `intensity_server.h`, `intensity_vis.h`, `interactive_slider.h`, `mesh_pcl.h`, `mesh_vis.h`, `np_tsdf_server.h`, `ptcloud_vis.h`, `ros_params.h` (15 `get*ConfigFromRosParam` functions, ~93 `param()` calls), `simulation_server.h`, `transformer.h`, `tsdf_server.h`, `voxblox_server.h`, `voxedt_server.h`, `voxfield_server.h`.

Sources (`src/`): `fiesta_server.cc`, `intensity_server.cc`, `interactive_slider.cc`, `np_tsdf_server.cc`, `simulation_server.cc`, `transformer.cc`, `tsdf_server.cc`, `voxblox_server.cc`, `voxedt_server.cc`, `voxfield_server.cc`, plus mains `*_server_node.cc` ×7, `voxblox_eval.cc`, `simulation_eval.cc`, `visualize_tsdf.cc`.

Class hierarchy:
```
TsdfServer ──┬── VoxbloxServer   (ESDF, original voxblox *method*)
             ├── FiestaServer    (ESDF via occupancy + FIESTA)
             ├── VoxedtServer    (ESDF via occupancy + EDT)
             └── IntensityServer
NpTsdfServer ─── VoxfieldServer  (non-projective TSDF + Voxfield ESDF)
SimulationServer (standalone)
Transformer (owned by TsdfServer/NpTsdfServer)
```
`TsdfServer` and `NpTsdfServer` are near-duplicates. **Keep them separate.** Deduplicating is out of scope.

### 3.3 Executables (must all exist after the port; names unchanged)
Executable names describe the **method**, not the project. `voxblox_server` runs the Voxblox ESDF method inside voxfield. Keep them all.

| New package | Executable | ROS 1 default node name (`ros::init`) |
|---|---|---|
| voxfield_ros | `tsdf_server` | `voxblox` |
| voxfield_ros | `np_tsdf_server` | `voxfield` |
| voxfield_ros | `voxblox_server` | `voxblox` |
| voxfield_ros | `voxfield_server` | `voxfield` |
| voxfield_ros | `fiesta_server` | `fiesta` |
| voxfield_ros | `voxedt_server` | `voxedt` |
| voxfield_ros | `intensity_server` | `voxblox` (check source) |
| voxfield_ros | `voxblox_eval` | `voxblox_node` |
| voxfield_ros | `simulation_eval` | `voxblox_sim` |
| voxfield_ros | `visualize_tsdf` | `visualize_tsdf_node` |
| voxfield | `tsdf_to_esdf`, `test_load_esdf` | (non-ROS tools) |

Keep the same default node names. Launch files override them anyway.

### 3.4 ROS interface surface (names are relative; `~` = node-private). Names stay unchanged.
Derived by grepping `advertise`/`subscribe`/`advertiseService`. **Re-verify with grep during Phase 5/6** and record the final table in `docs/ROS2_PORT_NOTES.md`.

- **Subscriptions (global/namespace-relative, meant to be remapped):** `pointcloud`, `freespace_pointcloud` (if `use_freespace_pointcloud`), `transform` (if `!use_tf_transforms`).
- **Subscriptions (private):** `~tsdf_map_in`, `~esdf_map_in`, `~intensity_image`.
- **Publishers (private):** `~mesh` (Mesh msg, latched), `~surface_pointcloud`, `~tsdf_pointcloud`, `~gsdf_pointcloud`, `~tsdf_slice`, `~gsdf_slice`, `~occupied_nodes` (MarkerArray), `~tsdf_map_out`, `~esdf_map_out` (Layer msg, not latched), `~Robot_model` (Marker, depth 100), `~icp_transform`, `~esdf_pointcloud`, `~esdf_slice`, `~traversable`, `~esdf_error_slice`, `~intensity_pointcloud`, `~intensity_mesh`, plus simulation/eval/visualize-only topics (`tsdf_gt`, `esdf_gt`, `tsdf_test`, `esdf_test`, `tsdf_gt_mesh`, `tsdf_test_mesh`, `view_ptcloud_pub`, `gt_ptcloud`, `mesh_pcl`, `mesh_as_pointcloud`, `all_tsdf_voxels`, `tsdf_voxels_near_surface`).
- **Services (private):** `~generate_mesh`, `~clear_map` (std_srvs/Empty); `~save_map`, `~load_map`, `~save_esdf_map`, `~save_occ_map`, `~save_all_map` (FilePath srv); `~publish_pointclouds`, `~publish_map` (Empty).
- **TF:** looks up `world_frame ← sensor_frame` (or the message frame); broadcasts `icp_corrected` / `pose_corrected` when ICP is enabled.

Note that the message **types** change (`voxfield_msgs/Mesh` → `voxfield_msgs/msg/Mesh`), while the topic **names** don't.

---

## 4. Target design & binding decisions

**D1. Rename map (authoritative).** Rename exactly these; nothing else.

| Kind | From | To |
|---|---|---|
| Package / dir | `voxblox/` | `voxfield/` |
| Package / dir | `voxfield_msgs/` | `voxfield_msgs/` |
| Package / dir | `voxfield_ros/` | `voxfield_ros/` |
| Package / dir | `voxfield_rviz_plugin/` | `voxfield_rviz_plugin/` |
| Include root | `voxblox/include/voxblox/…` → `${1}voxfield/core/…>` | `voxfield/include/voxfield/…` → `#include <voxfield/core/…>` |
| Include root | `include/voxfield_ros/…`, `"voxfield_ros/…"` | `include/voxfield_ros/…`, `"voxfield_ros/…"` |
| Include root | `include/voxfield_rviz_plugin/…` | `include/voxfield_rviz_plugin/…` |
| C++ namespace | `namespace voxfield`, `voxfield::` | `namespace voxfield`, `voxfield::` |
| C++ namespace | `namespace voxfield_rviz_plugin` | `namespace voxfield_rviz_plugin` |
| Include guards | `VOXFIELD_…_H_`, `VOXFIELD_ROS_…`, `VOXFIELD_RVIZ_PLUGIN_…` | `VOXFIELD_…_H_`, `VOXFIELD_ROS_…`, `VOXFIELD_RVIZ_PLUGIN_…` |
| Protobuf | `proto/voxfield/*.proto`, `package voxfield;`, `"voxblox/Block.pb.h"` | `proto/voxfield/*.proto`, `package voxfield;`, `"voxfield/Block.pb.h"` |
| Messages | `voxfield_msgs::…` / `voxfield_msgs/…` | `voxfield_msgs::msg::…` / `voxfield_msgs/msg/…` |
| RViz plugin classes | `VoxfieldMeshDisplay`, `VoxfieldMultiMeshDisplay`, `VoxfieldMeshVisual` | `VoxfieldMeshDisplay`, `VoxfieldMultiMeshDisplay`, `VoxfieldMeshVisual` |
| RViz plugin lookup names | `voxfield_rviz_plugin/VoxfieldMesh`, `/VoxfieldMultiMesh` | `voxfield_rviz_plugin/VoxfieldMesh`, `/VoxfieldMultiMesh` |
| Ogre resources | `voxblox.material`, materials `VoxfieldMaterial`, `VoxfieldMaterialTransparent`, resource group `VoxfieldMaterials` | `voxfield.material`, `VoxfieldMaterial`, `VoxfieldMaterialTransparent`, `VoxfieldMaterials` |
| CMake targets | `voxblox`, `voxblox_proto`, `voxfield_ros`, `voxfield_rviz_plugin` | `voxfield`, `voxfield_proto`, `voxfield_ros`, `voxfield_rviz_plugin` |
| Launch node name | `name="voxblox_node"` in every `.launch` → topics `/voxblox_node/...` | `name='voxfield_node'` → topics `/voxfield_node/...`. Update RViz configs, the smoke test, and README examples to match. |

**Do NOT rename** (these name the Voxblox *method* or are external interfaces):
- Class `VoxbloxServer`, files `voxblox_server.{h,cc}`, `voxblox_server_node.cc`, executable `voxblox_server`.
- Executable/file `voxblox_eval`, class `VoxbloxEvaluator`, and its ROS params `voxblox_file_path`, `voxblox_esdf_file_path`, `voxblox_occ_file_path`.
- Free functions such as `generateVoxbloxMeshMsg`, `colorVoxbloxToMsg`. Renaming them adds churn and no isolation benefit, since they're already inside the renamed namespace.
- Launch dir `launch/voxblox_launch/` and files `*_voxblox.launch*` (method name).
- The default `ros::init` node names in §3.3 (only used with bare `ros2 run`; launch files set the name explicitly).
- All ROS topic, service, and parameter names *relative to the node* (`~/mesh`, `~/save_map`, `tsdf_voxel_size`, …). Only the node-name prefix changes, via the launch node-name row above.
- Protobuf **message** names (`BlockProto`, `LayerProto`) and field numbers. With those unchanged, the binary map file format (`.vxblx`, `.tsdf`, …) stays wire-compatible, so maps saved by ROS 1 voxfield still load (protobuf wire format doesn't encode the package name).
- Upstream credits, citation, and prose describing voxblox history in README/docs.

Why the full rename and not just package names: both libraries could end up linked into one process (e.g. a node comparing voxblox and voxfield maps). Then identical C++ symbols in `namespace voxfield` are an ODR violation, identical protobuf full names (`voxblox.LayerProto`) abort at startup with "File already exists in database", and identical Ogre material names throw in RViz when both plugins load.

**D2. Distro/toolchain.** ROS 2 Jazzy, `ament_cmake` (plain, not `ament_cmake_auto`, so dependencies stay explicit), `CMAKE_CXX_STANDARD 17`, keep `-Wall -Wextra`. Don't add `-Werror`.

**D3. minkindr is vendored inside the `voxfield` package, not as its own package.** The core headers publicly use `kindr::minimal::QuatTransformationTemplate` (in `core/common.h`). A separate package named `minkindr` would collide with the Hector one (§2.1), so:
  - Copy the header-only minkindr headers into `voxfield/third_party/minkindr/include/kindr/…`, keeping its `LICENSE` (BSD) next to them.
  - Source preference: (1) the `minkindr` directory from `https://github.com/tu-darmstadt-ros-pkg/voxblox_ros2` (branch `ros2`), (2) upstream `https://github.com/ethz-asl/minkindr`, (3) if there's no network, the installed headers at `/opt/hector/jazzy/include/kindr/`. Record in the notes which source and commit you used.
  - Add that directory to the `voxfield` target's include dirs (BUILD_INTERFACE), and install the headers alongside voxfield's own under `include/voxfield/` (so `#include <kindr/minimal/…>` resolves through the exported `include/voxfield` dir).
  - Keep the `kindr::minimal` namespace unchanged. It's identical upstream code, so sharing it with another install is harmless.

**D4. Drop `minkindr_conversions`.** Only ~5 conversions are used (`transformKindrToMsg`, `transformMsgToKindr`, `transformKindrToTF`, `transformTFToKindr`, `xmlRpcToKindr`). Implement them in `voxfield_ros/include/voxfield_ros/kindr_conversions.h` against `geometry_msgs::msg::Transform` / `TransformStamped`. There is no separate TF type in ROS 2; use the msg type everywhere. Add a helper that builds a transform from a flat 16-element row-major `std::vector<double>`. It must renormalize the rotation via `RotationQuaternion::constructAndRenormalize`, matching the old `xmlRpcToKindr` behavior. Unit-test these conversions (Phase 5).

**D5. Node ownership model.** Server classes **hold** an `rclcpp::Node::SharedPtr` rather than inheriting from `rclcpp::Node`. This preserves the "use as a library inside another node" pattern.
  - Constructors: `explicit TsdfServer(rclcpp::Node::SharedPtr node)` and `TsdfServer(rclcpp::Node::SharedPtr node, const TsdfMap::Config&, const TsdfIntegratorBase::Config&, const MeshIntegratorConfig&)`. Same pattern for every server. The ROS 1 `(nh, nh_private)` pair collapses into one node:
    - ROS 1 `nh_.subscribe("pointcloud")` → `node_->create_subscription(... "pointcloud" ...)` (namespace-relative).
    - ROS 1 `nh_private_.advertise("mesh")` → `node_->create_publisher(... "~/mesh" ...)` (private).
    - ROS 1 `nh_private.param(...)` → node parameter (all ROS 2 params are node-local).
  - Mains: `rclcpp::init` → `auto node = std::make_shared<rclcpp::Node>("<default name>")` → construct server → `rclcpp::spin(node)` → `rclcpp::shutdown()`.

**D6. Parameters: tolerant declare-or-get helper (mandatory).** ROS 1 code reads the same parameter name from several places (e.g. `world_frame` in both `Transformer` and the server; `tsdf_voxel_size` in several `get*Config` functions), and the shipped YAML files mix ints and floats freely (e.g. `update_esdf_every_n_sec: 0` read as `double`). Plain `declare_parameter<T>` throws in both cases. Implement in `voxfield_ros/include/voxfield_ros/param_utils.h`:

```cpp
namespace voxfield {
// Mirrors ros::NodeHandle::param(name, value, default): declares once, returns
// the override if present, otherwise the default. Coerces int<->double.
template <typename T>
T getParam(rclcpp::Node& node, const std::string& name, const T& default_value);

// Same, in-place like ROS 1 nh.param(name, var, var).
template <typename T>
void param(rclcpp::Node& node, const std::string& name, T& value) { value = getParam(node, name, value); }

// Reads a 4x4 row-major transform stored as a flat 16-double array, plus an
// optional "invert_<name>" bool. Returns false (and leaves T unchanged) if absent.
bool getTransformationParam(rclcpp::Node& node, const std::string& name,
                            const std::string& invert_name, Transformation* T);
}
```
  Requirements:
  - Declare with `rcl_interfaces::msg::ParameterDescriptor{}.dynamic_typing = true` and the default value, guarded by `node.has_parameter(name)`.
  - Coercions: `PARAMETER_INTEGER` → `double`/`float`; `PARAMETER_DOUBLE` → `int` only when it's integral-valued (otherwise log an error and use the default); `PARAMETER_INTEGER_ARRAY` → `std::vector<double>`.
  - `float` and `FloatingPoint` targets go through `double`. `int`, `size_t`, and `unsigned` go through `int64_t`.
  - A parameter that's declared but unset (`PARAMETER_NOT_SET`) → return the default.
  - On a type error, log `RCLCPP_ERROR` naming the parameter and its expected and actual types, then fall back to the default. Never throw.
  - Rewrite every `get*ConfigFromRosParam(const ros::NodeHandle&)` in `ros_params.h` to take `rclcpp::Node&`, keeping names and defaults **identical**. Mechanical rule: `nh_private.param("x", v, v)` → `param(node, "x", v)`.
  - Unit-test the helper (Phase 5): int→double coercion, repeated reads, missing parameter → default, and the 16-array transform with and without inversion.

**D7. QoS mapping.**
  | ROS 1 | ROS 2 |
  |---|---|
  | `advertise<T>(name, 1, true)` (latched) | `rclcpp::QoS(1).transient_local().reliable()` |
  | `advertise<T>(name, N)` / `(name, N, false)` | `rclcpp::QoS(N)` (reliable, volatile) |
  | `subscribe(name, queue)` for `pointcloud` / `freespace_pointcloud` | `rclcpp::QoS(pointcloud_queue_size)` reliable by default. Add param `input_qos_best_effort` (default `false`) that switches to `rclcpp::SensorDataQoS().keep_last(pointcloud_queue_size)` for live sensors. |
  | `subscribe("transform", 40)` | `rclcpp::QoS(40)` |
  | `~tsdf_map_in`/`~esdf_map_in` depth 1 | `rclcpp::QoS(1)` |

**D8. Timers follow ROS time.** ROS 1 `nh.createTimer` uses ROS time, which is sim time during bag playback. Use `rclcpp::create_timer(node, node->get_clock(), rclcpp::Duration::from_seconds(p), cb)` (or `node->create_timer(...)` on Jazzy), **not** `create_wall_timer`. Keep the "negative period = every N frames" semantics exactly.

**D9. Executor/threading.** Use the default `SingleThreadedExecutor` (`rclcpp::spin`). The code assumes callbacks never run concurrently: timers mutate the map that subscriptions write to. Don't introduce multithreaded executors. `tf2_ros::TransformListener` may spin its own internal thread (default). That's fine because `tf2_ros::Buffer` is thread-safe.

**D10. TF.** `tf::TransformListener` → `std::shared_ptr<tf2_ros::Buffer>` (constructed with `node->get_clock()`) + `std::shared_ptr<tf2_ros::TransformListener>`. `tf::TransformBroadcaster` → `std::unique_ptr<tf2_ros::TransformBroadcaster>` (constructed from the node). Keep the lookup semantics: `canTransform(to, from, stamp)` with no wait, then `lookupTransform(to, from, stamp)`, catching `tf2::TransformException`.

**D11. Point cloud publishing.** ROS 1 published `pcl::PointCloud<T>` directly through `pcl_ros`. ROS 2 has no such adapter. Add to `conversions.h`:
```cpp
template <typename PointT>
void publishPclCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
                     const pcl::PointCloud<PointT>& cloud, const std::string& frame_id,
                     const rclcpp::Time& stamp);
```
It should call `pcl::toROSMsg` and set `header.frame_id` and `header.stamp`. All such publishers become `rclcpp::Publisher<sensor_msgs::msg::PointCloud2>`. Drop the `pcl_ros` dependency. Use `pcl_conversions` only. Always publish, even with zero subscribers: a latched topic must hold the latest value.

**D12. Stamps.** Header-only helpers that call `ros::Time::now()` (`mesh_vis.h: generateVoxbloxMeshMsg`, `fillMarkerWithMesh`, etc.) must take an explicit `const rclcpp::Time& stamp` argument, passed by callers from `node_->now()`. A free-standing `rclcpp::Clock` would ignore `use_sim_time`.

**D13. Services.** Callback signature: `void cb(const std::shared_ptr<Req> req, std::shared_ptr<Res> res)`. ROS 1 returned `bool` (false = call failed). ROS 2 has no failure channel, so log `RCLCPP_ERROR` on failure. **Keep `FilePath.srv` contents unchanged** (empty response), and record in the notes that adding `bool success` is a possible follow-up.

**D14. Messages.** Keep all 7 `.msg` and 1 `.srv` identical in content, except that same-package type references change from `voxfield_msgs/Block` to `voxfield_msgs/Block` (or plain `Block`). That includes `MultiMesh.msg` and `LayerWithTrajectory.msg`. Field names already satisfy ROS 2 naming rules. C++: `voxfield_msgs::msg::Layer`, header `voxfield_msgs/msg/layer.hpp`.

**D15. Include layout.** Follow the Jazzy convention and install headers to `include/${PROJECT_NAME}/`. For example, `voxfield` installs `include/voxfield/voxfield/core/...`, and targets export `$<INSTALL_INTERFACE:include/${PROJECT_NAME}>`.

**D16. gflags/glog in mains.** ROS 2 appends `--ros-args ...` to argv, and gflags aborts on unknown flags. Every `main()`:
```cpp
rclcpp::init(argc, argv);
std::vector<std::string> non_ros = rclcpp::remove_ros_arguments(argc, argv);
// build a char* argv from non_ros, then:
google::InitGoogleLogging(argv[0]);
google::ParseCommandLineFlags(&n, &nargv, /*remove_flags=*/true);
google::InstallFailureSignalHandler();
FLAGS_alsologtostderr = true;   // replaces `args="-alsologtostderr"` in old launch files
```
Put this in one shared helper, `voxfield_ros/include/voxfield_ros/node_main.h`, used by all mains.

**D17. Launch files: Python.** `.launch.py` everywhere. Provide one **generic** launch file plus thin per-dataset wrappers keeping the old names (details in Phase 10). Every node gets `use_sim_time` explicitly. In ROS 2 there's no global `<param name="use_sim_time">`.

**D18. Datasets are ROS 1 bags.** Convert them with `rosbags-convert` (`pip install rosbags`) into ROS 2 bags. Launch files play them with `ros2 bag play <bag> --clock -r <speed>`. Document this in the README.

---

## 5. Target repository layout

```
voxfield_ros2/
├── ROS2_PORT_PLAN.md                 (this file)
├── README.md                         (rewritten install/run for ROS 2)
├── docs/ROS2_PORT_NOTES.md           (new: decisions, rename log, interface table, known issues)
├── scripts/
│   ├── clean_env.sh                  (source only /opt/ros/jazzy + workspace)
│   ├── convert_ros1_params.py        (one-shot ROS1 YAML → ROS2 param YAML converter)
│   └── fake_sensor_publisher.py      (synthetic cloud + TF for smoke tests)
├── voxfield/                         (ament_cmake; core lib + proto + tests + 2 tools)
│   ├── include/voxfield/...
│   ├── proto/voxfield/{Block,Layer}.proto
│   └── third_party/minkindr/{LICENSE, include/kindr/...}
├── voxfield_msgs/                    (ament_cmake + rosidl)
├── voxfield_ros/
│   ├── include/voxfield_ros/{param_utils.h, kindr_conversions.h, node_main.h, ...}
│   ├── src/...
│   ├── launch/*.launch.py
│   ├── cfg/param/*.yaml, cfg/calib/*.yaml   (converted in place)
│   ├── cfg/rviz/*.rviz                      (RViz2 format)
│   └── test/test_smoke.launch.py            (launch_testing)
├── voxfield_rviz_plugin/             (rviz_common plugin)
└── .github/workflows/ros2.yml        (optional CI, Phase 11)
```
Delete `voxfield_https.rosinstall`, `voxfield_ssh.rosinstall`, and `rosdoc.yaml`. Update `.gitignore` for `build/ install/ log/`.

---

## 6. Phased implementation

### Phase 0: Workspace & environment
1. `git checkout -b ros2-port`.
2. Create workspace `~/voxfield_ws/src` and symlink the repo: `ln -s ~/src/voxfield_ros2 ~/voxfield_ws/src/voxfield_ros2`.
3. Write `scripts/clean_env.sh` per §2.1.
4. Check system deps: `rosdep` keys `eigen`, `libgoogle-glog-dev`, `libgflags-dev`, `protobuf-dev`, `libpcl-all-dev`, `libopencv-dev`, `cv_bridge`, `pcl_conversions`, `tf2_ros`, `tf2_eigen`, `tf2_geometry_msgs`, `interactive_markers`, `visualization_msgs`, `std_srvs`, `nav_msgs`, `rviz_common`, `rviz_rendering`, `rviz_default_plugins`, `pluginlib`, `ament_index_cpp`, `launch_testing_ament_cmake`. Most are already installed (§2).
5. `pip install --user rosbags` (for Phase 12). If pip is blocked by PEP 668, use `pipx` or a venv. Don't `--break-system-packages` without asking.

**Accept:** `colcon list` from the workspace shows the 4 packages (they won't build yet).

### Phase 1: Rename voxblox → voxfield (ROS 1 code, no porting yet)
Do the rename on the unported code as a pure mechanical change, following D1 exactly. No porting or behavior change in this phase.
1. `git mv` the four package dirs, `voxblox/include/voxblox` → `voxfield/include/voxfield`, `voxfield_ros/include/voxfield_ros` → `voxfield_ros/include/voxfield_ros`, `voxfield_rviz_plugin/include/voxfield_rviz_plugin` → `…/voxfield_rviz_plugin`, `proto/voxblox` → `proto/voxfield`, `voxblox.material` → `voxfield.material`, and the rviz plugin source files `voxblox_mesh_*.{h,cc}` / `voxblox_multi_mesh_display.*` → `voxfield_*`.
2. Rewrite identifiers with targeted, word-aware substitutions (e.g. `perl -pi -e` with `\b` anchors), **one rule per row of the D1 table**, not a blanket `s/voxblox/voxfield/`. Suggested order:
   - `#include [<"]voxfield_ros/` → `voxfield_ros/`; `voxfield_rviz_plugin/` → `voxfield_rviz_plugin/`; `voxfield_msgs/` → `voxfield_msgs/`; `#include [<"]voxblox/` → `voxfield/`.
   - `\bnamespace voxblox\b` → `namespace voxfield`; `\bvoxblox::` → `voxfield::`; the same for `voxfield_rviz_plugin` and `voxfield_msgs` tokens.
   - Include guards `\bVOXFIELD_` → `VOXFIELD_`.
   - Proto `package voxfield;` → `package voxfield;`.
   - Rviz plugin class/lookup names and Ogre material/resource-group names per D1.
   - `package.xml` `<name>` + `<depend>` entries and CMake `project()` names.
3. Afterwards, run `grep -rniE "voxblox" --exclude-dir=.git .` and classify **every** remaining hit as "keep (method name / credit / param name)" per D1's do-not-rename list, or fix it. Put the summary in `docs/ROS2_PORT_NOTES.md` ("Rename log").
4. Update `plugin_description.xml` and `.rviz` class references (`voxfield_rviz_plugin/VoxfieldMesh`). The old `.rviz` files will be regenerated in Phase 10 anyway.

**Accept:** `grep -rnE "namespace voxfield|voxfield::|VOXFIELD_|voxfield_msgs|voxfield_ros|voxfield_rviz_plugin|package voxfield;|#include [<\"]voxblox/" --exclude-dir=.git .` returns nothing outside docs/credits. Every remaining `voxblox` hit is on the keep-list. Commit as `Rename voxblox packages/namespace to voxfield`. (It can't be built yet; ROS 1 isn't available. Correctness is checked from Phase 3 on.)

### Phase 2: Vendor minkindr into `voxfield`
Per D3: copy the headers to `voxfield/third_party/minkindr/include/kindr/...` + `LICENSE`, and note the source and commit. If the minkindr headers include glog/gflags, voxfield already links those.

**Accept:** `voxfield/third_party/minkindr/include/kindr/minimal/quat-transformation.h` exists, and there is no `package.xml` in `third_party/` (it must not become a colcon package).

### Phase 3: `voxfield` core library
1. `package.xml` (format 3): `buildtool_depend ament_cmake`; `depend` on `eigen`, `libgoogle-glog-dev`, `libgflags-dev`, `protobuf-dev`; `test_depend ament_cmake_gtest`; `<export><build_type>ament_cmake</build_type></export>`.
2. `CMakeLists.txt`:
   - `find_package(ament_cmake Eigen3 Protobuf glog gflags REQUIRED)`. glog 0.6 ships a CMake config (`glog::glog`). For gflags, use the `gflags` target, or `${GFLAGS_LIBRARIES}` if no config is found.
   - Protobuf: `add_library(voxfield_proto SHARED)`, then `protobuf_generate(TARGET voxfield_proto PROTOS proto/voxfield/Block.proto proto/voxfield/Layer.proto IMPORT_DIRS proto PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto_gen)`. Headers must be includable as `"voxfield/Block.pb.h"`, so add `${CMAKE_CURRENT_BINARY_DIR}/proto_gen` as a BUILD_INTERFACE include dir. Install the generated `*.pb.h` to `include/voxfield/voxfield/`. Link `protobuf::libprotobuf` PUBLIC.
   - `add_library(voxfield SHARED <same 26 sources as today>)`, PUBLIC links to `voxfield_proto Eigen3::Eigen glog::glog gflags`. Include dirs (BUILD_INTERFACE): `include/`, `third_party/minkindr/include/`, proto_gen. INSTALL_INTERFACE: `include/${PROJECT_NAME}`. Install both `include/` and `third_party/minkindr/include/` into `include/${PROJECT_NAME}`.
   - Tools: `tsdf_to_esdf`, `test_load_esdf` → `install(TARGETS ... DESTINATION lib/${PROJECT_NAME})`.
   - `ament_export_targets(voxfieldTargets HAS_LIBRARY_TARGET)` and `ament_export_dependencies(Eigen3 Protobuf glog gflags)`.
3. Tests: `if(BUILD_TESTING)` → `ament_add_gtest` for the 10 tests (`test_approx_hash_array`, `test_tsdf_map`, `test_protobuf`, `test_tsdf_interpolator`, `test_layer`, `test_merge_integration`, `test_layer_utils`, `test_sdf_integrators`, `test_bucket_queue`, `test_clear_spheres`). Tests that need `test/test_data` should get `WORKING_DIRECTORY` set so relative paths resolve (check each test's file paths). Replace the old `add_custom_target(test_data)` hack with this.
4. `eigen_checks` replacement: two tests include `<eigen-checks/gtest.h>` / `<eigen-checks/entrypoint.h>`. Create a **test-only shim** at `voxfield/test/include/eigen-checks/{gtest.h,entrypoint.h}` implementing exactly the macros the tests use (grep for `EIGEN_MATRIX_*` and `UNITTEST_ENTRYPOINT`). `EIGEN_MATRIX_EQUAL(a,b)` returns `::testing::AssertionResult`, and so does `EIGEN_MATRIX_NEAR(a,b,tol)`. The entrypoint defines `main` with `InitGoogleTest` + glog init. Add that include dir only to test targets.
5. Fix compile errors caused by the newer toolchain (GCC 13, C++17, protobuf 3.21, Eigen 3.4). **Only minimal, behavior-neutral fixes.** Log each non-trivial one in the notes.
6. Map-file compatibility test: add a gtest that loads a small `.tsdf`/`.vxblx` layer file saved by the **original** code, if one is available in `test/test_data`, or writes a file using the renamed proto and checks the raw bytes round-trip. The goal is to show the rename didn't change the wire format.

**Accept:** `colcon build --packages-up-to voxfield` has zero errors, in both the clean env and the user's normal shell (§2.1). `colcon test --packages-select voxfield && colcon test-result --verbose` shows all tests passing. If a test fails, determine whether it's a port regression (fix it) or pre-existing (document with evidence in the notes; don't disable it silently).

### Phase 4: `voxfield_msgs`
- `package.xml`: `buildtool_depend ament_cmake rosidl_default_generators`; `depend std_msgs nav_msgs`; `exec_depend rosidl_default_runtime`; `<member_of_group>rosidl_interface_packages</member_of_group>`.
- `CMakeLists.txt`: `rosidl_generate_interfaces(${PROJECT_NAME} "msg/Block.msg" ... "srv/FilePath.srv" DEPENDENCIES std_msgs nav_msgs)` and `ament_export_dependencies(rosidl_default_runtime)`.
- Contents per D14.

**Accept:** builds, and `ros2 interface show voxfield_msgs/msg/Layer`, `.../MultiMesh`, and `voxfield_msgs/srv/FilePath` all print correctly.

### Phase 5: `voxfield_ros` infrastructure (no servers yet)
Do these first because every server depends on them:
1. `package.xml` + `CMakeLists.txt` skeleton. Deps: `rclcpp`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `std_srvs`, `nav_msgs`, `visualization_msgs`, `pcl_msgs`, `tf2`, `tf2_ros`, `tf2_eigen`, `tf2_geometry_msgs`, `pcl_conversions`, `cv_bridge`, `interactive_markers`, `voxfield`, `voxfield_msgs`, `ament_index_cpp`, system `PCL` (common, io, kdtree — `voxfield_server.cc` includes `pcl/kdtree/kdtree_flann.h`), `OpenCV` (core, imgproc), glog, gflags. `exec_depend voxfield_rviz_plugin`, `rviz2`, `ros2bag`. Build one shared library `voxfield_ros` (all non-main `.cc`) plus one executable per main. Install executables to `lib/${PROJECT_NAME}` and `launch/` + `cfg/` to `share/${PROJECT_NAME}`.
2. `param_utils.h` (D6) + gtest `test/test_param_utils.cc`.
3. `kindr_conversions.h` (D4) + gtest `test/test_kindr_conversions.cc` (round-trip msg↔kindr, 16-array → kindr with and without inversion, renormalization).
4. `ros_params.h`: port all 15 functions to `rclcpp::Node&` via `param()`. Diff-review that every name and default is unchanged.
5. `conversions.h` / `conversions_inl.h`: msg types → `voxfield_msgs::msg::` / `std_msgs::msg::`, add `publishPclCloud` (D11). Include `<pcl_conversions/pcl_conversions.h>`, **not** `pcl_ros/point_cloud.h`.
6. `mesh_vis.h`, `ptcloud_vis.h`, `intensity_vis.h`, `mesh_pcl.h`: msg types → `::msg::`, stamps as arguments (D12), `Marker::TRIANGLE_LIST` → `visualization_msgs::msg::Marker::TRIANGLE_LIST`, etc.
7. `transformer.h/.cc`: port per D10/D6.
   - Constructor `Transformer(rclcpp::Node::SharedPtr node)`.
   - Params: `world_frame`, `sensor_frame`, `timestamp_tolerance_sec`, `use_tf_transforms`, `T_B_D` + `invert_T_B_D`, `T_B_C` + `invert_T_B_C`, `T_C_CH` + `invert_T_C_CH` (the last one is read unconditionally), via `getTransformationParam`.
   - `transform` subscription only when `!use_tf_transforms`.
   - Queue logic: compare stamps as `rclcpp::Time` with **`RCL_ROS_TIME`** (see §8.1). `(a-b).toNSec()` → `(a-b).nanoseconds()`.
   - Keep interpolation code identical.
8. `interactive_slider.h/.cc`: `interactive_markers::InteractiveMarkerServer(name, node)` (ROS 2 constructor takes node interfaces). Feedback type `visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr`. Nothing in the servers currently instantiates it (verify). Port it so it compiles, but no runtime test is needed.
9. `node_main.h` (D16).

**Accept:** the `voxfield_ros` library target compiles with only the infra files, and `test_param_utils` + `test_kindr_conversions` pass.

### Phase 6: `TsdfServer` and `NpTsdfServer`
Port `tsdf_server.{h,cc}` and `np_tsdf_server.{h,cc}` together; they're parallel. Checklist per file:
- Members: `ros::NodeHandle nh_, nh_private_` → `rclcpp::Node::SharedPtr node_`. Publishers, subscriptions, services, and timers become `...::SharedPtr` (§7).
- `Transformer transformer_` must be constructed from `node_`. It stays a member initialized in the initializer list.
- Construction order: the ROS 1 constructors call `getServerConfigFromRosParam` and then create publishers. Keep that order.
- Callback signatures: `void insertPointcloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)`. It's **non-const** because the code mutates `fields[d].datatype` for the "rgb" hack. `std::queue<sensor_msgs::PointCloud2::Ptr>` → `std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>`.
- `min_time_between_msgs_` → `rclcpp::Duration`. `last_msg_time_ptcloud_` and `last_msg_time_freespace_ptcloud_` → `rclcpp::Time(0, 0, RCL_ROS_TIME)` (§8.1).
- `ros::WallTime` timing in `processPointCloudMessageAndInsert` → `std::chrono::steady_clock`.
- ICP block: the TF broadcasts use `geometry_msgs::msg::TransformStamped` built via `kindr_conversions.h`. Stamp from `pointcloud_msg->header.stamp`.
- `publishRobotMesh`: `Marker::MODIFY` doesn't exist in ROS 2; use `Marker::ADD` (same numeric value 0 in ROS 1). The stamp is zero `builtin_interfaces::msg::Time()` as before, and `lifetime` is zero `builtin_interfaces::msg::Duration()`. `mesh_resource = "file://" + robot_model_file_` stays unchanged. The ROS 1 code calls it unconditionally from `processPointCloudMessageAndInsert` regardless of `publish_robot_model_`. Keep that behavior and note it.
- `publishMap`: `getNumSubscribers()` → `get_subscription_count()`.
- Logging: `ROS_INFO` → `RCLCPP_INFO(node_->get_logger(), ...)`. `*_STREAM` variants exist. For `*_THROTTLE`, see §8.3. `ROS_INFO_ONCE` → `RCLCPP_INFO_ONCE`.
- Services per D13.
- Timers per D8. `publish_map_every_n_sec` / `update_mesh_every_n_sec` semantics unchanged.
- `getServerConfigFromRosParam(const ros::NodeHandle&)` → `getServerConfigFromRosParam()` using `node_`. Note that NpTsdfServer reads `sensor_is_lidar`, `width`, `height`, `fov_up/down`, `vx/vy/fx/fy`, `smooth_thre_ratio`, `min_z`, `min_dist`. `width_`, `height_`, `vx_`, and `fx_` are **uninitialized ints** in ROS 1 if they're not given. Initialize them to 0 in the header and add a startup check that logs an error if `width_ <= 0 || height_ <= 0` (behavior otherwise unchanged).
- Main files: `tsdf_server_node.cc` / `np_tsdf_server_node.cc` use `node_main.h`.

**Accept:** both executables build. `ros2 run voxfield_ros np_tsdf_server --ros-args -p width:=1024 -p height:=64 -p sensor_is_lidar:=true -p fov_up:=3.0 -p fov_down:=-25.0` starts without exceptions. `ros2 node info` shows the expected topics/services (§3.4) under `/voxfield/...`. `ros2 service call /voxfield/clear_map std_srvs/srv/Empty` returns.

### Phase 7: Derived servers & tools
Port in this order, reusing the Phase 6 patterns: `VoxfieldServer` (most important), `VoxbloxServer`, `FiestaServer`, `VoxedtServer`, `IntensityServer`, `SimulationServer`.
- `FiestaServer` and `VoxedtServer` are near-identical. Port one, then mirror the changes into the other.
- `VoxfieldServer::setupRos()` and the others create `eval_esdf_timer_` only if `eval_esdf_on`. Keep that.
- `IntensityServer` subscribes to `~intensity_image` (sensor_msgs/Image). Use `cv_bridge/cv_bridge.hpp` (the `.h` is deprecated in Jazzy).
- `SimulationServer` has no inputs. It generates GT and test maps and publishes them. It's the best **dataset-free runtime test**, so make sure `simulation_eval` runs to completion.
- `voxblox_eval.cc`: `T_V_G` via `getTransformationParam`. Required params `voxblox_file_path` and `gt_file_path` → `getParam<std::string>(..., "")` + `CHECK(!empty)`, keeping the old fail-fast behavior.
- `visualize_tsdf.cc`: `ros::shutdown()` on fatal → log + `rclcpp::shutdown(); return 1;`. `ros::spinOnce()` → `rclcpp::spin_some(node)`.

**Accept:** all 10 `voxfield_ros` executables build, in both the clean env and the normal shell. `ros2 run voxfield_ros simulation_eval` runs, publishes GT/test clouds, and logs evaluation results without crashing. Each ESDF server starts and advertises the §3.4 interface.

### Phase 8: `voxfield_rviz_plugin`
Port to `rviz_common` (Qt5, Ogre 1.12 via `rviz_ogre_vendor`):
- `rviz::MessageFilterDisplay<voxfield_msgs::Mesh>` → `rviz_common::MessageFilterDisplay<voxfield_msgs::msg::Mesh>`. `processMessage(voxfield_msgs::msg::Mesh::ConstSharedPtr msg)`.
- `VoxfieldMultiMeshDisplay` subscribes to `MultiMesh` and manages its own subscriber/property tree. Port it to `rviz_common::RosTopicDisplay<voxfield_msgs::msg::MultiMesh>` or `MessageFilterDisplay`, whichever matches its structure. Keep the per-namespace visibility tree.
- `#include <OGRE/OgreSceneNode.h>` → `#include <OgreSceneNode.h>` (and the same for other Ogre headers).
- `context_->getFrameManager()->getTransform(header, position, orientation)` has the same shape in `rviz_common::FrameManagerIface`. `ros::Time` → `rclcpp::Time`, and `header` is `std_msgs::msg::Header`.
- `material_loader.cc`: `ros::package::getPath(...)` → `ament_index_cpp::get_package_share_directory("voxfield_rviz_plugin")`. The resource group and material names are the renamed ones from D1. Install `content/materials/voxfield.material` and `icons/` to `share/${PROJECT_NAME}`. Rename the icons to `VoxfieldMesh.png` / `VoxfieldMultiMesh.png`, since rviz looks them up by class name.
- `plugin_description.xml`: `<library path="voxfield_rviz_plugin">`, classes `voxfield_rviz_plugin/VoxfieldMesh` and `voxfield_rviz_plugin/VoxfieldMultiMesh`, `base_class_type="rviz_common::Display"`, and `message_type` `voxfield_msgs/msg/Mesh` / `voxfield_msgs/msg/MultiMesh`. In CMake: `pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)`. Also `PLUGINLIB_EXPORT_CLASS(voxfield_rviz_plugin::VoxfieldMeshDisplay, rviz_common::Display)`.
- CMake: `set(CMAKE_AUTOMOC ON)`, `find_package(Qt5 REQUIRED COMPONENTS Widgets)`, add headers with `Q_OBJECT` to the sources so moc sees them, and link `rviz_common::rviz_common`, `rviz_rendering::rviz_rendering`, `rviz_ogre_vendor::OgreMain`, `pluginlib::pluginlib`, and `Qt5::Widgets`. Keep `-DQT_NO_KEYWORDS`.
- The Hector reference (`/opt/hector/jazzy/lib/libvoxfield_rviz_plugin.so`, built from the same upstream sources) can be used as a sanity check for the mesh display approach.

**Accept:** plugin builds. `rviz2` lists "VoxfieldMesh" and "VoxfieldMultiMesh" under Add → By display type. Adding VoxfieldMesh on a server's `~/mesh` topic renders the simulation or smoke-test mesh. In the user's normal shell, where Hector's voxblox plugin is also discoverable, both plugins load in the same RViz2 session without errors (checks the Ogre material rename).

### Phase 9: Configuration files
The repo ships ROS 1 parameter files for the example datasets. Convert them; don't create new ones.
1. Write `scripts/convert_ros1_params.py` (PyYAML). It must:
   - Load with a full YAML loader so anchors and aliases resolve, then dump **without** anchors. Don't rely on `rcl_yaml_param_parser` supporting aliases.
   - Flatten 4×4 list-of-lists (`T_B_C`, `T_B_D`, `T_C_CH`, `T_V_G`, and any other `T_*`) into 16-element float lists (row-major). Force every element to float (`1` → `1.0`) so ROS 2 sees `double_array`.
   - Wrap as `/**:\n  ros__parameters:\n    ...`. The `/**` wildcard makes the file apply regardless of node name/namespace.
   - Preserve key order and comments where practical. Comment preservation is nice-to-have; if you use `ruamel.yaml` you get it.
2. Convert in place: `cfg/param/*.yaml`, `cfg/calib/*.yaml`, `cfg/kitti_lidar.yaml`, `cfg/rgbd_dataset.yaml`. `cfg/stereo/*` is only for the non-ported stereo pipeline; convert it anyway for completeness, but it isn't used.
3. Spot-check that `ros2 param dump` on a running node shows the expected values (for example `tsdf_voxel_size: 0.25` from `kitti_param.yaml`, `T_C_CH` as 16 doubles).

**Accept:** every YAML loads via `ros2 run voxfield_ros voxfield_server --ros-args --params-file <file>` without parse errors, and the converted values match the ROS 1 originals (write a tiny check in the script: compare the flattened original vs the converted output).

### Phase 10: Launch files & RViz configs
1. **Generic launch** `launch/mapping.launch.py`. Arguments:
   - `method` ∈ {`voxfield`, `voxblox`, `fiesta`, `voxedt`, `np_tsdf`, `tsdf`} → executable `<method>_server`.
   - `dataset` ∈ {`cow`, `kitti`, `mai`, `basement`, `vicon`} → loads `cfg/param/<dataset>_param.yaml` + `cfg/calib/<dataset>_calib.yaml` in the same order as the old launch file.
   - `bag_file` (default empty → don't play), `play_bag` (true), `speed`.
   - `rviz` (true), `rviz_config` (default `cfg/rviz/<per-dataset>.rviz`).
   - `robot_model_file` (default per dataset, from the old launch files: `car.dae` for KITTI, `camera.dae` for cow, etc.).
   - `pointcloud_topic` and `transform_topic` (per-dataset defaults taken from the old `<remap>` lines, e.g. KITTI → `/velodyne_points`, cow → `/camera/depth_registered/points` + transform `/kinect/vrpn_client/estimated_transform`).
   - `use_sim_time` (true).

   Node: `Node(package='voxfield_ros', executable=..., name='voxfield_node', output='screen', parameters=[param_yaml, calib_yaml, {'robot_model_file': ..., 'use_sim_time': ...}], remappings=[('pointcloud', ...), ('transform', ...)])`. glog-to-stderr comes from D16.

   Bag: `ExecuteProcess(cmd=['ros2','bag','play',bag,'--clock','-r',speed], condition=...)`. RViz: `Node(package='rviz2', executable='rviz2', arguments=['-d', cfg], parameters=[{'use_sim_time': True}])`.
2. **Per-file wrappers** with the old names (`kitti_voxfield.launch.py`, `cow_voxblox.launch.py`, `mai_fiesta.launch.py`, …, and `eval_*.launch.py`). Each should `IncludeLaunchDescription(mapping.launch.py)` with fixed `method`/`dataset`. Read every old `.launch` and carry over **any per-file deviation** (extra params, different remaps, `process_every_nth_frame`, etc.). Port `voxblox_launch/`, `voxfield_launch/`, `fiesta_launch/`, and `voxedt_launch/`. Port `eval/*.launch` → `voxblox_eval` launches. Skip `bak/` and note that in the notes.
3. Replace hard-coded bag paths (`/media/yuepan/...`) with a required `bag_file` argument or an empty default + warning.
4. Delete the old `.launch` XML files once their `.py` equivalents exist.
5. **RViz2 configs:** recreate the 8 `.rviz` files in RViz2 format. Easiest: launch rviz2, add displays, save. Or hand-write YAML using existing RViz2 configs as templates.
   - Map old display classes: `rviz/PointCloud`/`PointCloud2` → `rviz_default_plugins/PointCloud2`, `rviz/MarkerArray` → `rviz_default_plugins/MarkerArray`, `rviz/Marker` → `rviz_default_plugins/Marker`, `rviz/Grid` → `rviz_default_plugins/Grid`, `rviz/Odometry` → `rviz_default_plugins/Odometry`, `rviz/Pose` → `rviz_default_plugins/Pose`, `rviz/InteractiveMarkers` → `rviz_default_plugins/InteractiveMarkers`, `voxfield_rviz_plugin/VoxfieldMesh` → `voxfield_rviz_plugin/VoxfieldMesh`.
   - Topics: `/voxfield_node/mesh`, `/voxfield_node/tsdf_slice`, etc. (the old files used `/voxblox_node/...`; see D1 node-name row).
   - For latched topics, set Durability Policy = Transient Local on those displays.
   - Keep the fixed frame (`world`), views, and color settings from the old files where practical.

**Accept:** `ros2 launch voxfield_ros mapping.launch.py method:=voxfield dataset:=kitti play_bag:=false` starts the node and RViz2 with no errors. `ros2 launch voxfield_ros kitti_voxfield.launch.py bag_file:=<converted bag>` runs end to end (Phase 12).

### Phase 11: Docs, cleanup, CI
- README: replace the catkin/wstool/`roslaunch` instructions with: prerequisites (Jazzy), clone into `~/voxfield_ws/src`, `rosdep install --from-paths src -y --ignore-src`, `colcon build`, bag conversion (`rosbags-convert --src data.bag --dst data_ros2`), and `ros2 launch voxfield_ros kitti_voxfield.launch.py bag_file:=...`.
  - Replace the paragraph "we keep the name of our package as voxblox" with a short explanation of the rename and what downstream voxblox-based projects must change (includes `voxblox/` → `voxfield/`, namespace, message package).
  - Keep citation and acknowledgements untouched.
- `docs/ROS2_PORT_NOTES.md`: decisions taken, the rename log, the final interface table (§3.4 re-verified), parameter-file format notes, and known upstream issues (§8.14).
- Remove `.rosinstall` files and `rosdoc.yaml`. Fix `.gitignore`.
- Optional CI `.github/workflows/ros2.yml` using `ros-tooling/setup-ros` + `ros-tooling/action-ros-ci` on `ubuntu-24.04` with `required-ros-distributions: jazzy`, building and testing all packages.
- Run `clang-format` (the repo has `.clang-format`) on changed C++ files **only**. Don't reformat untouched files; it inflates diffs.

### Phase 12: End-to-end validation (Definition of Done gate)
1. **Automated smoke test** (`voxfield_ros/test/test_smoke.launch.py`, registered via `add_launch_test`):
   - Start `scripts/fake_sensor_publisher.py`. It publishes a synthetic `sensor_msgs/PointCloud2` of a closed box room (e.g. 10×10×3 m, points on walls/floor, 64×1024 lidar-like sampling), plus a moving `world → velodyne` TF at 10 Hz.
   - Start `voxfield_server` with `kitti_param.yaml` + `kitti_calib.yaml` (with `update_esdf_every_n_sec: 1.0` overridden so ESDF runs) and `use_sim_time:=false`.
   - Assert within 30 s: `~/mesh` receives a message with ≥1 non-empty `mesh_blocks`; `~/tsdf_slice` or `~/esdf_slice` receives a non-empty cloud; `~/save_map` with a temp path creates a non-empty file; `~/load_map` on that file succeeds (no error log).
   - Repeat quickly for `np_tsdf_server`, `voxblox_server`, `fiesta_server`, and `voxedt_server` (mesh only).
2. **Unit tests:** `colcon test && colcon test-result --verbose` → 0 failures across all packages.
3. **Coexistence check:** in the user's normal shell (Hector underlay sourced), do a full build, run the smoke test, and open RViz2 with both the voxfield and voxblox mesh plugins available.
4. **Dataset run (manual, needs the user's data):** convert at least one dataset (KITTI seq 07 bag or MaiCity) with `rosbags-convert`, then run `kitti_voxfield.launch.py` / `mai_voxfield.launch.py`. Checks: the mesh builds progressively in RViz2, there are no TF extrapolation spam errors, `timing: true` output appears, and `~/generate_mesh` with `mesh_filename` set writes a PLY. If ROS 1 reference outputs exist, compare the PLY vertex counts or visually. If no dataset is available locally, stop here, report that, and tell the user exactly which commands to run.
5. **TF-queue mode:** exercise `use_tf_transforms: false` + `transform` topic (cow & lady config) with the fake publisher emitting `geometry_msgs/TransformStamped` instead of TF, so the queue and interpolation path runs.

---

## 7. ROS 1 → ROS 2 API cheat sheet

| ROS 1 | ROS 2 (rclcpp, Jazzy) |
|---|---|
| `#include <ros/ros.h>` | `#include <rclcpp/rclcpp.hpp>` |
| `sensor_msgs::PointCloud2::Ptr` | `sensor_msgs::msg::PointCloud2::SharedPtr` |
| `#include <sensor_msgs/PointCloud2.h>` | `#include <sensor_msgs/msg/point_cloud2.hpp>` |
| `#include <voxfield_msgs/FilePath.h>` | `#include <voxfield_msgs/srv/file_path.hpp>` |
| `#include <voxfield_msgs/Mesh.h>` | `#include <voxfield_msgs/msg/mesh.hpp>` |
| `#include <std_srvs/Empty.h>` | `#include <std_srvs/srv/empty.hpp>` |
| `nh.advertise<T>("x", n, latch)` | `node->create_publisher<T>("x" or "~/x", qos)` (D7) |
| `nh.subscribe("x", n, &C::cb, this)` | `node->create_subscription<T>("x", qos, std::bind(&C::cb, this, std::placeholders::_1))` |
| `nh.advertiseService("x", &C::cb, this)` | `node->create_service<S>("~/x", std::bind(&C::cb, this, _1, _2))` |
| `nh.createTimer(ros::Duration(s), &C::cb, this)` | `rclcpp::create_timer(node, node->get_clock(), rclcpp::Duration::from_seconds(s), std::bind(&C::cb, this))` — callback takes **no** args |
| `void cb(const ros::TimerEvent&)` | `void cb()` |
| `pub.publish(msg)` | `pub->publish(msg)` |
| `pub.getNumSubscribers()` | `pub->get_subscription_count()` |
| `nh.param("x", v, v)` | `voxfield::param(*node, "x", v)` (D6) |
| `nh.getParam("T", xmlrpc)` + `xmlRpcToKindr` | `voxfield::getTransformationParam(*node, "T", "invert_T", &T)` |
| `ros::Time::now()` | `node->now()` |
| `ros::Time` from header | `rclcpp::Time(msg->header.stamp, RCL_ROS_TIME)` |
| `ros::Duration(s)` / `.toSec()` / `.toNSec()` | `rclcpp::Duration::from_seconds(s)` / `.seconds()` / `.nanoseconds()` |
| `ros::WallTime::now()` | `std::chrono::steady_clock::now()` |
| `ROS_INFO(...)` | `RCLCPP_INFO(node_->get_logger(), ...)` |
| `ROS_INFO_STREAM(x)` | `RCLCPP_INFO_STREAM(node_->get_logger(), x)` |
| `ROS_ERROR_THROTTLE(60, ...)` | `RCLCPP_ERROR_THROTTLE(logger, *node_->get_clock(), 60000, ...)` — **milliseconds** |
| `tf::TransformListener` | `tf2_ros::Buffer` + `tf2_ros::TransformListener` |
| `tf::TransformBroadcaster` / `tf::StampedTransform` | `tf2_ros::TransformBroadcaster` / `geometry_msgs::msg::TransformStamped` |
| `tf::TransformException` | `tf2::TransformException` |
| `visualization_msgs::Marker::MODIFY` | `visualization_msgs::msg::Marker::ADD` |
| `ros::package::getPath(p)` | `ament_index_cpp::get_package_share_directory(p)` |
| `pcl_ros` direct publish of `pcl::PointCloud` | `pcl::toROSMsg` → `PointCloud2` (D11) |
| `ros::spin()` / `ros::spinOnce()` | `rclcpp::spin(node)` / `rclcpp::spin_some(node)` |
| `ros::shutdown()` | `rclcpp::shutdown()` |
| `$(find voxfield_ros)` | `FindPackageShare('voxfield_ros')` / `get_package_share_directory('voxfield_ros')` |
| `<rosparam file=...>` | `parameters=[path]` in `Node(...)` |
| `<remap from= to=>` | `remappings=[(from, to)]` |
| `<param name="use_sim_time" value="true"/>` (global) | `{'use_sim_time': True}` **on every node** |
| `rosbag play --clock -r s f` | `ros2 bag play f --clock -r s` |

---

## 8. Pitfalls (read before coding)

1. **Mixed clock types throw.** `rclcpp::Time()` defaults to `RCL_SYSTEM_TIME`, while `rclcpp::Time(msg.header.stamp)` is `RCL_ROS_TIME`, and subtracting or comparing them throws `std::runtime_error("can't subtract times with different time sources")`. Initialize every stored time (`last_msg_time_*`, queue stamps) with `RCL_ROS_TIME`. This directly affects `insertPointcloud`'s throttle check and `Transformer::lookupTransformQueue`.
2. **Parameter redeclaration / type mismatch throws** (`ParameterAlreadyDeclaredException`, `InvalidParameterTypeException`). That's why D6 is mandatory. Don't call raw `declare_parameter` anywhere in server code.
3. **Throttle units:** ROS 1 throttle periods are seconds; `RCLCPP_*_THROTTLE` takes **milliseconds** and needs a clock reference.
4. **YAML anchors (`&voxel_size` / `*voxel_size`)** are used in every shipped `cfg/param/*.yaml`. Expand them (Phase 9). Nested lists (4×4 matrices) aren't valid ROS 2 parameter types, so flatten them.
5. **Integer-looking YAML values** (`update_esdf_every_n_sec: 0`, `fx: 580`) come in as `PARAMETER_INTEGER`. D6 must coerce. In converted files, prefer writing floats for anything read as double.
6. **Every node needs `use_sim_time`.** Otherwise TF lookups at bag timestamps fail forever, and pointclouds pile up in the queue ("Input pointcloud queue getting too long!").
7. **`/tf_static` from bags:** `ros2 bag play` may publish `/tf_static` as volatile, so late-joining listeners miss it. If static transforms are missing during dataset replay, pass `--qos-profile-overrides-path` with `/tf_static: {durability: transient_local, history: keep_all}`. Document this in the README.
8. **Big PointCloud2 + best-effort QoS** can silently drop messages (fragment loss). That's why reliable is the default in D7.
9. **gflags vs `--ros-args`:** see D16. Without the fix, every node exits at startup with "unknown command line flag".
10. **Header-only helpers calling `now()`:** see D12. A local `rclcpp::Clock` ignores sim time, and then mesh stamps and bag time disagree.
11. **Eigen alignment:** the classes use `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`. Keep it. When holding servers in `std::make_shared`/`std::make_unique`, C++17 aligned new handles it, but don't remove the macros.
12. **Rename slips.** A blanket `sed s/voxblox/voxfield/` would break method names (`voxblox_server`), param names (`voxblox_file_path`), and upstream credits. Use the D1 rules and the Phase 1 grep audit. Conversely, a *missed* rename (e.g. a leftover `${1}voxfield/...>`) may still compile in the user's normal shell, because Hector's headers satisfy it. That's why Phase 1 has a grep gate and the build is checked in the clean env too.
13. **`kDefaultMaxIntensity`** is defined in both `tsdf_server.h` and `np_tsdf_server.h` in the same namespace. Any translation unit including both fails to compile. Don't include both. If you must, move it to one shared header (behavior-neutral).
14. **Known upstream issues. Record them; don't fix them in the port commits:**
    - `NpTsdfServer::computeNormalImage`: the condition `if (v == height_)` can never be true inside `for (v = 0; v < height_; ...)`, so for `v = height_-1` it reads row `height_` of `vertex_map`/`depth_image`, an out-of-bounds read. The intended condition is probably `v == height_ - 1`.
    - `NpTsdfServer::projectPointToImageCamera` returns `bool`, but its result is assigned to `float depth`. Depth for cameras is therefore always `0.0` or `1.0`: the `depth > min_d` filter passes every in-image point, and the "keep nearest point per pixel" logic keeps the *first* point instead. Verify against RGB-D (cow) output and record it.
    - `width_`, `height_`, `vx_`, `vy_`, `fx_`, and `fy_` are uninitialized if the parameters are missing (Phase 6 initializes them to 0 and adds a check).
    - The camera intrinsics `fx_`/`fy_`/`vx_`/`vy_` are `int`, which truncates non-integer calibrations. Keep this; note it.

---

## 9. Definition of Done

- [ ] No package, C++ namespace, include root, protobuf package, message package, plugin name, or Ogre material in this repo uses the `voxblox` name, except the D1 keep-list. The grep audit is recorded in the notes.
- [ ] `colcon build` of all packages: 0 errors, both in the clean env and in the user's normal shell with the Hector underlay sourced.
- [ ] `colcon test`: 0 failures (any pre-existing failure documented with evidence in `docs/ROS2_PORT_NOTES.md`).
- [ ] Map files written by the original voxfield/voxblox format still load (wire-format check, Phase 3 step 6).
- [ ] All 10 `voxfield_ros` executables + 2 `voxfield` tools are installed and start.
- [ ] Topic, service, and parameter names match §3.4 (re-verified table in the notes).
- [ ] `test_smoke.launch.py` passes for voxfield, np_tsdf, voxblox, fiesta, and voxedt servers.
- [ ] `simulation_eval` runs to completion.
- [ ] RViz2 displays the mesh via `voxfield_rviz_plugin/VoxfieldMesh`. MultiMesh display loads. Voxfield and voxblox plugins load together without conflicts.
- [ ] Every old `.launch` (excluding `bak/`) has a `.launch.py` equivalent, and every old `.rviz` has an RViz2 equivalent.
- [ ] All shipped YAML converted. No anchors, no nested lists.
- [ ] README updated for ROS 2 and the rename. `docs/ROS2_PORT_NOTES.md` written. `.rosinstall` files removed.
- [ ] No references to `ros/ros.h`, `catkin`, `tf/`, `pcl_ros/point_cloud.h`, `XmlRpc`, or `ros::` remain.
- [ ] Dataset validation done, **or** explicitly reported as pending with exact commands for the user.
- [ ] Branch `ros2-port` with one commit per phase (rename commit separate). Nothing pushed without the user's approval.

---

## 10. Open decisions for the user (proceed with the default unless told otherwise)

1. ~~Package names~~ **Decided:** full rename to `voxfield*` (D1), so voxfield and voxblox can be used independently and side by side.
2. **`FilePath.srv` response.** Default: unchanged (empty). Alternative: add `bool success` + `string message`.
3. **Fixing the known upstream issues (§8.14).** Default: document only, and fix in a separate follow-up PR after the port is validated.
4. **Components.** Default: plain executables. Stretch: register each server as an `rclcpp_components` component. This needs a `NodeOptions` constructor wrapper; do it only after the Definition of Done is met.
5. ~~Launch node name~~ **Decided:** `voxblox_node` → `voxfield_node` in all launch files, so topics are `/voxfield_node/...` (D1).
