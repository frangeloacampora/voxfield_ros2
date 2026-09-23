# ROS 2 Port Notes

Decisions, the rename log, the interface table, and known upstream issues
accumulate here as the port (see `ROS2_PORT_PLAN.md`) proceeds. Sections are
added phase by phase.

## Phase 1: Rename log (voxblox → voxfield)

Mechanical, behavior-neutral rename per plan §4 D1. Method: targeted
word-aware regex substitutions (Perl, `\b`-anchored so `voxblox_server`,
`voxblox_eval`, `voxblox_file_path`, etc. never matched the bare `voxblox`
token), followed by a full-repo grep audit
(`grep -rniE "voxblox" --exclude-dir=.git .`) with every hit classified below.
No `.launch`/porting/build-behavior changes in this phase — pure renaming of
the ROS 1 tree.

### Structural renames applied
- Dirs: `voxblox/` → `voxfield/`, `voxblox_msgs/` → `voxfield_msgs/`,
  `voxblox_ros/` → `voxfield_ros/`, `voxblox_rviz_plugin/` → `voxfield_rviz_plugin/`.
- Include roots: `*/include/voxblox` → `*/include/voxfield`,
  `voxblox_ros/include/voxblox_ros` → `voxfield_ros/include/voxfield_ros`,
  `voxblox_rviz_plugin/include/voxblox_rviz_plugin` → `.../voxfield_rviz_plugin`.
- Proto dir: `voxfield/proto/voxblox/` → `voxfield/proto/voxfield/`;
  `package voxblox;` → `package voxfield;` in both `.proto` files. Message
  names (`BlockProto`, `LayerProto`) and field numbers unchanged (wire format
  preserved, to be verified with a round-trip test in Phase 3).
- Material: `voxblox.material` → `voxfield.material`; Ogre material/resource
  names `VoxbloxMaterial(Transparent)`/`VoxbloxMaterials` → `Voxfield...`.
- RViz plugin source files: `voxblox_mesh_display.{h,cc}`,
  `voxblox_mesh_visual.{h,cc}`, `voxblox_multi_mesh_display.{h,cc}` →
  `voxfield_*`. Classes `VoxbloxMeshDisplay`/`VoxbloxMeshVisual`/
  `VoxbloxMultiMeshDisplay` → `Voxfield*`. Plugin lookup names
  `voxblox_rviz_plugin/VoxbloxMesh(MultiMesh)` → `voxfield_rviz_plugin/Voxfield...`.
  `plugin_description.xml`'s `<library path="lib/libvoxblox_rviz_plugin">` →
  `libvoxfield_rviz_plugin`.
- Include guards: every `VOXBLOX_*` → `VOXFIELD_*` (plain substring
  replace, not `\b`-anchored, because guards concatenate, e.g.
  `VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_VISUAL_H_` has two occurrences to fix).
- `package.xml`: `<name>` and `<depend>` entries for all 4 packages retargeted
  (`voxblox`→`voxfield`, `voxblox_msgs`→`voxfield_msgs`,
  `voxblox_rviz_plugin`→`voxfield_rviz_plugin`). Package `<description>`
  fields that said "Voxblox ROS interface" / "...voxblox mesh messages..."
  reworded to "Voxfield..." (not itself in the D1 table, but these describe
  the packages being renamed, not the upstream method, so they don't belong
  on the do-not-rename list either).
- CMake: `project(voxblox)` → `project(voxfield)` (voxfield_msgs/_ros/_rviz_plugin
  CMakeLists already used `voxblox_msgs`/`voxblox_ros`/`voxblox_rviz_plugin`
  literal tokens that the word-aware pass caught). Target names derive from
  `${PROJECT_NAME}`, so they follow automatically.
- Misc comments/strings that describe our own renamed types (not the
  upstream method): `esdf_integrator.cc` timing label keys
  (`upate_esdf/voxblox/...` → `.../voxfield/...`, typo `upate` left as-is —
  behavior/string-key preservation isn't required for internal profiling
  labels, but the rename should still be consistent), `conversions.h`
  "convert... to a voxblox pointcloud" comments, one comment in
  `voxfield_mesh_visual.cc`, `MultiMesh.msg`'s doc comment, and the
  transient round-trip test fixture names in `test_clear_spheres.cc`,
  `test_sdf_integrators.cc`, `test_protobuf.cc` (`*.voxblox` → `*.voxfield`;
  these are write-then-read-back temp files within a single test run, not
  checked-in fixtures — confirmed no `test_data/*.voxblox` files exist).

### Remaining `voxblox` hits — classified (grep audit, case-insensitive)
All kept as-is; each falls on the D1 do-not-rename list or is explicitly
scheduled for a later phase:

- **Method name (keep, D1):** `VoxbloxServer` class + `voxblox_server.{h,cc}`
  + executable `voxblox_server` (`voxfield_ros/include/voxfield_ros/voxblox_server.h`,
  `voxfield_ros/src/voxblox_server*.cc`, `voxfield_ros/CMakeLists.txt`);
  `voxblox_eval` executable + `VoxbloxEvaluator` class + its params
  `voxblox_file_path`/`voxblox_esdf_file_path`/`voxblox_occ_file_path`
  (`voxfield_ros/src/voxblox_eval.cc`, `voxfield_ros/CMakeLists.txt`, and the
  `eval_*.launch` files' `<arg>`/`<param>` entries); free functions
  `generateVoxbloxMeshMsg`, `colorVoxbloxToMsg`, `colorMsgToVoxblox`,
  `recolorVoxbloxMeshMsgByIntensity` (`mesh_vis.h`, `conversions.h`,
  `ptcloud_vis.h`, `intensity_vis.h`, `intensity_server.cc`, `tsdf_server.cc`,
  `np_tsdf_server.cc`, `visualize_tsdf.cc`); launch dir `voxblox_launch/` and
  files `*_voxblox.launch*`; comparisons of the Voxblox *method* to
  Voxfield/FIESTA/EDT in algorithm-description comments
  (`voxel.h`, `esdf_voxfield_integrator.cc`, `esdf_occ_fiesta_integrator.cc`,
  `esdf_occ_edt_integrator.cc`, `approx_hash_array.h`).
- **Default `ros::init` node names (keep, §3.3 — only used with bare
  `ros2 run`; launch files set the name explicitly):** `"voxblox"` in
  `tsdf_server_node.cc`, `voxblox_server_node.cc`, `intensity_server_node.cc`;
  `"voxblox_sim"` in `simulation_eval.cc`; `"voxblox_node"` in
  `voxblox_eval.cc`.
- **Citation / credit (keep):** `esdf_integrator.h`'s link to the Voxblox
  paper; `LICENSE`'s BSD clause naming voxblox (legal text, must stay
  verbatim).
- **Deferred to Phase 10 (launch files & RViz configs are rewritten/regenerated
  there, not mechanically renamed here):** every remaining `name="voxblox_node"`
  node name and `<!-- Run voxblox_node-->` comment in the per-dataset
  `.launch` files (`voxblox_launch/`, `voxfield_launch/`, `fiesta_launch/`,
  `voxedt_launch/`, `bak/`); the hardcoded example data paths in
  `eval_*.launch` (e.g. `/Users/helen/data/.../voxblox/...`); every
  `/voxblox_node/...`-prefixed `Topic:`/`Marker Topic:` and the stale
  `/VoxbloxMesh1` expanded-tree entry in the old `.rviz` files (these 8 files
  get fully regenerated in RViz2 format per plan §10.5); the
  `mav_planning_rviz` panel's `planner_name: voxblox_rrt_planner` field
  (that whole panel is dropped per plan §1.3, non-goal).
- **Deferred to Phase 11 (README/docs rewrite):** `README.md` prose;
  `docs/pages/*.rst`, `docs/index.rst`, `docs/conf.py` (legacy Sphinx/rosdoc
  source — `rosdoc.yaml` itself is deleted in Phase 11 per plan §5, and this
  content is superseded by the ROS 2 README rewrite).
- **This plan document:** `ROS2_PORT_PLAN.md` intentionally documents the
  rename using the old names throughout; not touched.

## Phase 2: minkindr vendoring

Per plan §4 D3, vendored (not a separate colcon package, to avoid colliding
with the Hector-provided `minkindr` package, §2.1):
- Source: `https://github.com/tu-darmstadt-ros-pkg/voxblox_ros2`, branch
  `ros2`, commit `9ed1e1a92e882089e9835cd74771a43e6cd4d35e` (2026-06-28),
  `minkindr/` subdirectory — plan's preference (1). Copied `include/` and
  `LICENSE` verbatim (BSD, ETH Zurich ASL, 2015) into
  `voxfield/third_party/minkindr/`; did not copy its `CMakeLists.txt` or
  `package.xml`, so `third_party/` cannot be picked up as its own colcon
  package. `kindr::minimal` namespace left unchanged.

### Map-file wire-compatibility
Protobuf message names (`BlockProto`, `LayerProto`) and field numbers are
unchanged, so `.vxblx`/`.tsdf` files written by ROS 1 voxfield/voxblox should
still load post-rename (protobuf wire format doesn't encode the `package`
name). A concrete round-trip verification test is added in Phase 3 step 6
per the plan.

## Phase 3: `voxfield` core library ported to `ament_cmake`

`package.xml` bumped to format 3 (`ament_cmake` buildtool; `depend` on
`eigen`, `libgoogle-glog-dev`, `libgflags-dev`, `protobuf-dev`; `test_depend
ament_cmake_gtest`). `CMakeLists.txt` rewritten for `ament_cmake` per plan
§6 Phase 3, with two deviations from the plan's shorthand, both because the
literal instructions don't work as written against this toolchain:

- **Protobuf generation.** `protobuf_generate(TARGET ... IMPORT_DIRS proto
  PROTOC_OUT_DIR ...)` (the plan's suggested call) always prepends
  `-I ${CMAKE_CURRENT_SOURCE_DIR}` (i.e. `voxfield/`) to protoc's include
  path *before* `IMPORT_DIRS`, in FindProtobuf.cmake as shipped with CMake
  3.28 -- and since that's also a valid prefix of
  `proto/voxfield/Block.proto`, protoc picks it first and names the file
  `proto.voxfield.Block`, generating `proto/voxfield/Block.pb.h`, not
  `voxfield/Block.pb.h` as the plan calls for. `CMakeLists.txt` instead
  invokes `protoc` directly via `add_custom_command`, passing only
  `-I ${CMAKE_CURRENT_SOURCE_DIR}/proto`, so the generated header is really
  `voxfield/Block.pb.h` and `#include "voxfield/Block.pb.h"` resolves as
  intended. (Needed `file(MAKE_DIRECTORY ...)` for the output dir up front
  too -- protoc doesn't create it.)
- **Build type default.** Plain `ament_cmake` (unlike a typical catkin
  profile) leaves `CMAKE_BUILD_TYPE` unset if colcon/the user doesn't pass
  one, meaning no `-DNDEBUG` and no optimization. `CMakeLists.txt` now
  defaults it to `Release` (the standard `if(NOT CMAKE_BUILD_TYPE ...)
  set(... "Release" ...)` boilerplate most ament packages use) when unset.
  Found necessary because of a real bug this surfaced -- see below.

**`eigen-checks` test shim** (`voxfield/test/include/eigen-checks/{gtest.h,
entrypoint.h}`, test-only, only on the 3 affected test targets' include
path): `EIGEN_MATRIX_EQUAL`/`EIGEN_MATRIX_NEAR` reimplemented as templates
returning `::testing::AssertionResult` (element-wise max-abs-diff vs.
tolerance); `entrypoint.h` just defines the unused `UNITTEST_ENTRYPOINT`
macro so the `#include` compiles (all 3 tests that pull it in define their
own `main()`). Used by `test_tsdf_map.cc`, `test_tsdf_interpolator.cc`,
`test_layer_utils.cc` (3 files, not the 2 the plan estimated).

**Tests:** all 10 gtests wired up via `ament_add_gtest`, linked against
`voxfield`. No `test_data` custom target ported -- `test/test_data/` doesn't
exist anywhere in this repo (the old `add_custom_command` copied from it
with `|| :` swallowing the resulting error), so there was nothing to copy.

**Found bug (pre-existing, not a port regression) -- documented per plan
§0 item 6, not fixed:** `Layer::allocateNewBlockByCoordinates()`
(`voxfield/include/voxfield/core/layer.h`) calls `allocateNewBlock()`
unconditionally, which `DCHECK(insert_status.second) << "Block already
exists..."`s if the block's already there, instead of checking
`getBlockPtrByIndex()` first the way `allocateBlockPtrByIndex()` does two
methods above it. `test_tsdf_map.cc`'s `BlockAllocation` test calls
`allocateNewBlockByCoordinates()` twice with coordinates that map to the
same block and expects it to be idempotent -- so the test only passes
because `DCHECK` compiles out under `NDEBUG` (i.e. only in a Release
build). This is pre-existing (unrelated to anything renamed or ported;
`layer.h`'s allocation logic is untouched apart from the mechanical
rename) and was latent under ROS 1 too for the same reason: nothing in the
original `catkin_simple`-based `CMakeLists.txt` set a build type either,
so it depended on whatever the workspace's catkin profile configured.
Confirmed via GDB-free evidence: this exact `DCHECK` fires (glog aborts)
when built without `CMAKE_BUILD_TYPE=Release`, and both this test and the
three below run and pass once it's set. Not fixed here per the plan's
behavior-preservation rule; a real fix (make
`allocateNewBlockByCoordinates()` check-then-allocate) is a candidate for
a follow-up, non-port PR.

**Also explained by the same unset-build-type issue:** `test_merge_integration`,
`test_sdf_integrators`, and `test_clear_spheres` integrate sizeable
simulated worlds (50 poses, fine voxel grids, multiple integrator variants
per test) and were timing out against `ament_add_gtest`'s 60s default
under an unoptimized build (4m36s total for the 4 affected tests). With
`CMAKE_BUILD_TYPE=Release` the full 10-test/50-assertion suite runs in
~13-34s. No `TIMEOUT` overrides were needed once the build type was fixed.

**Map-file compatibility (plan step 6):** no original ROS 1-written
`.tsdf`/`.vxblx` file exists anywhere in this repo (`test/test_data/`
doesn't exist, confirmed above) to load and check. `test_protobuf.cc`
already exercises `SaveLayer`/`LoadLayer` round-trips extensively
(`BlockSerialization`, `LayerSerialization`, `LayerSerializationToFile`,
`LayerSubsetSerializationToFile`, `LayerSubsetSerializationFromFile`,
`MultipleLayerSerialization`, across Tsdf/Occupancy/Esdf/Intensity voxel
types) using the renamed `voxfield.BlockProto`/`voxfield.LayerProto`
messages, and all pass. Per D1, protobuf wire format doesn't encode the
`package` name, so this is sufficient evidence the rename didn't change
the wire format; no additional test was added.

**Acceptance (plan §6 Phase 3):** `colcon build --packages-up-to voxfield`
-- 0 errors, verified both in `scripts/clean_env.sh` (only `/opt/ros/jazzy`
sourced) and the normal shell (Hector underlay sourced). `colcon test
--packages-select voxfield && colcon test-result --verbose` -- 50 tests, 0
errors, 0 failures, 0 skipped.

### Environment fix: miniconda's `python3` breaks `rosidl` (affects every phase from here on)

Not part of the plan's §2 environment facts, found in Phase 4: `PATH` puts
`~/miniconda3/bin` before `/opt/ros/jazzy/bin` and `/usr/bin`, so every ROS 2
build/tool that shells out to `python3` (rosidl's generators, in particular)
runs under miniconda's Python 3.13, not the system Python 3.12 the apt
`python3-*` ROS 2 dependencies were installed against. Two miniconda-side
gaps, both fixed by installing into miniconda's site-packages (reversible;
`pip uninstall`/pin back if this environment is used for anything else):
- `lark` wasn't installed at all (`rosidl_generator_type_description`,
  `rosidl_generator_rs` import `rosidl_parser.parser`, which needs it) ->
  `pip install lark` (got 1.3.1).
- miniconda had `empy` 4.2.1 (pip's newer, API-incompatible fork) where
  ROS 2 Jazzy's `rosidl_generator_rs` needs the classic EmPy 3.3.x API
  (matching the apt `python3-empy` 3.3.4-2 that's actually installed for
  the system Python) -> `pip install "empy==3.3.4"`.

## Phase 4: `voxfield_msgs`

`package.xml` format 3 (`ament_cmake` + `rosidl_default_generators`
buildtool, `std_msgs`/`nav_msgs` depend, `rosidl_default_runtime`
exec_depend, `rosidl_interface_packages` group). `CMakeLists.txt`:
`rosidl_generate_interfaces()` over all 7 `.msg` + `FilePath.srv`,
`DEPENDENCIES std_msgs nav_msgs`. Contents unchanged from D14 (cross-package
type refs already read `voxfield_msgs/...` from the Phase 1 rename;
`FilePath.srv` already had its empty-response `---` and is untouched).

**Accept:** builds (0 errors, clean env + normal shell). `ros2 interface
show voxfield_msgs/msg/Layer`, `.../MultiMesh`, and
`voxfield_msgs/srv/FilePath` all print correctly (verified manually).

## Phase 5: `voxfield_ros` infrastructure (no servers yet)

`package.xml` per plan §6 Phase 5 step 1. One deviation worth flagging:
even though `voxfield_rviz_plugin` is only an `exec_depend` (no C++ code in
`voxfield_ros` includes its headers -- confirmed by grep before writing
this), colcon still refuses to build `voxfield_ros` while that package is
present-but-unbuilt in the workspace: colcon's environment-hook sourcing
step for a package's *build* env walks its full package.xml dependency
list (build **and** exec) and hard-fails with "Failed to find the
following files: .../voxfield_rviz_plugin/share/voxfield_rviz_plugin/
package.sh" if a listed dependency exists as a discoverable package.xml in
the workspace but was never built. Since `voxfield_rviz_plugin/` still has
its ROS 1/catkin `package.xml`/`CMakeLists.txt` (it's ported in Phase 8),
this is a hard build blocker for every phase from here until Phase 8 --
not specific to the clean env or the normal shell, reproduced identically
in both. Fix: `touch voxfield_rviz_plugin/COLCON_IGNORE`, so colcon's
package discovery doesn't see it as a workspace package at all. This is
temporary and must be removed as part of Phase 8 (once
`voxfield_rviz_plugin` has its own ported `package.xml`, it needs to be
discoverable and buildable again, and the same colcon behavior then
correctly enforces build order between it and `voxfield_ros`).

Ported/added, all header-only unless noted:
- **`param_utils.h`** (D6): `getParam<T>`/`param()` implemented with
  `if constexpr` branches on `T` (bool, `std::string`,
  `std::vector<double>`, floating types -> declared/read as `double`,
  integral types -> declared/read as `int64_t`), each declaring once via
  `has_parameter()` + `ParameterDescriptor{dynamic_typing=true}` and
  coercing/erroring/defaulting per D6's rules. `getTransformationParam()`
  reads the flat 16-double array, checks presence via
  `has_parameter()`/`get_parameter_overrides()` before declaring anything
  (so an absent transform param doesn't get force-declared), builds a
  `Transformation::TransformationMatrix` and calls kindr's own
  `constructAndRenormalizeRotation()` (exactly what the plan's
  "renormalize via `constructAndRenormalize`" note is getting at, just
  via the transformation-level factory rather than composing
  `RotationQuaternion::constructAndRenormalize` by hand), then applies
  `invert_<name>` if set.
- **`kindr_conversions.h`** (D4): `transformKindrToMsg`/`transformMsgToKindr`
  templated on `Scalar`, replacing ROS 1's `transformKindrToTF`/
  `transformTFToKindr`/`transformKindrToMsg`/`transformMsgToKindr` (the TF
  pair collapses into the Msg pair -- no separate `tf::Transform` type in
  ROS 2). `xmlRpcToKindr` isn't reimplemented separately; its job is
  `getTransformationParam()`'s, which needs the same
  declare-and-coerce machinery anyway. Found and fixed one real bug while
  bringing this up: `Rotation::Implementation(...).normalized()` doesn't
  implicitly convert to `Rotation` (kindr's constructor from
  `Implementation` is `explicit`) -- needs a direct-init call, not
  copy-init.
- **`ros_params.h`**: all 15 `get*ConfigFromRosParam` functions retargeted
  to `rclcpp::Node&`, `nh_private.param(name, v, v)` -> `param(node, name,
  v)` mechanically (every call in this file was already in that
  self-referential `v, v` form). Names/defaults diffed unchanged against
  the pre-port version.
- **`conversions.h`/`conversions_inl.h`**: message types -> `::msg::`,
  dropped `pcl_ros/point_cloud.h` for `pcl_conversions.h`, added
  `publishPclCloud()` (D11), `std_msgs::ColorRGBA` ->
  `std_msgs::msg::ColorRGBA`. `colorVoxbloxToMsg`/`colorMsgToVoxblox` kept
  (D1 keep-list, free functions).
- **`mesh_vis.h`, `ptcloud_vis.h`**: message/marker types -> `::msg::`;
  `eigen_conversions/eigen_msg.h` + `tf::pointEigenToMsg` ->
  `tf2_eigen/tf2_eigen.hpp` + `tf2::toMsg()`; per D12,
  `generateVoxbloxMeshMsg()` (both overloads) and `fillMarkerWithMesh()`
  now take an explicit `const rclcpp::Time& stamp` parameter instead of
  calling `ros::Time::now()` internally (callers, added in Phase 6/7,
  must pass `node_->now()`). `ptcloud_vis.h`'s `eigen_conversions`/
  `pcl_ros` includes turned out to be unused (grepped for call sites) --
  dropped rather than translated.
- **`intensity_vis.h`, `mesh_pcl.h`**: message types -> `::msg::`;
  `mesh_pcl.h`'s unused `pcl_ros/point_cloud.h` include dropped.
- **`transformer.h`/`.cc`** (D10): `Transformer(rclcpp::Node::SharedPtr)`
  replaces the `(nh, nh_private)` pair; `tf::TransformListener` member ->
  `std::shared_ptr<tf2_ros::Buffer>` + `std::shared_ptr<tf2_ros::
  TransformListener>`, both still constructed unconditionally in the
  constructor (ROS 1 built `tf::TransformListener` unconditionally too,
  even when `use_tf_transforms_` is false -- kept identical rather than
  making it conditional, per the plan's behavior-preservation rule).
  `canTransform`/`lookupTransform` moved to the `tf2_ros::Buffer` API,
  catching `tf2::TransformException`. Every stamp comparison/subtraction
  in `lookupTransformQueue` now goes through an explicit
  `rclcpp::Time(msg.header.stamp, RCL_ROS_TIME)` per pitfall §8.1 (mixing
  `RCL_ROS_TIME` and the default `RCL_SYSTEM_TIME` throws). `T_B_D`/`T_B_C`/
  `T_C_CH` now come from `getTransformationParam()` (replacing the
  `XmlRpc::XmlRpcValue` + `xmlRpcToKindr()` + manual invert-flag reading);
  behavior is identical since `getTransformationParam()` already only
  applies the invert when the base transform was present, matching the
  original `if (nh_private_.getParam(...))`-gated logic.
- **`interactive_slider.h`/`.cc`**: constructor now also takes
  `rclcpp::Node::SharedPtr node` (needed to construct
  `interactive_markers::InteractiveMarkerServer`, which has no
  no-node ROS 2 constructor). Confirmed nothing in the server sources
  instantiates `InteractiveSlider` yet (grepped) -- ported to compile per
  the plan, no runtime test.
- **`node_main.h`** (D16): `initGflagsAndGlog(argc, argv)` strips ROS args
  via `rclcpp::remove_ros_arguments()`, then `InitGoogleLogging` +
  `FLAGS_alsologtostderr = true` (replacing the old
  `args="-alsologtostderr"` launch convention with a default, still
  command-line-overridable) + `ParseCommandLineFlags` +
  `InstallFailureSignalHandler()`.

**Tests:** `test/test_param_utils.cc` (missing param -> default; int
override coerced to double/float; double override coerced to int when
integral vs. falls back to default when not; repeated reads of the same
name; the in-place `param()` helper; `getTransformationParam()` absent /
round-trip / inverted / wrong-size-array) and
`test/test_kindr_conversions.cc` (msg->kindr->msg round trip; identity;
a non-unit-norm quaternion gets renormalized). Both define their own
`main()` (the first needs `rclcpp::init`/`shutdown`), so
`ament_add_gtest(... SKIP_LINKING_MAIN_LIBRARIES)` + explicit `gtest` link.

**Accept:** `voxfield_ros` library target (currently just
`interactive_slider.cc` + `transformer.cc` -- servers land in Phases 6-7)
builds with 0 errors in both the clean env and the normal shell (both
freshly rebuilt from an empty `install/`, with `voxfield_rviz_plugin`
ignored per above). `test_param_utils` + `test_kindr_conversions`: 15
tests, 0 failures.

## Phase 6: `TsdfServer` and `NpTsdfServer`

Both ported together per plan §6 Phase 6, following D5-D16 mechanically:
`(nh, nh_private)` -> single `rclcpp::Node::SharedPtr node_` held (not
inherited from); every publisher/subscription/service/timer becomes a
`...::SharedPtr` member; private topics/services use the `"~/name"`
convention (e.g. `nh_private_.advertise("mesh")` ->
`node_->create_publisher<...>("~/mesh", qos)`); QoS per D7 (a
`transient_local().reliable()` depth-1 QoS for every latched topic; `~/
tsdf_map_out` stays plain `QoS(1)`, matching the ROS 1 `advertise(..., 1,
false)`); timers via `rclcpp::create_timer(node_, node_->get_clock(), ...)`
(D8, so they follow `use_sim_time`, not wall time); `ros::WallTime` timing
-> `std::chrono::steady_clock`; every stored/compared timestamp
(`last_msg_time_ptcloud_`, `last_msg_time_freespace_ptcloud_`, queue
lookups) built as `rclcpp::Time(..., RCL_ROS_TIME)` per pitfall §8.1;
`Marker::MODIFY` -> `Marker::ADD` (D says same numeric value, confirmed:
both are `0`) with `publishRobotMesh()` still called unconditionally from
`processPointCloudMessageAndInsert()` regardless of `publish_robot_model_`,
matching ROS 1's (seemingly accidental) behavior exactly. ICP correction
publishes both a TF broadcast and a `~/icp_transform` message, all three
`geometry_msgs::msg::Transform`s built via the same `transformKindrToMsg()`
call (ROS 2 has no separate `tf::Transform` type, so the ROS 1 code's
`tf::transformKindrToTF()` + `tf::transformKindrToMsg()` split collapses
into one conversion path).

**Callback signature deviation from the plan's literal suggestion:**
`rclcpp::Subscription`'s accepted callback signatures (`rclcpp/
any_subscription_callback.hpp`) include `void(sensor_msgs::msg::
PointCloud2::SharedPtr)` (by value) but *not* `void(const
sensor_msgs::msg::PointCloud2::SharedPtr&)` (by const-ref) -- the latter
fails to compile against `std::bind`'s result type with a wall of
`std::variant`/`enable_if` template errors. `insertPointcloud()`,
`insertFreespacePointcloud()`, and `processPointCloudMessageAndInsert()`
all take the `SharedPtr` by value instead (still non-const, so the
`fields[d].datatype` rewrite hack still works); `getNextPointcloudFromQueue()`
and the two `std::queue<...>` members follow the same type.

**Found and fixed (toolchain-only, not a rename or behavior issue):**
`mesh_vis.h`'s `fillMarkerWithMesh()` (Phase 5, header-only, never actually
compiled until Phase 6 put it in a translation unit for the first time --
`interactive_slider.cc`/`transformer.cc` don't include it) had
`tf2::toMsg(mesh->vertices[i].cast<double>())`: the `cast<double>()` result
is an Eigen expression template that implicitly converts to *both*
`tf2::toMsg(const Eigen::Vector3d&)` and `tf2::toMsg(const
Eigen::Matrix<double,6,1>&)`, an ambiguous overload under GCC 13/Eigen 3.4.
Fixed by binding the cast to a named `Eigen::Vector3d` first. Also fixed a
`-Wreorder` warning in both servers' constructors (`transformer_` must be
initialized before `last_msg_time_ptcloud_`/`last_msg_time_freespace_ptcloud_`
to match declaration order, once the ROS 2 member types changed the
natural initializer-list ordering that used to fall out of the ROS 1
`ros::Time`/`Transformer` types).

**Found, documented, not fixed (pre-existing, per plan §0 item 6):**
- `timing_` (both servers) and `publish_robot_model_` (`TsdfServer`) are
  read via `nh_private.param("x", member_, member_)` -- i.e. the ROS 1 code
  used the member's own current value as its default -- without the member
  ever being given a value first, anywhere in the constructor's initializer
  list. This is an indeterminate-value read (UB), not merely "wrong
  default"; unlike the `width_`/`height_`/`vx_`/`fx_` case below (which the
  plan explicitly calls out and prescribes a fix for), the plan doesn't
  mention these two, but the risk category is identical and the same fix
  applies: both are now given an explicit `= false` in-class initializer.
  Effective default value is unchanged (both were `false` in every
  practical build observed); only the UB is removed.
- `NpTsdfServer::computeNormalImage()`'s dead `if (v == height_)` branch
  (already documented in the plan's §8.14 as a known upstream issue) is
  left exactly as-is, with a code comment pointing back to the plan.
- Per the plan's explicit instruction, `width_`, `height_`, `vx_`, `fx_`
  are now initialized to `0` in the header, and
  `getServerConfigFromRosParam()` logs `RCLCPP_ERROR` if `width_ <= 0 ||
  height_ <= 0` after loading params -- behavior (range-image
  preprocessing silently producing garbage if these are missing) is
  otherwise unchanged, only the initial-read UB and the missing-params
  case are now diagnosable.

**Acceptance (plan §6 Phase 6):** both executables build (0 errors). `ros2
run voxfield_ros np_tsdf_server --ros-args -p width:=1024 -p height:=64 -p
sensor_is_lidar:=true -p fov_up:=3.0 -p fov_down:=-25.0` starts without
exceptions; `ros2 node info /voxfield` shows exactly the §3.4 topic/service
set (`~/mesh`, `~/surface_pointcloud`, `~/tsdf_pointcloud`, `~/gsdf_pointcloud`,
`~/tsdf_slice`, `~/gsdf_slice`, `~/occupied_nodes`, `~/tsdf_map_out`,
`~/Robot_model`; `pointcloud`, `~/tsdf_map_in` subscriptions; `~/clear_map`,
`~/generate_mesh`, `~/save_map`, `~/load_map`, `~/publish_pointclouds`,
`~/publish_map` services); `ros2 service call /voxfield/clear_map
std_srvs/srv/Empty` returns. `tsdf_server` (default node name `voxblox`)
also confirmed to start and run its `updateMeshEvent` timer without error.
`test_param_utils` + `test_kindr_conversions` still pass (no infra
regressions).

## Phase 7: Derived servers and eval/tool executables

Ported in the plan's prescribed order: `VoxfieldServer`, `VoxbloxServer`,
`FiestaServer`, `VoxedtServer` (mirrored from `FiestaServer`, per the
plan), `IntensityServer`, `SimulationServer`, then `voxblox_eval.cc`,
`simulation_eval.cc`, `visualize_tsdf.cc` -- completing all 10
`voxfield_ros` executables from plan §3.3. Each `*_server` follows the
Phase 6 patterns (`"~/name"` private topics, D7 QoS, D8 timers via
`rclcpp::create_timer`, `RCL_ROS_TIME` stamps, `publishPclCloud()` for
every `pcl::PointCloud<T>` that used to publish directly via `pcl_ros`).

**`VoxedtServer` generated by mirroring `FiestaServer`** (plan's explicit
suggestion): after porting `FiestaServer` by hand, `voxedt_server.{h,cc}`
was produced with `sed` (class name swap, `EsdfOccFiestaIntegrator` ->
`EsdfOccEdtIntegrator`, `getEsdfOccFiestaIntegratorConfigFromRosParam` ->
`getEsdfEdtIntegratorConfigFromRosParam`), then hand-verified against a
`diff` of the two ROS 1 originals to confirm nothing else differs. Two
mechanical artifacts from the substitution were caught and fixed: a
duplicated `#include <voxfield/integrator/esdf_occ_edt_integrator.h>` (the
file already included both fiesta and edt integrator headers; the
class-name substitution turned the fiesta one into a second copy of the
edt one), and one sed rule's output being re-matched by an earlier rule
(`EsdfOccFiestaIntegrator` -> `EsdfOccEdtIntegrator` fired inside the
longer identifier `getEsdfOccFiestaIntegratorConfigFromRosParam` before
the intended whole-identifier replacement could match it, leaving
`getEsdfOccEdtIntegratorConfigFromRosParam` -- not a real function --
instead of the correct `getEsdfEdtIntegratorConfigFromRosParam`). Also
corrected a stale `// ... via FIESTA` comment carried over into
`voxedt_server.cc` from the original (pre-existing upstream copy-paste
leftover, zero behavior impact, fixed as a comment-only change).

**Found and fixed (toolchain-only, an ODR bug only reachable once Phase 7
links two independent translation units together):** `voxfield` core's
`SimulationWorld::setVoxel<TsdfVoxel>`/`setVoxel<EsdfVoxel>` -- full
template specializations -- are defined in the header
`simulation/simulation_world_inl.h` without `inline`. A full specialization
is an ordinary (non-template) definition for ODR purposes, so this is only
safe if at most one translation union in a given link both includes the
header. That held throughout Phase 3-6 (only `simulation_server.cc`
included it), but `simulation_eval.cc` also includes
`voxfield_ros/simulation_server.h` (for its `SimulationServerImpl`
subclass) and gets linked into the same `simulation_eval` executable as
`libvoxfield_ros.a` (which already contains `simulation_server.cc.o`) --
hence `ld: multiple definition of ... setVoxel<...>`. Fixed by marking both
specializations `inline`; this is the same category of latent,
toolchain-exposed bug as the Phase 6 `mesh_vis.h` ambiguous-overload fix,
not a rename or behavior change.

**Found and fixed (Phase 1 rename-script artifact, not a Phase 7
regression, but only just discovered here):** `voxblox_server.h`'s include
guard was `VOXFIELD_ROS_VOXFIELD_SERVER_H_` -- byte-identical to
`voxfield_server.h`'s own guard. Root cause: Phase 1's blanket `\bVOXBLOX_`
-> `VOXFIELD_` substitution (per D1's include-guard rule) also matched the
"VOXBLOX_SERVER" portion of the original `VOXBLOX_ROS_VOXBLOX_SERVER_H_`
guard (which names the *file*, not just the package), even though D1
explicitly keeps `VoxbloxServer`/`voxblox_server.{h,cc}` unrenamed (they
name the method, not the project). The two files were never included
together in the same translation unit before now, so the collision was
silent; Phase 1's grep audit couldn't have caught it either, since neither
resulting guard string contains the literal substring "voxblox" anymore.
Fixed by renaming `voxblox_server.h`'s guard to
`VOXFIELD_ROS_VOXBLOX_SERVER_H_` (prefix renamed per D1, "VOXBLOX_SERVER"
kept per the do-not-rename list -- consistent with every other identifier
in that file).

**Dead code kept faithfully (not cleaned up, per behavior-preservation):**
`VoxfieldServer::generateEsdfCallback()` and `VoxbloxServer::
generateEsdfCallback()` are both declared and (for `Voxblox`Server) even
defined, with a `generate_esdf_srv_` member declared alongside, but
neither is ever bound to an actual service in `setupRos()` in either ROS 1
class -- both are unreachable via ROS, only callable from C++ code that
holds the object directly. Ported with the same shape (ROS 2 service
callback signature, still unbound) rather than either wiring them up (a
behavior change) or deleting them (also a behavior/API change for anyone
using these classes as a library).

**Acceptance (plan §6 Phase 7):** all 10 executables build with 0 errors,
confirmed via a full clean rebuild (`rm -rf build install log`) in both
`scripts/clean_env.sh` and the user's normal shell with the Hector
underlay sourced (`ros2 pkg list` shows both `voxblox*` and `voxfield*`
packages simultaneously; a live `voxfield_server` run under that shell
starts and runs its mesh-update timer without error, confirming no Ogre/
symbol/param clashes at runtime, not just at link time). `colcon
test-result --verbose`: 65/65 tests, 0 failures. `ros2 run voxfield_ros
simulation_eval` runs to completion -- publishes GT/test clouds, prints
`TSDF RMSE: ... ESDF RMSE: ...` and the mesh timing table, then blocks in
`rclcpp::spin()` exactly as the ROS 1 original did (matches "Done." log
line before spinning). Each of `voxfield_server`, `voxblox_server`,
`fiesta_server`, `voxedt_server`, `intensity_server` starts and holds its
`updateMeshEvent`/`updateEsdfEvent` timer loop for several ticks with no
errors in the log. `voxfield_server`'s full `ros2 node info` output
(publishers, subscriptions, services) matches plan §3.4 exactly, including
the ESDF-specific topics (`~/esdf_pointcloud`, `~/esdf_slice`, `~/
esdf_map_out`, `~/esdf_map_in`, `~/save_esdf_map`) layered on top of the
Phase 6 NpTsdfServer set; a live `~/clear_map` service call returns.

## Phase 8: `voxfield_rviz_plugin`

Ported `voxfield_mesh_display.{h,cc}`, `voxfield_multi_mesh_display.{h,cc}`,
`voxfield_mesh_visual.{h,cc}`, and `material_loader.{h,cc}` from `rviz`
(Qt5/Ogre1, catkin_simple) to `rviz_common` (Qt5/Ogre1 via
`rviz_ogre_vendor`, `ament_cmake`). The rename to `voxfield_*` identifiers,
`VoxfieldMaterial*`/`VoxfieldMaterials` Ogre names, and `VoxfieldMesh.png`/
`VoxfieldMultiMesh.png` icons had already landed in Phase 1; this phase is
the ROS 1 → ROS 2 API port only, no further renaming.

### Mechanical translations
- `rviz::MessageFilterDisplay<voxfield_msgs::Mesh>` →
  `rviz_common::MessageFilterDisplay<voxfield_msgs::msg::Mesh>`;
  `processMessage(const T::ConstPtr&)` → `processMessage(T::ConstSharedPtr)`
  (by value, matching the new pure-virtual signature).
- `#include <OGRE/OgreFoo.h>` → `#include <OgreFoo.h>` (Ogre headers are no
  longer nested under an `OGRE/` prefix in `rviz_ogre_vendor`).
- `ros::package::getPath("voxfield_rviz_plugin")` →
  `ament_index_cpp::get_package_share_directory("voxfield_rviz_plugin")`
  in `material_loader.cc`.
- `context_->getFrameManager()->getTransform(frame, stamp, position,
  orientation)` — same argument order and return convention in
  `rviz_common::FrameManagerIface`; only the header and Ogre includes
  changed. `ros::Time::now()` → `context_->getFrameManager()->getTime()`
  (the rviz2-idiomatic replacement that respects the display's sync mode
  during bag playback, rather than a free-standing `rclcpp::Clock`, per the
  same reasoning as plan D12 for the server code). Message stamps go
  through `rclcpp::Time(msg->header.stamp, RCL_ROS_TIME)`, consistent with
  every other stamp conversion in this port.
- `PLUGINLIB_EXPORT_CLASS(..., rviz::Display)` →
  `PLUGINLIB_EXPORT_CLASS(..., rviz_common::Display)`;
  `#include <pluginlib/class_list_macros.h>` →
  `<pluginlib/class_list_macros.hpp>`.
- `plugin_description.xml`: `base_class_type="rviz::Display"` →
  `"rviz_common::Display"`, `message_type` `voxfield_msgs/Mesh` →
  `voxfield_msgs/msg/Mesh` (and the `MultiMesh` equivalent), `<library
  path="lib/libvoxfield_rviz_plugin">` → `path="voxfield_rviz_plugin"`
  (ament_cmake's pluginlib loader resolves the library by package-relative
  name, not a `lib/` path).

### Design decision: dropped the ROS 1 custom `subscribe()`/`onInitialize()` override in `VoxfieldMultiMeshDisplay`
The ROS 1 class overrode `subscribe()` to get a 1000-deep subscriber queue
(vs. the base class's default) and a manual reliable/unreliable transport
toggle, plus overrode `onInitialize()` to push the same queue depth into
`tf_filter_`. In `rviz_common`, `_RosTopicDisplay` already ships a
`QosProfileProperty` in every topic-based display's property panel, which
gives the user the reliable/best-effort toggle for free — reimplementing
the ROS 1 raw-transport-hint logic would just be a worse duplicate of that
built-in control. So Phase 8 keeps `MessageFilterDisplay<MultiMesh>` as
the base (unchanged from ROS 1) but, instead of overriding `subscribe()`/
`onInitialize()`, sets the *defaults* the base class already exposes:
`qos_profile = rclcpp::QoS(kSubscriberQueueLength)` (subscription depth)
and `message_queue_property_->setInt(kSubscriberQueueLength)` (tf2
message-filter queue depth), both in the constructor. Net behavior is the
same — a 1000-deep queue and a user-editable reliability setting — with
less code and no reimplemented ROS 2 transport plumbing. Both values stay
user-editable in the property panel afterwards, same as before.

### Everything else ported 1:1
`fixedFrameChanged()` in both display classes deliberately does *not* call
the `MessageFilterDisplay` base implementation (which would call `reset()`
and clear the built-up mesh) — it replicates the ROS 1 override exactly:
`tf_filter_->setTargetFrame(...)` followed by an in-place transform update,
so an incrementally-built mesh survives a fixed-frame change exactly as it
did in ROS 1. The `VisibilityField` per-namespace visibility tree, the
`VoxfieldMultiMeshDisplay::update()` throttled polling of all visual poses,
and the alpha/normal/color mesh-block-to-Ogre-object translation in
`VoxfieldMeshVisual::setMessage()` are unchanged apart from the message
type.

### Verification
- Clean build with zero errors and zero warnings (`-Wall -Wextra`) in both
  `scripts/clean_env.sh` and the user's normal shell with the Hector
  underlay sourced.
- `nm -D` on the built `libvoxfield_rviz_plugin.so` confirms the
  `class_loader`-registered symbols are namespaced under
  `voxfield_rviz_plugin::VoxfieldMeshDisplay` /
  `::VoxfieldMultiMeshDisplay`, and the ament resource index entry
  (`share/ament_index/resource_index/rviz_common__pluginlib__plugin/
  voxfield_rviz_plugin`) is separate from Hector's `voxblox_rviz_plugin`
  entry — the two are independently discoverable by rviz2's "Add → By
  display type" dialog.
- Launched `rviz2` (in an isolated `env -i` shell sourcing only
  `/opt/ros/jazzy` + the workspace install, to sidestep an unrelated,
  pre-existing environment issue where this machine's VS Code/snap
  environment breaks `rviz2`'s libpthread resolution — not a port bug,
  reproduces identically with Hector's own unmodified `rviz2` binary) with
  a config instantiating both `VoxfieldMesh` and `VoxfieldMultiMesh`: no
  pluginlib load errors, no Ogre resource-group exceptions, ran cleanly
  for the full duration.
- Repeated with Hector's `voxblox_rviz_plugin` also sourced and a third
  `VoxbloxMesh` display added to the same rviz2 session (same isolated
  shell, now sourcing `/opt/ros/jazzy` + Hector's underlay + the
  workspace): all three displays load together with no Ogre material
  clash and no pluginlib class-name collision, confirming the Phase 1
  rename achieved its goal for the RViz layer, not just for the core
  library/messages.
- No dataset/bag was available for this phase, so mesh *rendering* itself
  (as opposed to plugin loading and material/resource setup) is unverified
  pixel-for-pixel; that falls under Phase 12 (needs real datasets).

**Accept (plan §6 Phase 8):** met, with the rendering caveat above noted
for Phase 12.

## Phase 9: Configuration files

Wrote `scripts/convert_ros1_params.py` (PyYAML) and ran it in place over
every shipped ROS 1 parameter file. It: loads with `yaml.safe_load` (which
already resolves `&anchor`/`*alias` references — no special handling
needed, PyYAML's safe loader supports anchors natively), recursively
flattens any list-of-lists of numbers found anywhere in the tree into a
row-major list of Python `float`s (catches `T_B_C`, `T_B_D`, `T_C_CH`,
`T_D_C`, `T_D_B` uniformly, by shape rather than by name, so it needs no
per-key allowlist), wraps the result as `/**:\n  ros__parameters:\n ...`,
and — before writing anything to disk — re-verifies its own output
(`verify()`): every scalar must round-trip byte-identical and every
matrix must flatten to the same numbers in the same order as the
original. Comment preservation was left out (PyYAML drops them on
re-dump); the plan calls this a nice-to-have, and none of the shipped
files' comments carry information not already in an adjacent key name.

### Files converted
`cfg/param/{basement,cow,kitti,mai,vicon}_param.yaml`,
`cfg/calib/{basement,cow,kitti,mai,vicon,euroc}_calib.yaml`,
`cfg/kitti_lidar.yaml`, `cfg/rgbd_dataset.yaml`,
`cfg/stereo/{kitti_stereo,kitti_stereo_bm,kitti_stereo_jager}.yaml` (the
last three convert cleanly but are unused — see plan §1.3, the stereo
pipeline is out of scope; converted anyway per the plan's "for
completeness" instruction).

### Decision: `cfg/calib/euroc_camchain.yaml` left unconverted
This is Kalibr camchain output (nested per-camera maps with
`distortion_coeffs`, `intrinsics`, `resolution`, `rostopic`, etc.), not a
`rosparam`-loaded file — no in-scope launch file loads it via
`<rosparam file=...>` or would load it via ROS 2 `--params-file`. Its
only reference anywhere in the repo is `launch/bak/euroc_dataset.launch`
(out of scope per plan §1.3/§6 Phase 10 step 2), and even that reference
is itself broken — it points at `cfg/calibrations/euroc_camchain.yaml`,
a directory that has never existed in this repo (the real path is
`cfg/calib/`). Wrapping this file in `ros__parameters` would misrepresent
its actual purpose (external Kalibr tooling reads it in its native
format) for no benefit, since nothing in this repo would ever load it as
a ROS parameter file either before or after the port. Left byte-identical.

### Found (pre-existing upstream bug, not touched): `T_B_D::` typo in `euroc_calib.yaml`
The original ROS 1 file has `T_B_D::` (double colon) instead of `T_B_D:`.
PyYAML parses this as a mapping with a literal key `"T_B_D:"` (trailing
colon included in the key name) rather than raising a syntax error, so
the conversion script's round-trip-preserving design carried the typo
through faithfully — the converted file has key `'T_B_D:'`, still not
`T_B_D`. This means `T_B_D` was never actually readable as a ROS
parameter under this name in the ROS 1 version either (`rosparam load`
would have set the same bogus key). Not fixed, per the plan's "don't fix
upstream bugs silently" rule (§0.5) — and moot in practice, since
`euroc_calib.yaml` isn't loaded by any in-scope (non-`bak/`) launch file.

### Verification (plan §6 Phase 9 accept criteria)
- `voxfield_server --ros-args --params-file cfg/param/kitti_param.yaml
  --params-file cfg/calib/kitti_calib.yaml` starts with no parse errors.
- `ros2 param dump` on the running node confirms `tsdf_voxel_size: 0.25`
  (from the `&voxel_size` anchor in the original `kitti_param.yaml`) and
  `T_C_CH` present as the same 16 values, same row-major order, as the
  original nested 4x4 list in `kitti_calib.yaml`:
  `[-1,0,0,0, 0,0,1,1.8, 0,1,0,0, 0,0,0,1]`.
- `update_esdf_every_n_sec: 0` (an int-looking value that's read as a
  `double` in `NpTsdfServerConfig`, plan pitfall §8.5) was deliberately
  left as the YAML-inferred integer type rather than forced to `0.0` --
  the tolerant `voxfield::param()` helper (D6, ported in Phase 5) already
  coerces `PARAMETER_INTEGER` -> `double` at load time, and forcing every
  scalar to float would make the converted files diverge from what a
  human reading the original would expect for genuinely-integer params
  (`width`, `height`, `num_buckets`, `integration_threads`, ...). Only
  the flattened matrix elements are unconditionally forced to `float`,
  per the plan's explicit instruction for those.
- Every conversion's `verify()` step passed (script would have raised
  `AssertionError` and left the file untouched on any mismatch).

**Accept (plan §6 Phase 9):** met.
