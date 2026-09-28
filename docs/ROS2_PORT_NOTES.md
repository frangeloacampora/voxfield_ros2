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
  labels, but the rename should still be consistent). *(Later corrected,
  multi-sensor Phase 10: that rename mislabeled the **voxblox** ESDF
  integrator's timings as `.../voxfield/...`, which reads like the Voxfield
  integrator's own `update_esdf/voxfield`. They are now
  `update_esdf/voxblox/...`, and the `upate` typo is fixed everywhere,
  including EDT's `update_esdf/edt/...`.)* `conversions.h`
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
a follow-up, non-port PR. *(Later fixed, multi-sensor Phase 10: the
`DCHECK` is a correct precondition ("new" block), and only tests call
`allocateNewBlockByCoordinates()`. The bug was `BlockAllocation` checking
idempotency with the "new" API, so it now uses the idempotent
`allocateBlockPtrByCoordinates()`. A Debug build of `voxfield` then passes
`test_tsdf_map` and the other Layer-level tests; see
`docs/MULTI_SENSOR_NOTES.md` Phase 10.)*

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
  *(Later fixed; see "Known upstream issues" #1 below.)*
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
using these classes as a library). *(Later deleted at the user's request;
see "Known upstream issues" #6 below.)*

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
*(Later fixed; see "Known upstream issues" #7 below.)*

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

## Phase 10: Launch files & RViz2 configs

### Generic launch + per-dataset-method wrappers
Wrote `launch/mapping.launch.py`: one `OpaqueFunction`-driven launch that
takes `method` (`voxfield`/`voxblox`/`fiesta`/`voxedt`/`np_tsdf`/`tsdf` ->
`<method>_server`) and `dataset` (`cow`/`kitti`/`mai`/`basement`/`vicon`)
and resolves per-dataset defaults (param/calib YAML, robot model, point
cloud/transform topic, RViz config, bag speed) from a table built by
reading every ROS 1 `.launch` file for that dataset. `bag_file` defaults
to empty (plan §6 Phase 10 step 3 -- the ROS 1 defaults were all
hard-coded to `/media/yuepan/...` or `/Users/helen/...`); passing one
plays it with `ros2 bag play ... --clock -r <speed>`. Every other
override (`speed`, `rviz`, `rviz_config`, `robot_model_file`,
`pointcloud_topic`, `transform_topic`, `use_sim_time`) defaults to empty
meaning "use this dataset's default," so `ros2 launch voxfield_ros
mapping.launch.py method:=voxfield dataset:=kitti play_bag:=false` (the
plan's own accept-criterion command) works with zero further arguments.
Verified live: starts `voxfield_node` with the exact §3.4 topic/service
graph (`ros2 node info /voxfield_node`, including
`/voxfield_node/esdf_map_out`, `~/save_esdf_map`, etc.) and RViz2 with no
errors.

Per-file wrappers under `launch/{voxfield,voxblox,fiesta,voxedt}_launch/`
and `launch/eval/` (19 total, one per surviving ROS 1 `.launch` file,
`bak/` skipped per plan §6 Phase 10 step 2) each `IncludeLaunchDescription`
`mapping.launch.py` with `method`/`dataset` fixed and every pass-through
arg re-declared so `ros2 launch voxfield_ros kitti_voxfield.launch.py
bag_file:=...` still works. Per-file deviations found by diffing each ROS
1 file against its dataset's "canonical" (most common) variant were
carried over as explicit non-empty defaults in that one wrapper, not
baked into `mapping.launch.py`'s dataset table:
- `kitti_voxblox.launch.py`, `kitti_fiesta.launch.py`,
  `kitti_voxedt.launch.py`: `pointcloud_topic` defaults to
  `/velodyne_points_filtered` and `speed` to `0.25`, matching their ROS 1
  originals (`kitti_voxfield.launch` alone used `/velodyne_points` /
  `0.5`, which is what `mapping.launch.py`'s `kitti` table entry uses).
- `kitti_fiesta.launch.py`, `kitti_voxedt.launch.py`: their ROS 1
  originals never set `robot_model_file` at all, so no robot-mesh marker
  is published (`TsdfServer::robot_model_file_` / `NpTsdfServer::
  robot_model_file_` default to `""`, confirmed by reading `ros_params.h`
  callers). Reproducing "don't set this parameter" through a launch
  argument default needed a way to distinguish that from "use this
  dataset's default" (which is what an empty string already means), so
  `mapping.launch.py`'s `robot_model_file` argument accepts a
  `__none__` sentinel for exactly this case, documented in the launch
  file's own docstring.
- `basement_voxfield.launch.py`, `basement_voxblox.launch.py`: the ROS 1
  originals never set `<param name="use_sim_time">` at all (an
  omission/inconsistency vs. every other dataset's launch file, not
  fixed here since it's upstream behavior -- but ROS 2 pitfall §8.6 makes
  `use_sim_time` mandatory on every node regardless, so both wrappers
  still default it to `true` like all the others).

### Dropped dead launch argument: `process_every_nth_frame`
Declared with a default in nearly every ROS 1 `.launch` file but never
read anywhere in `voxfield_ros` (`grep` for it outside `.launch` files
returns nothing) -- a no-op argument, not a real deviation to preserve.
Dropped rather than ported.

### Eval launches (`launch/eval/*.launch.py`)
Ported all 4 as standalone `voxblox_eval` launches (no `mapping.launch.py`
involvement -- the node, params, and shape differ too much from the
mapping servers to share it). `eval_cow_and_lady.launch.py` also starts
RViz2 with `cfg/rviz/eval.rviz`; the other three don't (matching their
ROS 1 originals). All map-file path arguments (`gt_file_path`,
`voxblox_file_path`, `voxblox_esdf_file_path`, `voxblox_occ_file_path`)
default to empty rather than the ROS 1 originals' hard-coded personal
paths, per plan §6 Phase 10 step 3.

**Found (pre-existing upstream issue, not fixed):** `eval_cow.launch` and
`eval_euroc.launch` each loaded `<rosparam file="$(find voxfield_ros)/cfg/
{cow,euroc}_dataset.yaml"/>`, but neither `cfg/cow_dataset.yaml` nor
`cfg/euroc_dataset.yaml` exists anywhere in this repository's history --
both ROS 1 launch files were already non-functional. `eval_kitti.launch`
has the same kind of line, but it was already commented out there. Since
in ROS 2 a `parameters=[<missing path>]` entry aborts the entire launch
(not just a few params, as `<rosparam file=...>` failing did in ROS 1),
porting the line byte-for-byte would make these launch files
unconditionally fail to start at all. The dead reference is omitted in
`eval_cow.launch.py` / `eval_euroc.launch.py` rather than ported; see the
docstring in each file for the same note.

### RViz2 configs
Converted all 8 files in `cfg/rviz/*.rviz` in place with a one-shot script
(not shipped -- scratch tool, per plan §6 Phase 10 step 5's "hand-write
YAML using existing RViz2 configs as templates" option), rather than the
"launch rviz2, add displays, save" alternative, since the latter can't be
driven headlessly/reproducibly. Verified the target format empirically
against several live Jazzy packages' shipped `.rviz` files (not just the
plan's class-rename table), since rviz2's on-disk schema changed more
than a class rename in several places:
- **Topic properties changed shape.** ROS 1's flat `Topic: /foo` (and
  Marker/MarkerArray's `Marker Topic: /foo`) became a nested QoS block --
  confirmed against `/opt/ros/jazzy/share/nav2_bringup/rviz/
  nav2_default_view.rviz`'s `PointCloud2`/`MarkerArray`/`SetInitialPose`
  entries: `Topic: {Depth, Durability Policy, History Policy,
  Reliability Policy, Value}`, plus `Filter size` for classes derived
  from `MessageFilterDisplay` (confirmed via that same file's `LaserScan`
  entry -- the same base class `VoxfieldMeshDisplay`/
  `VoxfieldMultiMeshDisplay` use, per Phase 8). `Marker Topic` renamed to
  plain `Topic`; the separate `Queue Size` / `Unreliable` properties are
  gone, superseded by the QoS block's `Depth` / `Reliability Policy`.
  `~mesh` is the only latched publisher (plan §3.4 / D7), so only
  `VoxfieldMesh`'s/`VoxfieldMultiMesh`'s `Topic` block gets `Durability
  Policy: Transient Local` + `Depth: 1`; everything else gets `Volatile`
  + `Depth: 5` (D7's non-latched default).
- **`Global Options` dropped two fields.** `Default Light` and `Frame
  Rate` no longer appear in any live Jazzy `.rviz` file checked
  (`nav2_bringup`, `ros_gz_sim_demos`, `depth_image_proc`); only
  `Background Color` and `Fixed Frame` remain. Dropped on conversion.
- **`Window Geometry`'s `QMainWindow State`** is an opaque hex-encoded Qt
  binary blob (saved dock/window layout only, no functional effect).
  Dropped; the rest of `Window Geometry` (dock collapse states) is kept.
- Per plan §1.3, dropped displays/panels using out-of-scope third-party
  plugins: `mav_planning_rviz/PlanningPanel` (a `Panels` entry in
  `vicon_10cm.rviz` and `path_planning_exp.rviz`) and
  `rviz_plugin_tutorials/Imu` (a `Displays` entry in the same two files).
  Their sibling displays that use plain built-in rviz classes but
  reference topics from the same out-of-scope `mav_local_planner`/
  `mavros`/`vins_fusion` stack (e.g. `Local Path` `MarkerArray`,
  `PlanningMarker` `InteractiveMarkers`, `Mavros Odometry`) were **kept**
  -- the plan's out-of-scope list names two specific plugin classes, not
  "anything path-planning-related," and those displays simply show
  nothing when nothing publishes to their topics, same as in the ROS 1
  original when run without the full mav_planning stack.
- Found and fixed a pre-existing Phase 1 rename-script miss, explicitly
  deferred to this phase by the plan itself (§6 Phase 1 step 4: "the old
  `.rviz` files will be regenerated in Phase 10 anyway"): `vicon_10cm.rviz`
  and `path_planning_exp.rviz`'s `Displays` panel `Expanded` lists still
  said `/VoxbloxMesh1` even though the display itself already read `Name:
  VoxfieldMesh`. Fixed to `/VoxfieldMesh1` during conversion.
- All topic paths `/voxblox_node/...` -> `/voxfield_node/...` (D1's
  launch node-name rename). Colors, alpha, slice min/max intensity ranges,
  Grid/Marker/PointCloud2 styling, and View (Orbit/TopDownOrtho)
  parameters carried over unchanged from each original file.

Verified: loaded all 8 converted files in a live `rviz2` (in the same
isolated shell used for the Phase 8 plugin-loading check, to route around
this machine's unrelated snap/libpthread issue) -- zero pluginlib/Ogre/
config-parse errors, and `kitti_25cm.rviz` / `eval.rviz` additionally
confirmed via log output that the enabled `PointCloud2` display correctly
subscribed to its (renamed) topic.

### Verification
- `ros2 launch voxfield_ros mapping.launch.py method:=voxfield
  dataset:=kitti play_bag:=false` (plan's own accept command): starts
  `voxfield_node` + RViz2 with no errors; `ros2 node info /voxfield_node`
  matches the plan §3.4 interface exactly.
- `ros2 launch voxfield_ros kitti_fiesta.launch.py play_bag:=false
  rviz:=false`: process argv confirms `-r pointcloud:=/velodyne_points_filtered`
  was applied, verifying the per-file deviation override plumbing.
- Full clean rebuild (`rm -rf build install log`, then
  `scripts/clean_env.sh colcon build --symlink-install`) succeeds with 0
  errors; `colcon test-result --verbose`: 65/65 tests still pass.
- No dataset/bag is available in this environment, so bag playback
  (`ExecuteProcess(['ros2', 'bag', 'play', ...])`) and the mesh actually
  building progressively in RViz2 are unverified; that's Phase 12.

**Accept (plan §6 Phase 10):** met, modulo the dataset-playback caveat
above (Phase 12).

## Phase 11: Docs, cleanup, CI

- **README.md:** rewrote the Installation section for `colcon`/`rosdep`
  (dropped `wstool`/`catkin build`), replaced the "we keep the name of our
  package as voxblox" paragraph with an explanation of the D1 rename and
  what it means for downstream projects (Cblox/Voxgraph/Kimera/Panmap need
  their includes/namespace/message package updated to `voxfield*`), and
  updated every `roslaunch pkg file.launch` example to `ros2 launch pkg
  file.launch.py`, adding the `rosbags-convert` bag-conversion step ahead
  of each one. Added a "Run on your own data" section using
  `mapping.launch.py` directly, replacing the ROS 1 original's "(TBA)"
  placeholder. Left the citation, acknowledgments, paper/video links, and
  external voxblox.readthedocs.io links untouched, per plan §0.5/§6 Phase
  11.
- **Removed** `voxfield_https.rosinstall`, `voxfield_ssh.rosinstall`,
  `rosdoc.yaml` (all catkin/rosdoc tooling, meaningless under `colcon`).
- **`.gitignore`:** dropped catkin-only entries (`devel/`, `msg_gen/`,
  `srv_gen/`, `build_isolated/`, `devel_isolated/`, `CATKIN_IGNORE`,
  dynamic-reconfigure `*.cfgc`, generated `srv/_*.py`/`msg/_*.py`, a
  `/planning/` block that doesn't exist in this repo); kept
  editor/generated-doc entries and `build/`/`install`/`log/` (already
  present, matches D2/§5's colcon layout) and added `COLCON_IGNORE` (used
  on `voxfield_rviz_plugin/COLCON_IGNORE` during Phases 3-7 before that
  package existed; kept for any future per-package opt-out).
- **CI:** added `.github/workflows/ros2.yml` using `ros-tooling/setup-ros`
  + `ros-tooling/action-ros-ci` on `ubuntu-24.04` with
  `required-ros-distributions: jazzy`, building and testing all 4
  packages (`voxfield`, `voxfield_msgs`, `voxfield_ros`,
  `voxfield_rviz_plugin`). Not yet run on real CI infrastructure (this
  environment has no GitHub Actions access) -- the workflow mirrors the
  exact `colcon build`/`colcon test` invocations already verified
  manually in every phase's acceptance check, so it should pass, but
  that's unverified until it actually runs.
- **clang-format:** ran `clang-format` (repo's own `.clang-format`:
  Google style, 80 cols, pointer-left) over exactly the 145 C++ files
  this port touched (`git diff main...HEAD --name-only --diff-filter=AMR
  -- '*.cc' '*.h'`, confirming the vendored `voxfield/third_party/minkindr`
  headers -- untouched by the port -- were correctly excluded).
  `clang-format` wasn't installed and this sandbox has no interactive
  `sudo` password prompt; installed a user-level copy with `pip install
  --user clang-format` instead of skipping the step (confirmed with the
  user first). Result: 86 files reformatted, 523 insertions / 594
  deletions, entirely whitespace/line-break/include-ordering -- confirmed
  behavior-neutral by a full clean rebuild (0 errors) and `colcon
  test-result`: 65/65 tests still passing after the reformat.

### Final interface table (plan §3.4, re-verified)

Re-checked live against a running `voxfield_server` (the most complete
interface: TsdfServer/NpTsdfServer's ESDF-capable servers get the full
set below; `tsdf_server`/`np_tsdf_server`/`intensity_server` lack the
`~/esdf_*` and `~/save_esdf_map` entries, since they have no ESDF
integrator) via `ros2 node info /voxfield_node`, launched through
`mapping.launch.py method:=voxfield dataset:=kitti` (Phase 10). Matches
plan §3.4 exactly; node-name prefix is `/voxfield_node` per D1 (was
`/voxblox_node`).

- **Subscriptions (remappable):** `pointcloud`, `freespace_pointcloud`
  (if `use_freespace_pointcloud`), `transform` (if `!use_tf_transforms`).
- **Subscriptions (private):** `~/tsdf_map_in`, `~/esdf_map_in`,
  `~/intensity_image` (`IntensityServer` only).
- **Publishers (private):** `~/mesh` (latched), `~/surface_pointcloud`,
  `~/tsdf_pointcloud`, `~/gsdf_pointcloud`, `~/tsdf_slice`, `~/gsdf_slice`,
  `~/occupied_nodes`, `~/tsdf_map_out`, `~/esdf_map_out`, `~/Robot_model`,
  `~/icp_transform` (via `/tf`), `~/esdf_pointcloud`, `~/esdf_slice`,
  `~/traversable`, `~/esdf_error_slice`, `~/intensity_pointcloud`,
  `~/intensity_mesh`.
- **Services (private):** `~/generate_mesh`, `~/clear_map`, `~/save_map`,
  `~/load_map`, `~/save_esdf_map`, `~/save_occ_map`, `~/save_all_map`,
  `~/publish_pointclouds`, `~/publish_map`.
- **TF:** looks up `world_frame ← sensor_frame`; broadcasts
  `icp_corrected`/`pose_corrected` on `/tf` when ICP is enabled.

### Known upstream issues (consolidated)

Everything below is pre-existing ROS 1 Voxfield/Voxblox behavior. It was
first carried over unfixed, per the plan's "document, don't silently fix"
rule (§0 item 6). Algorithmic ones are already named in plan §8.14; the rest
were found during the port and logged in their own phase's section above
(linked here for a single point of reference).

After the port was complete, the user asked for #12 to be fixed, and then
for every other item that was still open to be fixed if it was a real bug
or deleted if it was dead code. Each item says **FIXED**, **REMOVED**, or
(for #4, #5, #8-#10) how it was already handled during the port. Each fix
was checked against the ROS 1 original (`git show main:voxblox_ros/...`)
to confirm the behavior was the same upstream, and has regression tests
that fail on the old code. Numbering is kept stable.

1. **FIXED.** `NpTsdfServer::computeNormalImage()`: `if (v == height_)`
   can never be true inside `for (v = 0; v < height_; ...)`, so row
   `height_ - 1`'s normals read one row out of bounds of
   `vertex_map`/`depth_image`. That is undefined behavior: in practice the
   last row got garbage or zero normals. (Plan §8.14; Phase 6 notes
   above.)

   The branch body (`n_y_v = v - 1; sign *= -1.0;`) shows what was
   intended. The last row should use the previous row as its y-neighbor
   and flip the normal's sign, because `dy` then points the other way. So
   the fix gives that row a real normal, not an empty one. The condition
   is now `v == height_ - 1`.

   Fixing it exposed the same boundary bug on the other axis, fixed
   alongside it. The last *column* always wrapped to column 0. That is
   right for a 360-degree LiDAR range image but wrong for a camera, where
   column 0 is the far edge of the image. The wrapped normal had the
   wrong sign, or was lost at the smoothness check. Camera mode now uses
   the previous column with the same sign flip, and LiDAR mode still
   wraps. A 1-pixel-wide or 1-pixel-tall image now returns all-zero
   normals instead of reading out of bounds.

   Tests in `voxfield_ros/test/test_np_tsdf_server.cc`:
   - `PlaneNormalsIncludingLastRowAndColumn`: a fronto-parallel plane
     must get a `+z` normal at every pixel, including the last row and
     column. Against the old code, 12 of the 13 last-row/last-column
     pixels fail.
2. **FIXED.** `NpTsdfServer::projectPointToImageCamera()` returned `bool`
   (in the image or not), but its result was assigned to a `float depth`.
   So `depth` for camera (non-LIDAR) sensors was always `0.0` or `1.0`:
   - the `depth > min_d` filter passed every in-image point;
   - "keep nearest point per pixel" kept the *first* point instead;
   - `computeNormalImage()`'s depth-discontinuity check
     (`|d_n - d_p| > smooth_thre_ratio * d_p`) never fired, so normals
     were computed across depth edges.

   This affected RGB-D datasets (Cow & Lady, Vicon) through
   `voxfield_server`/`np_tsdf_server` (Plan §8.14).

   The function now returns `float`: the point's range (Euclidean norm,
   the same convention as its sibling `projectPointToImageLiDAR()`), or
   `-1` if the point doesn't project into the image, which the caller's
   `depth > min_d` always rejects. Points behind the camera (`z <= 0`),
   on the image plane, and non-finite points are also rejected. Upstream
   pushed them through the pinhole model anyway: into the mirrored pixel,
   or through an undefined float-to-int conversion. The in-bounds check
   now runs on the float before rounding, for the same reason.

   Nothing downstream relied on the depths being 0/1.
   `extractPointCloud()`, `extractNormals()` and `extractColors()` use
   `depth_image > 0` only as a touched/untouched mask; the 3D point comes
   from `vertex_map`. `computeNormalImage()` now gets the real depths its
   smoothness check was written for.

   Behavior change on camera data:
   - `min_dist` now takes effect (default 0.1 m range).
   - Normals are no longer computed across depth edges, using the
     default `smooth_thre_ratio: 1.0` or a stricter configured value.

   Tests in `test_np_tsdf_server.cc`:
   - `ProjectionReturnsRangeAndRejectsInvalidPoints`
   - `NearestPointPerPixelWins`: two points on the same ray, in both
     orders; the nearer one must win in `depth_image`, `vertex_map` and
     the extracted cloud.
   - `PointsBehindCameraAreDropped`
   - `NoNormalAcrossDepthDiscontinuity`

   All four fail against the old code.
3. **FIXED.** Camera intrinsics `fx_`/`fy_`/`vx_`/`vy_` were `int`,
   truncating non-integer calibrations (Plan §8.14). A real camera's
   `fx = 451.51` became 451, and a half-pixel principal point such as
   `vx = 3.5` was rejected outright by the D6 param helper ("expected an
   integer"), falling back to 0. `vy_`/`fy_` also had no initializer
   (the same missing-param undefined behavior as #4).

   All four are now `float = 0.0f` (`np_tsdf_server.h`). `float` matches
   the projection math, which is all in `Point`'s `float`. They are
   declared and used only in `NpTsdfServer`: no subclass shadows them,
   and nothing else reads them. The `param()` calls needed no change.
   YAML integers (the shipped `cow_calib.yaml`/`vicon_calib.yaml` write
   `fx: 580`) still coerce through D6's integer-to-float path.

   Tests in `test_np_tsdf_server.cc`:
   - `NonIntegerIntrinsicsAreNotTruncated`: `fx = 200.6`, `vx = 100.6`
     must project `(1, 1, 1)` to column 301, not the truncated 300. The
     same check covers `v`.
   - `IntegerIntrinsicsStillAccepted`: integer-typed parameters still
     work.

   `NonIntegerIntrinsicsAreNotTruncated` fails against the old code.
4. `width_`/`height_`/`vx_`/`fx_` were read via
   `nh_private.param("x", member_, member_)` with no prior initializer --
   an indeterminate-value (UB) read if the ROS param is absent. Now
   initialized to `0` with an `RCLCPP_ERROR` if `width_ <= 0 ||
   height_ <= 0` after param loading; the missing-param *behavior*
   (garbage range-image preprocessing) is unchanged, only the UB and the
   silent failure mode are gone. (Phase 6 notes above.)
5. `timing_` (`TsdfServer`, `NpTsdfServer`) and `publish_robot_model_`
   (`TsdfServer`) have the same "read own value as own default with no
   prior initializer" UB as #4, just not called out in the plan. Same
   fix applied (explicit `= false` in-class initializer); observed
   effective default (`false`) unchanged. (Phase 6 notes above.)
6. **REMOVED.** `VoxfieldServer::generateEsdfCallback()` /
   `VoxbloxServer::generateEsdfCallback()` were declared (and, for
   `VoxbloxServer`, defined) with a `generate_esdf_srv_` member, but
   neither was ever bound to a ROS service in `setupRos()`. They were
   unreachable via ROS in both the ROS 1 original and here. In ROS 1,
   `VoxfieldServer`'s definition is commented out, so it was only a
   dangling declaration there. (Phase 7 notes above.)

   Deleted:
   - both declarations and their port comments (`voxfield_server.h`,
     `voxblox_server.h`);
   - `VoxbloxServer`'s definition (`voxblox_server.cc`);
   - the unused `generate_esdf_srv_` member from all four headers that
     carried it: `voxfield_server.h`, `voxblox_server.h`, and also
     `fiesta_server.h` and `voxedt_server.h`, which declared it too but
     never had a callback.

   The `std_srvs/srv/empty.hpp` include stays, since the base classes'
   `~/generate_mesh`/`~/clear_map`/etc. services still use it. The
   upstream Voxblox doc page `docs/pages/The-Voxblox-Node.rst` listed a
   `generate_esdf` service that no server in this repo has ever offered;
   that entry is removed too. A repo-wide grep finds no remaining
   references, and the full workspace rebuilds with 0 errors.
7. **FIXED.** `cfg/calib/euroc_calib.yaml` had a `T_B_D::` (double colon)
   typo. Both ROS 1's `rosparam` and PyYAML parse it as a literal key
   `"T_B_D:"`, so `T_B_D` was never readable under that name in ROS 1
   either. It is not used by any in-scope (non-`bak/`) launch file.
   (Phase 9 notes above.)

   The key is now `T_B_D`, edited in place in the already-converted ROS 2
   file. The values are untouched, and they form a valid rigid transform.
   New test `ParamUtils.EurocCalibTransformsLoadFromYaml` in
   `test_param_utils.cc` loads the real file via `--params-file` and
   reads both `T_B_C` and `T_B_D` through `getTransformationParam()`.
8. `launch/eval/eval_cow.launch` and `eval_euroc.launch` each loaded a
   `cfg/{cow,euroc}_dataset.yaml` that has never existed in this
   repository -- already non-functional in ROS 1. The dead reference is
   dropped (rather than ported byte-for-byte) in the `.launch.py`
   equivalents, since in ROS 2 it would abort the whole launch rather
   than just failing to set a few params. (Phase 9/10 notes above.)
9. `launch/voxfield_launch/basement_voxfield.launch` and
   `launch/voxblox_launch/basement_voxblox.launch` never set
   `use_sim_time` at all (every other dataset's launch file does) --
   `mapping.launch.py`'s `basement_*.launch.py` wrappers default it to
   `true` regardless, since ROS 2 requires it on every node. (Phase 10
   notes above.)
10. `process_every_nth_frame` was declared as a launch argument in nearly
    every ROS 1 `.launch` file but never read by any node -- a dead
    argument, dropped rather than ported. (Phase 10 notes above.)
11. `VoxfieldServer::saveMap()` has its TSDF-layer save commented out:
    ```cpp
    bool VoxfieldServer::saveMap(const std::string& file_path) {
      // Output TSDF map first, then ESDF.
      // const bool success = NpTsdfServer::saveMap(file_path);
      bool success = true;
      constexpr bool kClearFile = false;
      return success &&
             io::SaveLayer(esdf_map_->getEsdfLayer(), file_path, kClearFile);
    }
    ```
    present verbatim in the ROS 1 original (`git show main:voxblox_ros/
    src/voxfield_server.cc`), so a `~/save_map`-written file only ever
    contains the ESDF layer, while `loadMap()` (not overridden the same
    way) still loads TSDF first, then ESDF, and fails (logs `Failed to
    load map from ...`) on the TSDF half every time. Found while writing
    the Phase 12 smoke test (below); reproduced deliberately in that
    test's `~/save_map` -> `~/load_map` check rather than worked around.

    **FIXED**, for `VoxbloxServer::saveMap()` too, which had exactly the
    same commented-out `TsdfServer::saveMap()`.

    Why upstream commented it out: `~/save_esdf_map`'s callback also
    called `saveMap()`, and `voxblox_eval` reads that file with
    single-layer `io::LoadLayer<EsdfVoxel>()`, which needs the ESDF layer
    first in the file. Simply uncommenting the TSDF save would have broken
    `~/save_esdf_map` for evaluation. So the two jobs are now separate:
    - `saveMap()` (`~/save_map`) calls the base `saveMap()`, which writes
      the TSDF layer with `SaveLayer`'s default `clear_file = true`
      (truncate/create), then appends the ESDF layer (`kClearFile =
      false`). That is the order `loadMap()` reads back, and the order
      `FiestaServer`'s combined saves already use.
    - A new `saveEsdfMap()` (`~/save_esdf_map`) writes only the ESDF
      layer, now with `kClearFile = true`. Upstream appended here, so
      re-saving to an existing path left the old content first in the
      file, which is where the single-layer reader looks.

    Tests:
    - `test_{voxfield,voxblox}_server_map_io` (new,
      `voxfield_ros/test/test_server_map_io.cc`, built once per server):
      a save/load round trip must restore both layers, and an ESDF-only
      save over an existing combined file must leave an ESDF-first file
      that `io::LoadLayer<EsdfVoxel>` reads.
    - `test_smoke.launch.py`'s voxfield round trip: previously it
      asserted only that `~/load_map` *completed*. It now asserts the
      load *succeeds*: the server logs "Successfully loaded TSDF layer."
      and no "Failed to load map".
    - `README.md`'s services and on-disk format sections are updated to
      match.

    Note for ROS 1 users: a `~/save_map` file written by ROS 1 Voxfield
    contains only the ESDF layer, so `~/load_map` still rejects it,
    correctly. It can still be read as a plain ESDF layer.

    Also fixed (cosmetic, multi-sensor Phase 10): `Layer::isCompatible()`'s
    warning printed the two layer types swapped. It is logged, as
    expected, when the multi-layer loader skips the other layer in a
    combined file, and now reads "loaded map is: tsdf but the current map
    is: esdf" for an ESDF layer skipping the TSDF one.
12. **FIXED as a deliberate deviation from the "document, don't fix"
    rule** (the user authorized it because of how severe it is):
    `EsdfVoxfieldIntegrator::setLocalRange()` (`voxfield/src/integrator/
    esdf_voxfield_integrator.cc`) allocated *every* ESDF block in the
    axis-aligned bounding box of all voxels whose occupancy changed since
    the last ESDF update, plus `local_range_offset_{x,y,z}`, and never
    freed them. The memory it allocated grows with the **cube** of how far
    the updated regions are from each other, so a fast-moving sensor, a
    large localization correction, or a diverging pose estimate allocates
    tens of GB within seconds. This caused the `voxfield_server` OOM
    incidents during real-robot validation (the maze bag's own
    `map -> sensor_init` TF diverges to kilometres between ~24 s and
    ~55 s). Code is identical in the ROS 1 original (the core library is
    unchanged from upstream `78ee640` apart from include order), so this
    is **not** a port regression. Fix: the dense allocation is skipped
    (unless the opt-in, default-off `allocate_tsdf_in_range` is set, which
    needs it), and `updateESDF()` treats a neighbor in an unallocated block
    as an unobserved neighbor. It only ever reads or writes `observed`
    voxels, and every observed voxel's block is already allocated by
    `updateFromTsdfBlocks()`, so the ESDF result does not change. Checked
    bit-for-bit against the upstream code on synthetic RGB-D-like
    sequences: 7 scenarios, from 75k to 1.27M observed voxels, voxel sizes
    0.05/0.1/0.15 m, with and without pose jumps or drift. Every observed
    voxel's `distance`/`raw_distance`/`coc_idx`/`behind`/`fixed` matched.
    ESDF blocks dropped 2.4-8.4x (the ESDF block count now equals the TSDF
    block count). New regression test: `voxfield/test/
    test_esdf_voxfield_integrator.cc`. Live on the maze bag: RSS peaked at
    139 MB instead of 20-25 GB. Details:
    `docs/BUG_voxfield_server_camera_mode_memory.md`.

13. **FIXED** (found in multi-sensor Phase 9 step 7, user-authorized):
    `EsdfOccFiestaIntegrator::setLocalRange()` and
    `EsdfOccEdtIntegrator::setLocalRange()` (`voxfield/src/integrator/
    esdf_occ_{fiesta,edt}_integrator.cc`) had the same dense allocation as
    #12. They allocated every ESDF block in the bounding box of all voxels
    whose occupancy changed since the last update, plus
    `local_range_offset_{x,y,z}`, and never freed them. On the Athena bag
    (two 360-degree Livox LiDARs, 30 m rays pitched up to 53 degrees,
    0.1 m voxels), the first ESDF update's box spans ~38 x 38 x 25 blocks
    of ~0.4 MB each. `fiesta_server` and `voxedt_server` reached 16 GB RSS
    within the first 20 s, and the run's 18 GB watchdog killed both. TSDF
    layer memory was only ~0.77 GB. Code is identical in the ROS 1
    original, so this is **not** a port regression.

    Fix, the same as #12: the dense loop is skipped (unless the new
    default-off `allocate_dense_local_range` config flag is set, kept only
    for the regression test). The neighbor lookups in `updateESDF()` (FIESTA)
    and `processRaise()`/`processLower()` (EDT) treat a neighbor in an
    unallocated block as unobserved. This doesn't change the result. Both
    algorithms only read or write `observed` neighbors, and a voxel only
    becomes observed in `updateFromOccBlocks()`, which allocates its block.
    The insert/delete-list voxels are observed occupancy voxels in blocks
    that `OccTsdfIntegrator` just flagged as updated, so their ESDF blocks
    are allocated too, and their `CHECK_NOTNULL`s stay.

    Test: `voxfield/test/test_esdf_occ_integrators.cc` (typed over both
    integrators) runs the real TSDF -> `OccTsdfIntegrator` -> ESDF pipeline
    twice, dense (upstream) and sparse (fixed). It uses 4 incremental frames
    with surface insertions, occupied-to-free deletions and 10 m jumps, and
    requires every observed voxel's `distance`/`coc_idx`/`behind`/`self_idx`
    to match bit-for-bit, and the ESDF block count to equal the occupancy
    block count (upstream allocated >10x more). Real bag: see
    `docs/MULTI_SENSOR_NOTES.md` Phase 9 step 7.

14. **FIXED** (fixed in multi-sensor Phase 10, user-authorized):
    `RayCaster::setupRayCaster()` (`voxfield/src/integrator/
    integrator_utils.cc`) guarded each axis with
    `std::abs(ray_scaled.x()) < 0.0 ? 2.0 : ...`. An absolute value is never
    negative, so the guard never fired. For a ray with an exactly-zero
    direction component, the time to the next boundary on that axis became
    `distance / 0` and its step `0 / 0 = NaN`:
    - If the ray started inside a voxel, that axis's time was `-inf`, so
      `nextRayIndex()` chose it once, "stepping" with sign 0 (the same voxel
      again). After that the time was `NaN`, and `minCoeff()` stopped
      choosing it. The step count is fixed, so each zero axis cost one
      duplicated voxel at the start and one **missing voxel at the end** of
      the ray, i.e. the far end of the truncation band behind the surface.
    - If the ray started exactly on a voxel boundary, it got `0/0 = NaN`
      straight away and was traversed correctly by luck.

    Rays with an exactly-zero component are rare in real data but common in
    synthetic scenes: an axis-aligned room seen by a sensor at yaw 0, and
    image rows or columns through the optical center. The code is identical
    in the ROS 1 original (and in voxblox), so this is **not** a port
    regression.

    Fix: test `ray_scaled.c() == 0` instead. `signum()` returns 0 exactly
    then, so a zero step sign always gets the `2.0` sentinel (beyond the
    ray's t in [0, 1], never chosen), which was clearly the intent. For any
    non-zero component the old and new expressions are identical, so only
    rays with an exactly-zero component change.

    Tests: `voxfield/test/test_ray_caster.cc` checks that axis-aligned rays
    (from inside a voxel and from a boundary, both directions), in-plane
    rays and general rays follow a valid 3D-DDA path from the start voxel to
    the end voxel, one unit step at a time. It also checks that
    `MergedTsdfIntegrator` integrates an axis-aligned ray from an off-grid
    sensor through the whole truncation band. The 4 zero-component tests
    fail on the old code, and the general-ray test passes on both.
    `test_legacy_golden_tsdf`'s box-room scene has such rays: 617 voxel
    values changed (same blocks), and its golden file was regenerated
    (`VOXFIELD_WRITE_GOLDEN=1`). The NP golden, from the projective
    integrator, is unchanged.

    Found via multi-sensor Phase 7, whose "a point at (x, 0, 0) only
    allocates the origin block" symptom was mostly a different effect:
    `TsdfIntegratorBase::getVoxelWeight()`'s `1 / z^2` depth-camera weight
    is 0 for a point with sensor-frame z = 0, so the point is skipped
    entirely.

15. **FIXED** (multi-sensor Phase 10, user's choice among three options):
    `TsdfIntegratorBase::getVoxelWeight()` weighted every point by
    `1 / z^2` in the sensor frame (when `use_const_weight` is false). That
    is a depth-camera noise model, where z is depth. For a LiDAR, z is
    height: points near its horizontal plane got huge weights (10000 at
    z = 1 cm), and points exactly level with it got weight 0 and were
    dropped before any ray was cast. Every LiDAR preset here (KITTI,
    MaiCity, basement, Athena) runs with `use_const_weight: false`, so this
    hit the `tsdf`, `voxblox`, `fiesta`, `voxedt` and `intensity` servers
    on LiDAR data. The NP integrator (`np_tsdf`, `voxfield`) already used
    `1 / range^weight_reduction_exp`. The code is identical in the ROS 1
    original and in voxblox.

    Fix: `TsdfIntegratorBase::Config` gains `sensor_is_lidar` and
    `weight_reduction_exp`, read from the existing parameters of the same
    name, which are also per-sensor overridable. A LiDAR now uses
    `1 / range^weight_reduction_exp`, the NP integrator's model, so
    voxblox-vs-voxfield comparisons differ only in algorithm. Cameras keep
    `1 / z^2`. The new map-global `lidar_z_weighting` (default false)
    restores the old LiDAR behavior, for reproducing earlier results.
    Configs without `sensor_is_lidar` (it defaults to false) are unchanged,
    including the legacy golden tests.

    Tests: `voxfield/test/test_point_weight.cc` integrates single points and
    checks the resulting voxel weight for a LiDAR (including z = 0, which
    was dropped before, and z = 1 cm, which got 10000), a configurable
    exponent, a camera (unchanged 1 / z^2), and `lidar_z_weighting` (which
    reproduces the old values, including the dropped z = 0 point).
    `test_sensor_config`'s new cases check that the parameters reach each
    sensor's TSDF config and that `lidar_z_weighting` is map-global.
    Real-bag effect: `docs/MULTI_SENSOR_NOTES.md` Phase 10.

16. **FIXED** (found in the multi-sensor end-to-end RViz2 runs):
    `TsdfServer`/`NpTsdfServer` read `publish_robot_model` but never
    checked it, identically in ROS 1, so the `~/Robot_model` marker was
    published for every cloud of the primary sensor regardless. With no
    `robot_model_file`, RViz2 logged `Could not load resource [file://]`
    for each marker. The marker is now published only when
    `publish_robot_model` is true. All dataset presets set it true, so
    they are unchanged. Test: `RobotModelMarker.*` in
    `test_multi_sensor_server.cc`, built for both servers; the disabled
    case fails without the fix.

### `rcl_yaml_param_parser` gotcha found while writing the smoke test
Multiple `--params-file` arguments for the same node merge with later
files overriding earlier ones for a given parameter -- *except* when the
same key's YAML-inferred type differs between files, in which case the
later override is silently dropped and the earlier value wins. Confirmed
empirically (not documented anywhere obvious): `update_esdf_every_n_sec:
0` (int) in `kitti_param.yaml`, overridden with `1.0` (double) via a
third `--params-file`, silently kept `0` -- switching the override to the
int `1` fixed it. Not a port bug (this is `rcl_yaml_param_parser`/rclcpp
core behavior, unrelated to D6's coercion, which never even sees a
dropped override), but worth knowing for anyone else layering parameter
overrides via `mapping.launch.py` or `ros2 launch ... --params-file`.

## Phase 12: End-to-end validation

### Step 1: dataset-free automated smoke test

Wrote `scripts/fake_sensor_publisher.py`: an rclpy node publishing a
synthetic, organized (`height`x`width`, matching `kitti_calib.yaml`'s
`fov_up`/`fov_down`/`width`/`height`) `sensor_msgs/PointCloud2` of a
closed, airtight, axis-aligned box room (default 10x10x3 m), ray-cast
from a sensor circling near the room's center -- every ray is guaranteed
to hit a wall/floor/ceiling (`is_dense: true`, verified: no NaNs, point
count == `height*width` every frame), so both the projective
(NpTsdfServer/VoxfieldServer) and non-projective
(TsdfServer/VoxbloxServer/FiestaServer/VoxedtServer) integration paths
get real geometry. Broadcasts a moving `world -> velodyne` TF at the same
10 Hz rate.

Wrote `voxfield_ros/test/test_smoke.launch.py` (one file, `method` read
from `sys.argv`, registered 5x via `add_launch_test(... ARGS
"method:=<name>")` in `CMakeLists.txt` -- one per server). Starts the
fake publisher + `<method>_server` against `kitti_param.yaml`/
`kitti_calib.yaml`, `use_sim_time:=false`,
`update_esdf_every_n_sec` overridden to the int `1` (see the
`rcl_yaml_param_parser` gotcha above -- this is exactly where it was
found: the override silently failed as `1.0` first). Every method
asserts `~/mesh` receives a message with a non-empty mesh block within
30s; `method=voxfield` additionally asserts `~/tsdf_slice`/`~/esdf_slice`
produce a non-empty cloud and exercises `~/save_map` -> `~/load_map`
(the load is expected to fail -- see known-upstream-issue #11 above,
found by this very test).

**Environment note:** this sandbox's normal shell has `~/miniconda3/bin`
ahead of `/usr/bin` on `PATH` (the same issue as the Phase 3/4
`rosidl`/miniconda note), which breaks `rclpy` entirely (`No module named
'rclpy._rclpy_pybind11'`) when `python3` resolves to miniconda's build.
Two independent fixes were needed: (1) the smoke test always launches the
fake publisher via the absolute path `/usr/bin/python3`, never a bare
`python3`; (2) `colcon build`/`colcon test` themselves must run via
`scripts/clean_env.sh` for this specific test, since `add_launch_test`'s
generated CTest command bakes in whatever `python3` CMake's Python
detection found *at configure time* -- configuring in the normal shell
would bake in miniconda's broken interpreter for the whole launch-test
invocation, not just the fake-publisher subprocess.

**Verified:** all 5 `test_smoke_<method>` launch tests pass (`colcon test
--packages-select voxfield_ros --ctest-args -R test_smoke_`, run via
`scripts/clean_env.sh`). Full-workspace `colcon test`: 80 tests, 0
errors, 0 failures, 4 skipped (the 4 non-`voxfield` methods'
slice/save/load subtest, intentionally skipped -- mesh-only per the plan).
*(Later corrected: skipping these was a coverage gap, not a design choice.
They hid a `fiesta`/`voxedt` save/load bug. The subtest now runs for every
method with 0 skips; see `docs/MULTI_SENSOR_NOTES.md` Phase 9, "Slice/save/load
smoke coverage".)*

### Step 2: unit tests

`colcon test` / `colcon test-result --verbose`: 80 tests, 0 errors, 0
failures, 4 intentionally skipped (above; later un-skipped). Re-confirmed in both the clean
env and the user's normal shell with the Hector underlay sourced (step 3
below covers the latter).

### Step 3: coexistence check

Full clean rebuild (`rm -rf build install log`, then `colcon build
--symlink-install`) and `colcon test` in an isolated shell sourcing
`/opt/ros/jazzy` **and** the Hector underlay (not `scripts/clean_env.sh`,
which deliberately excludes Hector -- a one-off variant of it for this
check): 0 build errors, 80/80 tests still passing, including all 5 smoke
tests. `rviz2` loaded `cfg/rviz/kitti_25cm.rviz` (which references
`voxfield_rviz_plugin/VoxfieldMesh`) with no errors in this same
Hector-sourced environment, consistent with the Phase 8/10 coexistence
checks.

### Step 5: TF-queue mode

`scripts/fake_sensor_publisher.py` gained a `publish_tf` parameter
(default `true`): set to `false`, it publishes a
`geometry_msgs/TransformStamped` on `transform_topic` (default
`transform`, matching `Transformer`'s subscription -- see
`transformer.cc`) instead of broadcasting TF, and -- so the queue
actually has something to interpolate between -- publishes the point
cloud at half the transform rate, guaranteeing every cloud's stamp falls
strictly between two queued transforms.

Verified manually (not added as a 6th permanent smoke-test target, since
the plan frames Phase 12 step 5 as a one-off exercise of the path, not a
regression gate): `voxblox_server` with `cow_param.yaml`/`cow_calib.yaml`
(`use_tf_transforms: false`, the dataset the plan names for this check)
against the fake publisher in `publish_tf:=false` mode. `ros2 node info`
confirmed the `/transform` subscription
(`geometry_msgs/msg/TransformStamped`); the log showed exactly two
`[WARN] No match found for transform timestamp` lines right at startup
(queue empty, then queue not yet full) and *no further occurrences* as
the queue filled -- i.e. `Transformer::lookupTransformQueue`'s
interpolation path ran and worked, not spammed. Mesh/ESDF integration
continued normally throughout.

### Step 4: dataset run, against a real ROS 2 bag (not KITTI/MaiCity)

No ROS 1 KITTI/MaiCity/Cow-and-Lady bag was available in this
environment. The user instead provided a **native ROS 2 bag** (mcap,
`rosbag2_2026_09_23-12_40_03/`, 4.1 GiB, 138.9 s) recorded from a real
mobile-manipulator robot ("Athena"), carrying `/athena/front_lidar/
points_raw_livox` and `/athena/back_lidar/points_raw_livox`
(`sensor_msgs/msg/PointCloud2` from a Livox lidar) plus a full URDF-
derived TF tree on `/athena/tf`/`/athena/tf_static` with a real
`map -> odom -> ... -> front_lidar_laser_frame` chain. This needed no
`rosbags-convert` step (already ROS 2), but did need a bespoke param
file (`world_frame: map`, `sensor_frame: front_lidar_laser_frame`,
`voxel_size: 0.1`, ICP disabled -- not committed to the repo, since it's
specific to this local bag, not a shipped dataset preset) and bag-side
topic remaps (`ros2 bag play --remap /athena/front_lidar/
points_raw_livox:=/pointcloud /athena/tf:=/tf /athena/tf_static:=/tf_static`)
so the server's un-namespaced subscriptions and `tf2_ros::TransformListener`
(which listens on the global `/tf`/`/tf_static`, not a namespaced variant)
lined up without any code changes.

A Livox point cloud isn't the organized, fixed-width x height range image
`NpTsdfServer`/`VoxfieldServer`'s projective integration expects (per
plan §7 -- solid-state/non-repetitive-scan lidars don't produce that
shape), so this run exercised the three **non-projective, ray-casting**
`TsdfServer` subclasses instead, per the user's request: `voxblox_server`,
`fiesta_server`, `voxedt_server`. All three, run against ~25s of the bag:

- Built a real mesh (`~/generate_mesh` + `mesh_filename` -> a non-empty
  ASCII PLY each time): `voxblox_server` 27550 vertices/23840 faces,
  `fiesta_server` 27318/23667, `voxedt_server` 27335/23694 -- all three
  independently reconstructing essentially the same real scene geometry,
  as expected.
- `timing: true` output present throughout (200+ `SM Timing` blocks
  logged per run).
- Exactly one transient `Invalid frame ID "..." passed to canTransform`
  warning per run, in the first ~3s before the TF tree was fully
  populated from `/tf_static` -- not ongoing spam, and it stopped once
  the tree filled in.
- Visually confirmed in `rviz2` for `fiesta_server`: launched the bag +
  server + `rviz2` (with a `voxfield_rviz_plugin/VoxfieldMesh` display on
  `~/mesh`) together and screenshotted the live result -- real
  reconstructed room/warehouse structure, not a blank or broken display.
  Screenshot: `docs/assets/phase12_fiesta_mesh_rviz.png`.

`voxfield_server`/`np_tsdf_server` were **not** exercised against this
bag (out of scope for this run, per the point above); they remain
validated only via the Phase 12 step 1 synthetic smoke test and the
Phase 6/7 manual checks. A genuine KITTI/MaiCity/Cow-and-Lady run, which
would exercise the projective path against real (rather than synthetic)
organized/RGB-D data, is still pending a suitable ROS 1 bag -- see the
Definition of Done note below.

**Accept (plan §6 Phase 12):** steps 1, 2, 3, and 5 fully met. Step 4 met
for the three ray-casting methods against real robot data (not the
plan's suggested KITTI/MaiCity), with the projective-path/KITTI-or-MaiCity
portion still pending external data -- reported here explicitly per the
plan's own fallback ("stop here, report that, and tell the user exactly
which commands to run"). To run it once a bag is available:
`rosbags-convert --src <bag>.bag --dst <bag>_ros2`, then `ros2 launch
voxfield_ros kitti_voxfield.launch.py bag_file:=<bag>_ros2` (or
`mai_voxfield.launch.py` / `cow_voxfield.launch.py`).
