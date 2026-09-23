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
