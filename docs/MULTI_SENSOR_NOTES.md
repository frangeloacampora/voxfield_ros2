# Multi-Sensor Fusion Notes

Decisions, measurements, and deviations accumulate here as
`MULTI_SENSOR_PLAN.md` proceeds. Sections are added phase by phase.

## Phase 0: Branch and baseline

Branch `multi-sensor` created off `ros2-port` at commit `335553d` (`Fix real
upstream bugs, remove dead code from known-issues list`).

`colcon build --symlink-install` in `~/voxfield_ws`: 4 packages finished
(`voxfield`, `voxfield_msgs`, `voxfield_rviz_plugin`, `voxfield_ros`), no
errors. Build was effectively a no-op (0.3 s wall) since the tree is
unchanged from the last Phase 12 build.

`colcon test --packages-select voxfield voxfield_msgs voxfield_rviz_plugin
voxfield_ros` + `colcon test-result --verbose`:

```
Summary: 100 tests, 0 errors, 0 failures, 4 skipped
```

The 4 skips are pre-existing and expected, per `docs/ROS2_PORT_NOTES.md`
(~line 1279): the 4 non-`voxfield` methods' (`fiesta`, `np_tsdf`, `voxblox`,
`voxedt`) slice/save/load subtest is intentionally skipped, mesh-only per the
port plan; `test_smoke_voxfield` has 0 skips. This is the baseline every
later phase is compared against — any regression here must be explained
before continuing.

## Phase 1: Golden regression tests for legacy mode

`test/test_legacy_golden.cc` pins today's (unmodified) `TsdfServer` and
`NpTsdfServer` TSDF integration output bit-for-bit, so Phases 2–6 can prove
"no behavior change in legacy mode" by re-running against the same golden
files. `TsdfServer`/`NpTsdfServer` can't share a translation unit (both
define `voxfield::kDefaultMaxIntensity`), so CMake builds the one source
file twice — `test_legacy_golden_tsdf` and `test_legacy_golden_np_tsdf` —
exactly like `test_server_map_io.cc`'s `TEST_VOXBLOX_SERVER` split.

**Scenario:** a `TestServer` subclass (`using TsdfServer::transformer_;`)
exposes the protected `Transformer` member so the test can inject synthetic
poses directly via the public `transformCallback()`, with
`use_tf_transforms: false` — this sidesteps TF-broadcast timing and executor
spinning entirely; no `rclcpp::spin` is needed anywhere in the test.
20 deterministic frames at 10 Hz are fed through a C++ port of
`scripts/fake_sensor_publisher.py`'s box-room raycasting (axis-aligned
10×10×3 m room, sensor orbiting radius 1.5 m / period 20 s, spinning in
place period 30 s — verified byte-for-byte against the Python source, no
randomness in either). For each frame the sensor pose is pushed into the
transform queue at *t*−5 ms and *t*+5 ms (bracketing the cloud's own stamp),
exercising `Transformer::lookupTransformQueue`'s interpolation path, then
`insertPointcloud()` is called with the cloud stamped at *t* — the real
production entry point, not `integratePointcloud()` directly, so PCL
parsing (and, for `NpTsdfServer`, the range-image reprojection) is exercised
too.

`TsdfServer` uses a wider, coarser scan (180×16, ±20° vertical) since it has
no per-pixel reprojection constraint; `NpTsdfServer` uses the plan's LiDAR
model exactly (256×16, `fov_up 3° / fov_down -25°`), matching its internal
range-image resolution so pixels get real neighbors. Both use
`tsdf_voxel_size: 0.25`, `max_ray_length_m: 20.0` (room diagonal from the
farthest orbit point is ~8.3 m), `method: merged`, `integrator_threads: 1`
(removes thread-order nondeterminism), identity `T_B_C`/`T_B_D`, and every
`publish_*`/`update_*_every_n_sec` param off (no timers, no side-channel
publishes).

**Comparison is exact (zero tolerance).** `voxfield::test::LayerTest`'s
`CompareLayers` (`voxfield/test/layer_test_utils.h`) uses `EXPECT_NEAR` with
a fixed `1e-10` absolute tolerance — tight, but not literally exact, and the
plan requires that a refactor may not change a single float. Per the plan's
explicit instruction, a local `CompareLayersExact()` was added in the test
file itself (`EXPECT_EQ` on `distance`/`weight`/color channels) rather than
loosening the shared util.

**Golden files:** `test/test_data/golden_{tsdf,np_tsdf}.tsdf`
(`io::SaveLayer`/`io::LoadLayer<TsdfVoxel>`, ~0.9–1.0 MB each), generated
once against this unmodified Phase 0 code with `VOXFIELD_WRITE_GOLDEN=1`
(the test then calls `saveMap()` and `GTEST_SKIP()`s instead of comparing),
and committed. The path is baked in via a CMake compile definition
(`VOXFIELD_ROS_TEST_DATA_DIR`), the same pattern `test_param_utils` uses for
`VOXFIELD_ROS_CFG_DIR`.

**Sanity check that the comparison actually compares** (not committed, done
manually against the built binaries): (1) flipped one byte ~500 KB into
`golden_tsdf.tsdf` — broke protobuf framing, `io::LoadLayer` itself failed
(`Could not read block protobuf message number 15`), test failed as
expected, though this only proves the file-load path is exercised. (2) A
tighter check: wrote a scratch program
(`voxfield::io::LoadLayer`/mutate one voxel's `weight` by +12345/
`voxfield::io::SaveLayer`) to change exactly one previously-unobserved
voxel's weight from 0 to 12345 and nothing else. Running the golden test
against that copy failed precisely at the `CompareLayersExact` value check:
`g.weight Which is: 12345` vs `a.weight Which is: 0`, trace `block
[-1,-2,0] voxel 0` — confirming the per-voxel exact comparison itself (not
just protobuf loading) is what catches a regression. Restored the real
golden file afterward and re-ran both `test_legacy_golden_{tsdf,np_tsdf}` to
confirm they pass again.

**Full suite after Phase 1:** `colcon test-result --verbose` →
`Summary: 104 tests, 0 errors, 0 failures, 4 skipped` (100 baseline + 2 new
gtest binaries, each counted once in the per-file breakdown and once in
CTest's own `Testing/Test.xml` aggregate — consistent with how the Phase 0
baseline was already being counted, not a new double-count). The same 4
pre-existing skips, unaffected.

## Phase 2: Pure refactor — extract `RangeImageProjector`, add string-array params

No behavior change; `test_legacy_golden_{tsdf,np_tsdf}` stay exact
throughout, and `git status` after this phase touches only `voxfield_ros`
C++/CMake/test files — no `.yaml`, launch file, or core-library (`voxfield/`)
change.

**`RangeImageProjector`** (`include/voxfield_ros/range_image_projector.h` +
`src/range_image_projector.cc`, new): `projectPointCloudToImage`,
`projectPointToImageLiDAR`, `projectPointToImageCamera`,
`computeNormalImage`, `extractPointCloud`, `extractNormals`, and
`extractColors` moved out of `NpTsdfServer` verbatim (member fields like
`width_`/`sensor_is_lidar_` became `config_.width`/`config_.sensor_is_lidar`;
every ROS2_PORT-deviation comment moved with its code). Added a `process()`
convenience method matching the inline pipeline that used to live in
`NpTsdfServer::processPointCloudMessageAndInsert()`, and a `Config::isValid()`
per M8's contract (width/height positive; camera needs `fx,fy > 0`; LiDAR
needs `fov_up != fov_down`) — unused until Phase 4's fail-fast validation,
but part of the class's contract from the start per the plan.

**`NpTsdfServer`** now owns one `std::unique_ptr<RangeImageProjector>
projector_` (still a single member at this stage — the `sensors_` vector
doesn't exist until Phase 4/5), built at the end of
`getServerConfigFromRosParam()` from the same params it already read.
`processPointCloudMessageAndInsert()`'s inline range-image block collapsed
to one `projector_->process(...)` call. The seven public methods
(`projectPointCloudToImage()` etc.) are now one-line wrappers delegating to
`projector_`, signatures byte-for-byte unchanged — `test_np_tsdf_server.cc`
required **zero** edits and still passes.

**`param_utils.h`:** added a `std::vector<std::string>` branch to
`getParam<T>()` (accepts `PARAMETER_STRING_ARRAY`, falls back to the default
on `PARAMETER_NOT_SET` or a type mismatch, matching every other branch's
pattern) and `listParameterOverrides(node, prefix)` (linear scan of
`get_parameter_overrides()` for keys starting with `prefix`) — both needed
by Phase 4's `sensor_names`/`sensors.<name>.*` parsing, added now per the
plan. 5 new `test_param_utils.cc` cases cover both.

**`test/test_range_image_projector.cc`** (new): the 8 `test_np_tsdf_server.cc`
camera-model cases ported 1:1 to call `RangeImageProjector` directly (no ROS
node needed — the class has no ROS dependency, so this target links the
default `gtest_main`, unlike every other test in this package), plus a new
LiDAR cylinder round-trip case. First attempt at the round-trip case
generated points via a plain `linspace`-style row/col → elevation/azimuth
mapping (like `scripts/fake_sensor_publisher.py`'s, a *different* generator)
and failed: `projectPointToImageLiDAR`'s actual `yaw`/`pitch` →
`proj_x`/`proj_y` formula has a different pixel-to-angle convention (e.g.
azimuth 0 lands at column `width/2`, not column 0). Fixed by generating
points from the exact algebraic inverse of that formula instead
(`yaw = π·(2·col/width − 1)`, `pitch = fov_down_rad + fov_rad·(1 − row/height)`),
which now round-trips every point in the test to <1e-5 of its original
position.

**Full suite after Phase 2:** `Summary: 119 tests, 0 errors, 0 failures, 4
skipped` (104 + 9 new `test_range_image_projector` cases + 1 new CTest
aggregate entry + 5 new `test_param_utils` cases). Same 4 pre-existing
skips.
