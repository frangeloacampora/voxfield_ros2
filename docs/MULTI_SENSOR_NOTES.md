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
