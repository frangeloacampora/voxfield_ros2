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

The 4 skips are the 4 non-`voxfield` methods' (`fiesta`, `np_tsdf`,
`voxblox`, `voxedt`) slice/save/load subtest in `test_smoke.launch.py`.
`docs/ROS2_PORT_NOTES.md` (~line 1279) called them intentional
("mesh-only per the plan"). **That was wrong:** slice and save/load
coverage matters for every server, not just `voxfield`. The skips hid a real
bug: `fiesta`/`voxedt` `~/save_map` → `~/load_map` always failed. Phase 9
removed the skips and fixed the bug; see "Slice/save/load smoke coverage"
there. From then on the suite has 0 skips. This is the baseline every later
phase is compared against — any regression here must be explained before
continuing.

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

## Phase 3: `Transformer` changes (M5, M6)

**M5 — frame resolution moved out of `Transformer`.** Removed
`Transformer::sensor_frame_` and `lookupTransformTf`'s override (it used to
silently substitute `sensor_frame_` for whatever `from_frame` a caller
passed, if set — so in TF mode every sensor would really be looked up in
one shared frame). `lookupTransform()`'s `from_frame` argument is now used
exactly as given. `TsdfServer`/`NpTsdfServer::getNextPointcloudFromQueue()`
now resolve the frame themselves before calling it:
`sensor_frame_.empty() ? pointcloud_msg->header.frame_id : sensor_frame_` —
this is also **F1**, the deliberate legacy fix: previously an empty
`sensor_frame` meant the lookup used frame `""` and failed forever; now it
falls back to the message's own header frame (upstream voxblox behavior).
`IntensityServer::intensityImageCallback` (A17) got the identical fix,
since it relied on the same override.

`grep -rn sensor_frame voxfield_ros/cfg` confirms F1 changes nothing for
any shipped config: `mai_param.yaml`, `kitti_param.yaml`, and
`basement_param.yaml` all set `sensor_frame` explicitly (non-empty, so F1's
fallback path is never taken); `cow_param.yaml` and `vicon_param.yaml`
don't set it, but both use `use_tf_transforms: false` (queue mode), where
`sensor_frame`/`from_frame` was already ignored entirely (queue-mode
lookups only match on timestamp) — F1 only ever mattered for a TF-mode
config with `sensor_frame` unset, and no shipped config is that.

**M6 — per-sensor extrinsic queue-mode API, retention window, `tfBuffer()`.**
New public `Transformer` API, additive (old `lookupTransform(from, to,
stamp, T)` kept working unchanged — its queue branch now calls
`lookupTransformQueue(timestamp, T_B_C_, transform)` with the global
`T_B_C_`, so legacy output is identical):
- `lookupTransformQueue(stamp, T_B_C, T_G_C)`: same match/interpolate logic
  as before, but `T_D_C = T_B_D_.inverse() * T_B_C` is computed per call
  from a caller-supplied `T_B_C` instead of a ctor-precomputed member
  (`T_D_C_` removed entirely — nothing else used it). `T_B_D` stays global,
  matching the plan's split.
- `lookupSensorTransform(frame, T_B_C_or_null, stamp, T_G_C)`: routes to TF
  or queue mode per `use_tf_transforms_`, taking a single already-resolved
  `frame` (the caller — Phase 5's per-sensor frontend — does the `cfg.frame
  .empty() ? header.frame_id : cfg.frame` resolution, matching M5).
- `tfBuffer()`: test-only accessor to `tf2_ros::Buffer`, so a test can
  inject static/dynamic transforms with `setTransform(...)` instead of
  broadcasting and spinning.
- **A8 fix:** the old `transform_queue_.erase(begin, it)` (erase everything
  before this lookup's own matched/bracketing entry) is replaced with
  "erase entries older than `stamp - transform_queue_retention_sec`" (new
  global param, default `1.0`) plus a 10 000-entry cap (drop oldest). The
  old policy broke with >1 sensor: sensor A's lookup would erase queue
  entries a *second* sensor's earlier-stamped lookup still needed as its
  interpolation bracket. Single-sensor output is unaffected except when two
  consecutive clouds are stamped closer together than
  `timestamp_tolerance_sec` (1 ms default) — negligible, undocumented
  upstream, and not exercised by any shipped config's real sensor rate.

**Sanity check (A8 reproduces on the old policy):** temporarily replaced
the new retention-window code with the literal old
`transform_queue_.erase(transform_queue_.begin(), it)` (the new
`lookupTransformQueue` already has `it` in the right state at that point,
so no other change was needed), rebuilt, and ran
`test_transformer_multi.DifferentSensorsCanLookUpOutOfOrder` alone: it
fails exactly as predicted (`No match found for transform timestamp:
995000000 Queue front: 1000000000` — the 900 ms entry sensor B's
interpolation needed was already gone, erased by sensor A's own lookup at
1000 ms). Reverted immediately after and confirmed the full suite green
again. (A literal "run against the actual Phase 2 `Transformer`" wasn't
possible since the new test uses API — `lookupTransformQueue`'s new
signature, `transformQueueSizeForTest()`, `tfBuffer()` — that class didn't
expose yet; isolating just the erase-policy line is the faithful
equivalent.)

**`test/test_transformer_multi.cc`** (new, 4 cases):
`DifferentSensorsCanLookUpOutOfOrder` (above), `RetentionWindowErasesOnlyOldEntries`
(20 entries 0–1900 ms, lookup at 1900 ms with `retention_sec=1.0` leaves
exactly the 11 entries from 900 ms onward), `SizeCapDropsOldestBeyondTenThousand`
(10 005 entries 1 ms apart within one large retention window, one lookup
caps the queue at exactly 10 000), and `TfModeUsesTheGivenFrameForEachSensor`
(two independent static TF child frames, proving no cross-sensor frame
override remains — this is also the mechanism F1 depends on, so a
separate literal "F1" test wasn't added: `Transformer` itself has no
`sensor_frame` concept left to test in isolation once M5 moved frame
resolution to the caller).

**Accept:** all tests pass; `test_legacy_golden_{tsdf,np_tsdf}` exact; the 5
existing smoke launch tests still pass. `Summary: 124 tests, 0 errors, 0
failures, 4 skipped` (119 + 4 new `test_transformer_multi` cases + 1 new
CTest aggregate entry). `git status` after this phase touches only
`voxfield_ros` C++/CMake/test files.

## Phase 4: Sensor config loading and validation (M2, M13)

**`sensor_input.h`** (new): `SensorConfig` and the `SensorInput<IntegratorBaseT>`
template, exactly as M4 specifies. No ROS-topic wiring yet (`sub`/`integrator`/
`projector` stay unpopulated) — those are Phase 5/6's job; this phase only
adds the struct shapes and the config-building logic that fills
`SensorConfig`.

**`sensor_config_loader.{h,cc}`** (new): `LoadedSensor` (a `SensorConfig`
plus both integrator `Config`s, a `RangeImageProjector::Config`, and
`method` — always all four, regardless of caller; a `TsdfServer` caller
just never reads `np_tsdf`/`projector`) and `loadSensors()`.

- **Legacy mode** (`sensor_names` not present — checked the same way
  `getTransformationParam` checks presence, `has_parameter() ||
  overrides.count() > 0`, *not* by declaring the param with an empty-vector
  default: an empty-vector default is itself a real `PARAMETER_STRING_ARRAY`
  value, not `PARAMETER_NOT_SET`, so it can't distinguish "unset" from "set
  to `[]`" — matches M1's requirement): returns exactly the caller-supplied
  `legacy_input` and base configs, unmodified, reading **no** `sensors.*`
  key at all. A stray `sensors.*` override while `sensor_names` is unset
  logs `RCLCPP_WARN` and is otherwise ignored.
- **Multi-sensor mode:** each sensor's `SensorConfig` starts as a **copy of
  `legacy_input`** (giving it inherited `queue_size`/`input_qos_best_effort`/
  `min_time_between_msgs_sec` for free, since those are exactly the
  top-level-resolved values `legacy_input` already carries), then
  `name`/`topic`/`freespace_topic`/`frame`/`has_T_B_C` are reset — `frame`
  deliberately does **not** inherit `legacy_input.frame` (which would be
  the legacy `sensor_frame` value), since that would force every sensor
  into one frame, defeating the point (M2). Integrator/projector configs
  copy from `tsdf_base`/`np_base`/`projector_base` (or default-construct if
  null) the same way.
- **Whitelist** (§7.1) is a fixed set of ~30 key names split into four
  categories in the `.cc`'s anonymous namespace: `SensorConfig` fields,
  integrator fields shared by both `TsdfIntegratorBase::Config` and
  `NpTsdfIntegratorBase::Config` (same name/type/units in both, verified by
  reading `voxfield/integrator/{tsdf,np_tsdf}_integrator.h`'s `Config`
  structs directly — they're independent structs, not a shared base class,
  but every §7.1 "both" key matches field-for-field), NP-integrator-only
  fields, and `RangeImageProjector::Config`-only fields. `anti_grazing` is
  the one key/field-name mismatch (`enable_anti_grazing`), matching
  `ros_params.h`'s own read. Each present override is read into **both**
  `sensor.tsdf` and `sensor.np_tsdf` for the shared category (harmless for
  whichever struct the calling server ignores) — this also means the
  whitelist/forbidden-key check doesn't need to know which server is
  calling; only the NP-only categories and the `RangeImageProjector::isValid()`
  check are gated on `np_base != nullptr` (a `TsdfServer` multi-sensor
  config never sets `width`/`height`, so it must not be validated as if it
  were an NP camera/LiDAR model).
- Only keys **actually present** under `sensors.<name>.` are read
  (`hasOverride()` checks the raw override map before calling `param()`),
  per Phase 4 step 2 — avoids declaring the full ~30-key set per sensor.
- **M13 validation**, collecting every problem before throwing (not
  stopping at the first): sensor name syntax/uniqueness; every
  `sensors.*` override key scanned via `listParameterOverrides(node,
  "sensors.")` and classified as unknown-sensor / forbidden (exact-name set
  plus an `esdf_`/`occ_`/`mesh_`/`publish_`/`update_` prefix check,
  covering the "every `esdf_*`/... key" part of §7.2 without enumerating
  each one) / unknown-key / valid; missing or duplicate `topic`; and, for
  NP servers only, `RangeImageProjector::Config::isValid()` per sensor. A
  non-empty error list is joined (`"N problem(s): \n  - ...\n  - ..."`)
  into one `std::invalid_argument`. `use_tf_transforms` (map-global) is
  read once to gate the M13.6 duplicate-`T_B_C` check (queue mode only):
  warns if 2+ sensors have no per-sensor `T_B_C` override (all silently
  inheriting the same global one — Transformer's `lookupSensorTransform()`
  fallback, from Phase 3), and separately if any two *explicit* overrides
  are numerically identical.
- On success, logs one `RCLCPP_INFO` line per sensor (name, topic,
  resolved frame or `<header>`, effective ray-length range; NP servers also
  log the projection model).

**`test/test_sensor_config.cc`** (new, 12 cases): legacy passthrough
(including a field-by-field check against a freshly-read top-level config,
to catch a loader that forgets to copy a field rather than just testing
compiler-generated copy semantics) and the stray-override warning;
multi-sensor inheritance (`max_ray_length_m: 45` top-level, sensor B
overriding to `5` → A stays 45, B is 5) and the frame-non-inheritance case;
one test per M13 error case named in the plan (unknown key, unknown
sensor, forbidden key, missing topic, duplicate topic, duplicate name,
invalid NP projector config) plus a combined "several errors → message
lists both" case.

**Accept:** all tests pass; `test_legacy_golden` exact. `Summary: 137
tests, 0 errors, 0 failures, 4 skipped` (124 + 12 new `test_sensor_config`
cases + 1 new CTest aggregate entry). One transient flake was observed
(`test_transformer_multi` failed to produce a result file when run under
the full `colcon test` alongside `voxfield`'s ~35s build; re-running it
standalone and via `--ctest-args -R test_transformer_multi` both passed
immediately, and a full re-run of the whole suite came back clean at 137/0/0/4
— treated as ctest-level resource contention under load, not a code
regression, since nothing in this phase touches `Transformer`. *(Later
corrected: this was a real, intermittent deadlock in `Transformer`'s
`tf2_ros::TransformListener` shutdown, which also shows up as a missing
result file after the 60 s timeout. Fixed in Phase 9, "A shutdown deadlock
in `Transformer`".)* `git status`
after this phase touches only new `voxfield_ros` files plus `CMakeLists.txt`.

## Phase 5: `TsdfServer` multi-sensor frontend

The core rewrite: `pointcloud_sub_`/`freespace_pointcloud_sub_`/
`pointcloud_queue_`/`freespace_pointcloud_queue_`/`last_msg_time_ptcloud_`/
`last_msg_time_freespace_ptcloud_`/`tsdf_integrator_`/`min_time_between_msgs_`
all removed from `TsdfServer`; replaced by
`std::vector<std::unique_ptr<SensorInput<TsdfIntegratorBase>>> sensors_`.
Grepped first to confirm nothing outside `TsdfServer` touches any of them
directly — `SimulationServer` has its own independent `tsdf_integrator_`
(doesn't inherit `TsdfServer` at all), and the four derived ESDF servers
(`VoxbloxServer`/`VoxfieldServer`/`FiestaServer`/`VoxedtServer`) only ever
pass `tsdf_integrator_config` as a constructor *parameter* to the base
class — exactly M14's "the programmatic constructor's `integrator_config`
becomes the base config per-sensor overrides apply to" — so no
`primaryIntegrator()` accessor (M3's fallback suggestion) was needed.

**Construction order:** `loadSensors()` runs *before* `tsdf_map_` exists
(it only reads params, no map dependency) so the sensor **count** is known
in time for M11's ICP guard, which must run before `icp_transform_pub_`
would be created. Then: `tsdf_map_.reset()` → mesh/ICP accessories → one
`SensorInput` + integrator per `LoadedSensor` (map now exists) →
subscriptions created **last**, only once every integrator exists, so no
callback can fire against a half-built sensor. Taking each `SensorInput`'s
raw pointer for the subscription lambda's capture is safe regardless of
`sensors_`'s own reallocation, since `vector<unique_ptr<T>>` reallocating
only moves the `unique_ptr` *pointers*, not the heap-allocated `T` objects
they point to — `sensors_.reserve()` is for efficiency, not correctness
here (documented in the header anyway, since the plan calls it out as a
requirement).

**Legacy mode is exact.** `legacy_input` (topic `"pointcloud"`,
`freespace_topic` from `use_freespace_pointcloud_`, `frame = sensor_frame_`,
etc.) is built from precisely the same top-level param reads as before,
just relocated from scattered points in the old constructor into one block;
`loadSensors()` with `sensor_names` unset returns it unchanged as the sole
`"default"` sensor. `getNextPointcloudFromQueue()` and the new
`processPointCloudMessageAndInsert(..., sensor)` now call
`transformer_.lookupSensorTransform(from_frame, T_B_C_or_null, stamp, &T)`
(Phase 3's API) instead of the old `lookupTransform(from, to, stamp, T)` —
traced through both code paths (TF mode: same `world_frame_` value read
independently by both `Transformer` and the server from the same param, so
identical; queue mode: `T_B_C_or_null == nullptr` routes to the exact same
`lookupTransformQueue(stamp, T_B_C_, T)` call as before) to confirm this is
a no-op for legacy mode before writing a line of the actual swap.
**`test_legacy_golden_tsdf` passed exactly on the first build** — no
iteration needed — and the 5 pre-existing smoke launch tests (which start
real `voxblox_server`/`fiesta_server`/`voxedt_server`/etc. executables, all
`TsdfServer` subclasses) passed too, confirming the derived servers work
unmodified.

**M9 (body pose)** implemented here, reusing Phase 3's
`lookupSensorTransform` with a neat trick: passing an **identity**
`Transformation` as `T_B_C_or_null` when looking up `body_frame_` gives
`T_G_D * T_B_D^-1 * Identity = T_G_D * T_B_D^-1 = T_G_B` in queue mode
(exactly the M9 formula), while in TF mode the same call just resolves
`body_frame_` via TF directly (the passed extrinsic is ignored there) — one
code path handles both, no new `Transformer` method needed. `body_frame_`
empty (default) leaves `T_pose == T_G_C`, so `removeDistantBlocks`/
`clearDistantMesh`/`newPoseCallback` are byte-identical to before in legacy
mode.

**M10** (robot marker, primary sensor only) is a single
`sensor == sensors_[0].get()` check around the existing
`publishRobotMesh()` call — trivially "every cloud" in legacy mode since
there's only one sensor.

**M11** (ICP guard): `enable_icp_ && loaded_sensors.size() > 1` →
`RCLCPP_ERROR` + `enable_icp_ = false`, checked before `icp_transform_pub_`
would be created.

**`test/test_multi_sensor_server.cc`** (new, 5 cases, all named in the
plan): the **equivalence** test — two sensors with identical config fed the
same 20-frame box-room sequence alternately, versus one legacy sensor fed
the same sequence on one topic — passed exactly, confirming M3's claim that
splitting integration across separate `MergedTsdfIntegrator` instances
(each finishing its own `updateLayerWithStoredBlocks()` before returning)
gives bit-identical results to one instance handling everything serially.
Chose `method: merged` deliberately for this test, since M3 flags
`FastTsdfIntegrator` as keeping per-stream state that legitimately *would*
differ between 1 and 2 instances — that's not this test's concern.
Different-extrinsics (`T_B_C` yaw 0 vs 180°, same world position — both
sensors sit at the body origin so only the *look direction* differs) and
different-ray-length cases use a small straight-ahead ray bundle rather
than the full spherical scan, checking voxels near ±x via
`Layer::getVoxelPtrByCoordinates()`. The throttle test exploits exactly the
bug M4's per-sensor `last_msg_time` fixes: two sensors fed clouds at the
*identical* stamp — if the throttle clock were still shared, the second
sensor's message would see "0 seconds since the last (other sensor's)
message" and be wrongly throttled; since each `SensorInput` starts its own
clock at epoch, neither is.

**Manual accept-criteria check** (real executables, not just gtest): built
a temp params file with `sensor_names: [front, back]` and started
`voxblox_server`, `fiesta_server`, and `voxedt_server` against it in turn;
`ros2 node info` on each showed `/front/points` and `/back/points` as
subscribers and **no** `pointcloud` topic, confirming the same multi-sensor
wiring reaches every `TsdfServer`-derived executable, not just the base
class under test.

**Full suite:** `Summary: 143 tests, 0 errors, 0 failures, 4 skipped` (137
+ 5 new `test_multi_sensor_server` cases + 1 new CTest aggregate entry).
`git status` after this phase touches only `tsdf_server.{h,cc}`,
`CMakeLists.txt`, and the new test file.

## Phase 6: `NpTsdfServer` multi-sensor frontend

Same rewrite as Phase 5, mirrored onto `NpTsdfServer`, plus M8: each
`SensorInput<NpTsdfIntegratorBase>` now also owns its own
`std::unique_ptr<RangeImageProjector> projector`, built from
`LoadedSensor::projector` (the per-sensor-resolved `RangeImageProjector::Config`
from `loadSensors()`). `processPointCloudMessageAndInsert()`'s range-image
step is one line, `sensor->projector->process(...)`, replacing Phase 2's
single shared `projector_` member (removed). The seven
`projectPointCloudToImage()`-style wrapper methods now delegate to
`sensors_[0]->projector` instead — `test_np_tsdf_server.cc`'s 8 cases
needed zero edits, same as Phase 2. Dropped two now-fully-dead members,
`fov_down_rad_`/`fov_rad_` (their only write was the old projector-config
block Phase 2 already made unreachable; grepped to confirm nothing read
them post-Phase-2 before removing).

`loadSensors()` is called with `tsdf_base = nullptr` (an NP server never
reads the TSDF-only struct) and both `np_base`/`projector_base` non-null —
this is also what gates `sensor_config_loader.cc`'s NP-only checks
(`RangeImageProjector::Config::isValid()`, the NP branch of the per-sensor
INFO log) via `np_base != nullptr`, so a `TsdfServer` multi-sensor config
(which never sets `width`/`height`) is never validated as if it needed a
camera/LiDAR model.

**Found and fixed a real bug while writing the NP equivalence test**
(caught by the test itself, not by inspection): `sensor_config_loader.cc`'s
per-sensor INFO log printed `sensor.tsdf.min/max_ray_length_m` in *both*
branches, including the `is_np_server` one — for an NP server (`tsdf_base
== nullptr`), `sensor.tsdf` is always default-constructed, so the log
silently showed the wrong struct's (coincidentally same-default, 5.0 m)
value instead of the real `sensor.np_tsdf.max_ray_length_m` the integrator
actually uses. Log-only bug (the integrator itself always read the correct
field), but real: fixed to read `sensor.np_tsdf.*` in that branch.

**`test/test_multi_sensor_server.cc` restructured** with the
`test_legacy_golden.cc`/`test_server_map_io.cc` `#ifdef TEST_NP_TSDF_SERVER`
split (can't include both server headers in one translation unit), so the
same source now builds two binaries: `test_multi_sensor_server` (the 5
Phase-5 TSDF cases, unchanged) and `test_multi_sensor_server_np_tsdf` (3
new cases, all three named in the plan):
- **Equivalence** (exact): mirrors Phase 5's, with both servers' `"default"`/
  `'a'`/`'b'` sensors sharing identical top-level LiDAR params (256×16, fov
  3/−25°) instead of per-sensor overrides, so legacy and multi-sensor mode
  inherit the identical projector config the same way real configs would.
- **Mixed LiDAR + camera**: sensor A (LiDAR, box-room scan) and sensor B
  (camera, a small pinhole grid on a 2 m fronto-parallel plane, chosen so
  the image center `u=320,v=240` lands at exactly local `(0,0,2)` — an
  exactly predictable world voxel with identity extrinsics). Checking "both
  sensors' voxels exist" by comparing allocated block *counts* before/after
  B turned out not to work: A's own spherical LiDAR scan already carves
  free space through most of the room's interior (including near B's z=2m
  plane), so B added weight to already-allocated blocks without changing
  the count. Replaced with two direct, sensor-specific voxel checks
  instead (A's wall hit, B's image-center hit), each still present after
  the other sensor integrates.
- **Invalid projector throws at construction**: sensor B's `width` left
  unset (defaults to 0) → `NpTsdfServer`'s constructor itself throws
  `std::invalid_argument` (via `loadSensors()`, uncaught, propagating out —
  exactly M13.7's "the process exits non-zero" behavior), with `'b'` and
  "invalid projector config" both present in the message.

**A real floating-point edge case, caught by the mixed test and fixed by
changing the test, not the code:** the first attempt probed the LiDAR
wall-hit voxel at row 0 of the configured image (elevation exactly
`fov_up`, the top boundary) and got weight `0` — not a bug: `asin`/`atan2`
round-tripping a point's 3D position back to an angle in
`projectPointToImageLiDAR()` can land a float epsilon on the wrong side of
exactly `fov_up`, and the deliberate (verbatim, pre-existing)
`floor(proj_y) < 0` boundary check then rejects the point. Switched the
probe to an interior row (row 8 of 16) where this can't happen — a
reminder that a boundary row is a bad choice for hand-computing an
"exact" expected pixel in *any* test of this projection math, not
something to loosen the production bounds check for.

**Manual accept-criteria check:** `voxfield_server` and `np_tsdf_server`
both started against a `sensor_names: [front, back]` config and showed
`/front/points`/`/back/points` (no `pointcloud`) in `ros2 node info`. Aside:
`np_tsdf_server`'s executable names its node `"voxfield"` (matching
`voxfield_server`'s own node name) — a pre-existing quirk in
`np_tsdf_server_node.cc`, unrelated to this plan; noted here only because
it wasted a few minutes guessing the wrong node name for `ros2 node info`.
*(Fixed in Phase 10: see "Pre-existing bugs fixed in Phase 10".)*

**Full suite:** `Summary: 147 tests, 0 errors, 0 failures, 4 skipped` (143
+ 3 new `test_multi_sensor_server_np_tsdf` cases + 1 new CTest aggregate
entry). `test_legacy_golden_np_tsdf` and all 8 `test_np_tsdf_server` cases
passed with zero edits. `git status` after this phase touches
`np_tsdf_server.{h,cc}`, `sensor_config_loader.cc` (the log fix),
`CMakeLists.txt`, and `test_multi_sensor_server.cc`.

## Phase 7: Body frame for derived ESDF servers (M9) + optional status topic

**Step 1 (verify, no code change expected):** grepped every `T_G_C.`
access in `voxblox_server.cc`, `voxfield_server.cc`, `fiesta_server.cc`,
and `voxedt_server.cc` — every one is `.getPosition()` (`removeDistantBlocks`,
`addNewRobotPosition`), confirmed by an empty grep for any *other* member
access. Since Phase 5 already made `TsdfServer::processPointCloudMessageAndInsert()`
pass the body pose (when `body_frame` is set) as the argument to the
virtual `newPoseCallback()` these four servers override, and none of them
read anything but the position out of that argument, M9 reaches all four
with **zero code changes** in this phase, exactly as predicted.

**Step 2 test:** `test_multi_sensor_server.cc`'s new
`BodyFrameChangesBlockRemovalReference` case — two servers (`body_frame`
set vs. unset), each with two sensors 1 m apart (`T_B_C` translations
(5,0,0) and (6,0,0), body/`T_G_D` at the world origin), `max_block_distance_from_body:
10`. Sensor A's cloud places a block whose *origin* (the block's min
corner — what `Layer::removeDistantBlocks()` actually compares against,
not the block's center) is at world (12,0,0): 12 m from the body but only
7 m from sensor A's own position. Passes on both sides: removed with
`body_frame` set, kept without it.

**A real bug found and isolated while writing this test, confirmed
out of scope, not fixed here.** The test's first version used points lying
exactly on the sensor's local x-axis (`y = z = 0`). Every such point
integrated correctly in every *previous* multi-sensor test (all of which
use box-room raycasts or camera grids with natural x/y/z spread) but here,
alone, the resulting map only ever had a block at the *ray's origin*, never
one anywhere near the actual far point — regardless of translation, and
regardless of using 1 or 20 such points. Isolated with a **standalone
program linked directly against `libvoxfield.so`**, bypassing this
package's server/test code entirely (`voxfield::MergedTsdfIntegrator`
constructed and called directly): a point at `(x, 0, 0)` for *any* `x`
(1 through 12.5 m tried) only ever allocates the block containing the
ray's origin; the same point with a nonzero z (e.g. `(5, 0, 0.437)`, copied
from a passing test) allocates the correct block too. The bug is a
**degenerate case in `MergedTsdfIntegrator`'s ray traversal when the ray
direction has two exactly-zero components** (i.e. a ray parallel to a
coordinate axis) — evidently never previously exercised by any test in
either this package or `voxfield`'s own `test_sdf_integrators` suite, since
real sensor data (and every synthetic raycast in this plan) essentially
never produces an exactly axis-aligned ray. This is squarely inside
`voxfield/src/integrator/tsdf_integrator.cc`, which §1.3 puts out of scope
("No `voxfield/src/integrator/*` changes are expected. If one turns out to
be necessary, stop and document why.") — so, per that instruction: **not
fixed here**, and flagged here for the user's awareness, since it's a
latent correctness issue that predates this plan and could affect any
caller (single- or multi-sensor) whose sensor happens to produce an
axis-aligned ray (e.g. a purely horizontal or vertical calibration ray in a
test fixture). The multi-sensor test itself was fixed by giving its points
a small deliberate z offset (0.3 m), matching how every other geometry in
this file already avoids the case incidentally. *(Corrected in Phase 10: the symptom above
was mainly `TsdfIntegratorBase::getVoxelWeight()`. Its `1 / z^2`
depth-camera weight is 0 for a point with sensor-frame z = 0, so such a
point is skipped entirely, and a z offset "fixes" it. Investigating it
still turned up a real ray-caster bug for rays with an exactly-zero
component, now fixed: "Known upstream issues" #14 in
`docs/ROS2_PORT_NOTES.md`. The weighting question is in Phase 10.)*

**Step 3 (optional `~/sensor_status` `DiagnosticArray`, M15): skipped,
time-boxed**, exactly as the plan allows ("Skip it if time-boxed; say so in
the notes"). The `SensorInput` stats fields it would report
(`num_received`/`num_throttled`/`num_dropped`/`num_integrated`, last
stamp, last integration ms) already exist on every `SensorInput` (M4) and
are exercised directly by tests (`ThrottleIsPerSensor`); wiring them to a
published topic is straightforward future work if wanted, but the INFO-level
per-sensor logging `sensor_config_loader.cc` already emits at construction
(M13.8) covers the "is my multi-sensor config doing what I think"
question this topic would mostly answer.

**Full suite:** `Summary: 148 tests, 0 errors, 0 failures, 4 skipped` (147
+ 1 new case, no new CTest binary since it was added to the existing
`test_multi_sensor_server` target). `git status` after this phase touches
only `test_multi_sensor_server.cc`.

## Phase 8: Launch, configs, fake publisher, smoke test

**Step 1: `scripts/fake_sensor_publisher.py`.** Added `num_sensors`
(default 1), `sensor_yaw_offsets_deg` (comma-separated string, default
evenly spaced), and `sensor_rig_offset_m` (default 0.5). Sensor *i* now
gets frame `<sensor_frame>_<i>`, mounted on a rig frame
`<sensor_frame>_rig` at `sensor_rig_offset_m` and yaw offset *i* from
`sensor_yaw_offsets_deg`, publishing on `<pointcloud_topic>_<i>`; each
sensor's emitted points are filtered to `cos(azimuth) >= 0` (±90° of its
own forward axis), computed once as a precomputed direction-index subset
so the per-tick cost is unchanged. `publish_tf` now broadcasts a moving
`world → <sensor_frame>_rig` transform (or publishes it on `~/transform`
when `publish_tf:=false`, matching the existing single-sensor behavior)
plus a *static* `<sensor_frame>_rig → <sensor_frame>_i` per sensor via
`StaticTransformBroadcaster`, published once at startup.

`num_sensors:=1` is byte-identical to the old code path (verified: the
`tick()` method dispatches to an untouched `_tick_single_sensor()` body
when `num_sensors == 1`, and all 5 pre-existing single-sensor smoke
launch tests pass unmodified). Manually verified `num_sensors:=2
sensor_yaw_offsets_deg:="0,180" width:=64 height:=8`: topics
`/pointcloud_0`/`/pointcloud_1` only (no unsuffixed `/pointcloud`), static
TF `velodyne_rig → velodyne_0` at `(0.5, 0, 0)` identity rotation and
`velodyne_rig → velodyne_1` at `(-0.5, ~0, 0)` yaw 180° (`ros2 topic echo
/tf_static --once`), and exactly 256 points per cloud (half of 64×8=512,
confirming the ±90° filter). `publish_tf:=false` confirmed publishing
`~/transform` with `frame_id: world, child_frame_id: velodyne_rig`.

**Step 2: `test/test_multi_sensor_smoke.launch.py`.** Registered for both
`method:=voxfield` and `method:=voxblox`. Starts the fake publisher with
`num_sensors:=2 sensor_yaw_offsets_deg:=0,180 width:=256 height:=16` and
the server with `sensor_names: [s0, s1]`, `sensors.s0.topic:
pointcloud_0`, `sensors.s1.topic: pointcloud_1`,
`update_esdf_every_n_sec: 1`. Four assertions, all within the 40 s
budget: `test_mesh_has_both_walls` (accumulates growing mesh messages
until both a block with world `x > 4` and one with world `x < -4` are
seen — world coordinates recovered from the message's quantized
`uint16` vertex encoding via `world_x = block_edge_length *
(block_index_x + 2.0 * (x_uint16 / 65535.0))`, the exact inverse of
`mesh_vis.h`'s `generateVoxbloxMeshMsg()`), `test_esdf_slice_nonempty`,
`test_save_load_roundtrip` (same pattern as the existing single-sensor
smoke test), and `test_no_error_lines` (filters `proc_output[server]` —
i.e., only the server process's captured stdout/stderr via
`IoHandler.__getitem__` — for `"ERROR"` substrings, so the fake
publisher's own output can't cause a false failure). Both
`test_multi_sensor_smoke_voxfield` and `test_multi_sensor_smoke_voxblox`
pass 4/4.

**Single-sensor sanity check (manual, not automated, per the plan's "a
sanity run in the notes").** Ran the same two-sensor fake publisher
(`num_sensors:=2 sensor_yaw_offsets_deg:=0,180 width:=256 height:=16
spin_period_sec:=30.0`) against a `voxblox_server` configured with only
`sensor_names:=[s0]` / `sensors.s0.topic:=/pointcloud_0` — deliberately
ignoring the second sensor's topic — and subscribed directly to the
server's own mesh topic, classifying each message by the same
quantized-vertex decode used in the smoke test. Over the first ~29 mesh
messages (`update_mesh_every_n_sec: 0.1`, ≈3 s of publisher time), only
the `+x` wall (the one visible from sensor `s0`'s own ±90° FOV at its
initial pose) ever appears; a single message (msg 29) also showed a
`-x` block, which reverted to absent on the very next message and is
attributed to a block spanning the `x = ±4` classification boundary
rather than genuine `-x` wall coverage — the rig's slow spin
(`spin_period_sec: 30`) means the sensor's forward axis eventually
sweeps far enough to graze the far wall's edge given enough elapsed
time, so this check only holds reliably over a short window, not
indefinitely. This confirms the plan's expectation: with only one
limited-FOV sensor subscribed, the fused map does not show both walls
(at least not early/for an extended window), in contrast to the
two-sensor automated test which does.

A naming pitfall hit while running this manually, worth recording:
running `voxblox_server` directly via `ros2 run` (rather than through a
launch file, which explicitly passes `name="voxfield_node"`) uses the
executable's own default node name, which for `voxblox_server` is
`voxblox` — so the mesh topic is `/voxblox/mesh`, not
`/voxfield_node/mesh`. This mirrors the Phase 6 finding that
`np_tsdf_server`'s default node name is `voxfield` (not `np_tsdf`), i.e.
neither executable's default node name matches its own filename; both
are pre-existing quirks unrelated to this plan, noted only because they
cost real debugging time (`ros2 topic echo` and a subscriber script both
silently saw zero messages against the wrong topic name, even though the
server's own log clearly showed successful periodic mesh updates).

**Step 3: `launch/multi_sensor_mapping.launch.py`.** Arguments: `method`,
`param_file`, `sensors_file`, `bag_file` (default `""`), `play_bag`
(default `true`), `speed`, `start_offset`, `rviz` (default `true`),
`rviz_config` (falls back to `cfg/rviz/multi_sensor.rviz`),
`use_sim_time` (default `true`), `tf_remap_prefix` (default `""`), and
`rgbd` (default `false`). Bag playback uses `--clock`, `-r <speed>`, an
optional `--start-offset`, and always applies
`cfg/multi_sensor/tf_static_qos_override.yaml` (port plan pitfall §8.7:
`ros2 bag play`'s `/tf_static` can be volatile, dropping late-joining
listeners' static TF unless overridden to `transient_local`/`keep_all`).
When `tf_remap_prefix` is set (e.g. `/athena`), `/tf → <prefix>/tf` and
`/tf_static → <prefix>/tf_static` remaps are applied to **both** the
server node and — per pitfall §9.9 — the `rviz2` node when `rviz:=true`.

When `rgbd:=true`, one `ComposableNodeContainer` per camera
(`_ATHENA_RGBD_CAMERAS = ["front", "back"]`) loads an
`image_transport::Republisher` (`in_transport: compressedDepth,
out_transport: raw`, remapping `in/compressedDepth` to
`/athena/<camera>_rgbd/depth/image_raw/compressedDepth` and `out` to a
new `.../image_raw_decoded` topic) feeding a
`depth_image_proc::PointCloudXyzNode` (remapping `image_rect` to that
decoded topic, `camera_info` to `/athena/<camera>_rgbd/depth/camera_info`,
and `points` to `/athena/<camera>_rgbd/points`). The exact plugin names
and remap keys were confirmed per the plan's explicit instruction to
check with `ros2 component types` (both plugins listed) and `ros2 param
list` on standalone-run instances of each: `image_transport::Republisher`
selects transports via the *declared parameters* `in_transport`/
`out_transport`, not positional CLI args or remap-only selection (an
earlier attempt using positional `compressedDepth raw` args silently
republished every transport under `/out/<transport>` with zero
subscribers — fixed by using `-p in_transport:=... -p
out_transport:=...`); `depth_image_proc::PointCloudXyzNode` takes plain
`image_rect`/`camera_info`/`points` topic remaps plus its own
`depth_image_transport` parameter.

Verified manually, twice, end-to-end via `ros2 launch`: (a)
`play_bag:=false rviz:=false` with `athena_param.yaml` +
`athena_dual_lidar.yaml` started cleanly and printed exactly the
expected M13 per-sensor INFO lines for `front_lidar` and `back_lidar`
(topic, frame, ray range, and LiDAR FOV); (b) the same with `rgbd:=true`
and `athena_lidar_rgbd.yaml` printed all 4 sensors' INFO lines, and both
RGB-D `ComposableNodeContainer`s loaded their two components with zero
errors.

**Step 4: configs.** `cfg/multi_sensor/athena_param.yaml` (map-global
params) starts from the local, uncommitted Phase 12 param file described
in `docs/ROS2_PORT_NOTES.md` Step 4 (`world_frame: map`, `tsdf_voxel_size:
0.1`, ICP off), converted to the full key set every other
`cfg/*/*_param.yaml` in this repo carries, with one deliberate fix:
`kitti_param.yaml` sets `integration_threads`, which is a pre-existing
typo (the real key, per `ros_params.h`, is `integrator_threads`) that
silently does nothing — not repeated here; `athena_param.yaml` sets
`integrator_threads: 6`. `body_frame: base_link` (M9) and
`update_{mesh,esdf}_every_n_sec` / `publish_map_every_n_sec` all left
positive (M12's multi-sensor "every N frames" pitfall: with multiple
sensors each publishing a cloud counted as one frame, a negative
"every N frames" value's effective rate would scale with sensor count).

`cfg/multi_sensor/athena_dual_lidar.yaml` (two Livox LiDARs) and
`cfg/multi_sensor/athena_lidar_rgbd.yaml` (adds two RGB-D cameras) both
carry a documented **placeholder**: `fov_up: 52.0` / `fov_down: -7.0` are
not yet measured from the real bag — that's Phase 9 step 1's job, and
both files say so in a comment. The RGB-D file's per-camera `fx`/`fy`/
`vx`/`vy` are real values already read from this machine's bag
(`/athena/{front,back}_rgbd/depth/camera_info`): front
`fx=fy=451.5, vx=325.8, vy=242.6`; back `fx=fy=452.0, vx=327.1,
vy=241.0`; both 640×480, `max_ray_length_m: 4.0`,
`min_time_between_msgs_sec: 0.4`, topic `/athena/<camera>_rgbd/points`
matching the launch file's RGB-D pipeline output.

**Step 5: `cfg/rviz/multi_sensor.rviz`.** Based on `kitti_25cm.rviz`'s
structure. `Fixed Frame: map`. `VoxfieldMesh` on `/voxfield_node/mesh`
(`Durability Policy: Transient Local`), ESDF slice on
`/voxfield_node/esdf_slice`, TF, a Robot Model marker, and one
`PointCloud2` per raw sensor topic in a distinct flat color (front =
red, `/athena/front_lidar/points_raw_livox`; back = blue,
`/athena/back_lidar/points_raw_livox`). YAML-validated with
`python3 -c "import yaml; yaml.safe_load(...)"`.

**`package.xml`:** added `rclcpp_components`, `image_transport`,
`compressed_depth_image_transport`, `depth_image_proc` as
`exec_depend`s for the `rgbd:=true` launch path.

**Full suite:** `Summary: 158 tests, 0 errors, 0 failures, 4 skipped` (148
+ 2 new launch-test binaries × ~5 tests each, no new gtest binaries this
phase). `ros2 launch voxfield_ros multi_sensor_mapping.launch.py
method:=voxfield param_file:=… sensors_file:=… play_bag:=false` starts
cleanly and M13 prints one INFO line per sensor (verified above, both
without and with `rgbd:=true`). `git status` after this phase touches
`scripts/fake_sensor_publisher.py`, `voxfield_ros/CMakeLists.txt`,
`voxfield_ros/package.xml` (new `exec_depend`s), and adds
`voxfield_ros/test/test_multi_sensor_smoke.launch.py`,
`voxfield_ros/launch/multi_sensor_mapping.launch.py`,
`voxfield_ros/cfg/multi_sensor/*.yaml`,
`voxfield_ros/cfg/rviz/multi_sensor.rviz`.

## Phase 9: Real-bag validation

Every measurement below comes from `~/src/rosbag2_2026_09_23-14_32_47`
(Athena robot: two Livox Mid-360-type LiDARs, two RGB-D cameras). The
section was paused once for a handoff (steps 1–4 and part of step 5 were
done then) and finished afterwards. Validation turned up three real bugs,
all fixed here: the `fiesta`/`voxedt` save/load bug (below), the FIESTA/EDT
memory blow-up (step 7), and a `Transformer` shutdown deadlock (after
step 7).

### Setup

**Code kept from the handoff.** Two verbose-only logs in
`tsdf_server.cc`/`np_tsdf_server.cc` are kept:
- `[<sensor>] stats: received=N throttled=N dropped=N integrated=N`, once
  per callback. These are M4's per-sensor counters, which the plan's M4
  comment asked to log when verbose.
- `[<sensor>] Range image: <raw> -> <kept> points, <n> with a valid
  normal (<%>)` in `NpTsdfServer`. The plan called this "temporary", but it
  is verbose-only and O(N), and it is the only way to see §3.3's
  resolution/normal trade-off on real data (step 5), so it stays.

**Scripts** (not in the repo, since they're specific to this bag and
machine) are in `~/voxfield_phase9_handoff/`:
- `p9run.sh <out_prefix> <binary> <rate> <bag_seconds> <server ros-args...>`
  is the run driver. It is self-contained and deterministic:
  - It starts the server with `verbose`/`timing` on, `mesh_filename=<out>.ply`, `__node:=voxfield_node`, and `/tf`/`/tf_static` remapped to `/athena/...`.
  - It plays **only** the 2 LiDAR + TF topics with `--playback-duration`, so it runs for a fixed amount of *bag* time.
  - It waits `P9_DRAIN` seconds, then calls `save_map` and `generate_mesh` with retries.
  - It grabs the latched `esdf_slice`/`tsdf_slice` into `.npy` using `grab_slices.py`.
  - It samples RSS every 2 s into `<out>.rss`, with a watchdog kill at 18 GB (**no cgroup cap**, per the user's correction). It also logs VmHWM.
  - It `pkill`s any running server first, so **never run two drivers at once**. Run it from `bash` (zsh doesn't word-split `set -- $wh` in loops). Don't put `pkill -f install/voxfield_ros/...` in the same shell command as other work: the pattern matches the calling shell's own command line and kills it.
- `p9run_rgbd.sh <out_prefix> <method> <rate> <bag_seconds>` does the same
  through `multi_sensor_mapping.launch.py rgbd:=true`, which provides the
  compressedDepth → points chain. `P9_SENSORS=<yaml>` swaps the sensor file.
- `parse.py <log>...` prints per-sensor integration ms, rate, final stats line, range-image and valid-normal fractions, and the final timing table.
- `raw_align.py <t0> <t1>` checks the step 3 extrinsics from raw clouds. It loads the TF and both LiDARs for that window, transforms the clouds into `map`, and reports front↔back point-to-plane residuals and a linearized 6-DoF correction. It also computes a front-vs-front baseline.
- `consistency.py` does mesh-vs-mesh nearest-neighbour and ICP between the (a), (b), and (c) PLYs.
- `mesh_quality.py <name>=<ply>...` computes step 5's accuracy/completeness metric. `render.py` makes the `docs/assets/multi_sensor_*.png` figures.
- `traj_scan.py` prints the robot trajectory. `measure_livox_geometry.py` and `tf_scan.py` are from step 1.
- Run the Python scripts with **`/usr/bin/python3`** inside `bash -c 'source /opt/ros/jazzy/setup.bash && ...'`. Conda's `python3` is first on `PATH` and can't import `rosbag2_py`.
- Logs, RSS traces and slices: `runs/` (handoff-era runs) and `work/` (later runs). Maps/PLYs: `maps/` and `work/`.

**Environment.** The user retracted the memory-cap instruction (§0.7 /
pitfall 13) for this phase, so all runs are **uncapped** with the RSS
watchdog. Before the step 7 fix, the watchdog killed `fiesta_server` and
`voxedt_server`. After it, peak RSS was 1.5–8.0 GB.

**Trajectory** (from `traj_scan.py`, `odom → base_footprint_link`; `map → odom` is about identity, within 0.1 m):
- 0–12 s: the robot is stationary at (30.2, −13.7).
- 12–60 s: it drives about 10 m with a yaw change of about 80°.
- It then continues for about 60 m up and down a ramp (z from −0.7 to +3 m) and returns to its start area.
- It is stationary again from about 380 s.

Every run below uses `--start-offset 0 --playback-duration 60`. A nonzero
offset would skip the `/tf_static` messages at the start of the bag.

### Slice/save/load smoke coverage for every method
The 4 skipped `test_smoke_<method>` subtests (Phase 0) were a coverage gap,
not an intended design. `test_slice_and_map_roundtrip` now runs for every
method:
- `~/tsdf_slice` must publish a non-empty cloud for all 5 methods.
- `~/esdf_slice` must also publish one for the ESDF servers (`voxfield`,
  `voxblox`, `fiesta`, `voxedt`).
- A `~/save_map` → `~/load_map` round trip through a temp file must succeed
  (`Successfully loaded TSDF layer.` and no `Failed to load map`).
- For `np_tsdf` the test also sets `publish_pointclouds: true`.
  `NpTsdfServer` publishes slices only from `publishPointclouds()`, and
  `kitti_param.yaml` turns that off. The ESDF servers publish slices from
  their ESDF timer instead.

**Bug found and fixed:** `FiestaServer` and `VoxedtServer` never overrode
`saveMap()`. `~/save_map` fell through to `TsdfServer::saveMap()` and wrote
a TSDF-only file, but their own `loadMap()` reads TSDF and then ESDF. Every
load therefore failed on the ESDF half ("Layer type of the loaded map is:
esdf but the current map is: tsdf check passed? 0", then `Failed to load
map`). This is the mirror image of "Known upstream issues" #11 in
`docs/ROS2_PORT_NOTES.md`, and it gets the same fix: `saveMap()` writes
TSDF (truncating) and then appends ESDF. Their `saveEsdfMap()`/
`saveOccMap()` also now truncate (`kClearFile = true`), like
`VoxbloxServer::saveEsdfMap()`. Each writes its own file (`~/save_esdf_map`,
`~/save_occ_map`, and `saveAllMap()`'s `.esdf`/`.occ`), so appending only
left stale content first in the file on a re-save.

Tests:
- `test_server_map_io.cc` is now built for `fiesta` and `voxedt` too
  (`test_{fiesta,voxedt}_server_map_io`, via `TEST_{FIESTA,VOXEDT}_SERVER`).
- Negative check: with `FiestaServer::saveMap()` temporarily reverted to
  TSDF-only, both `test_fiesta_server_map_io` (`SaveLoadRoundTrip`) and
  `test_smoke_fiesta` fail as described above. With the fix, both pass.

**Full suite:** `Summary: 164 tests, 0 errors, 0 failures, 0 skipped`. This
run includes the two verbose-only logs from the handoff below.

### Step 1: Livox geometry
Measured with a `rosbag2_py` script over 50 clouds per LiDAR:
- Front LiDAR pitch: −7.739° to 53.017°.
- Back LiDAR pitch: −7.312° to 51.847°.
- Config set to `fov_up: 53.5` and `fov_down: −8.0`, which covers both with some margin.

A new finding while checking step 5: each cloud has about 20 000 points,
but **about 38 % of them are at the origin** (r < 0.1 m; the Livox driver
emits (0,0,0) placeholders for no-return). **About 45 % have r < 0.3 m**,
and about 11 % lie between 0.1 m and 1 m. That last band is likely the
robot body and mounts seen by the LiDAR itself (pitfall 11). The median
range is about 1.1 m. `min_ray_length_m: 0.3` already drops the
placeholders and the closest self-hits.

**Do the 0.3–1 m self-hits leave obstacles next to the robot (pitfall 11)?
No.** Checked on the voxblox (c), voxfield (i) and fiesta meshes:
- At the start pose, where the robot is stationary for 12 s on flat
  ground, there are 0–2 mesh vertices within 1 m horizontally and 0.15–1.5 m
  above the footprint, all at dz ≈ 0.16 m (ground texture), against ~400
  ground vertices.
- At the 60 s pose (on the ramp), all vertices within 1 m fit a single
  24–28° plane (voxfield median residual 1.5 cm, max 0.29 m, only 9 of 500
  more than 0.2 m off).

Nothing body-shaped persists. `min_ray_length_m: 0.3` drops the closest
returns, and the remaining self-hits are carved away by rays from other
poses.

### Step 2: ray-casting fusion, `voxblox_server`
Settings for these runs:
- Rate 0.25 and 60 s of bag, so the server keeps up.
- `update_esdf_every_n_sec: 0`, which turns ESDF off. The block count is TSDF-only, and ESDF would otherwise stall the executor (see step 4).
- (a) and (b) run in **legacy mode**: no `sensor_names`, and `-r pointcloud:=/athena/<x>_lidar/points_raw_livox`. `sensor_frame` is empty, so the frame comes from the header (F1). (c) runs with `athena_dual_lidar.yaml`.

| Run | Final blocks | Clouds integrated | RSS peak | PLY / TSDF |
|---|---|---|---|---|
| (a) front only | **8557** | 595 / 600 recv (2 dropped at TF warm-up) | 1.37 GB | 93 MB / 390 MB |
| (b) back only | **8544** | 598 / 600 | 1.36 GB | 90 MB / 386 MB |
| (c) both | **9445** | front 594/599, back 596/598 | 1.60 GB | 123 MB / 432 MB |

The plan's expectation `max(a,b) < c ≤ a+b` holds: 8557 < 9445 ≤ 17101.
The overlap is large because both Mid-360-type sensors see 360° in yaw,
and `voxel_carving` allocates free-space blocks along every 30 m ray.
The only ERROR line in each run is the expected `[front_lidar] Input
pointcloud queue getting too long` during the first ~4 s while TF warms
up (`map` doesn't exist yet). The saved files are in
`~/voxfield_phase9_handoff/maps/vb_{a_front,b_back,c_both}.{tsdf,ply}`.
Figure: `docs/assets/multi_sensor_voxblox.png` (left: the fused (c) mesh
coloured by height; right: the start area with the raw front/back clouds
over the fused mesh). This and the other Phase 9 figures are matplotlib
top-down renders (`render.py`) rather than RViz2 screenshots. They are
reproducible from the saved maps, and they don't open windows on the
user's live `DISPLAY=:1` desktop, which a scripted `xwd` screenshot would
need.

### Step 3: consistency and extrinsics
From `raw_align.py`: raw clouds from 40 front and 40 back sweeps,
transformed into `map` with the bag's own TF, while the robot is
stationary.

| Window | Planar-overlap \|residual\| median / p90 | Front-vs-front baseline | Best rigid correction back→front |
|---|---|---|---|
| 2–6 s | 0.024 / 0.111 m | 0.003 / 0.010 m | rot (−0.90, 1.23, −0.21)°, t 0.023 m |
| 6–10 s | 0.023 / 0.112 m | 0.004 / 0.012 m | rot (−0.77, 1.24, −0.27)°, t 0.036 m |
| 392–398 s | 0.063 / 0.140 m | 0.004 / 0.014 m | rot (0.83, 0.91, −0.17)°, t 0.020 m |

The residual grows with range: 0.02 m median at 0–6 m, 0.095 m at
6–9 m, and 0.145 m at 9–12 m. That is the signature of an angular
error, not a translation. Rotating the map-frame corrections into the
robot's yaw at each window (64.7° and −14.7°) gives about (0.6–0.7°,
1.1–1.3°) of roll and pitch in the **body** frame, consistent across the
start and the end of the bag.

**Conclusion:**
- The bag's `/athena/tf_static` places the back LiDAR about 1.4° tilted relative to the front one.
- The static extrinsics look CAD-nominal, because front and back are perfectly symmetric: `*_sensor_mount_link → *_lidar_link` is rpy (26.78°, 0, 180°), and `chassis_link → *_sensor_mount_link` is rpy (6.78°, 0, ±90°) at x = ±0.246 m.
- This is a **calibration issue in the data**, not a code bug. The independent Python check uses the same TF chain as the server and sees the same thing.

What this means for the fused map:
- Within about 6 m, the two sensors agree to about 2 cm, well under the 0.1 m voxel. **No doubled walls near the robot.**
- At 6–12 m, surfaces from the two sensors separate by about 1–1.5 voxels. That thickens distant walls and can double them faintly.

Mesh-level cross-check from `consistency.py` (5 cm-deduplicated vertices):
- Vertex nearest-neighbour distance between (a) and (b): median 0.10 m, p90 0.27–0.29 m.
- (a)→(c): median 0.026 m. (b)→(c): median 0.014 m.
- Vertex ICP between the two meshes gave |t| = 0.39 m, mostly along z. This is **not trustworthy**, because it slides along the dominant ground plane on sloped terrain. Use the raw-cloud result above instead.

Figure: `docs/assets/multi_sensor_wall_slice.png`, the front-only (a) and
back-only (b) mesh vertices in the z = 0.5 ± 0.05 m band, with 6 m / 12 m
rings around the start. Inside 6 m the two coincide. Toward 12 m the
right-hand wall visibly separates by about one voxel, matching the raw-cloud
residuals above.

Recommendation: calibrate the back LiDAR's extrinsic (about 1.4° of
pitch/roll). Until then, the shipped `max_ray_length_m` of 12 m (step 4)
also keeps most of the doubling out of the map.

### Step 4: per-sensor stats, the M7 budget, and the real-time default
From `parse.py`. Per-sensor integration time (`merged`, 6 threads,
0.1 m voxels, 30 m rays): front **45.7 ms** mean (median 43, p95 58.5)
and back **41.7 ms** mean (median 41, p95 53). The first few clouds take
up to about 0.4 s.

| Run (dual LiDAR, 60 s bag) | Front recv / integrated | Back recv / integrated | Rate |
|---|---|---|---|
| (c) rate 0.25, ESDF off | 599 / 594 | 598 / 596 | 10 Hz in bag time for each |
| rate 1.0, ESDF off | **448 / 444** | **493 / 491** | 7.6 and 8.2 Hz |
| rate 1.0, ESDF every 1 s (the shipped `athena_param.yaml`) | **38 / 36** | **33 / 32** | about 0.6 Hz |

- `num_dropped` is 0–2 per sensor, and only during TF warm-up. The losses at rate 1.0 happen **at the DDS subscription** (`pointcloud_queue_size` 1) while the single-threaded executor is busy. `num_dropped` doesn't count those. Compare `received` with 600 offered.
- **M7 budget:** 20 clouds/s × about 43.6 ms is about 0.87 s/s of TSDF alone, plus about 0.04 s/s of mesh updates. That is right at the 1 s/s limit, which is why about 20 % of clouds are lost even without ESDF.
- **The ESDF is what really blows the budget.** `voxblox_server`'s ESDF update takes 3.2 s mean and 6.7 s max with `local_range_offset` 20/20/10 blocks, and it runs every 1 s. With the shipped config, only about 6 % of clouds get integrated in real time. RSS reached 6.1 GB. This isn't specific to multi-sensor: the ESDF works from TSDF blocks. A single LiDAR would suffer the same way.

**Choosing the real-time default.** Two candidates at rate 1.0, 60 s of
bag, both LiDARs, `athena_param.yaml` + `athena_dual_lidar.yaml`
(`work/rt_*`, `work/rt2_*`). Received is out of 600 offered per sensor.

| Candidate | Server | Received (front / back) | ESDF update mean (max) | Blocks | Peak RSS |
|---|---|---|---|---|---|
| 1: 30 m rays, ESDF every 5 s, `local_range_offset` 10/10/5, queue 10 | voxblox | 115 / 115 | 4.2 s (9.9) | 8662 | 6.9 GB |
| 1 | voxfield | 173 / 173 | 7.8 s (15.3) | 8568 | 6.8 GB |
| **2: 12 m rays, ESDF every 2 s, queue 10** | voxblox | 443 / 443 | 1.0 s (1.8) | 1875 | 1.6 GB |
| **2** | voxfield | **558 / 563** | 0.9 s (1.9) | 1869 | 1.5 GB |

- Candidate 1 barely helps. With 30 m rays, a single ESDF update takes
  longer than the 5 s interval, and nothing else can run on the
  single-threaded executor while it does. A shorter `local_range_offset`
  doesn't shrink the work, because the ESDF still has to cover every TSDF
  block the 30 m rays touch.
- Candidate 2 cuts every stage: TSDF integration 30 ms (voxblox) / 16 ms
  (voxfield) per cloud, and ESDF ~1 s. `voxfield_server` integrates
  **~93 %** of clouds live (the rest are lost at the DDS queue during ESDF
  updates). `voxblox_server` manages 74 %: its TSDF alone is ~0.6 s/s for
  two LiDARs at 10 Hz, before the ESDF.
- **Shipped** in `athena_param.yaml`: `max_ray_length_m: 12.0`,
  `update_esdf_every_n_sec: 2.0`, `pointcloud_queue_size: 10`. The
  trade-off is range: the map covers ~12 m around the trajectory (1.9 k
  blocks instead of 9.4 k over this minute of bag). 12 m is also where the
  back LiDAR's extrinsic error starts doubling walls (step 3). For offline
  mapping at reduced playback speed, override `max_ray_length_m:=30.0`.
  All the other runs in this section used 30 m, the value shipped before
  this step.
- A multi-threaded executor that runs the ESDF update off the
  subscription thread would lift the budget further. That needs locking
  between TSDF integration and the ESDF reading the TSDF layer, so it isn't
  part of this plan.
- A cosmetic note: every Livox cloud prints `Failed to find match for field 'intensity'`. The Livox `intensity` field is UINT8, and PCL's `PointXYZI` expects FLOAT32, so intensity/colour is dropped. Geometry isn't affected.

### Step 5: `voxfield_server` with both LiDARs
**Range-image resolution sweep.** Rate 0.5, 30 s of bag, ESDF off,
`normal_available: true`. The valid-normal fraction doesn't depend on
`normal_available`; it's computed in the projector. The percentages
below are the fraction of the ~20 000 raw points that land in the
image. Recall that only about 55 % of raw points are valid returns
(r > 0.3 m).

| width × height | Points kept (front / back) | Valid normals among kept | Integration ms (front / back) | Blocks |
|---|---|---|---|---|
| 360 × 32 | 30.4 % / 28.5 % (about 6 000) | **62–65 %** | 23.6 / 20.7 | 7429 |
| 720 × 64 | 50.3 % / 47.9 % (about 9 800) | 11.5–11.8 % | 36.2 / 32.8 | 7526 |
| 1440 × 128 | 57.9 % / 55.2 % (about 11 300, essentially every valid return) | 1.0–1.1 % | 39.1 / 36.4 | 7539 |

This is the trade-off §3.3 predicted:
- The coarse image keeps usable normals but throws away about 45 % of valid returns, because several points share each pixel.
- The fine image keeps every return, but almost none of them get a normal.

**Run (i): 360 × 32, `normal_available: true`.** Rate 0.25, 60 s of
bag, `update_esdf_every_n_sec: 5`, and `pointcloud_queue_size: 200`.
The deep queue is needed because each ~9 s ESDF update would otherwise
drop clouds at the subscription. A first attempt without it received
only 358 of 600 clouds.
- 596 of 600 front and 598 of 600 back clouds integrated. Final blocks: **9185**.
- Integration: 26.1 ms front and 23.9 ms back.
- Voxfield ESDF update: **9.2 s** mean over 12 updates.
- RSS peak: **7.3 GB**.
- The ESDF and TSDF slices were captured with 130 k points each.
- Files: `~/voxfield_phase9_handoff/maps/vf_i_360_nt.{tsdf,ply}` and `runs/vf_i_360_nt.{esdf,tsdf}_slice.npy`.

**Runs (ii) and (iii)** use the same command as (i), changing only
`normal_available` and `width`/`height`:
- (ii) 360 × 32, `normal_available: false` (the plan's literal comparison).
- (iii) 1440 × 128, `normal_available: false`. With normals off, a coarse
  image only loses points, so this is the natural pairing.

| Run | Integrated (front / back) | Integration ms (front / back) | ESDF mean | Blocks | Peak RSS |
|---|---|---|---|---|---|
| (i) 360 × 32, normals on | 596 / 598 | 26.1 / 23.9 | 9.2 s | 9185 | 7.3 GB |
| (ii) 360 × 32, normals off | 596 / 598 | 25.3 / 23.1 | 8.7 s | 9185 | 7.3 GB |
| (iii) 1440 × 128, normals off | 596 / 598 | 44.1 / 41.4 | 9.4 s | 9445 | 7.5 GB |

**Mesh quality** (`mesh_quality.py`). The reference is the stationary raw
clouds from both LiDARs at t = 2–6 s (`runs/rawF_2.npy`/`rawB_2.npy`,
already in `map`, 2 cm-deduplicated), scored within 1–8 m horizontally
of the start position, where step 3 showed the two LiDARs agree to
~2 cm. Accuracy is the mesh-vertex → nearest-raw-point distance. Vertices
more than 0.5 m from any raw point are counted as "unsupported" (surfaces
first seen later in the run). Completeness is the fraction of raw points
with a mesh vertex within 0.1 m (one voxel) / 0.2 m.

| Mesh | Accuracy median / p90 | Unsupported | Completeness 0.1 m / 0.2 m |
|---|---|---|---|
| voxblox (c), reference | 0.126 / 0.354 m | 29.0 % | 58.9 % / **82.2 %** |
| voxfield (i) 360 × 32, normals on | **0.103** / 0.304 m | 29.2 % | 57.8 % / 74.3 % |
| voxfield (ii) 360 × 32, normals off | 0.104 / 0.302 m | 28.7 % | 57.9 % / 74.0 % |
| voxfield (iii) 1440 × 128, normals off | 0.116 / 0.350 m | 32.9 % | 53.2 % / 66.3 % |

- (i) and (ii) are indistinguishable. At 360 × 32, the normals neither
  help nor hurt the mesh near the robot on this data.
- (iii) keeps nearly twice as many points and still scores **worse** on
  every metric, at ~1.7× the integration cost. A plausible explanation, not verified:
  with only ~1 % valid normals and one return per pixel, the fine image
  spreads single noisy hits over more voxels instead of averaging several
  per pixel.
- voxfield is ~2 cm more accurate than voxblox's ray casting, and voxblox is
  more complete within 0.2 m. The median accuracy of ~0.1 m is about one
  voxel. Marching-cubes vertices sit on voxel edges, so this is near the
  floor for 0.1 m voxels.

**Default chosen:** 360 × 32 with `normal_available: true`, set explicitly
in `athena_dual_lidar.yaml` and `athena_lidar_rgbd.yaml`, where it sits
next to the resolution it depends on. Quality equals (ii), the cost is the
same, and it matches every other `cfg/param/*.yaml`. Before this,
`athena_param.yaml` didn't set it, so the effective value was the library
default, `false`.

Figure: `docs/assets/multi_sensor_voxfield.png` (run (i): mesh vertices
and the latched ESDF slice at z = 0.55 m).

Command for (i):
```
bash -c 'S=~/voxfield_phase9_handoff; C=~/src/voxfield_ros2/voxfield_ros/cfg/multi_sensor
P9_DRAIN=30 bash $S/p9run.sh /tmp/vf_i voxfield_server 0.25 60 --params-file $C/athena_param.yaml \
  --params-file $C/athena_dual_lidar.yaml -p update_esdf_every_n_sec:=5.0 -p normal_available:=true \
  -p pointcloud_queue_size:=200'
```

### Step 6: RGB-D
Run through `multi_sensor_mapping.launch.py rgbd:=true` (`p9run_rgbd.sh`):
`voxfield_server`, rate 0.25, 60 s of bag, `athena_lidar_rgbd.yaml` (both
LiDARs plus both cameras), ESDF every 5 s, queue 200 (`work/rgbd_vf.*`).

- All 4 sensors integrate. The LiDARs are unchanged from run (i) (596 / 597
  of 599). Each camera received ~267 decoded clouds, throttled 140 of them
  (`min_time_between_msgs_sec: 0.4`) and integrated 126, at 12.5–13.6 ms
  per ~170 k-point cloud. Rays are capped at 4 m and the `merged`
  integrator bundles points per voxel, which is why the cost is lower than
  for the 6 k-point LiDAR clouds with 30 m rays.
- 99 % of camera points get a valid normal in the range image (dense
  640 × 480 depth), against ~65 % for the LiDARs.
- The map, mesh and slices were saved. Peak RSS was 7.4 GB, the same as
  LiDAR-only.
- Noise, not a problem: each `depth_image_proc::PointCloudXyzNode` logs
  `image_transport ... do not appear to be synchronized` every 10 s. The
  bag has depth at 4.5 Hz but `camera_info` at 30 Hz. Every depth image has
  a `camera_info` with the identical stamp (checked over 201 images), so
  every image is paired and the warning is only about the unequal counts.

**Are the cameras placed correctly?** A second run with only the two
cameras (`sensor_names: [front_rgbd, back_rgbd]`, `work/rgbd_only_vf.*`)
gave 30 blocks and 2229 mesh vertices (2 cm-deduplicated). Their distance to
the LiDAR-only run (i)'s mesh: median **0.044 m**, p90 0.078 m, 97.9 %
within one voxel. So the camera intrinsics, the TF chain and the
compressedDepth → points pipeline all put surfaces where the LiDARs see
them. In the 4-sensor map, the cameras add almost nothing (42 of 447 k
vertices are > 15 cm from the LiDAR-only mesh). Everything within the
cameras' 4 m range is already covered by the two 360° LiDARs as the robot
moves, and the step 5 metric scores the two maps identically. On this robot
the cameras' value would be close-range detail below the LiDARs' −8°
field of view, which this 60 s window doesn't exercise.

(A first RGB-D-only attempt aborted at start-up with M13's `'sensors.
back_lidar.topic' names sensor 'back_lidar', which is not in sensor_names`.
That was the test YAML still carrying the LiDAR entries: validation
working as designed.)

### Step 7: `fiesta_server` / `voxedt_server`
One dual-LiDAR run of each: rate 0.25, 60 s of bag,
`pointcloud_queue_size:=200`, ESDF every 1 s (30 m rays, as shipped
before step 4).

**First attempt: both killed at 18 GB.** RSS jumped to ~16 GB within the
first 20 s of wall time, while TSDF layer memory was only ~0.77 GB. The
cause was the dense ESDF allocation in `EsdfOcc{Fiesta,Edt}Integrator::
setLocalRange()`. It's the FIESTA/EDT twin of "Known upstream issues" #12,
which Phase 12 had fixed only in the Voxfield integrator. With 30 m rays
pitched up to 53°, a single update's bounding box is ~38 × 38 × 25 blocks
of ~0.4 MB (`EsdfVoxel` is ~100 B). **Fixed** with the user's
authorization. Details and the bit-for-bit equivalence test
(`voxfield/test/test_esdf_occ_integrators.cc`) are in
`docs/ROS2_PORT_NOTES.md` "Known upstream issues" #13.

**With the fix** (`work/s7fix_*`):

| Server | Integrated (front / back of 600) | Dropped | ESDF update mean (max) | Blocks | Peak RSS |
|---|---|---|---|---|---|
| fiesta | 585 / 586 | 12 / 12 | 3.6 s (10.2) | 9448 | 8.0 GB |
| voxedt | 441 / 448 | 156 / 150 | 7.8 s (16.4) | 9231 | 7.7 GB |

- Both save the map (`save_map`) and mesh, and publish non-empty ESDF/TSDF
  slices (128–137 k points). The only ERROR line is the TF warm-up
  `queue getting too long` one.
- `voxedt`'s drops are overload, not a bug. Its ESDF update (7.8 s mean)
  takes longer than its 1 s bag-time interval (4 s wall at rate 0.25). The
  backlog grows until queued clouds are older than tf2's 10 s cache, and
  those TF lookups fail (`num_dropped` counts only clouds dropped after a
  failed TF lookup with 10 already queued). The shipped real-time config
  (step 4) makes ESDF updates ~1 s.
- Figure: `docs/assets/multi_sensor_fiesta.png` (mesh and ESDF slice). Its
  ESDF matches voxfield's (`multi_sensor_voxfield.png`).

### A shutdown deadlock in `Transformer` (found by a flaky test)
A full `colcon test` run once hit a 60 s timeout in
`TransformerMulti.RetentionWindowErasesOnlyOldEntries`. The test body
can't block. The hang was `tf2_ros::TransformListener(buffer)`'s
destructor. That constructor makes its own node and spins it on a
dedicated thread, and the destructor calls `executor_->cancel()` and then
`join()`. If the `Transformer` is destroyed before that thread has entered
`spin()`, rclcpp's `Executor::spin()` still starts spinning afterwards
and the cancel is lost, so `join()` waits forever. Tests that build and
destroy a `Transformer` within milliseconds hit it occasionally. So would
a server shut down right after start-up. **Fixed:** `Transformer` now owns
the thread. The listener is attached to the server's own node with
`spin_thread=false`, and its `/tf`/`/tf_static` subscriptions (default
listener QoS and options) go in a callback group that isn't added to the
node's executor. That's the same isolation as the listener's own
spin-thread mode, so TF still arrives during long ESDF updates.
`~Transformer()` retries `cancel()` until the thread has really left
`spin()`. New test `TransformerMulti.ImmediateDestructionDoesNotHang`
builds and destroys 300 `Transformer`s: against the old code it hung 5 of 5
runs (30 s timeout), and with the fix the whole suite passes 10 of 10 runs
in 0.9 s.

### Wrap-up
- **Bugs found and fixed in this phase:**
  - The `fiesta`/`voxedt` `~/save_map` → `~/load_map` round trip always
    failed, hidden by the 4 "intentionally" skipped smoke subtests.
  - The FIESTA/EDT dense ESDF allocation used ~16 GB on the first update
    ("Known upstream issues" #13).
  - The `Transformer`/`tf2_ros::TransformListener` shutdown deadlock.
- **Config changes:**
  - Measured `fov_up`/`fov_down` (step 1).
  - `normal_available: true` at 360 × 32 (step 5).
  - Real-time `athena_param.yaml`: 12 m rays, ESDF every 2 s, queue 10
    (step 4).
- **Figures:** `docs/assets/multi_sensor_{voxblox,wall_slice,voxfield,fiesta}.png`.
- **Not done / recommended:**
  - Calibrate the back LiDAR's extrinsic (~1.4°, step 3).
  - A multi-threaded executor with TSDF/ESDF locking, to get full-range
    (30 m) mapping in real time (step 4).
- **Full suite:** `Summary: 170 tests, 0 errors, 0 failures, 0 skipped`
  (164 after the smoke-coverage fix, plus the new FIESTA/EDT integrator
  and `Transformer` tests).

## Phase 10: Docs and cleanup

**README.** New "Multiple sensors (one map)" section:
- The concept, legacy vs multi-sensor mode, and an example config (the
  measured Athena values, not the plan's §7.5 placeholders).
- The per-sensor key table and the map-global and new global keys.
- The frame-counting (M12) and ICP (M11) caveats.
- `multi_sensor_mapping.launch.py` with `tf_remap_prefix`, and the
  RGB-D-via-`depth_image_proc` recipe.

The `pointcloud` input row now says "legacy mode only". The pose
paragraph mentions F1 (an empty `sensor_frame` uses `header.frame_id`).
The services table was already updated for the `fiesta`/`voxedt`
`save_map` fix.

**Where the decisions are recorded** (all in this file):
- F1 (empty frame → the cloud's own frame, instead of failing forever):
  Phases 3 and 5.
- M6 (the transform queue keeps a retention window instead of erasing
  everything before each lookup), with the test that fails on the old
  code: Phase 3.
- M9 body frame: Phases 5 and 7. M11/M10: Phase 5. M12: Phase 8's configs
  and the README.
- Real-bag measurements and defaults: Phase 9.

**Legacy `ros2 param dump` diff (pitfall 1).** Never recorded in Phases
2–6, so it was done here. Commit `82eed25` (Phase 1, before the refactor)
was built in a worktree with its own build and install directories. The
same legacy servers were started from both builds with `kitti_param.yaml`
+ `kitti_calib.yaml` (no `sensor_names`), and `ros2 param dump
/voxfield_node` was diffed. `voxfield_server`, `voxblox_server`,
`fiesta_server` and `np_tsdf_server` all differ in exactly the same way:
- `body_frame: ''` and `transform_queue_retention_sec: 1.0`, the two new
  globals the plan allows.
- `qos_overrides./tf.subscription.*` and
  `qos_overrides./tf_static.subscription.*` (depth 100, reliable;
  read-only). rclcpp declares these for the TF listener's subscriptions.
  Since the Phase 9 `Transformer` deadlock fix, those subscriptions live
  on the server's node instead of tf2's hidden internal node, so they now
  appear in the dump. The values are the listener's defaults, and
  behavior is unchanged.

No existing parameter changed its name, type or default.

**Pre-existing bugs fixed in Phase 10** (the user asked for every
documented bug to be fixed, including ones earlier plans put out of
scope):
- `RayCaster` zero-component guard ("Known upstream issues" #14):
  axis-aligned rays repeated their first voxel and dropped their last one.
  The legacy TSDF golden file was regenerated (617 voxel values; same
  blocks).
- `test_tsdf_map`'s `BlockAllocation` aborted in Debug builds (a
  `DCHECK`). It was checking idempotency with the "new" allocation API. In
  a Debug build of `voxfield`, 11 tests now pass: `test_tsdf_map`,
  `test_protobuf`, `test_layer`, `test_approx_hash_array`,
  `test_bucket_queue`, `test_ray_caster`, `test_esdf_voxfield_integrator`,
  `test_esdf_occ_integrators`, `test_tsdf_interpolator`, `test_layer_utils`
  and `test_merge_integration`, and later `test_clear_spheres` too.
  `test_sdf_integrators` takes over half an hour unoptimized, and its
  Debug result wasn't recorded.
- The `setLocalRange()` block ranges (Voxfield, FIESTA, EDT) now floor
  explicitly. The previous `int64 / size_t` division was only correct by
  accident (`docs/BUG_voxfield_server_camera_mode_memory.md`).
- `Layer::isCompatible()`'s warning printed the loaded and current layer
  types swapped.
- Timing labels: the voxblox ESDF integrator reported as
  `upate_esdf/voxfield/...` (a rename-script mislabel plus a typo). It is
  now `update_esdf/voxblox/...`, and EDT's is `update_esdf/edt/...`.
- Default node names collided: `tsdf_server`, `voxblox_server` and
  `intensity_server` were all `voxblox`, and `np_tsdf_server` and
  `voxfield_server` were both `voxfield`, as in ROS 1. Two such nodes
  started side by side would share a name, and so their parameters and
  services. `tsdf_server`, `np_tsdf_server` and `intensity_server` now
  default to `tsdf`, `np_tsdf` and `intensity`. The launch files always set
  `name="voxfield_node"`, so only a bare `ros2 run` is affected.

**LiDAR point weighting (user's decision: range weighting).**
`TsdfIntegratorBase::getVoxelWeight()` (used by the `tsdf`, `voxblox`,
`fiesta`, `voxedt` and `intensity` servers) weighted every point by
`1 / z²` in the sensor frame, a depth-camera model. For a LiDAR, z is
height: points near its horizontal plane got huge weights (10000 at
z = 1 cm), and points with z = 0 were dropped. That was the real cause of
Phase 7's "a point at (x, 0, 0) only allocates the origin block". Every
LiDAR preset sets `use_const_weight: false`. Three options were put to the
user: range weighting for LiDARs, constant weighting in the configs, or
leaving it. The user chose range weighting ("Known upstream issues" #15 in
`docs/ROS2_PORT_NOTES.md`):
- LiDARs (`sensor_is_lidar: true`) now use `1 / range^weight_reduction_exp`,
  the NP integrator's model.
- Cameras keep `1 / z²`.
- `lidar_z_weighting: true` restores the old behavior.

Real bag, `voxblox_server`, rate 0.25, 60 s of bag, 30 m rays, ESDF off,
scored with step 5's metric (`work/w_*`):

| Run | Vertices | Accuracy median / p90 | Completeness 0.1 m / 0.2 m |
|---|---|---|---|
| both LiDARs, old `1/z²` (`lidar_z_weighting:=true`) | 68 366 | 0.126 / 0.355 m | 58.7 % / 82.0 % |
| both LiDARs, range weighting | 42 795 | 0.116 / 0.352 m | 53.0 % / 65.9 % |
| both LiDARs, range weighting + `anti_grazing` | 42 680 | 0.119 / 0.354 m | 51.4 % / 65.4 % |
| front only, old `1/z²` | 55 352 | 0.115 / 0.324 m | 52.0 % / 80.1 % |
| front only, range weighting | 40 141 | **0.094** / 0.309 m | **53.2 %** / 68.3 % |

The `lidar_z_weighting` runs reproduce Phase 9's (c) and (a) almost
exactly (68 301 / 55 371 vertices, same metric values), so the switch is
faithful.

Why the old model looked more complete: parsing the saved TSDFs
(`tsdf_weights.py`) shows 5.7 % of all observed voxels (6.6 % of
near-surface ones), 1.35 M voxels, pinned at `max_weight` (10000) under
`1/z²`. A single near-horizontal point saturates a voxel, and no later
observation can move it. Under range weighting none are at the cap (p99
weight 110), so later rays can revise surfaces.
- **Single LiDAR:** range weighting is 2 cm more accurate and at least as
  complete within one voxel. The old model's extra completeness only
  appears at 0.1–0.2 m: thick or offset surfaces frozen in place, not
  extra real detail.
- **Both LiDARs:** completeness within 0.1 m also drops (58.7 → 53.0 %).
  That is the back LiDAR's ~1.4° extrinsic error (step 3). Its rays now
  carve the front LiDAR's surfaces at range instead of being masked by
  saturated voxels. `anti_grazing` doesn't change it, so it isn't
  within-scan grazing.

Calibrating the back LiDAR is the fix for that. The shipped 12 m
`max_ray_length_m` also keeps most of it out.

**Formatting and builds.** `clang-format` was run on the 14 C++ files this
branch changed that were clean on its base. `test_server_map_io.cc`'s 6
unformatted lines predate the branch, so it was left alone. `colcon build`
succeeds both in the clean environment (`scripts/clean_env.sh`) and from
scratch with the Hector underlay sourced (separate build and install
directories). Full suite after the Phase 10 fixes above: `Summary: 176
tests, 0 errors, 0 failures, 0 skipped`, and `Summary: 184 tests, 0
errors, 0 failures, 0 skipped` after the LiDAR weighting change.

**Follow-ups (not in this plan):**
- Multi-sensor ICP: one designated `icp_sensor` whose correction applies
  to all sensors (M11).
- Motion deskew of LiDAR sweeps, since the robot moves during a 100 ms
  Livox scan.
- A self-filter (robot body box). Phase 9 found no persistent self-hits
  on Athena with `min_ray_length_m: 0.3`, but other robots may differ.
- Per-sensor voxel downsampling before integration. Dense RGB-D clouds
  currently rely on the `merged` integrator's per-voxel bundling and on
  throttling.
- `CameraInfo`-driven intrinsics: a per-sensor `camera_info_topic` instead
  of `fx`/`fy`/`vx`/`vy` copied into YAML.
- A multi-threaded executor, running the ESDF update off the subscription
  thread with TSDF/ESDF locking, for full-range (30 m) real-time mapping
  (Phase 9 step 4).
- Calibrate the Athena back LiDAR's extrinsic (~1.4°, Phase 9 step 3).
- Livox's UINT8 `intensity` field doesn't match PCL's FLOAT32 `PointXYZI`
  (`Failed to find match for field 'intensity'`, cosmetic). A converter
  would restore intensity colouring.

## End-to-end runs in RViz2 (after Phase 10)

`multi_sensor_mapping.launch.py` was run with bag playback and RViz2 on
four Athena bags with `voxfield_server`: `rosbag2_2026_09_23-14_32_47`
(1× with the shipped 12 m config, and 0.5× with 30 m rays),
`-14_11_34`, `-13_40_18` (both 1×), and `maze_semi_finals_1_bag`.
Memory was sampled only once, early in `-13_40_18` (about 2 GB), but no
run showed memory trouble.

Three bugs showed up only in the live RViz2 view, all fixed with the
tests noted:
- **The mesh never rendered.** `voxfield_rviz_plugin` passed no resource
  group to `ManualObject::begin()`. Ogre 1.9 (ROS 1) searched all
  groups; Ogre 1.12+ (`rviz_ogre_vendor`) only `General`, while the
  materials live in `VoxfieldMaterials`. A port bug, found only by looking.
- **The robot-model marker was published even with
  `publish_robot_model: false`** ("Known upstream issues" #16), which
  flooded RViz2 with `Could not load resource [file://]`. New
  `RobotModelMarker` tests.
- **The raw point cloud displays covered the mesh and ESDF slice.**
  `multi_sensor.rviz` now starts with them off.

Maze bag: its localization diverges at about 24–55 s, so it was played
from `start_offset:=56`. The bag records `/tf_static` only at start-up
(12 messages), so the static transforms were replayed separately once
the server was up (`ros2 bag play ... --topics /athena/tf_static
--playback-duration 10` with `tf_static_qos_override.yaml`). The map then
built normally.

**Final state:** `Summary: 188 tests, 0 errors, 0 failures, 0 skipped`.
The README has a quick start for Athena bags and for other robots.
