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
regression, since nothing in this phase touches `Transformer`. `git status`
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

**Full suite:** `Summary: 147 tests, 0 errors, 0 failures, 4 skipped` (143
+ 3 new `test_multi_sensor_server_np_tsdf` cases + 1 new CTest aggregate
entry). `test_legacy_golden_np_tsdf` and all 8 `test_np_tsdf_server` cases
passed with zero edits. `git status` after this phase touches
`np_tsdf_server.{h,cc}`, `sensor_config_loader.cc` (the log fix),
`CMakeLists.txt`, and `test_multi_sensor_server.cc`.
