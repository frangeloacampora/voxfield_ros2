# Bug: `voxfield_server` runaway memory growth (tens of GB in seconds) on real robot data

**Status:** root-caused and **fixed** (2026-09-24), not yet committed. The
bug is in the Voxfield ESDF integrator's local-range allocation
(`EsdfVoxfieldIntegrator::setLocalRange()`). It was triggered by a
diverging pose estimate in the maze bag's own TF. It is **pre-existing
upstream behavior** (identical in the ROS 1 original), not a port
regression. The fix is a deliberate, user-authorized deviation from the
port's "document, don't fix" rule. See "Known upstream issues" #12 in
`docs/ROS2_PORT_NOTES.md`.

The original title ("camera-mode integration ...") was misleading.
Camera mode (`sensor_is_lidar: false`) is **not** the cause. See
"Corrections to the original write-up" below.

**Safety note (still applies to any live experiment):** before running any
mapping server against a real bag, put it under a hard memory cap. Use a
cgroup, e.g. `systemd-run --user --scope -p MemoryMax=1500M -p
MemorySwapMax=0 <real binary> --ros-args ...`, which kills a runaway
inside that scope only, or `ulimit -v`. Also poll the real binary's
`VmRSS`, not the `ros2 run` wrapper's (see "Diagnostic methodology
pitfall"). Before this fix, the incident below twice made the kernel
OOM-killer kill processes in the same cgroup as the user's VS Code
session.

## Symptom (as originally observed)

`voxfield_server` (`sensor_is_lidar: false`, 640x480 organized depth-derived
`PointCloud2`, `voxel_size: 0.15`, `update_esdf_every_n_sec: 1`) running on
`/home/frangeloac/src/maze_semi_finals_1_bag/`:

- RSS stayed at tens of MB for about 25-30 s.
- RSS then grew by about 0.5 GB/s to 20-25 GB, until something killed it.

The `voxblox_server` instances on the same robot's lidars stayed small.

## Root cause

### 1. Trigger: the bag's localization diverges

The robot's own pose estimate in this bag diverges. The `map ->
sensor_init -> base_footprint_link -> ...` TF chain was read offline from
the bag with `rosbag2_py` + `tf2_ros.Buffer`; the script is
`scratchpad/rc/tfcheck.py` in the debugging session. Position of
`back_rgbd_color_optical_frame` in `map`:

| bag time | position (m) | motion |
|---|---|---|
| 0-24 s | about (6, -0.9, -9.8) | essentially static |
| 24-29 s | runs away | 5-50 m/s |
| 29-54 s | reaches (670, -700, **-3300**) | accelerating to about 250 m/s |
| 54.4-54.9 s | snaps back to about (0, 0, 0) | reset |
| after 55 s | about (0, 0, 0) | sane |

This is bad input data in the bag (an upstream state-estimator blowup),
not something the server does. The other two local bags have no such
divergence:

- `rosbag2_2026_09_23-12_40_03`: stationary the whole time.
- `rosbag2_2026_09_23-13_40_18`: smooth motion at about 0.5 m/s.

### 2. Mechanism: dense bounding-box allocation in `setLocalRange()`

`EsdfVoxfieldIntegrator::updateFromTsdfBlocks()` collects every voxel whose
occupancy flipped since the last ESDF update (`insert_list_` /
`delete_list_`). `getUpdateRange()` takes their axis-aligned bounding box.
`setLocalRange()` then pads the box by `local_range_offset_{x,y,z}` and
**allocated every ESDF block inside it**:

```cpp
for (int x = block_range_min(0); x <= block_range_max(0); x++)
  for (int y = ...; ...)
    for (int z = ...; ...)
      esdf_layer_->allocateBlockPtrByIndex(BlockIndex(x, y, z));
```

These blocks are never freed. `EsdfVoxel` is about 160 bytes (five
`GlobalIndex` members). At `voxels_per_side: 8`, that is about 82 KB per
block.

The allocated volume grows with the **cube** of the distance between the
places that were updated in one ESDF interval. When the pose moves tens to
hundreds of metres per second in x, y and z, one 1 s ESDF update covers a
box of millions of blocks. The server's own `timing: true` log from the
original incident shows this: `update_esdf/voxfield/allocate_vox` took
0.36 s, then 1.36 s, then 5.5 s per call, starting at the 18th ESDF
update. Meanwhile `update_esdf/voxfield/update`, the actual ESDF
propagation, stayed in the microseconds. The node's time went into
allocating and constructing empty blocks. The flat-then-linear RSS curve
is this allocation loop running at memory-bandwidth speed.

`voxblox_server` (the lidar servers) uses Voxblox's `EsdfIntegrator`. It
only allocates ESDF blocks next to updated TSDF blocks, so the same
diverging TF gives it a thin tube of blocks along the bogus trajectory
instead of a filled cube. That is why it stayed small.

### 3. Why the dense allocation was unnecessary

`updateESDF()` only ever reads or writes voxels that are `observed`:

- Every neighbor access checks `nbr_vox->observed` before using the
  voxel.
- List and queue members are observed by construction.

A voxel becomes observed only in `updateFromTsdfBlocks()`, which already
allocates that voxel's ESDF block (`allocateBlockPtrByIndex(block_index)`
for every updated TSDF block). So every block that only `setLocalRange()`
would allocate holds nothing but unobserved default voxels, which the
algorithm skips anyway. Visualization (`ptcloud_vis.h`) and evaluation
also filter on `observed`. The only reason for the loop was so that the
`CHECK_NOTNULL(nbr_vox)` calls in `updateESDF()` would not fire. The
upstream comment above the adjacent `allocate_tsdf_in_range` branch
already says "TO CHECK !!! ... might be not neccessary".

## Fix

The fix is in `voxfield/src/integrator/esdf_voxfield_integrator.cc`:

- **`setLocalRange()`** still computes `range_min_`/`range_max_`, which
  `voxInRange()` uses. It now returns before the dense allocation loop
  unless `config_.allocate_tsdf_in_range` is set. That option is opt-in,
  off by default, and not exposed as a ROS parameter; it needs the loop to
  allocate TSDF blocks, so it keeps the legacy behavior. A comment at that
  point explains the deviation.
- **`updateESDF()`** has three neighbor lookups: the delete pass, the
  patch step, and the main BFS. In each, `CHECK_NOTNULL(nbr_vox)` became
  `if (nbr_vox == nullptr) continue;`, so an unallocated block counts as
  an unobserved neighbor.
- **`resetFixed()`** is deprecated and unused, but it also iterated over
  the range assuming dense allocation. It got the same null guard.

Nothing else changed. No parameters, no public API, no server code.

## Verification

1. **Bit-for-bit equivalence with the upstream algorithm.** A scratch
   harness integrates synthetic RGB-D-like frames with
   `MergedNpTsdfIntegrator`. The frames model a room plus a sphere,
   320x240, 43% invalid pixels, depth noise, a moving and yawing camera,
   and the camera-mode config from the incident (`finer_esdf_on`,
   `patch_on`, `early_break`, offsets 20/20/5). ESDF updates every 5
   frames. The harness is linked once against the upstream `.cc` and once
   against the fixed `.cc`, and dumps every observed ESDF voxel's
   `distance`, `raw_distance`, `coc_idx`, `behind` and `fixed`.

   The dumps were **identical** in all 7 scenarios. The scenarios covered
   75k to 1.27M observed voxels, voxel sizes 0.05/0.1/0.15 m, a single
   pose jump, and a continuous pose drift. ESDF block counts dropped
   2.4-8.4x, from 870 to 275 and up to 26803 to 3185. Peak RSS dropped
   2-7x. With the ESDF block count equal to the TSDF block count, the
   ESDF layer no longer adds memory beyond the TSDF's footprint.

   In a faster-drift scenario under `ulimit -v 2000000`, the upstream code
   hit the cap and aborted. The fixed code finished at 378 MB.
2. **New regression gtest** `voxfield/test/test_esdf_voxfield_integrator.cc`
   (3 tests):
   - Two surface patches about 10 m apart per axis must allocate exactly 2
     ESDF blocks. The upstream code allocates 2475, about 200 MB, which is
     deliberately small enough that a regression fails the test instead
     of OOMing CI.
   - ESDF distances around a plane must be exact.
   - An occupied-to-free delete pass across a distant update must leave
     the reset voxels at the default distance with no closest-occupied
     link, and still allocate only 2 blocks.

   Against the upstream `.cc`, the two memory tests fail and the distance
   test passes. Against the fix, all 3 pass.
3. **Full test suite:** `scripts/clean_env.sh colcon build --symlink-install`,
   then `scripts/clean_env.sh colcon test` and `colcon test-result
   --verbose`. Result: 84 tests, 0 errors, 0 failures, 4 skipped. That is
   the previous 80, plus the 3 new gtests and their CTest entry. The 4
   skipped are the same intentional skips as before. This includes the
   Phase 12 launch smoke test that runs `voxfield_server` end to end.
4. **Guarded live runs** of the rebuilt `voxfield_server`. Same pipeline
   and param file as the incident, the server inside a `systemd-run
   --user --scope -p MemoryMax=1500M -p MemorySwapMax=0` cgroup, plus a
   1 s `VmRSS` poll of the real binary that kills at 1.2 GB:

   | bag | code | peak RSS | notes |
   |---|---|---|---|
   | maze, 85 s, covering the whole divergence | fixed | **139 MB** | 243 frames integrated, 62 ESDF updates, max `allocate_vox` 0.13 ms (5.5 s before) |
   | maze | upstream ESDF `.cc` (via `LD_PRELOAD`) | killed | 100 MB, then 389 MB at t=24 s, then 1.24 GB at t=25 s, then killed by the watchdog, exactly at the divergence onset. Confirms the causal chain end to end. |
   | `13_40_18`, 90 s | fixed | 92 MB | |
   | `12_40_03`, 90 s | fixed | 89 MB | |

## Corrections to the original write-up

- **"Camera mode vs. lidar mode" was a confound.** The "lidar-mode
  control" runs used `voxblox_server` (`TsdfServer` + Voxblox
  `EsdfIntegrator`), not `voxfield_server` with `sensor_is_lidar: true`.
  The real difference was Voxfield's ESDF integrator versus Voxblox's.
  `voxfield_server` in lidar mode on the same TF would hit the same
  allocation. `np_tsdf_server`, which has no ESDF integrator, is not
  affected by this mechanism.
- **"Reproduced against two different bags" is not borne out for the
  second bag.** The guarded runs above used the incident's param file
  (`voxel_size: 0.15`) on both non-maze bags for 90 s. Neither grew with
  either the upstream or the fixed ESDF code: upstream stayed at 138 MB
  or less on `13_40_18`. Those bags' camera poses do not jump. The
  earlier incidents on those bags were detected by watchdogs that
  measured **system-wide** `free` usage, not the server's RSS. They also
  ran several servers at once, sometimes at `voxel_size: 0.05`. See
  "Open" below.
- The `projectPointToImageCamera` bool-as-depth bug (Known upstream issues
  #2), the int-typed intrinsics (#3), and the `computeNormalImage`
  out-of-bounds row (#1) are real. None of them contributes to this
  memory growth. All three have since been fixed separately; see those
  items in `docs/ROS2_PORT_NOTES.md`.
- The leading hypothesis about `local_range_offset_x/y/z: 20` was
  partially right. The offsets only pad the box. The box itself spans the
  diverging trajectory, and that is what explodes. Offsets of 20 voxels
  on a 0.15 m grid are merely wasteful. With the fix, they cost nothing
  beyond the `voxInRange()` bounds.

## Open / not fixed here

- **The maze bag's localization divergence itself** (bag time about
  24-55 s) is bad input. Any map built over that window is garbage for
  every server, now with bounded memory for `voxfield_server` as well.
  Use `--start-offset 56` or later with `ros2 bag play` to skip it for
  validation runs.
- **The earlier incidents attributed to `rosbag2_2026_09_23-12_40_03` /
  `-13_40_18` are unexplained.** They could not be reproduced with the
  current param file. They were measured system-wide while several
  servers ran concurrently, some at 0.05 m voxels. If they come back, the
  first step is to measure per-process RSS of the real binaries under a
  cgroup cap.
- `setLocalRange()`'s legacy loop, still used only when
  `allocate_tsdf_in_range` is set, converts voxel indices to block indices
  with C++ integer division, which truncates toward zero instead of
  flooring. For negative coordinates, one block row can be missed at the
  low edge. Upstream behavior, off by default, left as-is.

## Diagnostic methodology pitfall (so you don't repeat it)

`ros2 run <pkg> <exe> ... &` backgrounds the **Python wrapper**
(`/opt/ros/jazzy/bin/ros2`), not the actual C++ binary. The real
`voxfield_server`/`voxblox_server` executable is a **child process** with a
different PID, found under
`/home/frangeloac/voxfield_ws/install/voxfield_ros/lib/voxfield_ros/`.
Monitoring `$!` sees a flat ~29 MB while the real process grows. Find the
real PID with `pgrep -f 'install/voxfield_ros/lib/voxfield_ros/<name>'`,
or launch the binary directly, as the verification runs above did:
`systemd-run --user --scope ... <full path to binary> --ros-args ...`.
