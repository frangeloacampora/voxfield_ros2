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
