#include <cmath>
#include <gtest/gtest.h>

#include "voxfield/core/layer.h"
#include "voxfield/core/voxel.h"
#include "voxfield/integrator/tsdf_integrator.h"

namespace voxfield {

// TsdfIntegratorBase's point weight (use_const_weight: false). Upstream used
// 1 / z^2 for every sensor: right for a depth camera, where sensor-frame z
// is depth, but wrong for a LiDAR, where z is height -- a point level with
// the LiDAR got weight 0 and was dropped, and points near that plane got
// huge weights. LiDARs now use 1 / range^weight_reduction_exp, like
// NpTsdfIntegratorBase (docs/MULTI_SENSOR_NOTES.md, Phase 10).
namespace {

constexpr FloatingPoint kVoxelSize = 0.1f;

// Integrates one point (sensor at the origin) and returns the weight of the
// voxel containing it, or -1 if that voxel was never allocated.
float weightAtPoint(const TsdfIntegratorBase::Config& config, const Point& p) {
  Layer<TsdfVoxel> layer(kVoxelSize, 16u);
  MergedTsdfIntegrator integrator(config, &layer);
  const Pointcloud points_C = {p};
  const Colors colors(1u);
  integrator.integratePointCloud(Transformation(), points_C, colors);
  const TsdfVoxel* voxel = layer.getVoxelPtrByCoordinates(p);
  return voxel == nullptr ? -1.0f : voxel->weight;
}

TsdfIntegratorBase::Config makeConfig(bool lidar) {
  TsdfIntegratorBase::Config config;
  config.default_truncation_distance = 0.3f;
  config.max_ray_length_m = 20.0f;
  config.use_const_weight = false;
  config.sensor_is_lidar = lidar;
  config.weight_reduction_exp = 1.0f;
  return config;
}

TEST(PointWeight, LidarUsesRange) {
  const TsdfIntegratorBase::Config config = makeConfig(true);
  // Level with the sensor (z == 0): used to be dropped entirely.
  EXPECT_NEAR(
      weightAtPoint(config, Point(5.03f, 0.0f, 0.0f)), 1.0f / 5.03f, 1e-5f);
  // Slightly above the plane: used to get 1 / 0.01^2 = 10000.
  const Point p(3.03f, 4.0f, 0.01f);
  EXPECT_NEAR(weightAtPoint(config, p), 1.0f / p.norm(), 1e-5f);
}

TEST(PointWeight, LidarExponentIsConfigurable) {
  TsdfIntegratorBase::Config config = makeConfig(true);
  config.weight_reduction_exp = 2.0f;
  EXPECT_NEAR(
      weightAtPoint(config, Point(5.03f, 0.0f, 0.0f)), 1.0f / (5.03f * 5.03f),
      1e-6f);
}

TEST(PointWeight, CameraKeepsDepthWeight) {
  const TsdfIntegratorBase::Config config = makeConfig(false);
  // Depth along the optical axis (z), not range.
  EXPECT_NEAR(
      weightAtPoint(config, Point(1.03f, 0.0f, 5.03f)), 1.0f / (5.03f * 5.03f),
      1e-6f);
}

TEST(PointWeight, LidarZWeightingRestoresUpstream) {
  TsdfIntegratorBase::Config config = makeConfig(true);
  config.lidar_z_weighting = true;
  // Upstream behavior: a point with z == 0 gets weight 0 and is skipped.
  EXPECT_EQ(weightAtPoint(config, Point(5.03f, 0.0f, 0.0f)), -1.0f);
  const Point p(3.03f, 4.0f, 0.5f);
  EXPECT_NEAR(weightAtPoint(config, p), 1.0f / (0.5f * 0.5f), 1e-4f);
}

TEST(PointWeight, ConstWeightWins) {
  TsdfIntegratorBase::Config config = makeConfig(true);
  config.use_const_weight = true;
  EXPECT_NEAR(weightAtPoint(config, Point(5.03f, 0.0f, 0.0f)), 1.0f, 1e-6f);
}

}  // namespace
}  // namespace voxfield
