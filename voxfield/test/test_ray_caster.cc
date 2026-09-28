#include <gtest/gtest.h>
#include <vector>

#include "voxfield/core/layer.h"
#include "voxfield/core/voxel.h"
#include "voxfield/integrator/integrator_utils.h"
#include "voxfield/integrator/tsdf_integrator.h"

namespace voxfield {

// Regression tests for RayCaster::setupRayCaster()'s zero-component guard.
// Upstream (identical in the ROS 1 original and in voxblox) checked
// `std::abs(ray_scaled.x()) < 0.0`, which is never true, so a ray with an
// exactly-zero direction component divided by zero: t_to_next_boundary_ got
// -inf/NaN on that axis, nextRayIndex() kept picking that axis, and its step
// sign is 0, so the ray never left its first voxel. Any point exactly on a
// coordinate axis or plane through the sensor was integrated as a single
// voxel at the sensor instead of along its ray (docs/MULTI_SENSOR_NOTES.md
// Phase 7 found it; fixed in Phase 10).
namespace {

std::vector<GlobalIndex> traverse(const Point& start, const Point& end) {
  RayCaster ray_caster(start, end);
  std::vector<GlobalIndex> indices;
  GlobalIndex index;
  while (ray_caster.nextRayIndex(&index)) {
    indices.push_back(index);
  }
  return indices;
}

// A correct 3D-DDA traversal starts in the start voxel, ends in the end
// voxel, and moves exactly one voxel along exactly one axis per step.
void expectValidTraversal(const Point& start, const Point& end) {
  SCOPED_TRACE(
      ::testing::Message() << "start " << start.transpose() << " end "
                           << end.transpose());
  const std::vector<GlobalIndex> indices = traverse(start, end);
  const GlobalIndex start_index = getGridIndexFromPoint<GlobalIndex>(start);
  const GlobalIndex end_index = getGridIndexFromPoint<GlobalIndex>(end);
  const size_t expected_size =
      static_cast<size_t>((end_index - start_index).cwiseAbs().sum()) + 1u;
  ASSERT_EQ(indices.size(), expected_size);
  EXPECT_EQ(indices.front(), start_index);
  EXPECT_EQ(indices.back(), end_index);
  for (size_t i = 1u; i < indices.size(); ++i) {
    EXPECT_EQ((indices[i] - indices[i - 1]).cwiseAbs().sum(), 1)
        << "step " << i << ": " << indices[i - 1].transpose() << " -> "
        << indices[i].transpose();
  }
}

TEST(RayCaster, AxisAlignedRays) {
  // Two zero components: the case Phase 7 hit.
  const Point start(0.3f, 0.2f, 0.1f);
  for (int axis = 0; axis < 3; ++axis) {
    for (const FloatingPoint sign : {1.0f, -1.0f}) {
      Point end = start;
      end[axis] += sign * 5.0f;
      expectValidTraversal(start, end);
    }
  }
}

TEST(RayCaster, AxisAlignedRaysFromVoxelBoundary) {
  // Starting exactly on a boundary makes the zero-component distance 0/0.
  const Point start(0.0f, 0.0f, 0.0f);
  for (int axis = 0; axis < 3; ++axis) {
    for (const FloatingPoint sign : {1.0f, -1.0f}) {
      Point end = start;
      end[axis] += sign * 4.5f;
      expectValidTraversal(start, end);
    }
  }
}

TEST(RayCaster, RaysInACoordinatePlane) {
  // One zero component.
  expectValidTraversal(Point(0.3f, 0.2f, 0.1f), Point(4.8f, -2.9f, 0.1f));
  expectValidTraversal(Point(0.0f, 0.0f, 0.0f), Point(-3.7f, 0.0f, 2.2f));
  expectValidTraversal(Point(1.5f, 0.5f, -0.5f), Point(1.5f, 6.1f, -4.4f));
}

TEST(RayCaster, GeneralRaysAreUnchanged) {
  // No zero component: the guard never applied here, before or after.
  expectValidTraversal(Point(0.3f, 0.2f, 0.1f), Point(5.3f, 0.7f, 0.437f));
  expectValidTraversal(Point(-1.2f, 3.4f, 0.9f), Point(2.6f, -1.1f, -3.3f));
}

TEST(RayCaster, MergedIntegratorIntegratesAxisAlignedRay) {
  // End-to-end: a point on the sensor's x axis must be integrated along its
  // whole ray, including the truncation band behind the surface.
  constexpr FloatingPoint kVoxelSize = 0.1f;
  constexpr size_t kVoxelsPerSide = 16u;
  Layer<TsdfVoxel> layer(kVoxelSize, kVoxelsPerSide);
  TsdfIntegratorBase::Config config;
  config.default_truncation_distance = 0.3f;
  config.max_ray_length_m = 20.0f;
  config.voxel_carving_enabled = true;
  // The default 1/z^2 point weight is a depth-camera model: a point with
  // sensor-frame z == 0 gets weight 0 and is skipped before any ray is
  // cast. Constant weights isolate the ray traversal under test.
  config.use_const_weight = true;
  MergedTsdfIntegrator integrator(config, &layer);

  // Sensor inside a voxel (not on a boundary, where the old code's 0/0 NaN
  // happened to be ignored by minCoeff()), looking along its x axis.
  const Point sensor_G(0.33f, 0.27f, 0.41f);
  const Transformation T_G_C(sensor_G, Rotation());
  const Pointcloud points_C = {Point(5.0f, 0.0f, 0.0f)};
  const Colors colors(1u);
  integrator.integratePointCloud(T_G_C, points_C, colors);

  const Point point_G = sensor_G + Point(5.0f, 0.0f, 0.0f);
  const TsdfVoxel* voxel = layer.getVoxelPtrByCoordinates(point_G);
  ASSERT_NE(voxel, nullptr) << "block at the measured point not allocated";
  EXPECT_GT(voxel->weight, 0.0f);
  EXPECT_NEAR(voxel->distance, 0.0f, kVoxelSize);
  // The ray continues one truncation distance behind the surface. The old
  // code wasted one step per zero axis and so stopped two voxels short.
  const TsdfVoxel* behind_voxel =
      layer.getVoxelPtrByCoordinates(point_G + Point(0.25f, 0.0f, 0.0f));
  ASSERT_NE(behind_voxel, nullptr);
  EXPECT_GT(behind_voxel->weight, 0.0f) << "voxel behind the surface missed";
  EXPECT_LT(behind_voxel->distance, 0.0f);
  // Free space along the ray (carving) is observed too.
  const TsdfVoxel* free_voxel =
      layer.getVoxelPtrByCoordinates(sensor_G + Point(2.5f, 0.0f, 0.0f));
  ASSERT_NE(free_voxel, nullptr);
  EXPECT_GT(free_voxel->weight, 0.0f);
  EXPECT_GT(free_voxel->distance, 0.0f);
}

}  // namespace
}  // namespace voxfield
