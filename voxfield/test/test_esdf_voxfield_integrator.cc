#include <gtest/gtest.h>

#include "voxfield/core/layer.h"
#include "voxfield/core/voxel.h"
#include "voxfield/integrator/esdf_voxfield_integrator.h"

namespace voxfield {

// Regression test for docs/BUG_voxfield_server_camera_mode_memory.md:
// EsdfVoxfieldIntegrator::setLocalRange() used to allocate every ESDF block
// in the bounding box of all voxels whose occupancy changed since the last
// update, so memory grew with the cube of the distance between updated
// regions (a fast-moving sensor or a pose jump produced tens of GB within
// seconds). It must now only allocate ESDF blocks backed by TSDF blocks,
// without changing the resulting distances.
class EsdfVoxfieldIntegratorTest : public ::testing::Test {
 protected:
  static constexpr FloatingPoint kVoxelSize = 0.15f;
  static constexpr size_t kVoxelsPerSide = 8u;
  // Local z index of the occupied plane inside each patch block.
  static constexpr int kPlaneZ = 3;

  EsdfVoxfieldIntegratorTest()
      : tsdf_layer_(kVoxelSize, kVoxelsPerSide),
        esdf_layer_(kVoxelSize, kVoxelsPerSide) {}

  // Fill one TSDF block with a horizontal surface at local z == kPlaneZ:
  // every voxel observed, signed distance = height above the plane.
  void addPlanePatch(const BlockIndex& block_index) {
    Block<TsdfVoxel>::Ptr block =
        tsdf_layer_.allocateBlockPtrByIndex(block_index);
    for (size_t i = 0u; i < block->num_voxels(); ++i) {
      const VoxelIndex voxel_index = block->computeVoxelIndexFromLinearIndex(i);
      TsdfVoxel& voxel = block->getVoxelByLinearIndex(i);
      voxel.distance = (voxel_index.z() - kPlaneZ) * kVoxelSize;
      voxel.weight = 1.0f;
    }
    block->setUpdatedAll();
  }

  EsdfVoxfieldIntegrator::Config makeConfig() const {
    EsdfVoxfieldIntegrator::Config config;
    // Same local-range margin as the shipped param presets.
    config.range_boundary_offset = GlobalIndex(20, 20, 5);
    config.default_distance_m = 2.0f;
    config.max_distance_m = 2.0f;
    config.finer_esdf_on = false;
    return config;
  }

  Layer<TsdfVoxel> tsdf_layer_;
  Layer<EsdfVoxel> esdf_layer_;
};

TEST_F(EsdfVoxfieldIntegratorTest, DistantUpdatesDoNotAllocateBoundingBox) {
  // Two surface patches ~10 m apart on every axis, updated in the same ESDF
  // pass (e.g. a pose jump between two ESDF updates). The old dense
  // bounding-box allocation created ~2000 ESDF blocks (~160 MB) here.
  addPlanePatch(BlockIndex(-4, -4, -2));
  addPlanePatch(BlockIndex(4, 4, 6));

  EsdfVoxfieldIntegrator integrator(makeConfig(), &tsdf_layer_, &esdf_layer_);
  integrator.updateFromTsdfLayer(true);

  EXPECT_EQ(
      esdf_layer_.getNumberOfAllocatedBlocks(),
      tsdf_layer_.getNumberOfAllocatedBlocks());
  EXPECT_EQ(esdf_layer_.getNumberOfAllocatedBlocks(), 2u);
}

TEST_F(EsdfVoxfieldIntegratorTest, DistancesToPlaneAreCorrect) {
  addPlanePatch(BlockIndex(-4, -4, -2));
  addPlanePatch(BlockIndex(4, 4, 6));

  EsdfVoxfieldIntegrator integrator(makeConfig(), &tsdf_layer_, &esdf_layer_);
  integrator.updateFromTsdfLayer(true);

  BlockIndexList blocks;
  esdf_layer_.getAllAllocatedBlocks(&blocks);
  size_t num_checked = 0u;
  for (const BlockIndex& block_index : blocks) {
    const Block<EsdfVoxel>& block = esdf_layer_.getBlockByIndex(block_index);
    for (size_t i = 0u; i < block.num_voxels(); ++i) {
      const EsdfVoxel& voxel = block.getVoxelByLinearIndex(i);
      if (!voxel.observed) {
        continue;
      }
      const VoxelIndex voxel_index = block.computeVoxelIndexFromLinearIndex(i);
      const FloatingPoint expected = (voxel_index.z() - kPlaneZ) * kVoxelSize;
      EXPECT_NEAR(voxel.distance, expected, 1e-4)
          << "block " << block_index.transpose() << " voxel "
          << voxel_index.transpose();
      ++num_checked;
    }
  }
  EXPECT_EQ(num_checked, 2u * kVoxelsPerSide * kVoxelsPerSide * kVoxelsPerSide);
}

TEST_F(EsdfVoxfieldIntegratorTest, IncrementalDeleteKeepsMemoryBounded) {
  // Exercise the "delete" (occupied -> free) path across a distant update:
  // first build a patch, then remove its surface while adding a far one.
  addPlanePatch(BlockIndex(0, 0, 0));
  EsdfVoxfieldIntegrator integrator(makeConfig(), &tsdf_layer_, &esdf_layer_);
  integrator.updateFromTsdfLayer(true);

  Block<TsdfVoxel>::Ptr block = tsdf_layer_.getBlockPtrByIndex(BlockIndex(0, 0, 0));
  ASSERT_TRUE(block != nullptr);
  for (size_t i = 0u; i < block->num_voxels(); ++i) {
    block->getVoxelByLinearIndex(i).distance = 1.0f;  // all free now
  }
  block->setUpdatedAll();
  addPlanePatch(BlockIndex(8, -8, 8));
  integrator.updateFromTsdfLayer(true);

  EXPECT_EQ(esdf_layer_.getNumberOfAllocatedBlocks(), 2u);
  // With no surface left in the first patch, every voxel there must have
  // lost its closest-occupied-voxel link and been reset to the default
  // (raw) distance by the delete pass.
  const Block<EsdfVoxel>& esdf_block =
      esdf_layer_.getBlockByIndex(BlockIndex(0, 0, 0));
  for (size_t i = 0u; i < esdf_block.num_voxels(); ++i) {
    const EsdfVoxel& voxel = esdf_block.getVoxelByLinearIndex(i);
    EXPECT_EQ(voxel.coc_idx(0), UNDEF);
    EXPECT_FLOAT_EQ(voxel.raw_distance, makeConfig().default_distance_m);
  }
}

}  // namespace voxfield
