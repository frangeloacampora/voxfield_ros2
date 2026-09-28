#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "voxfield/core/layer.h"
#include "voxfield/core/voxel.h"
#include "voxfield/integrator/esdf_occ_edt_integrator.h"
#include "voxfield/integrator/esdf_occ_fiesta_integrator.h"
#include "voxfield/integrator/occupancy_tsdf_integrator.h"

namespace voxfield {

// Regression test for the FIESTA/EDT twin of docs/ROS2_PORT_NOTES.md "Known
// upstream issues" #12 (docs/MULTI_SENSOR_NOTES.md Phase 9 step 7):
// EsdfOcc{Fiesta,Edt}Integrator::setLocalRange() allocated every ESDF block
// in the bounding box of all changed voxels plus range_boundary_offset, so a
// real 30 m 360-degree LiDAR at 0.1 m voxels allocated ~15 GB on its first
// ESDF update. The fix skips that dense allocation; this test checks that
// the ESDF result is bit-for-bit unchanged against the upstream behavior
// (`allocate_dense_local_range = true`) over a sequence of incremental
// updates with surface insertions, deletions and distant jumps.
namespace {

constexpr FloatingPoint kVoxelSize = 0.2f;
constexpr size_t kVoxelsPerSide = 8u;
constexpr FloatingPoint kTruncation = 0.6f;

struct Sphere {
  Point center;
  FloatingPoint radius;
};

// One frame of the scene: TSDF blocks to (re)write and the spheres whose
// union's signed distance they hold.
struct Frame {
  std::vector<BlockIndex> blocks;
  std::vector<Sphere> spheres;
};

FloatingPoint sceneSdf(const std::vector<Sphere>& spheres, const Point& p) {
  FloatingPoint d = kTruncation;
  for (const Sphere& s : spheres) {
    d = std::min(d, (p - s.center).norm() - s.radius);
  }
  return std::max(-kTruncation, std::min(kTruncation, d));
}

std::vector<BlockIndex> blocksAround(const Point& center, int radius_blocks) {
  const FloatingPoint block_size = kVoxelSize * kVoxelsPerSide;
  const BlockIndex c =
      (center / block_size).array().floor().cast<IndexElement>();
  std::vector<BlockIndex> blocks;
  for (int x = -radius_blocks; x <= radius_blocks; ++x) {
    for (int y = -radius_blocks; y <= radius_blocks; ++y) {
      for (int z = -radius_blocks; z <= radius_blocks; ++z) {
        blocks.push_back(c + BlockIndex(x, y, z));
      }
    }
  }
  return blocks;
}

// Frames: a sphere; a far second sphere (distant update); the first sphere
// shrinks and moves (occupied -> free: delete path) together with an even
// farther third one (a pose-jump-sized update); then the first region is
// re-observed with nothing in it (pure deletes).
std::vector<Frame> makeFrames() {
  const Sphere a{Point(0.3f, 0.1f, -0.2f), 0.9f};
  const Sphere a_moved{Point(0.9f, -0.4f, 0.1f), 0.5f};
  const Sphere b{Point(6.1f, -5.3f, 2.2f), 0.7f};
  const Sphere c{Point(-7.4f, 8.2f, -3.1f), 1.1f};
  std::vector<Frame> frames;
  frames.push_back({blocksAround(a.center, 1), {a}});
  frames.push_back({blocksAround(b.center, 1), {a, b}});
  std::vector<BlockIndex> blocks = blocksAround(a.center, 1);
  const std::vector<BlockIndex> c_blocks = blocksAround(c.center, 1);
  blocks.insert(blocks.end(), c_blocks.begin(), c_blocks.end());
  frames.push_back({blocks, {a_moved, b, c}});
  frames.push_back({blocksAround(a.center, 1), {b, c}});
  return frames;
}

template <typename EsdfIntegrator>
class Pipeline {
 public:
  explicit Pipeline(bool dense)
      : tsdf_layer_(kVoxelSize, kVoxelsPerSide),
        occ_layer_(kVoxelSize, kVoxelsPerSide),
        esdf_layer_(kVoxelSize, kVoxelsPerSide),
        occ_integrator_(
            OccTsdfIntegrator::Config(), &tsdf_layer_, &occ_layer_) {
    typename EsdfIntegrator::Config config;
    // Same local-range margin (in voxels) and distances as the shipped
    // athena_param.yaml.
    config.range_boundary_offset = GlobalIndex(20, 20, 10);
    config.default_distance_m = 3.0f;
    config.max_distance_m = 3.0f;
    config.max_behind_surface_m = 1.0f;
    config.num_buckets = 50;
    config.allocate_dense_local_range = dense;
    esdf_integrator_ =
        std::make_unique<EsdfIntegrator>(config, &occ_layer_, &esdf_layer_);
  }

  // Mirrors {Fiesta,Voxedt}Server::updateOccFromTsdf() + updateEsdfFromOcc().
  void apply(const Frame& frame) {
    for (const BlockIndex& block_index : frame.blocks) {
      Block<TsdfVoxel>::Ptr block =
          tsdf_layer_.allocateBlockPtrByIndex(block_index);
      for (size_t i = 0u; i < block->num_voxels(); ++i) {
        const Point p = block->computeCoordinatesFromLinearIndex(i);
        TsdfVoxel& voxel = block->getVoxelByLinearIndex(i);
        voxel.distance = sceneSdf(frame.spheres, p);
        voxel.weight = 1.0f;
      }
      block->setUpdatedAll();
    }
    occ_integrator_.updateFromTsdfLayer(true, false);
    esdf_integrator_->loadInsertList(occ_integrator_.getInsertList());
    esdf_integrator_->loadDeleteList(occ_integrator_.getDeleteList());
    esdf_integrator_->updateFromOccLayer(true);
  }

  Layer<OccupancyVoxel> occ_layer_;
  Layer<EsdfVoxel> esdf_layer_;

 private:
  Layer<TsdfVoxel> tsdf_layer_;
  OccTsdfIntegrator occ_integrator_;
  std::unique_ptr<EsdfIntegrator> esdf_integrator_;
};

// Every observed voxel of `a` must exist in `b` with identical fields.
// Counts `a`'s observed voxels into *num_observed.
void expectObservedVoxelsMatch(
    const Layer<EsdfVoxel>& a, const Layer<EsdfVoxel>& b,
    size_t* num_observed) {
  BlockIndexList blocks;
  a.getAllAllocatedBlocks(&blocks);
  *num_observed = 0u;
  for (const BlockIndex& block_index : blocks) {
    const Block<EsdfVoxel>& block_a = a.getBlockByIndex(block_index);
    for (size_t i = 0u; i < block_a.num_voxels(); ++i) {
      const EsdfVoxel& va = block_a.getVoxelByLinearIndex(i);
      if (!va.observed) {
        continue;
      }
      ++*num_observed;
      ASSERT_TRUE(b.hasBlock(block_index)) << block_index.transpose();
      const EsdfVoxel& vb =
          b.getBlockByIndex(block_index).getVoxelByLinearIndex(i);
      EXPECT_TRUE(vb.observed);
      EXPECT_EQ(va.distance, vb.distance)
          << block_index.transpose() << " " << i;
      EXPECT_EQ(va.coc_idx, vb.coc_idx) << block_index.transpose() << " " << i;
      EXPECT_EQ(va.behind, vb.behind);
      EXPECT_EQ(va.self_idx, vb.self_idx);
    }
  }
}

}  // namespace

template <typename EsdfIntegrator>
class EsdfOccIntegratorTest : public ::testing::Test {};

using Integrators =
    ::testing::Types<EsdfOccFiestaIntegrator, EsdfOccEdtIntegrator>;
TYPED_TEST_SUITE(EsdfOccIntegratorTest, Integrators);

TYPED_TEST(EsdfOccIntegratorTest, SparseAllocationMatchesDenseResult) {
  Pipeline<TypeParam> dense(true);
  Pipeline<TypeParam> sparse(false);
  size_t frame_index = 0u;
  for (const Frame& frame : makeFrames()) {
    SCOPED_TRACE(::testing::Message() << "frame " << frame_index++);
    dense.apply(frame);
    sparse.apply(frame);

    // Memory: ESDF blocks now track the occupancy blocks one-to-one, while
    // the upstream dense allocation fills the whole bounding box.
    EXPECT_EQ(
        sparse.esdf_layer_.getNumberOfAllocatedBlocks(),
        sparse.occ_layer_.getNumberOfAllocatedBlocks());
    EXPECT_GE(
        dense.esdf_layer_.getNumberOfAllocatedBlocks(),
        sparse.esdf_layer_.getNumberOfAllocatedBlocks());

    size_t num_sparse = 0u;
    size_t num_dense = 0u;
    expectObservedVoxelsMatch(
        sparse.esdf_layer_, dense.esdf_layer_, &num_sparse);
    expectObservedVoxelsMatch(
        dense.esdf_layer_, sparse.esdf_layer_, &num_dense);
    EXPECT_EQ(num_sparse, num_dense);
    EXPECT_GT(num_sparse, 0u);
  }
  // The distant updates must actually have exercised the dense path.
  EXPECT_GT(
      dense.esdf_layer_.getNumberOfAllocatedBlocks(),
      10u * sparse.esdf_layer_.getNumberOfAllocatedBlocks());
}

TYPED_TEST(EsdfOccIntegratorTest, DistancesAreFinite) {
  // Sanity check on the fixed path alone: every observed voxel got a finite
  // distance no larger than the configured default.
  Pipeline<TypeParam> sparse(false);
  for (const Frame& frame : makeFrames()) {
    sparse.apply(frame);
  }
  BlockIndexList blocks;
  sparse.esdf_layer_.getAllAllocatedBlocks(&blocks);
  for (const BlockIndex& block_index : blocks) {
    const Block<EsdfVoxel>& block =
        sparse.esdf_layer_.getBlockByIndex(block_index);
    for (size_t i = 0u; i < block.num_voxels(); ++i) {
      const EsdfVoxel& voxel = block.getVoxelByLinearIndex(i);
      if (voxel.observed) {
        EXPECT_TRUE(std::isfinite(voxel.distance));
        EXPECT_LE(std::abs(voxel.distance), 3.0f + 1e-4f);
      }
    }
  }
}

}  // namespace voxfield
