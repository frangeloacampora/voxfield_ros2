#include <gtest/gtest.h>

#include "voxfield/Block.pb.h"
#include "voxfield/Layer.pb.h"
#include "voxfield/core/block.h"
#include "voxfield/core/layer.h"
#include "voxfield/core/voxel.h"
#include "voxfield/test/layer_test_utils.h"
#include "voxfield/utils/layer_utils.h"

using namespace voxfield;  // NOLINT

template <typename VoxelType>
class LayerTest : public ::testing::Test,
                  public voxfield::test::LayerTest<VoxelType> {
 protected:
  virtual void SetUp() {
    layer_.reset(new Layer<VoxelType>(voxel_size_, voxels_per_side_));
    SetUpLayer();
  }

  void SetUpLayer() const {
    voxfield::test::SetUpTestLayer(kBlockVolumeDiameter, layer_.get());
  }

  typename Layer<VoxelType>::Ptr layer_;

  const double voxel_size_ = 0.02;
  const size_t voxels_per_side_ = 16u;

  static constexpr size_t kBlockVolumeDiameter = 10u;
};

typedef LayerTest<TsdfVoxel> TsdfLayerTest;
typedef LayerTest<EsdfVoxel> EsdfLayerTest;
typedef LayerTest<OccupancyVoxel> OccupancyLayerTest;

TEST_F(TsdfLayerTest, DeepCopyConstructor) {
  Layer<TsdfVoxel> new_layer(*layer_);
  EXPECT_TRUE(voxfield::utils::isSameLayer(new_layer, *layer_));
}

TEST_F(EsdfLayerTest, DeepCopyConstructor) {
  Layer<EsdfVoxel> new_layer(*layer_);
  EXPECT_TRUE(voxfield::utils::isSameLayer(new_layer, *layer_));
}

TEST_F(OccupancyLayerTest, DeepCopyConstructor) {
  Layer<OccupancyVoxel> new_layer(*layer_);
  EXPECT_TRUE(voxfield::utils::isSameLayer(new_layer, *layer_));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}
