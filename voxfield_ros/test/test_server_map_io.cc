// Regression tests for the ~/save_map / ~/load_map / ~/save_esdf_map file
// handling of VoxfieldServer, VoxbloxServer, FiestaServer and VoxedtServer
// (docs/ROS2_PORT_NOTES.md "Known upstream issues" #11). Upstream saveMap()
// had its TSDF save commented out (voxfield, voxblox) or wasn't overridden
// at all (fiesta, voxedt: TSDF only), so a saved map could never be loaded
// back.
#include <gtest/gtest.h>

#include <cstdio>
#include <memory>
#include <string>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <voxfield/io/layer_io.h>

// The servers' headers can't all be included together (tsdf_server.h and
// np_tsdf_server.h both define kDefaultMaxIntensity), so CMake builds this
// file once per server, selecting it with TEST_{VOXBLOX,FIESTA,VOXEDT}_SERVER
// (none set: VoxfieldServer).
#if defined(TEST_VOXBLOX_SERVER)
#include "voxfield_ros/voxblox_server.h"
#define TEST_SERVER VoxbloxServer
#define TEST_SERVER_NAME "voxblox"
#elif defined(TEST_FIESTA_SERVER)
#include "voxfield_ros/fiesta_server.h"
#define TEST_SERVER FiestaServer
#define TEST_SERVER_NAME "fiesta"
#elif defined(TEST_VOXEDT_SERVER)
#include "voxfield_ros/voxedt_server.h"
#define TEST_SERVER VoxedtServer
#define TEST_SERVER_NAME "voxedt"
#else
#include "voxfield_ros/voxfield_server.h"
#define TEST_SERVER VoxfieldServer
#define TEST_SERVER_NAME "voxfield"
#endif

namespace voxfield {
namespace {

const BlockIndex kTsdfBlock(0, 0, 0);
const BlockIndex kEsdfBlock(1, 2, 3);

rclcpp::Node::SharedPtr makeNode(const std::string& name) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("update_mesh_every_n_sec", 0.0),
      rclcpp::Parameter("update_esdf_every_n_sec", 0.0),
      rclcpp::Parameter("publish_map_every_n_sec", 0.0),
      // NpTsdfServer logs an error without these; irrelevant here.
      rclcpp::Parameter("width", 8),
      rclcpp::Parameter("height", 8),
  });
  return std::make_shared<rclcpp::Node>(name, options);
}

std::string tempPath(const std::string& tag) {
  char path[] = "/tmp/voxfield_map_io_XXXXXX";
  const int fd = mkstemp(path);
  EXPECT_GE(fd, 0);
  close(fd);
  return std::string(path) + "_" + tag;
}

template <typename Server>
void fillMaps(Server* server) {
  TsdfVoxel& tsdf = server->getTsdfMapPtr()
                        ->getTsdfLayerPtr()
                        ->allocateBlockPtrByIndex(kTsdfBlock)
                        ->getVoxelByLinearIndex(0);
  tsdf.distance = 0.05f;
  tsdf.weight = 2.0f;
  EsdfVoxel& esdf = server->getEsdfMapPtr()
                        ->getEsdfLayerPtr()
                        ->allocateBlockPtrByIndex(kEsdfBlock)
                        ->getVoxelByLinearIndex(0);
  esdf.distance = 1.25f;
  esdf.observed = true;
}

template <typename Server>
void checkMapRoundTrip(const std::string& name) {
  const std::string path = tempPath(name + ".map");
  {
    Server saver(makeNode(name + "_saver"));
    fillMaps(&saver);
    ASSERT_TRUE(saver.saveMap(path));
  }
  Server loader(makeNode(name + "_loader"));
  ASSERT_TRUE(loader.loadMap(path));
  const Layer<TsdfVoxel>& tsdf = loader.getTsdfMapPtr()->getTsdfLayer();
  const Layer<EsdfVoxel>& esdf = loader.getEsdfMapPtr()->getEsdfLayer();
  ASSERT_TRUE(tsdf.hasBlock(kTsdfBlock));
  ASSERT_TRUE(esdf.hasBlock(kEsdfBlock));
  EXPECT_FLOAT_EQ(
      tsdf.getBlockByIndex(kTsdfBlock).getVoxelByLinearIndex(0).weight, 2.0f);
  EXPECT_FLOAT_EQ(
      esdf.getBlockByIndex(kEsdfBlock).getVoxelByLinearIndex(0).distance,
      1.25f);
  std::remove(path.c_str());
}

template <typename Server>
void checkEsdfOnlySave(const std::string& name) {
  const std::string path = tempPath(name + ".esdf");
  Server server(makeNode(name));
  fillMaps(&server);
  // Write a combined TSDF+ESDF file first, then save the ESDF layer alone to
  // the same path: the ESDF-only save must replace the file, not append,
  // so single-layer readers (voxblox_eval's io::LoadLayer<EsdfVoxel>) see
  // the ESDF layer first.
  ASSERT_TRUE(server.saveMap(path));
  ASSERT_TRUE(server.saveEsdfMap(path));
  Layer<EsdfVoxel>::Ptr esdf_layer;
  ASSERT_TRUE(io::LoadLayer<EsdfVoxel>(path, &esdf_layer));
  EXPECT_TRUE(esdf_layer->hasBlock(kEsdfBlock));
  Layer<TsdfVoxel>::Ptr tsdf_layer;
  constexpr bool kMultipleLayerSupport = true;
  EXPECT_FALSE(
      io::LoadLayer<TsdfVoxel>(path, kMultipleLayerSupport, &tsdf_layer));
  std::remove(path.c_str());
}

TEST(ServerMapIo, SaveLoadRoundTrip) {
  checkMapRoundTrip<TEST_SERVER>(TEST_SERVER_NAME "_roundtrip");
}

TEST(ServerMapIo, EsdfOnlySave) {
  checkEsdfOnlySave<TEST_SERVER>(TEST_SERVER_NAME "_esdf_only");
}

}  // namespace
}  // namespace voxfield

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
