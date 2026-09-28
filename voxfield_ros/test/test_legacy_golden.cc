// Golden regression tests for legacy (single-sensor) mode, added before the
// multi-sensor refactor begins (MULTI_SENSOR_PLAN.md Phase 1). These pin
// today's TSDF integration output bit-for-bit so later phases can prove "no
// behavior change in legacy mode" by re-running against the same golden
// files.
//
// TsdfServer and NpTsdfServer can't be included in the same translation
// unit (both define voxfield::kDefaultMaxIntensity), so CMake builds this
// file twice, once per server, with TEST_NP_TSDF_SERVER on/off -- the same
// pattern as test_server_map_io.cc's TEST_VOXBLOX_SERVER.
#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <vector>
#include <voxfield/io/layer_io.h>

#ifdef TEST_NP_TSDF_SERVER
#include "voxfield_ros/np_tsdf_server.h"
#define TEST_SERVER NpTsdfServer
#define TEST_SERVER_NAME "np_tsdf"
#else
#include "voxfield_ros/tsdf_server.h"
#define TEST_SERVER TsdfServer
#define TEST_SERVER_NAME "tsdf"
#endif

namespace voxfield {
namespace {

// ---- Box-room synthetic LiDAR, ported from scripts/fake_sensor_publisher.py
// (deterministic, no randomness) ----
//
// A closed, axis-aligned box room (walls + floor + ceiling), seen by a
// spinning sensor orbiting near the room's center. Every ray from the
// sensor's origin is guaranteed to hit a wall, floor, or ceiling, so the
// cloud is fully dense every frame, matching the Python script's geometry
// and motion model exactly.
class BoxRoomLidar {
 public:
  BoxRoomLidar(int width, int height, double fov_up_deg, double fov_down_deg)
      : width_(width), height_(height) {
    const double fov_up = fov_up_deg * M_PI / 180.0;
    const double fov_down = fov_down_deg * M_PI / 180.0;
    dirs_local_.reserve(static_cast<size_t>(width) * height);
    for (int row = 0; row < height; ++row) {
      const double frac =
          height > 1 ? static_cast<double>(row) / (height - 1) : 0.0;
      const double el = fov_up + (fov_down - fov_up) * frac;
      const double cos_el = std::cos(el);
      const double sin_el = std::sin(el);
      for (int col = 0; col < width; ++col) {
        const double az = 2.0 * M_PI * static_cast<double>(col) / width;
        dirs_local_.emplace_back(
            cos_el * std::cos(az), cos_el * std::sin(az), sin_el);
      }
    }
  }

  // Position (world frame) and yaw (rad) of the sensor at time t_sec, always
  // inside the room. Matches FakeSensorPublisher.sensor_pose().
  void sensorPose(double t_sec, Eigen::Vector3d* position, double* yaw) const {
    const double omega_orbit = 2.0 * M_PI / kOrbitPeriodSec;
    position->x() = kOrbitRadius * std::cos(omega_orbit * t_sec);
    position->y() = kOrbitRadius * std::sin(omega_orbit * t_sec);
    position->z() = kRoomHeight / 2.0;
    *yaw = 2.0 * M_PI * t_sec / kSpinPeriodSec;
  }

  // Ray/box exit distances for every precomputed local direction, given the
  // sensor is at `position` with yaw `yaw` inside the axis-aligned room.
  // Returns sensor-local-frame hit points. Matches
  // FakeSensorPublisher.raycast().
  std::vector<Eigen::Vector3f> raycast(
      const Eigen::Vector3d& position, double yaw) const {
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    Eigen::Matrix3d rot;
    rot << cos_yaw, -sin_yaw, 0.0, sin_yaw, cos_yaw, 0.0, 0.0, 0.0, 1.0;

    const Eigen::Vector3d box_min(-kRoomHalfX, -kRoomHalfY, 0.0);
    const Eigen::Vector3d box_max(kRoomHalfX, kRoomHalfY, kRoomHeight);

    std::vector<Eigen::Vector3f> hits_local;
    hits_local.reserve(dirs_local_.size());
    for (const Eigen::Vector3d& dir_local : dirs_local_) {
      const Eigen::Vector3d dir_world = rot * dir_local;
      double t_exit = std::numeric_limits<double>::infinity();
      for (int axis = 0; axis < 3; ++axis) {
        if (dir_world[axis] > 0.0) {
          t_exit = std::min(
              t_exit, (box_max[axis] - position[axis]) / dir_world[axis]);
        } else if (dir_world[axis] < 0.0) {
          t_exit = std::min(
              t_exit, (box_min[axis] - position[axis]) / dir_world[axis]);
        }
      }
      const Eigen::Vector3d hit_world = position + t_exit * dir_world;
      const Eigen::Vector3d hit_local =
          rot.transpose() * (hit_world - position);
      hits_local.emplace_back(hit_local.cast<float>());
    }
    return hits_local;
  }

 private:
  static constexpr double kRoomHalfX = 5.0;
  static constexpr double kRoomHalfY = 5.0;
  static constexpr double kRoomHeight = 3.0;
  static constexpr double kOrbitRadius = 1.5;
  static constexpr double kOrbitPeriodSec = 20.0;
  static constexpr double kSpinPeriodSec = 30.0;

  int width_;
  int height_;
  std::vector<Eigen::Vector3d> dirs_local_;
};

sensor_msgs::msg::PointCloud2::SharedPtr makeCloudMsg(
    const std::vector<Eigen::Vector3f>& points, const rclcpp::Time& stamp,
    const std::string& frame_id) {
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id;
  msg->height = 1;
  msg->width = static_cast<uint32_t>(points.size());
  msg->is_bigendian = false;
  msg->is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(*msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
  for (const Eigen::Vector3f& p : points) {
    *iter_x = p.x();
    *iter_y = p.y();
    *iter_z = p.z();
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  return msg;
}

// Exposes the protected `transformer_` member so the test can inject
// synthetic poses directly into its queue (use_tf_transforms: false),
// bypassing TF and the "transform" topic entirely for determinism.
class TestServer : public TEST_SERVER {
 public:
  explicit TestServer(rclcpp::Node::SharedPtr node) : TEST_SERVER(node) {}
  using TEST_SERVER::transformer_;
};

void pushTransform(
    TestServer* server, const BoxRoomLidar& lidar, int64_t stamp_ns,
    const std::string& world_frame) {
  Eigen::Vector3d position;
  double yaw;
  lidar.sensorPose(static_cast<double>(stamp_ns) * 1e-9, &position, &yaw);

  auto msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
  msg->header.stamp = rclcpp::Time(stamp_ns, RCL_ROS_TIME);
  msg->header.frame_id = world_frame;
  msg->transform.translation.x = position.x();
  msg->transform.translation.y = position.y();
  msg->transform.translation.z = position.z();
  msg->transform.rotation.x = 0.0;
  msg->transform.rotation.y = 0.0;
  msg->transform.rotation.z = std::sin(yaw / 2.0);
  msg->transform.rotation.w = std::cos(yaw / 2.0);
  server->transformer_.transformCallback(msg);
}

// Feeds 20 deterministic synthetic clouds of the box room into `server`.
// Before each cloud, pushes the sensor pose at t-5ms and t+5ms into the
// transform queue (exercising Transformer::lookupTransformQueue's
// interpolation path), then calls insertPointcloud() with the cloud stamped
// at t, exactly as plan §8 Phase 1 step 1 specifies.
void runScenario(
    TestServer* server, const BoxRoomLidar& lidar, const std::string& frame_id,
    const std::string& world_frame) {
  constexpr int kNumFrames = 20;
  constexpr int64_t kFrameStepNs = 100000000LL;  // 100 ms (10 Hz)
  constexpr int64_t kHalfBracketNs = 5000000LL;  // 5 ms
  constexpr int64_t kBaseNs = 1000000000LL;      // avoid t=0 edge cases

  for (int i = 0; i < kNumFrames; ++i) {
    const int64_t frame_ns = kBaseNs + static_cast<int64_t>(i) * kFrameStepNs;
    const int64_t minus_ns = frame_ns - kHalfBracketNs;
    const int64_t plus_ns = frame_ns + kHalfBracketNs;

    pushTransform(server, lidar, minus_ns, world_frame);
    pushTransform(server, lidar, plus_ns, world_frame);

    Eigen::Vector3d position;
    double yaw;
    lidar.sensorPose(static_cast<double>(frame_ns) * 1e-9, &position, &yaw);
    const std::vector<Eigen::Vector3f> points = lidar.raycast(position, yaw);

    const auto cloud_msg =
        makeCloudMsg(points, rclcpp::Time(frame_ns, RCL_ROS_TIME), frame_id);
    server->insertPointcloud(cloud_msg);
  }
}

// Exact (zero-tolerance) layer comparison. voxfield::test::LayerTest's
// CompareLayers (voxfield/test/layer_test_utils.h) uses EXPECT_NEAR with a
// 1e-10 absolute tolerance, which is tight but not literally exact; per plan
// §8 Phase 1 step 3, a refactor may not change a single float, so this local
// helper uses EXPECT_EQ instead, rather than loosening the shared util.
void CompareLayersExact(
    const Layer<TsdfVoxel>& golden, const Layer<TsdfVoxel>& actual) {
  ASSERT_FLOAT_EQ(golden.voxel_size(), actual.voxel_size());
  ASSERT_EQ(golden.voxels_per_side(), actual.voxels_per_side());

  BlockIndexList golden_blocks, actual_blocks;
  golden.getAllAllocatedBlocks(&golden_blocks);
  actual.getAllAllocatedBlocks(&actual_blocks);
  ASSERT_EQ(golden_blocks.size(), actual_blocks.size());

  for (const BlockIndex& index : golden_blocks) {
    ASSERT_TRUE(actual.hasBlock(index))
        << "block [" << index.transpose() << "] missing from the fresh layer";
    const Block<TsdfVoxel>& golden_block = golden.getBlockByIndex(index);
    const Block<TsdfVoxel>& actual_block = actual.getBlockByIndex(index);
    ASSERT_EQ(golden_block.num_voxels(), actual_block.num_voxels());
    for (size_t v = 0; v < golden_block.num_voxels(); ++v) {
      const TsdfVoxel& g = golden_block.getVoxelByLinearIndex(v);
      const TsdfVoxel& a = actual_block.getVoxelByLinearIndex(v);
      SCOPED_TRACE(
          "block [" + std::to_string(index.x()) + "," +
          std::to_string(index.y()) + "," + std::to_string(index.z()) +
          "] voxel " + std::to_string(v));
      EXPECT_EQ(g.distance, a.distance);
      EXPECT_EQ(g.weight, a.weight);
      EXPECT_EQ(g.color.r, a.color.r);
      EXPECT_EQ(g.color.g, a.color.g);
      EXPECT_EQ(g.color.b, a.color.b);
      EXPECT_EQ(g.color.a, a.color.a);
    }
  }
}

std::string goldenPath() {
  return std::string(VOXFIELD_ROS_TEST_DATA_DIR) + "/golden_" +
         TEST_SERVER_NAME ".tsdf";
}

bool writeGoldenRequested() {
  const char* env = std::getenv("VOXFIELD_WRITE_GOLDEN");
  return env != nullptr && std::string(env) == "1";
}

rclcpp::Node::SharedPtr makeNode(const std::string& name) {
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> overrides = {
      rclcpp::Parameter("world_frame", "map"),
      rclcpp::Parameter("sensor_frame", "lidar"),
      rclcpp::Parameter("use_tf_transforms", false),
      rclcpp::Parameter(
          "T_B_C",
          std::vector<double>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}),
      rclcpp::Parameter(
          "T_B_D",
          std::vector<double>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}),
      rclcpp::Parameter("method", "merged"),
      rclcpp::Parameter("integrator_threads", 1),
      rclcpp::Parameter("tsdf_voxel_size", 0.25),
      rclcpp::Parameter("max_ray_length_m", 20.0),
      rclcpp::Parameter("verbose", false),
      rclcpp::Parameter("timing", false),
      rclcpp::Parameter("update_mesh_every_n_sec", 0.0),
      rclcpp::Parameter("publish_map_every_n_sec", 0.0),
      rclcpp::Parameter("publish_pointclouds_on_update", false),
      rclcpp::Parameter("publish_slices", false),
      rclcpp::Parameter("publish_pointclouds", false),
      rclcpp::Parameter("publish_tsdf_map", false),
#ifdef TEST_NP_TSDF_SERVER
      rclcpp::Parameter("sensor_is_lidar", true),
      rclcpp::Parameter("width", 256),
      rclcpp::Parameter("height", 16),
      rclcpp::Parameter("fov_up", 3.0),
      rclcpp::Parameter("fov_down", -25.0),
#endif
  };
  options.parameter_overrides(overrides);
  return std::make_shared<rclcpp::Node>(name, options);
}

#ifdef TEST_NP_TSDF_SERVER
// Matches the plan's explicit NP LiDAR model (§8 Phase 1 step 1).
BoxRoomLidar makeLidar() {
  return BoxRoomLidar(
      /*width=*/256, /*height=*/16, /*fov_up_deg=*/3.0,
      /*fov_down_deg=*/-25.0);
}
#else
// TsdfServer doesn't reproject onto a range image, so any dense, deterministic
// coverage of the room works; wider vertical FOV than the NP case since
// there's no per-pixel reprojection constraint to satisfy.
BoxRoomLidar makeLidar() {
  return BoxRoomLidar(
      /*width=*/180, /*height=*/16, /*fov_up_deg=*/20.0,
      /*fov_down_deg=*/-20.0);
}
#endif

TEST(LegacyGolden, PinnedTsdfOutput) {
  auto node = makeNode(std::string(TEST_SERVER_NAME) + "_golden");
  TestServer server(node);
  const BoxRoomLidar lidar = makeLidar();
  runScenario(&server, lidar, "lidar", "map");

  if (writeGoldenRequested()) {
    ASSERT_TRUE(server.saveMap(goldenPath()))
        << "failed to write golden file " << goldenPath();
    GTEST_SKIP() << "VOXFIELD_WRITE_GOLDEN=1: wrote " << goldenPath()
                 << " instead of comparing.";
  }

  Layer<TsdfVoxel>::Ptr golden_layer;
  ASSERT_TRUE(io::LoadLayer<TsdfVoxel>(goldenPath(), &golden_layer))
      << "failed to load golden file " << goldenPath()
      << " -- generate it first with VOXFIELD_WRITE_GOLDEN=1";
  CompareLayersExact(*golden_layer, server.getTsdfMapPtr()->getTsdfLayer());
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
