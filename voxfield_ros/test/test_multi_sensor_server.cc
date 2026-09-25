// Unit tests for TsdfServer's and NpTsdfServer's multi-sensor frontends
// (MULTI_SENSOR_PLAN.md Phase 5/6): sensor equivalence, per-sensor
// extrinsics/ray limits/throttle, the M11 ICP guard, and (NP only) a mixed
// LiDAR+camera sensor pair and M13's fail-fast projector validation.
//
// TsdfServer and NpTsdfServer can't be included in the same translation
// unit (both define voxfield::kDefaultMaxIntensity), so CMake builds this
// file twice, once per server, with TEST_NP_TSDF_SERVER on/off -- the same
// pattern as test_legacy_golden.cc's.
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "voxfield_ros/kindr_conversions.h"

#ifdef TEST_NP_TSDF_SERVER
#include "voxfield_ros/np_tsdf_server.h"
#define TEST_SERVER NpTsdfServer
#else
#include "voxfield_ros/tsdf_server.h"
#define TEST_SERVER TsdfServer
#endif

namespace voxfield {
namespace {

// ---- Box-room synthetic LiDAR, same convention as test_legacy_golden.cc
// (ported from scripts/fake_sensor_publisher.py). ----
class BoxRoomLidar {
 public:
  BoxRoomLidar(int width, int height, double fov_up_deg, double fov_down_deg) {
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

  void sensorPose(double t_sec, Eigen::Vector3d* position, double* yaw) const {
    const double omega_orbit = 2.0 * M_PI / kOrbitPeriodSec;
    position->x() = kOrbitRadius * std::cos(omega_orbit * t_sec);
    position->y() = kOrbitRadius * std::sin(omega_orbit * t_sec);
    position->z() = kRoomHeight / 2.0;
    *yaw = 2.0 * M_PI * t_sec / kSpinPeriodSec;
  }

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
          t_exit =
              std::min(t_exit, (box_max[axis] - position[axis]) / dir_world[axis]);
        } else if (dir_world[axis] < 0.0) {
          t_exit =
              std::min(t_exit, (box_min[axis] - position[axis]) / dir_world[axis]);
        }
      }
      const Eigen::Vector3d hit_world = position + t_exit * dir_world;
      const Eigen::Vector3d hit_local = rot.transpose() * (hit_world - position);
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

// Exposes the protected members the tests need to reach directly.
class TestServer : public TEST_SERVER {
 public:
  explicit TestServer(rclcpp::Node::SharedPtr node) : TEST_SERVER(node) {}
  using TEST_SERVER::enable_icp_;
  using TEST_SERVER::icp_transform_pub_;
  using TEST_SERVER::sensors_;
  using TEST_SERVER::transformer_;
};

void pushTransform(
    TestServer* server, const Transformation& T_G_D, int64_t stamp_ns,
    const std::string& world_frame) {
  auto msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
  msg->header.stamp = rclcpp::Time(stamp_ns, RCL_ROS_TIME);
  msg->header.frame_id = world_frame;
  transformKindrToMsg(T_G_D.cast<double>(), &msg->transform);
  server->transformer_.transformCallback(msg);
}

std::vector<rclcpp::Parameter> commonParams() {
  return {
      rclcpp::Parameter("world_frame", "map"),
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
  };
}

rclcpp::Node::SharedPtr makeLegacyNode(
    const std::string& name,
    std::vector<rclcpp::Parameter> extra_params = {}) {
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> params = commonParams();
  params.insert(params.end(), extra_params.begin(), extra_params.end());
  options.parameter_overrides(params);
  return std::make_shared<rclcpp::Node>(name, options);
}

rclcpp::Node::SharedPtr makeMultiSensorNode(
    const std::string& name, std::vector<rclcpp::Parameter> extra_params) {
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> params = commonParams();
  params.insert(params.end(), extra_params.begin(), extra_params.end());
  options.parameter_overrides(params);
  return std::make_shared<rclcpp::Node>(name, options);
}

void CompareLayersExact(
    const Layer<TsdfVoxel>& expected, const Layer<TsdfVoxel>& actual) {
  ASSERT_FLOAT_EQ(expected.voxel_size(), actual.voxel_size());
  BlockIndexList expected_blocks, actual_blocks;
  expected.getAllAllocatedBlocks(&expected_blocks);
  actual.getAllAllocatedBlocks(&actual_blocks);
  ASSERT_EQ(expected_blocks.size(), actual_blocks.size());
  for (const BlockIndex& index : expected_blocks) {
    ASSERT_TRUE(actual.hasBlock(index))
        << "block [" << index.transpose() << "] missing from actual";
    const Block<TsdfVoxel>& expected_block = expected.getBlockByIndex(index);
    const Block<TsdfVoxel>& actual_block = actual.getBlockByIndex(index);
    ASSERT_EQ(expected_block.num_voxels(), actual_block.num_voxels());
    for (size_t v = 0; v < expected_block.num_voxels(); ++v) {
      const TsdfVoxel& e = expected_block.getVoxelByLinearIndex(v);
      const TsdfVoxel& a = actual_block.getVoxelByLinearIndex(v);
      SCOPED_TRACE(
          "block [" + std::to_string(index.x()) + "," +
          std::to_string(index.y()) + "," + std::to_string(index.z()) +
          "] voxel " + std::to_string(v));
      EXPECT_EQ(e.distance, a.distance);
      EXPECT_EQ(e.weight, a.weight);
    }
  }
}

#ifndef TEST_NP_TSDF_SERVER

// ---- Equivalence: identical multi-sensor sensors == a legacy sensor ----

TEST(MultiSensorServer, AlternatingIdenticalSensorsMatchLegacy) {
  auto legacy_node = makeLegacyNode("equivalence_legacy");
  TestServer legacy_server(legacy_node);

  auto multi_node = makeMultiSensorNode(
      "equivalence_multi",
      {rclcpp::Parameter(
           "sensor_names", std::vector<std::string>{"a", "b"}),
       rclcpp::Parameter("sensors.a.topic", "a_pointcloud"),
       rclcpp::Parameter("sensors.b.topic", "b_pointcloud")});
  TestServer multi_server(multi_node);
  ASSERT_EQ(multi_server.sensors_.size(), 2u);

  const BoxRoomLidar lidar(180, 16, 20.0, -20.0);
  constexpr int kNumFrames = 20;
  constexpr int64_t kFrameStepNs = 100000000LL;
  constexpr int64_t kHalfBracketNs = 5000000LL;
  constexpr int64_t kBaseNs = 1000000000LL;

  for (int i = 0; i < kNumFrames; ++i) {
    const int64_t frame_ns = kBaseNs + static_cast<int64_t>(i) * kFrameStepNs;
    const int64_t minus_ns = frame_ns - kHalfBracketNs;
    const int64_t plus_ns = frame_ns + kHalfBracketNs;

    Eigen::Vector3d position;
    double yaw;
    lidar.sensorPose(static_cast<double>(minus_ns) * 1e-9, &position, &yaw);
    const Transformation T_minus(
        Transformation::Position(position.x(), position.y(), position.z()),
        Transformation::Rotation(Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
                                      .cast<float>()));
    lidar.sensorPose(static_cast<double>(plus_ns) * 1e-9, &position, &yaw);
    const Transformation T_plus(
        Transformation::Position(position.x(), position.y(), position.z()),
        Transformation::Rotation(Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
                                      .cast<float>()));

    pushTransform(&legacy_server, T_minus, minus_ns, "map");
    pushTransform(&legacy_server, T_plus, plus_ns, "map");
    pushTransform(&multi_server, T_minus, minus_ns, "map");
    pushTransform(&multi_server, T_plus, plus_ns, "map");

    lidar.sensorPose(static_cast<double>(frame_ns) * 1e-9, &position, &yaw);
    const std::vector<Eigen::Vector3f> points = lidar.raycast(position, yaw);
    const auto legacy_msg = makeCloudMsg(
        points, rclcpp::Time(frame_ns, RCL_ROS_TIME), "lidar");
    const auto multi_msg = makeCloudMsg(
        points, rclcpp::Time(frame_ns, RCL_ROS_TIME), "lidar");

    legacy_server.insertPointcloud(legacy_msg);
    multi_server.insertPointcloud(
        multi_msg, multi_server.sensors_[i % 2].get());
  }

  CompareLayersExact(
      legacy_server.getTsdfMapPtr()->getTsdfLayer(),
      multi_server.getTsdfMapPtr()->getTsdfLayer());
}

// ---- Different extrinsics: each sensor sees its own half of the room ----

TEST(MultiSensorServer, DifferentExtrinsicsEachSeeOwnWall) {
  auto node = makeMultiSensorNode(
      "different_extrinsics",
      {rclcpp::Parameter(
           "sensor_names", std::vector<std::string>{"a", "b"}),
       rclcpp::Parameter("sensors.a.topic", "a_pointcloud"),
       rclcpp::Parameter("sensors.b.topic", "b_pointcloud"),
       // A: T_B_C = identity (looks along local/body +x).
       rclcpp::Parameter(
           "sensors.b.T_B_C",
           // B: yaw 180 deg (looks along local -x, i.e. body/world -x).
           std::vector<double>{
               -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1})});
  TestServer server(node);
  ASSERT_EQ(server.sensors_.size(), 2u);

  constexpr int64_t kMinusNs = 995000000LL;
  constexpr int64_t kFrameNs = 1000000000LL;
  constexpr int64_t kPlusNs = 1005000000LL;
  pushTransform(&server, Transformation(), kMinusNs, "map");
  pushTransform(&server, Transformation(), kPlusNs, "map");

  // A narrow bundle of rays straight ahead (local +x), in the sensor-local
  // frame; T_B_C (world position stays at the body's origin, only rotation
  // differs) decides which wall each sensor actually reaches.
  const BoxRoomLidar straight_ahead(8, 4, 5.0, -5.0);
  const std::vector<Eigen::Vector3f> points =
      straight_ahead.raycast(Eigen::Vector3d::Zero(), 0.0);
  const rclcpp::Time stamp(kFrameNs, RCL_ROS_TIME);

  server.insertPointcloud(
      makeCloudMsg(points, stamp, "a_frame"), server.sensors_[0].get());
  server.insertPointcloud(
      makeCloudMsg(points, stamp, "b_frame"), server.sensors_[1].get());

  const Layer<TsdfVoxel>& layer = server.getTsdfMapPtr()->getTsdfLayer();
  const TsdfVoxel* plus_x_voxel =
      layer.getVoxelPtrByCoordinates(Point(4.9f, 0.0f, 0.0f));
  const TsdfVoxel* minus_x_voxel =
      layer.getVoxelPtrByCoordinates(Point(-4.9f, 0.0f, 0.0f));
  ASSERT_NE(plus_x_voxel, nullptr);
  ASSERT_NE(minus_x_voxel, nullptr);
  EXPECT_GT(plus_x_voxel->weight, 0.0f);
  EXPECT_GT(minus_x_voxel->weight, 0.0f);
  EXPECT_LT(std::abs(plus_x_voxel->distance), 0.5f);
  EXPECT_LT(std::abs(minus_x_voxel->distance), 0.5f);
}

// ---- Different ray limits: a short max_ray_length_m stays local ----

TEST(MultiSensorServer, ShortRayLengthDoesNotReachFarVoxels) {
  auto node = makeMultiSensorNode(
      "short_ray_length",
      {rclcpp::Parameter(
           "sensor_names", std::vector<std::string>{"a", "b"}),
       rclcpp::Parameter("sensors.a.topic", "a_pointcloud"),
       rclcpp::Parameter("sensors.b.topic", "b_pointcloud"),
       rclcpp::Parameter("sensors.b.max_ray_length_m", 2.0)});
  TestServer server(node);
  ASSERT_EQ(server.sensors_.size(), 2u);

  constexpr int64_t kMinusNs = 995000000LL;
  constexpr int64_t kFrameNs = 1000000000LL;
  constexpr int64_t kPlusNs = 1005000000LL;
  pushTransform(&server, Transformation(), kMinusNs, "map");
  pushTransform(&server, Transformation(), kPlusNs, "map");

  const BoxRoomLidar straight_ahead(8, 4, 5.0, -5.0);
  const std::vector<Eigen::Vector3f> points =
      straight_ahead.raycast(Eigen::Vector3d::Zero(), 0.0);
  const rclcpp::Time stamp(kFrameNs, RCL_ROS_TIME);

  // Only sensor B integrates -- isolates its own max_ray_length_m: the
  // actual points are ~5m away (the wall), well beyond B's 2m limit, so
  // nothing beyond 2m + truncation should ever be touched.
  server.insertPointcloud(
      makeCloudMsg(points, stamp, "b_frame"), server.sensors_[1].get());

  const Layer<TsdfVoxel>& layer = server.getTsdfMapPtr()->getTsdfLayer();
  const TsdfVoxel* far_voxel =
      layer.getVoxelPtrByCoordinates(Point(4.9f, 0.0f, 0.0f));
  if (far_voxel != nullptr) {
    EXPECT_FLOAT_EQ(far_voxel->weight, 0.0f)
        << "a voxel ~5m from sensor B's origin was touched despite "
           "max_ray_length_m: 2.0";
  }
}

// ---- Throttle is per sensor ----

TEST(MultiSensorServer, ThrottleIsPerSensor) {
  auto node = makeMultiSensorNode(
      "per_sensor_throttle",
      {rclcpp::Parameter(
           "sensor_names", std::vector<std::string>{"a", "b"}),
       rclcpp::Parameter("sensors.a.topic", "a_pointcloud"),
       rclcpp::Parameter("sensors.b.topic", "b_pointcloud"),
       rclcpp::Parameter("min_time_between_msgs_sec", 0.5)});
  TestServer server(node);
  ASSERT_EQ(server.sensors_.size(), 2u);

  constexpr int64_t kMinusNs = 995000000LL;
  constexpr int64_t kFrameNs = 1000000000LL;
  constexpr int64_t kPlusNs = 1005000000LL;
  pushTransform(&server, Transformation(), kMinusNs, "map");
  pushTransform(&server, Transformation(), kPlusNs, "map");

  const BoxRoomLidar straight_ahead(4, 4, 5.0, -5.0);
  const std::vector<Eigen::Vector3f> points =
      straight_ahead.raycast(Eigen::Vector3d::Zero(), 0.0);
  const rclcpp::Time stamp(kFrameNs, RCL_ROS_TIME);

  // A and B at the identical stamp: if the throttle clock were wrongly
  // shared across sensors, B's message (arriving "second") would be
  // throttled against A's just-updated clock. It isn't, because each
  // sensor's last_msg_time starts independently.
  server.insertPointcloud(
      makeCloudMsg(points, stamp, "a_frame"), server.sensors_[0].get());
  server.insertPointcloud(
      makeCloudMsg(points, stamp, "b_frame"), server.sensors_[1].get());

  EXPECT_EQ(server.sensors_[0]->num_integrated, 1u);
  EXPECT_EQ(server.sensors_[1]->num_integrated, 1u);
  EXPECT_EQ(server.sensors_[0]->num_throttled, 0u);
  EXPECT_EQ(server.sensors_[1]->num_throttled, 0u);
}

// ---- ICP guard (M11) ----

TEST(MultiSensorServer, IcpIsDisabledWithMultipleSensors) {
  auto node = makeMultiSensorNode(
      "icp_guard",
      {rclcpp::Parameter(
           "sensor_names", std::vector<std::string>{"a", "b"}),
       rclcpp::Parameter("sensors.a.topic", "a_pointcloud"),
       rclcpp::Parameter("sensors.b.topic", "b_pointcloud"),
       rclcpp::Parameter("enable_icp", true)});
  TestServer server(node);
  ASSERT_EQ(server.sensors_.size(), 2u);
  EXPECT_FALSE(server.enable_icp_);
  EXPECT_EQ(server.icp_transform_pub_, nullptr);
}

#else  // TEST_NP_TSDF_SERVER

// ---- NP equivalence: identical multi-sensor sensors == a legacy sensor ----

TEST(MultiSensorServerNp, AlternatingIdenticalSensorsMatchLegacy) {
  // Top-level sensor-model params (LiDAR, 256x16, fov 3/-25 deg): legacy
  // mode's "default" sensor and multi-sensor mode's un-overridden 'a'/'b'
  // both inherit these identically.
  const std::vector<rclcpp::Parameter> np_sensor_model = {
      rclcpp::Parameter("sensor_is_lidar", true),
      rclcpp::Parameter("width", 256), rclcpp::Parameter("height", 16),
      rclcpp::Parameter("fov_up", 3.0), rclcpp::Parameter("fov_down", -25.0)};

  auto legacy_node = makeLegacyNode("np_equivalence_legacy", np_sensor_model);
  TestServer legacy_server(legacy_node);

  std::vector<rclcpp::Parameter> multi_params = np_sensor_model;
  multi_params.push_back(rclcpp::Parameter(
      "sensor_names", std::vector<std::string>{"a", "b"}));
  multi_params.push_back(rclcpp::Parameter("sensors.a.topic", "a_pointcloud"));
  multi_params.push_back(rclcpp::Parameter("sensors.b.topic", "b_pointcloud"));
  auto multi_node = makeMultiSensorNode("np_equivalence_multi", multi_params);
  TestServer multi_server(multi_node);
  ASSERT_EQ(multi_server.sensors_.size(), 2u);

  const BoxRoomLidar lidar(256, 16, 3.0, -25.0);
  constexpr int kNumFrames = 20;
  constexpr int64_t kFrameStepNs = 100000000LL;
  constexpr int64_t kHalfBracketNs = 5000000LL;
  constexpr int64_t kBaseNs = 1000000000LL;

  for (int i = 0; i < kNumFrames; ++i) {
    const int64_t frame_ns = kBaseNs + static_cast<int64_t>(i) * kFrameStepNs;
    const int64_t minus_ns = frame_ns - kHalfBracketNs;
    const int64_t plus_ns = frame_ns + kHalfBracketNs;

    Eigen::Vector3d position;
    double yaw;
    lidar.sensorPose(static_cast<double>(minus_ns) * 1e-9, &position, &yaw);
    const Transformation T_minus(
        Transformation::Position(position.x(), position.y(), position.z()),
        Transformation::Rotation(Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
                                      .cast<float>()));
    lidar.sensorPose(static_cast<double>(plus_ns) * 1e-9, &position, &yaw);
    const Transformation T_plus(
        Transformation::Position(position.x(), position.y(), position.z()),
        Transformation::Rotation(Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
                                      .cast<float>()));

    pushTransform(&legacy_server, T_minus, minus_ns, "map");
    pushTransform(&legacy_server, T_plus, plus_ns, "map");
    pushTransform(&multi_server, T_minus, minus_ns, "map");
    pushTransform(&multi_server, T_plus, plus_ns, "map");

    lidar.sensorPose(static_cast<double>(frame_ns) * 1e-9, &position, &yaw);
    const std::vector<Eigen::Vector3f> points = lidar.raycast(position, yaw);
    const auto legacy_msg = makeCloudMsg(
        points, rclcpp::Time(frame_ns, RCL_ROS_TIME), "lidar");
    const auto multi_msg = makeCloudMsg(
        points, rclcpp::Time(frame_ns, RCL_ROS_TIME), "lidar");

    legacy_server.insertPointcloud(legacy_msg);
    multi_server.insertPointcloud(
        multi_msg, multi_server.sensors_[i % 2].get());
  }

  CompareLayersExact(
      legacy_server.getTsdfMapPtr()->getTsdfLayer(),
      multi_server.getTsdfMapPtr()->getTsdfLayer());
}

// ---- Mixed LiDAR + camera sensors, each with its own projector (M8) ----

TEST(MultiSensorServerNp, MixedLidarAndCameraSensorsBothIntegrate) {
  auto node = makeMultiSensorNode(
      "np_mixed_lidar_camera",
      {rclcpp::Parameter(
           "sensor_names", std::vector<std::string>{"a", "b"}),
       rclcpp::Parameter("sensors.a.topic", "a_pointcloud"),
       rclcpp::Parameter("sensors.a.sensor_is_lidar", true),
       rclcpp::Parameter("sensors.a.width", 256),
       rclcpp::Parameter("sensors.a.height", 16),
       rclcpp::Parameter("sensors.a.fov_up", 3.0),
       rclcpp::Parameter("sensors.a.fov_down", -25.0),
       rclcpp::Parameter("sensors.b.topic", "b_pointcloud"),
       rclcpp::Parameter("sensors.b.sensor_is_lidar", false),
       rclcpp::Parameter("sensors.b.width", 640),
       rclcpp::Parameter("sensors.b.height", 480),
       rclcpp::Parameter("sensors.b.fx", 450.0),
       rclcpp::Parameter("sensors.b.fy", 450.0),
       rclcpp::Parameter("sensors.b.vx", 320.0),
       rclcpp::Parameter("sensors.b.vy", 240.0)});
  TestServer server(node);
  ASSERT_EQ(server.sensors_.size(), 2u);

  constexpr int64_t kMinusNs = 995000000LL;
  constexpr int64_t kFrameNs = 1000000000LL;
  constexpr int64_t kPlusNs = 1005000000LL;
  pushTransform(&server, Transformation(), kMinusNs, "map");
  pushTransform(&server, Transformation(), kPlusNs, "map");
  const rclcpp::Time stamp(kFrameNs, RCL_ROS_TIME);

  // Sensor A: a LiDAR box-room scan (static pose at the room's center),
  // matching its own configured projector resolution/FOV. Since the sensor
  // pose is identity, the row-8 (an interior row, not the fov_up/fov_down
  // boundary row -- projectPointToImageLiDAR's asin/atan2 round trip can
  // push a boundary-row point a float epsilon out of [0, height), which
  // its strict floor/ceil bounds check then rejects), col-0 hit is itself
  // the exact world-frame wall hit; probe 2% short of it along the same
  // ray (just inside the wall) to avoid depending on which side of a
  // voxel/truncation boundary the raw float hit lands on.
  constexpr int kWidth = 256, kHeight = 16;
  const BoxRoomLidar lidar(kWidth, kHeight, 3.0, -25.0);
  const std::vector<Eigen::Vector3f> a_points =
      lidar.raycast(Eigen::Vector3d::Zero(), 0.0);
  const Point a_probe = a_points[8 * kWidth] * 0.98f;
  const Layer<TsdfVoxel>& layer = server.getTsdfMapPtr()->getTsdfLayer();
  server.insertPointcloud(
      makeCloudMsg(a_points, stamp, "a_frame"), server.sensors_[0].get());

  const TsdfVoxel* lidar_wall_voxel = layer.getVoxelPtrByCoordinates(a_probe);
  ASSERT_NE(lidar_wall_voxel, nullptr);
  EXPECT_GT(lidar_wall_voxel->weight, 0.0f);

  // Sensor B: a small camera-model grid at a 2m fronto-parallel plane
  // (pinhole convention: x = (u - vx) * z / fx, y = (v - vy) * z / fy).
  // Includes the image center exactly (u=320, v=240 -> local (0, 0, 2)),
  // so its world position is exactly predictable (identity T_B_C/T_B_D).
  constexpr double kFx = 450.0, kFy = 450.0, kVx = 320.0, kVy = 240.0;
  constexpr float kZ = 2.0f;
  std::vector<Eigen::Vector3f> b_points;
  for (int v = 0; v <= 480; v += 40) {
    for (int u = 0; u <= 640; u += 40) {
      b_points.emplace_back(
          static_cast<float>((u - kVx) * kZ / kFx),
          static_cast<float>((v - kVy) * kZ / kFy), kZ);
    }
  }
  server.insertPointcloud(
      makeCloudMsg(b_points, stamp, "b_frame"), server.sensors_[1].get());

  // The exact camera-center voxel: just short of z=2 (in front of the
  // surface, along the ray from the origin).
  const TsdfVoxel* camera_center_voxel =
      layer.getVoxelPtrByCoordinates(Point(0.0f, 0.0f, 1.9f));
  ASSERT_NE(camera_center_voxel, nullptr);
  EXPECT_GT(camera_center_voxel->weight, 0.0f);

  // Both sensors' evidence must coexist in the one shared map: sensor A's
  // wall voxel is still there after sensor B's later integration.
  const TsdfVoxel* lidar_wall_voxel_after_b =
      layer.getVoxelPtrByCoordinates(a_probe);
  ASSERT_NE(lidar_wall_voxel_after_b, nullptr);
  EXPECT_GT(lidar_wall_voxel_after_b->weight, 0.0f);
}

// ---- Invalid per-sensor projector config fails fast at construction ----

TEST(MultiSensorServerNp, InvalidProjectorConfigThrowsAtConstruction) {
  auto node = makeMultiSensorNode(
      "np_invalid_projector",
      {rclcpp::Parameter(
           "sensor_names", std::vector<std::string>{"a", "b"}),
       rclcpp::Parameter("sensors.a.topic", "a_pointcloud"),
       rclcpp::Parameter("sensors.a.sensor_is_lidar", true),
       rclcpp::Parameter("sensors.a.width", 256),
       rclcpp::Parameter("sensors.a.height", 16),
       rclcpp::Parameter("sensors.a.fov_up", 3.0),
       rclcpp::Parameter("sensors.a.fov_down", -25.0),
       rclcpp::Parameter("sensors.b.topic", "b_pointcloud"),
       // Invalid: width defaults to 0 (never set).
       rclcpp::Parameter("sensors.b.height", 16)});

  std::string message;
  try {
    TestServer server(node);
  } catch (const std::invalid_argument& e) {
    message = e.what();
  }
  EXPECT_NE(message.find("'b'"), std::string::npos) << message;
  EXPECT_NE(message.find("invalid projector config"), std::string::npos)
      << message;
}

#endif  // TEST_NP_TSDF_SERVER

}  // namespace
}  // namespace voxfield

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
