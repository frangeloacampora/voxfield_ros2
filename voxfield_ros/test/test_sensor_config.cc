// Unit tests for loadSensors() (MULTI_SENSOR_PLAN.md Phase 4, M2/M13):
// legacy-mode passthrough, multi-sensor inheritance, and fail-fast
// validation.
#include <functional>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "voxfield_ros/ros_params.h"
#include "voxfield_ros/sensor_config_loader.h"

namespace voxfield {
namespace {

rclcpp::Node::SharedPtr makeNode(
    const std::string& name,
    const std::vector<rclcpp::Parameter>& overrides = {}) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  return std::make_shared<rclcpp::Node>(name, options);
}

SensorConfig legacySensorConfig() {
  SensorConfig config;
  config.name = "default";
  config.topic = "pointcloud";
  config.freespace_topic = "";
  config.frame = "";
  config.queue_size = 1;
  config.input_qos_best_effort = false;
  config.min_time_between_msgs_sec = 0.0;
  return config;
}

std::string throwMessage(const std::function<void()>& fn) {
  try {
    fn();
  } catch (const std::invalid_argument& e) {
    return e.what();
  }
  return "";  // did not throw
}

// ---- Legacy mode ----

TEST(SensorConfig, LegacyModeReturnsOneDefaultSensor) {
  auto node = makeNode("legacy_default");
  const TsdfIntegratorBase::Config tsdf_base =
      getTsdfIntegratorConfigFromRosParam(*node);

  const std::vector<LoadedSensor> sensors = loadSensors(
      *node, &tsdf_base, nullptr, nullptr, "merged", legacySensorConfig());

  ASSERT_EQ(sensors.size(), 1u);
  EXPECT_EQ(sensors[0].input.name, "default");
  EXPECT_EQ(sensors[0].input.topic, "pointcloud");
  EXPECT_EQ(sensors[0].method, "merged");
  // Field-by-field: legacy mode must be an unmodified copy of the
  // already-read top-level config, not re-derived independently.
  EXPECT_FLOAT_EQ(
      sensors[0].tsdf.default_truncation_distance,
      tsdf_base.default_truncation_distance);
  EXPECT_FLOAT_EQ(sensors[0].tsdf.max_weight, tsdf_base.max_weight);
  EXPECT_FLOAT_EQ(sensors[0].tsdf.min_ray_length_m, tsdf_base.min_ray_length_m);
  EXPECT_FLOAT_EQ(sensors[0].tsdf.max_ray_length_m, tsdf_base.max_ray_length_m);
  EXPECT_EQ(
      sensors[0].tsdf.voxel_carving_enabled, tsdf_base.voxel_carving_enabled);
  EXPECT_EQ(sensors[0].tsdf.integrator_threads, tsdf_base.integrator_threads);
  EXPECT_EQ(
      sensors[0].tsdf.integration_order_mode, tsdf_base.integration_order_mode);
}

TEST(SensorConfig, LegacyModeWithStraySensorsOverrideIsIgnored) {
  auto node = makeNode(
      "legacy_stray_override",
      {rclcpp::Parameter("sensors.foo.topic", "should_be_ignored")});
  const TsdfIntegratorBase::Config tsdf_base =
      getTsdfIntegratorConfigFromRosParam(*node);

  // sensor_names is unset, so this is still legacy mode: the stray
  // sensors.foo.topic override is logged (RCLCPP_WARN) and has no effect.
  const std::vector<LoadedSensor> sensors = loadSensors(
      *node, &tsdf_base, nullptr, nullptr, "merged", legacySensorConfig());
  ASSERT_EQ(sensors.size(), 1u);
  EXPECT_EQ(sensors[0].input.topic, "pointcloud");
}

// ---- Multi-sensor mode: inheritance ----

TEST(SensorConfig, PerSensorOverrideInheritsFromTopLevel) {
  auto node = makeNode(
      "inheritance",
      {rclcpp::Parameter("max_ray_length_m", 45.0),
       rclcpp::Parameter("sensor_names", std::vector<std::string>{"a", "b"}),
       rclcpp::Parameter("sensors.a.topic", "topic_a"),
       rclcpp::Parameter("sensors.b.topic", "topic_b"),
       rclcpp::Parameter("sensors.b.max_ray_length_m", 5.0)});
  const TsdfIntegratorBase::Config tsdf_base =
      getTsdfIntegratorConfigFromRosParam(*node);
  ASSERT_FLOAT_EQ(tsdf_base.max_ray_length_m, 45.0f);

  const std::vector<LoadedSensor> sensors = loadSensors(
      *node, &tsdf_base, nullptr, nullptr, "merged", legacySensorConfig());

  ASSERT_EQ(sensors.size(), 2u);
  EXPECT_EQ(sensors[0].input.name, "a");
  EXPECT_FLOAT_EQ(sensors[0].tsdf.max_ray_length_m, 45.0f);
  EXPECT_EQ(sensors[1].input.name, "b");
  EXPECT_FLOAT_EQ(sensors[1].tsdf.max_ray_length_m, 5.0f);
}

TEST(SensorConfig, LidarWeightingReachesTsdfIntegrator) {
  // sensor_is_lidar / weight_reduction_exp select the TSDF integrator's
  // point weight model too (not just the NP projector/integrator), both at
  // top level and per sensor; lidar_z_weighting is map-global.
  auto node = makeNode(
      "lidar_weighting",
      {rclcpp::Parameter("sensor_is_lidar", true),
       rclcpp::Parameter("weight_reduction_exp", 1.0),
       rclcpp::Parameter("lidar_z_weighting", true),
       rclcpp::Parameter("sensor_names", std::vector<std::string>{"a", "b"}),
       rclcpp::Parameter("sensors.a.topic", "topic_a"),
       rclcpp::Parameter("sensors.b.topic", "topic_b"),
       rclcpp::Parameter("sensors.b.sensor_is_lidar", false),
       rclcpp::Parameter("sensors.b.weight_reduction_exp", 2.0)});
  const TsdfIntegratorBase::Config tsdf_base =
      getTsdfIntegratorConfigFromRosParam(*node);
  EXPECT_TRUE(tsdf_base.sensor_is_lidar);
  EXPECT_TRUE(tsdf_base.lidar_z_weighting);

  const std::vector<LoadedSensor> sensors = loadSensors(
      *node, &tsdf_base, nullptr, nullptr, "merged", legacySensorConfig());
  ASSERT_EQ(sensors.size(), 2u);
  EXPECT_TRUE(sensors[0].tsdf.sensor_is_lidar);
  EXPECT_FLOAT_EQ(sensors[0].tsdf.weight_reduction_exp, 1.0f);
  EXPECT_FALSE(sensors[1].tsdf.sensor_is_lidar);
  EXPECT_FLOAT_EQ(sensors[1].tsdf.weight_reduction_exp, 2.0f);
  EXPECT_TRUE(sensors[1].tsdf.lidar_z_weighting);
}

TEST(SensorConfig, LidarZWeightingIsMapGlobal) {
  auto node = makeNode(
      "lidar_z_weighting_per_sensor",
      {rclcpp::Parameter("sensor_names", std::vector<std::string>{"a"}),
       rclcpp::Parameter("sensors.a.topic", "topic_a"),
       rclcpp::Parameter("sensors.a.lidar_z_weighting", true)});
  const TsdfIntegratorBase::Config tsdf_base =
      getTsdfIntegratorConfigFromRosParam(*node);
  EXPECT_THROW(
      loadSensors(
          *node, &tsdf_base, nullptr, nullptr, "merged", legacySensorConfig()),
      std::invalid_argument);
}

TEST(SensorConfig, FrameIsNotInheritedFromLegacySensorFrame) {
  // legacySensorConfig() here simulates a legacy sensor_frame of "unit_test"
  // (as if the top-level `sensor_frame` param were set); a multi-sensor
  // sensor must NOT inherit it -- that would force every sensor into one
  // frame, defeating the point of per-sensor frames (M2).
  SensorConfig legacy = legacySensorConfig();
  legacy.frame = "unit_test_legacy_frame";
  auto node = makeNode(
      "frame_not_inherited",
      {rclcpp::Parameter("sensor_names", std::vector<std::string>{"a"}),
       rclcpp::Parameter("sensors.a.topic", "topic_a")});
  const TsdfIntegratorBase::Config tsdf_base =
      getTsdfIntegratorConfigFromRosParam(*node);

  const std::vector<LoadedSensor> sensors =
      loadSensors(*node, &tsdf_base, nullptr, nullptr, "merged", legacy);
  ASSERT_EQ(sensors.size(), 1u);
  EXPECT_TRUE(sensors[0].input.frame.empty());
}

// ---- Multi-sensor mode: validation errors ----

TEST(SensorConfig, UnknownKeyTypoThrows) {
  auto node = makeNode(
      "unknown_key",
      {rclcpp::Parameter("sensor_names", std::vector<std::string>{"a"}),
       rclcpp::Parameter("sensors.a.topic", "t"),
       rclcpp::Parameter("sensors.a.max_ray_lenght_m", 1.0)});
  const TsdfIntegratorBase::Config tsdf_base =
      getTsdfIntegratorConfigFromRosParam(*node);

  const std::string message = throwMessage([&] {
    loadSensors(
        *node, &tsdf_base, nullptr, nullptr, "merged", legacySensorConfig());
  });
  EXPECT_NE(message.find("max_ray_lenght_m"), std::string::npos) << message;
  EXPECT_NE(message.find("unknown"), std::string::npos) << message;
}

TEST(SensorConfig, UnknownSensorNameThrows) {
  auto node = makeNode(
      "unknown_sensor",
      {rclcpp::Parameter("sensor_names", std::vector<std::string>{"a"}),
       rclcpp::Parameter("sensors.a.topic", "t"),
       rclcpp::Parameter("sensors.c.topic", "x")});
  const TsdfIntegratorBase::Config tsdf_base =
      getTsdfIntegratorConfigFromRosParam(*node);

  const std::string message = throwMessage([&] {
    loadSensors(
        *node, &tsdf_base, nullptr, nullptr, "merged", legacySensorConfig());
  });
  EXPECT_NE(message.find("'c'"), std::string::npos) << message;
  EXPECT_NE(message.find("not in sensor_names"), std::string::npos) << message;
}

TEST(SensorConfig, ForbiddenMapGlobalKeyThrows) {
  auto node = makeNode(
      "forbidden_key",
      {rclcpp::Parameter("sensor_names", std::vector<std::string>{"a"}),
       rclcpp::Parameter("sensors.a.topic", "t"),
       rclcpp::Parameter("sensors.a.truncation_distance", -3.0)});
  const TsdfIntegratorBase::Config tsdf_base =
      getTsdfIntegratorConfigFromRosParam(*node);

  const std::string message = throwMessage([&] {
    loadSensors(
        *node, &tsdf_base, nullptr, nullptr, "merged", legacySensorConfig());
  });
  EXPECT_NE(message.find("truncation_distance"), std::string::npos) << message;
  EXPECT_NE(message.find("map-global"), std::string::npos) << message;
}

TEST(SensorConfig, MissingTopicThrows) {
  auto node = makeNode(
      "missing_topic",
      {rclcpp::Parameter("sensor_names", std::vector<std::string>{"a"})});
  const TsdfIntegratorBase::Config tsdf_base =
      getTsdfIntegratorConfigFromRosParam(*node);

  const std::string message = throwMessage([&] {
    loadSensors(
        *node, &tsdf_base, nullptr, nullptr, "merged", legacySensorConfig());
  });
  EXPECT_NE(message.find("missing required key 'topic'"), std::string::npos)
      << message;
}

TEST(SensorConfig, DuplicateTopicThrows) {
  auto node = makeNode(
      "duplicate_topic",
      {rclcpp::Parameter("sensor_names", std::vector<std::string>{"a", "b"}),
       rclcpp::Parameter("sensors.a.topic", "same"),
       rclcpp::Parameter("sensors.b.topic", "same")});
  const TsdfIntegratorBase::Config tsdf_base =
      getTsdfIntegratorConfigFromRosParam(*node);

  const std::string message = throwMessage([&] {
    loadSensors(
        *node, &tsdf_base, nullptr, nullptr, "merged", legacySensorConfig());
  });
  EXPECT_NE(message.find("used by another sensor"), std::string::npos)
      << message;
}

TEST(SensorConfig, DuplicateNameThrows) {
  auto node = makeNode(
      "duplicate_name",
      {rclcpp::Parameter("sensor_names", std::vector<std::string>{"a", "a"})});
  const TsdfIntegratorBase::Config tsdf_base =
      getTsdfIntegratorConfigFromRosParam(*node);

  const std::string message = throwMessage([&] {
    loadSensors(
        *node, &tsdf_base, nullptr, nullptr, "merged", legacySensorConfig());
  });
  EXPECT_NE(message.find("duplicated"), std::string::npos) << message;
}

TEST(SensorConfig, InvalidProjectorConfigThrowsForNpServer) {
  auto node = makeNode(
      "invalid_projector",
      {rclcpp::Parameter("sensor_names", std::vector<std::string>{"a"}),
       rclcpp::Parameter("sensors.a.topic", "t")});
  const TsdfIntegratorBase::Config tsdf_base =
      getTsdfIntegratorConfigFromRosParam(*node);
  const NpTsdfIntegratorBase::Config np_base =
      getNpTsdfIntegratorConfigFromRosParam(*node);
  // Default-constructed: width/height are 0, so isValid() must fail.
  const RangeImageProjector::Config projector_base;

  const std::string message = throwMessage([&] {
    loadSensors(
        *node, &tsdf_base, &np_base, &projector_base, "merged",
        legacySensorConfig());
  });
  EXPECT_NE(message.find("invalid projector config"), std::string::npos)
      << message;
}

TEST(SensorConfig, MultipleErrorsAreAllListed) {
  auto node = makeNode(
      "multiple_errors",
      {rclcpp::Parameter("sensor_names", std::vector<std::string>{"a", "b"}),
       // 'a' is missing its required topic.
       rclcpp::Parameter("sensors.b.topic", "topic_b"),
       // 'b' also has an unknown key.
       rclcpp::Parameter("sensors.b.max_ray_lenght_m", 1.0)});
  const TsdfIntegratorBase::Config tsdf_base =
      getTsdfIntegratorConfigFromRosParam(*node);

  const std::string message = throwMessage([&] {
    loadSensors(
        *node, &tsdf_base, nullptr, nullptr, "merged", legacySensorConfig());
  });
  EXPECT_NE(message.find("missing required key 'topic'"), std::string::npos)
      << message;
  EXPECT_NE(message.find("max_ray_lenght_m"), std::string::npos) << message;
  EXPECT_NE(message.find("2 "), std::string::npos) << message;
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
