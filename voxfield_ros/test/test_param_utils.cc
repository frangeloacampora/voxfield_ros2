#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>

#include "voxfield_ros/param_utils.h"

namespace voxfield {
namespace {

rclcpp::Node::SharedPtr makeNode(
    const std::string& name,
    const std::vector<rclcpp::Parameter>& overrides = {}) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  return std::make_shared<rclcpp::Node>(name, options);
}

TEST(ParamUtils, MissingParameterReturnsDefault) {
  auto node = makeNode("test_missing_param");
  EXPECT_DOUBLE_EQ(getParam<double>(*node, "not_set", 4.5), 4.5);
  EXPECT_EQ(getParam<int>(*node, "also_not_set", 7), 7);
  EXPECT_EQ(getParam<bool>(*node, "bool_not_set", true), true);
  EXPECT_EQ(
      getParam<std::string>(*node, "str_not_set", "fallback"), "fallback");
}

TEST(ParamUtils, IntegerOverrideCoercedToDouble) {
  // A YAML file with "tsdf_voxel_size: 0" parses as an integer; the target
  // C++ type is double/float. D6 requires this to coerce, not throw.
  auto node =
      makeNode("test_int_to_double", {rclcpp::Parameter("voxel_size", 2)});
  EXPECT_DOUBLE_EQ(getParam<double>(*node, "voxel_size", 0.25), 2.0);
  EXPECT_FLOAT_EQ(getParam<float>(*node, "voxel_size", 0.25f), 2.0f);
}

TEST(ParamUtils, DoubleOverrideCoercedToIntWhenIntegral) {
  auto node =
      makeNode("test_double_to_int_ok", {rclcpp::Parameter("count", 3.0)});
  EXPECT_EQ(getParam<int>(*node, "count", 1), 3);
}

TEST(ParamUtils, NonIntegralDoubleOverrideFallsBackToDefaultForIntTarget) {
  auto node =
      makeNode("test_double_to_int_bad", {rclcpp::Parameter("count", 3.5)});
  EXPECT_EQ(getParam<int>(*node, "count", 1), 1);
}

TEST(ParamUtils, RepeatedReadsDoNotThrow) {
  auto node = makeNode("test_repeated_reads", {rclcpp::Parameter("value", 42)});
  EXPECT_EQ(getParam<int>(*node, "value", 0), 42);
  // A second read of the same name must not hit
  // ParameterAlreadyDeclaredException.
  EXPECT_EQ(getParam<int>(*node, "value", 0), 42);
  EXPECT_EQ(getParam<int>(*node, "value", 0), 42);
}

TEST(ParamUtils, ParamInPlaceHelperMatchesGetParam) {
  auto node = makeNode("test_param_in_place", {rclcpp::Parameter("x", 1.5)});
  double x = 0.0;
  param(*node, "x", x);
  EXPECT_DOUBLE_EQ(x, 1.5);
}

TEST(ParamUtils, TransformationParamAbsentReturnsFalse) {
  auto node = makeNode("test_transform_absent");
  Transformation T;
  T.setIdentity();
  const Transformation before = T;
  EXPECT_FALSE(getTransformationParam(*node, "T_B_C", "invert_T_B_C", &T));
  EXPECT_TRUE(T == before);
}

TEST(ParamUtils, TransformationParamRoundTrip) {
  // Row-major 4x4: translation (1, 2, 3), identity rotation.
  const std::vector<double> flat = {1.0, 0.0, 0.0, 1.0,  //
                                    0.0, 1.0, 0.0, 2.0,  //
                                    0.0, 0.0, 1.0, 3.0,  //
                                    0.0, 0.0, 0.0, 1.0};
  auto node =
      makeNode("test_transform_round_trip", {rclcpp::Parameter("T_B_C", flat)});

  Transformation T;
  ASSERT_TRUE(getTransformationParam(*node, "T_B_C", "invert_T_B_C", &T));
  EXPECT_NEAR(T.getPosition().x(), 1.0, 1e-5);
  EXPECT_NEAR(T.getPosition().y(), 2.0, 1e-5);
  EXPECT_NEAR(T.getPosition().z(), 3.0, 1e-5);
  EXPECT_NEAR(T.getRotation().w(), 1.0, 1e-5);
}

TEST(ParamUtils, TransformationParamInverted) {
  const std::vector<double> flat = {1.0, 0.0, 0.0, 1.0,  //
                                    0.0, 1.0, 0.0, 2.0,  //
                                    0.0, 0.0, 1.0, 3.0,  //
                                    0.0, 0.0, 0.0, 1.0};
  auto node = makeNode(
      "test_transform_inverted", {rclcpp::Parameter("T_B_C", flat),
                                  rclcpp::Parameter("invert_T_B_C", true)});

  Transformation T_forward;
  ASSERT_TRUE(
      getTransformationParam(*node, "T_B_C", "invert_T_B_C_unset", &T_forward));

  Transformation T_inverted;
  ASSERT_TRUE(
      getTransformationParam(*node, "T_B_C", "invert_T_B_C", &T_inverted));

  EXPECT_TRUE(T_inverted == T_forward.inverse());
  // Inverting a pure translation of (1, 2, 3) gives (-1, -2, -3).
  EXPECT_NEAR(T_inverted.getPosition().x(), -1.0, 1e-5);
  EXPECT_NEAR(T_inverted.getPosition().y(), -2.0, 1e-5);
  EXPECT_NEAR(T_inverted.getPosition().z(), -3.0, 1e-5);
}

TEST(ParamUtils, TransformationParamWrongSizeReturnsFalse) {
  auto node = makeNode(
      "test_transform_wrong_size",
      {rclcpp::Parameter("T_B_C", std::vector<double>{1.0, 2.0, 3.0})});
  Transformation T;
  T.setIdentity();
  EXPECT_FALSE(getTransformationParam(*node, "T_B_C", "invert_T_B_C", &T));
}

TEST(ParamUtils, StringArrayMissingReturnsDefault) {
  auto node = makeNode("test_string_array_missing");
  const std::vector<std::string> default_value = {"a", "b"};
  EXPECT_EQ(
      getParam<std::vector<std::string>>(*node, "not_set", default_value),
      default_value);
}

TEST(ParamUtils, StringArrayOverride) {
  auto node = makeNode(
      "test_string_array_override",
      {rclcpp::Parameter(
          "sensor_names", std::vector<std::string>{"front_lidar",
                                                     "back_lidar"})});
  const std::vector<std::string> names =
      getParam<std::vector<std::string>>(*node, "sensor_names", {});
  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "front_lidar");
  EXPECT_EQ(names[1], "back_lidar");
}

TEST(ParamUtils, StringArrayWrongTypeFallsBackToDefault) {
  auto node =
      makeNode("test_string_array_wrong_type", {rclcpp::Parameter("x", 1.5)});
  const std::vector<std::string> default_value = {"fallback"};
  EXPECT_EQ(
      getParam<std::vector<std::string>>(*node, "x", default_value),
      default_value);
}

TEST(ParamUtils, ListParameterOverridesFindsPrefixedKeysOnly) {
  auto node = makeNode(
      "test_list_overrides",
      {rclcpp::Parameter("sensors.front_lidar.topic", "front"),
       rclcpp::Parameter("sensors.back_lidar.topic", "back"),
       rclcpp::Parameter("world_frame", "map")});
  std::vector<std::string> names = listParameterOverrides(*node, "sensors.");
  std::sort(names.begin(), names.end());
  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "sensors.back_lidar.topic");
  EXPECT_EQ(names[1], "sensors.front_lidar.topic");
}

TEST(ParamUtils, ListParameterOverridesEmptyWhenNoneMatch) {
  auto node = makeNode(
      "test_list_overrides_empty", {rclcpp::Parameter("world_frame", "map")});
  EXPECT_TRUE(listParameterOverrides(*node, "sensors.").empty());
}

TEST(ParamUtils, EurocCalibTransformsLoadFromYaml) {
  // Regression for "Known upstream issues" #7: euroc_calib.yaml spelled the
  // key `T_B_D::`, which YAML parses as a literal `T_B_D:` key, so T_B_D was
  // never readable under its real name.
  rclcpp::NodeOptions options;
  options.arguments(
      {"--ros-args", "--params-file",
       std::string(VOXFIELD_ROS_CFG_DIR) + "/calib/euroc_calib.yaml"});
  auto node = std::make_shared<rclcpp::Node>("test_euroc_calib", options);

  Transformation T_B_C, T_B_D;
  ASSERT_TRUE(getTransformationParam(*node, "T_B_C", "invert_T_B_C", &T_B_C));
  ASSERT_TRUE(getTransformationParam(*node, "T_B_D", "invert_T_B_D", &T_B_D));
  // Translation column of the file's T_B_D.
  EXPECT_NEAR(T_B_D.getPosition().x(), 0.06901, 1e-5);
  EXPECT_NEAR(T_B_D.getPosition().y(), -0.02781, 1e-5);
  EXPECT_NEAR(T_B_D.getPosition().z(), -0.12395, 1e-5);
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
