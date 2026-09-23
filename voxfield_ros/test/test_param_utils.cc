#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

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
  EXPECT_EQ(getParam<std::string>(*node, "str_not_set", "fallback"), "fallback");
}

TEST(ParamUtils, IntegerOverrideCoercedToDouble) {
  // A YAML file with "tsdf_voxel_size: 0" parses as an integer; the target
  // C++ type is double/float. D6 requires this to coerce, not throw.
  auto node = makeNode(
      "test_int_to_double",
      {rclcpp::Parameter("voxel_size", 2)});
  EXPECT_DOUBLE_EQ(getParam<double>(*node, "voxel_size", 0.25), 2.0);
  EXPECT_FLOAT_EQ(getParam<float>(*node, "voxel_size", 0.25f), 2.0f);
}

TEST(ParamUtils, DoubleOverrideCoercedToIntWhenIntegral) {
  auto node = makeNode(
      "test_double_to_int_ok",
      {rclcpp::Parameter("count", 3.0)});
  EXPECT_EQ(getParam<int>(*node, "count", 1), 3);
}

TEST(ParamUtils, NonIntegralDoubleOverrideFallsBackToDefaultForIntTarget) {
  auto node = makeNode(
      "test_double_to_int_bad",
      {rclcpp::Parameter("count", 3.5)});
  EXPECT_EQ(getParam<int>(*node, "count", 1), 1);
}

TEST(ParamUtils, RepeatedReadsDoNotThrow) {
  auto node = makeNode(
      "test_repeated_reads",
      {rclcpp::Parameter("value", 42)});
  EXPECT_EQ(getParam<int>(*node, "value", 0), 42);
  // A second read of the same name must not hit
  // ParameterAlreadyDeclaredException.
  EXPECT_EQ(getParam<int>(*node, "value", 0), 42);
  EXPECT_EQ(getParam<int>(*node, "value", 0), 42);
}

TEST(ParamUtils, ParamInPlaceHelperMatchesGetParam) {
  auto node = makeNode(
      "test_param_in_place",
      {rclcpp::Parameter("x", 1.5)});
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
  const std::vector<double> flat = {
      1.0, 0.0, 0.0, 1.0,   //
      0.0, 1.0, 0.0, 2.0,   //
      0.0, 0.0, 1.0, 3.0,   //
      0.0, 0.0, 0.0, 1.0};
  auto node = makeNode(
      "test_transform_round_trip",
      {rclcpp::Parameter("T_B_C", flat)});

  Transformation T;
  ASSERT_TRUE(getTransformationParam(*node, "T_B_C", "invert_T_B_C", &T));
  EXPECT_NEAR(T.getPosition().x(), 1.0, 1e-5);
  EXPECT_NEAR(T.getPosition().y(), 2.0, 1e-5);
  EXPECT_NEAR(T.getPosition().z(), 3.0, 1e-5);
  EXPECT_NEAR(T.getRotation().w(), 1.0, 1e-5);
}

TEST(ParamUtils, TransformationParamInverted) {
  const std::vector<double> flat = {
      1.0, 0.0, 0.0, 1.0,   //
      0.0, 1.0, 0.0, 2.0,   //
      0.0, 0.0, 1.0, 3.0,   //
      0.0, 0.0, 0.0, 1.0};
  auto node = makeNode(
      "test_transform_inverted",
      {rclcpp::Parameter("T_B_C", flat),
       rclcpp::Parameter("invert_T_B_C", true)});

  Transformation T_forward;
  ASSERT_TRUE(getTransformationParam(
      *node, "T_B_C", "invert_T_B_C_unset", &T_forward));

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

}  // namespace
}  // namespace voxfield

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
