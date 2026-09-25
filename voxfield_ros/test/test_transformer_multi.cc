// Unit tests for Transformer's multi-sensor support (MULTI_SENSOR_PLAN.md
// Phase 3, M5/M6): per-sensor T_B_C in queue mode, the transform-queue
// retention window (fixes A8: the old "erase everything up to this lookup's
// bracket" policy broke with >1 sensor), and TF-mode frame resolution with
// no hidden sensor_frame_ override (the mechanism F1 relies on).
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "voxfield_ros/kindr_conversions.h"
#include "voxfield_ros/transformer.h"

namespace voxfield {
namespace {

rclcpp::Node::SharedPtr makeQueueModeNode(
    const std::string& name, double retention_sec = 1.0) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("use_tf_transforms", false),
      rclcpp::Parameter("world_frame", "map"),
      rclcpp::Parameter("transform_queue_retention_sec", retention_sec),
  });
  return std::make_shared<rclcpp::Node>(name, options);
}

rclcpp::Node::SharedPtr makeTfModeNode(const std::string& name) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("use_tf_transforms", true),
      rclcpp::Parameter("world_frame", "map"),
  });
  return std::make_shared<rclcpp::Node>(name, options);
}

Transformation translation(double x, double y, double z) {
  return Transformation(
      Transformation::Position(x, y, z), Transformation::Rotation());
}

geometry_msgs::msg::TransformStamped::SharedPtr makeStamped(
    const Transformation& T, int64_t stamp_ns) {
  auto msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
  msg->header.stamp = rclcpp::Time(stamp_ns, RCL_ROS_TIME);
  transformKindrToMsg(T.cast<double>(), &msg->transform);
  return msg;
}

// ---- Queue mode: per-sensor T_B_C, out-of-order multi-sensor lookups ----

// Regression for A8. With the old "erase everything before this lookup's
// matched/bracketing entry" policy, sensor A's lookup at t=1.000s would
// erase every queue entry strictly before its own match, so a *second*
// sensor's lookup at t=0.9995s (earlier, for a different sensor with a
// different extrinsic) would find its own bracketing poses already gone,
// and fail. Confirmed against the pre-M6 code (docs/MULTI_SENSOR_NOTES.md):
// it does fail there. The retention-window replacement fixes it: both
// lookups succeed, and interpolation for B is exact since T_G_D here is a
// pure, constant-velocity translation.
TEST(TransformerMulti, DifferentSensorsCanLookUpOutOfOrder) {
  auto node = makeQueueModeNode("multi_out_of_order");
  Transformer transformer(node);

  // T_G_D(t) = translation(t, 0, 0): poses at 100 ms steps, t = 0.9s..2.1s.
  for (int i = 0; i <= 12; ++i) {
    const int64_t stamp_ns = 900000000LL + i * 100000000LL;
    const double t = 0.9 + static_cast<double>(i) * 0.1;
    transformer.transformCallback(
        makeStamped(translation(t, 0.0, 0.0), stamp_ns));
  }

  const Transformation T_B_C_a = translation(0.1, 0.0, 0.0);
  const Transformation T_B_C_b = translation(-0.1, 0.0, 0.0);

  // Sensor A at t=1.000s: exact match to a queued pose.
  Transformation T_G_C_a;
  ASSERT_TRUE(transformer.lookupTransformQueue(
      rclcpp::Time(1000000000LL, RCL_ROS_TIME), T_B_C_a, &T_G_C_a));
  EXPECT_NEAR(T_G_C_a.getPosition().x(), 1.1, 1e-6);

  // Sensor B at t=0.995s: earlier than A's own lookup timestamp and more
  // than timestamp_tolerance_sec (1 ms default) away from any queued
  // entry, so it genuinely interpolates between the queue's 0.9s and 1.0s
  // entries -- both of which the old erase-up-to-match policy would have
  // discarded once A's lookup ran (A's match sits at the 1.0s entry).
  Transformation T_G_C_b;
  ASSERT_TRUE(transformer.lookupTransformQueue(
      rclcpp::Time(995000000LL, RCL_ROS_TIME), T_B_C_b, &T_G_C_b));
  // T_G_D(0.995) * T_B_D^-1 * T_B_C_b = translation(0.995, 0, 0) *
  // translation(-0.1, 0, 0) = translation(0.895, 0, 0) (T_B_D is identity;
  // pure-translation composition along the same axis is exact).
  EXPECT_NEAR(T_G_C_b.getPosition().x(), 0.895, 1e-6);
  EXPECT_NEAR(T_G_C_b.getPosition().y(), 0.0, 1e-6);
  EXPECT_NEAR(T_G_C_b.getPosition().z(), 0.0, 1e-6);
}

// ---- Queue mode: retention window and size cap ----

TEST(TransformerMulti, RetentionWindowErasesOnlyOldEntries) {
  auto node = makeQueueModeNode("retention_window", /*retention_sec=*/1.0);
  Transformer transformer(node);

  // 20 entries, 0ms..1900ms.
  for (int i = 0; i < 20; ++i) {
    const int64_t stamp_ns = static_cast<int64_t>(i) * 100000000LL;
    transformer.transformCallback(
        makeStamped(translation(0.0, 0.0, 0.0), stamp_ns));
  }
  ASSERT_EQ(transformer.transformQueueSizeForTest(), 20u);

  // Exact match at the newest (1900ms) entry. Retention cutoff = 1900ms -
  // 1000ms = 900ms: the 9 entries strictly older than 900ms (0..800ms) are
  // erased; the 11 entries from 900ms onward (inclusive) are kept.
  Transformation T_G_C;
  ASSERT_TRUE(transformer.lookupTransformQueue(
      rclcpp::Time(1900000000LL, RCL_ROS_TIME), Transformation(), &T_G_C));
  EXPECT_EQ(transformer.transformQueueSizeForTest(), 11u);
}

TEST(TransformerMulti, SizeCapDropsOldestBeyondTenThousand) {
  // A large retention window so only the size cap (not retention) is
  // exercised: every pushed entry is within 1 second of the lookup below.
  auto node = makeQueueModeNode("size_cap", /*retention_sec=*/100.0);
  Transformer transformer(node);

  constexpr int kNumEntries = 10005;
  for (int i = 0; i < kNumEntries; ++i) {
    const int64_t stamp_ns = static_cast<int64_t>(i) * 1000000LL;  // 1 ms apart
    transformer.transformCallback(
        makeStamped(translation(0.0, 0.0, 0.0), stamp_ns));
  }
  ASSERT_EQ(transformer.transformQueueSizeForTest(), 10005u);

  Transformation T_G_C;
  const int64_t last_stamp_ns =
      static_cast<int64_t>(kNumEntries - 1) * 1000000LL;
  ASSERT_TRUE(transformer.lookupTransformQueue(
      rclcpp::Time(last_stamp_ns, RCL_ROS_TIME), Transformation(), &T_G_C));
  EXPECT_EQ(transformer.transformQueueSizeForTest(), 10000u);
}

// ---- TF mode: frame is used exactly as given, no hidden override ----

// Before this phase, Transformer silently substituted its own sensor_frame_
// for whatever from_frame a caller passed in, if sensor_frame_ was set --
// so every sensor was really looked up in one shared frame. Now the frame
// string a caller passes through is used exactly as given. This is also
// the mechanism F1 relies on: TsdfServer/NpTsdfServer now resolve an unset
// per-server `sensor_frame` param to the message's own header.frame_id
// *before* calling lookupTransform()/lookupSensorTransform() -- simulated
// here by passing two different frame names directly, standing in for
// "cfg.frame empty -> use this cloud's header.frame_id" and "cfg.frame set
// -> use it", per sensor.
TEST(TransformerMulti, TfModeUsesTheGivenFrameForEachSensor) {
  auto node = makeTfModeNode("tf_mode_no_override");
  Transformer transformer(node);

  constexpr int64_t kStampNs = 5000000000LL;
  const rclcpp::Time stamp(kStampNs, RCL_ROS_TIME);

  auto broadcastStatic = [&](const std::string& child_frame,
                              const Transformation& T) {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = "map";  // world_frame_
    msg.header.stamp = stamp;
    msg.child_frame_id = child_frame;
    transformKindrToMsg(T.cast<double>(), &msg.transform);
    transformer.tfBuffer()->setTransform(msg, "test", /*is_static=*/true);
  };

  // Stands in for sensor A, whose cfg.frame is empty (F1: falls back to its
  // cloud's header.frame_id, "from_header_a").
  broadcastStatic("from_header_a", translation(1.0, 0.0, 0.0));
  // Stands in for sensor B, whose cfg.frame is explicitly "explicit_b".
  broadcastStatic("explicit_b", translation(0.0, 2.0, 0.0));

  Transformation T_G_C_a, T_G_C_b;
  ASSERT_TRUE(
      transformer.lookupTransform("from_header_a", "map", stamp, &T_G_C_a));
  ASSERT_TRUE(
      transformer.lookupTransform("explicit_b", "map", stamp, &T_G_C_b));

  EXPECT_NEAR(T_G_C_a.getPosition().x(), 1.0, 1e-6);
  EXPECT_NEAR(T_G_C_a.getPosition().y(), 0.0, 1e-6);
  EXPECT_NEAR(T_G_C_b.getPosition().x(), 0.0, 1e-6);
  EXPECT_NEAR(T_G_C_b.getPosition().y(), 2.0, 1e-6);

  // lookupSensorTransform() (the entry point Phase 5's per-sensor frontend
  // will call) resolves the same way.
  Transformation T_G_C_a2;
  ASSERT_TRUE(transformer.lookupSensorTransform(
      "from_header_a", /*T_B_C_or_null=*/nullptr, stamp, &T_G_C_a2));
  EXPECT_NEAR(T_G_C_a2.getPosition().x(), 1.0, 1e-6);
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
