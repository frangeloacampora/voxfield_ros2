#include <cmath>
#include <gtest/gtest.h>

#include "voxfield_ros/kindr_conversions.h"

namespace voxfield {
namespace {

using Transformation = kindr::minimal::QuatTransformationTemplate<double>;

TEST(KindrConversions, RoundTripMsgToKindrToMsg) {
  geometry_msgs::msg::Transform msg;
  msg.translation.x = 1.0;
  msg.translation.y = -2.5;
  msg.translation.z = 3.25;
  // A valid, already-normalized quaternion (90 deg about Z).
  msg.rotation.w = std::sqrt(0.5);
  msg.rotation.x = 0.0;
  msg.rotation.y = 0.0;
  msg.rotation.z = std::sqrt(0.5);

  Transformation transform;
  transformMsgToKindr(msg, &transform);

  geometry_msgs::msg::Transform round_tripped;
  transformKindrToMsg(transform, &round_tripped);

  EXPECT_NEAR(round_tripped.translation.x, msg.translation.x, 1e-9);
  EXPECT_NEAR(round_tripped.translation.y, msg.translation.y, 1e-9);
  EXPECT_NEAR(round_tripped.translation.z, msg.translation.z, 1e-9);
  EXPECT_NEAR(round_tripped.rotation.w, msg.rotation.w, 1e-9);
  EXPECT_NEAR(round_tripped.rotation.x, msg.rotation.x, 1e-9);
  EXPECT_NEAR(round_tripped.rotation.y, msg.rotation.y, 1e-9);
  EXPECT_NEAR(round_tripped.rotation.z, msg.rotation.z, 1e-9);
}

TEST(KindrConversions, IdentityRoundTrip) {
  geometry_msgs::msg::Transform msg;
  msg.rotation.w = 1.0;

  Transformation transform;
  transformMsgToKindr(msg, &transform);

  EXPECT_TRUE(transform.getPosition().isZero());
  EXPECT_NEAR(transform.getRotation().w(), 1.0, 1e-9);
}

TEST(KindrConversions, NonNormalizedQuaternionIsRenormalized) {
  // Deliberately not unit-norm; transformMsgToKindr must renormalize rather
  // than propagate an invalid rotation.
  geometry_msgs::msg::Transform msg;
  msg.rotation.w = 2.0;
  msg.rotation.x = 0.0;
  msg.rotation.y = 0.0;
  msg.rotation.z = 0.0;

  Transformation transform;
  transformMsgToKindr(msg, &transform);

  const auto& q = transform.getRotation();
  const double norm_sq =
      q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z();
  EXPECT_NEAR(norm_sq, 1.0, 1e-9);
  EXPECT_NEAR(q.w(), 1.0, 1e-9);
}

}  // namespace
}  // namespace voxfield

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
