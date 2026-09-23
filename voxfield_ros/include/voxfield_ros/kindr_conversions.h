#ifndef VOXFIELD_ROS_KINDR_CONVERSIONS_H_
#define VOXFIELD_ROS_KINDR_CONVERSIONS_H_

// Replaces the ROS 1 minkindr_conversions package (ROS2_PORT_PLAN.md D4):
// only the ~5 conversions voxfield_ros actually uses
// (transformKindrToMsg, transformMsgToKindr; transformKindrToTF and
// transformTFToKindr collapse into the same two, since ROS 2 has no
// separate tf::Transform type -- geometry_msgs::msg::Transform is used
// everywhere). The flat-16-element-array conversion (formerly
// xmlRpcToKindr) lives in param_utils.h's getTransformationParam(),
// which already needs the same parameter-declaration machinery.

#include <geometry_msgs/msg/transform.hpp>
#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>

namespace voxfield {

template <typename Scalar>
void transformKindrToMsg(
    const kindr::minimal::QuatTransformationTemplate<Scalar>& kindr,
    geometry_msgs::msg::Transform* msg) {
  CHECK_NOTNULL(msg);
  const auto& rotation = kindr.getRotation();
  msg->translation.x = kindr.getPosition().x();
  msg->translation.y = kindr.getPosition().y();
  msg->translation.z = kindr.getPosition().z();
  msg->rotation.w = rotation.w();
  msg->rotation.x = rotation.x();
  msg->rotation.y = rotation.y();
  msg->rotation.z = rotation.z();
}

template <typename Scalar>
void transformMsgToKindr(
    const geometry_msgs::msg::Transform& msg,
    kindr::minimal::QuatTransformationTemplate<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  using Transformation = kindr::minimal::QuatTransformationTemplate<Scalar>;
  const typename Transformation::Position position(
      static_cast<Scalar>(msg.translation.x),
      static_cast<Scalar>(msg.translation.y),
      static_cast<Scalar>(msg.translation.z));
  const typename Transformation::Rotation rotation(
      typename Transformation::Rotation::Implementation(
          static_cast<Scalar>(msg.rotation.w),
          static_cast<Scalar>(msg.rotation.x),
          static_cast<Scalar>(msg.rotation.y),
          static_cast<Scalar>(msg.rotation.z))
          .normalized());
  *kindr = Transformation(position, rotation);
}

}  // namespace voxfield

#endif  // VOXFIELD_ROS_KINDR_CONVERSIONS_H_
