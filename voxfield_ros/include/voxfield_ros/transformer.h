#ifndef VOXFIELD_ROS_TRANSFORMER_H_
#define VOXFIELD_ROS_TRANSFORMER_H_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <voxfield/core/common.h>

namespace voxfield {

/**
 * Class that binds to either the TF tree or resolves transformations from the
 * ROS parameter server, depending on settings loaded from ROS params.
 */
class Transformer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit Transformer(rclcpp::Node::SharedPtr node);

  // Resolves T_G_C for `from_frame` at `timestamp`, via TF or the queue
  // (queue mode uses the global T_B_C_, so single-sensor callers get
  // exactly today's behavior). Kept working, unchanged signature, so
  // existing single-sensor callers (TsdfServer, NpTsdfServer,
  // IntensityServer) don't need to change (MULTI_SENSOR_PLAN.md M6).
  bool lookupTransform(
      const std::string& from_frame, const std::string& to_frame,
      const rclcpp::Time& timestamp, Transformation* transform);

  // MULTI_SENSOR_PLAN.md M6: queue-mode lookup with a per-call extrinsic,
  // for multi-sensor mode (each sensor has its own T_B_C).
  // T_G_C = T_G_D(stamp) * T_B_D^-1 * T_B_C, where T_G_D is interpolated
  // from the transform queue and T_B_D is the (still global) dynamic-frame
  // extrinsic.
  bool lookupTransformQueue(
      const rclcpp::Time& stamp, const Transformation& T_B_C,
      Transformation* T_G_C);

  // Resolves T_G_C for `frame` at `stamp`, via TF (T_B_C_or_null ignored)
  // or the queue (T_B_C_or_null if non-null, else the global T_B_C_),
  // depending on use_tf_transforms_. The multi-sensor per-sensor frontend
  // (Phase 5) calls this instead of lookupTransform().
  bool lookupSensorTransform(
      const std::string& frame, const Transformation* T_B_C_or_null,
      const rclcpp::Time& stamp, Transformation* T_G_C);

  void transformCallback(
      const geometry_msgs::msg::TransformStamped::SharedPtr transform_msg);

  Transformation getStaticTransform();

  Transformation getModelTransform();

  // Test-only accessor: lets a test inject TF transforms directly with
  // tfBuffer()->setTransform(...), bypassing broadcast timing. Harmless in
  // production (MULTI_SENSOR_PLAN.md M6).
  std::shared_ptr<tf2_ros::Buffer> tfBuffer() {
    return tf_buffer_;
  }

  // Test-only accessor for the queue-mode transform queue's current size,
  // so a test can observe the retention window/size cap in
  // lookupTransformQueue() without a production reason to expose the queue
  // itself. Harmless in production.
  size_t transformQueueSizeForTest() const {
    return transform_queue_.size();
  }

 private:
  bool lookupTransformTf(
      const std::string& from_frame, const std::string& to_frame,
      const rclcpp::Time& timestamp, Transformation* transform);

  rclcpp::Node::SharedPtr node_;

  /**
   * Global/map coordinate frame. Will always look up TF transforms to this
   * frame.
   */
  std::string world_frame_;
  /**
   * Whether to use TF transform resolution (true) or fixed transforms from
   * parameters and transform topics (false).
   */
  bool use_tf_transforms_;
  int64_t timestamp_tolerance_ns_;
  // MULTI_SENSOR_PLAN.md M6: how long a queue-mode transform is kept after
  // it's older than the most recent lookup's timestamp. Replaces the old
  // "erase everything before this lookup's bracket" policy, which broke
  // with >1 sensor (A8): a later-timestamped cloud from one sensor,
  // followed by an earlier-timestamped cloud from another, would find its
  // bracketing poses already erased.
  double transform_queue_retention_sec_;
  /**
   * B is the body frame of the robot, C is the camera/sensor frame creating
   * the pointclouds, and D is the 'dynamic' frame; i.e., incoming messages
   * are assumed to be T_G_D.
   */
  Transformation T_B_C_;
  Transformation T_B_D_;
  Transformation T_C_CH_;
  /**
   * If we use topic transforms, we have 2 parts: a dynamic transform from a
   * topic and a static transform from parameters.
   * Dynamic transform should be T_G_D (where D is whatever sensor the
   * dynamic coordinate frame is in) and the static should be T_D_C (where
   * C is the sensor frame that produces the depth data). It is possible to
   * specify T_C_D and set invert_static_transform to true.
   */

  /**
   * To be replaced (at least optionally) with odometry + static transform
   * from IMU to visual frame.
   */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Only used if use_tf_transforms_ set to false.
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
      transform_sub_;

  // Transform queue, used only when use_tf_transforms is false.
  AlignedDeque<geometry_msgs::msg::TransformStamped> transform_queue_;
};

}  // namespace voxfield

#endif  // VOXFIELD_ROS_TRANSFORMER_H_
