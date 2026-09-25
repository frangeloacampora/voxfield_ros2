#include "voxfield_ros/transformer.h"

#include <algorithm>

#include "voxfield_ros/kindr_conversions.h"
#include "voxfield_ros/param_utils.h"

namespace voxfield {
// TODO(py): change the name of these transformation, current name
// is hard to understand
Transformer::Transformer(rclcpp::Node::SharedPtr node)
    : node_(node),
      world_frame_("world"),
      use_tf_transforms_(true),
      timestamp_tolerance_ns_(1000000),
      transform_queue_retention_sec_(1.0) {
  param(*node_, "world_frame", world_frame_);

  const double kNanoSecondsInSecond = 1.0e9;
  double timestamp_tolerance_sec =
      timestamp_tolerance_ns_ / kNanoSecondsInSecond;
  param(*node_, "timestamp_tolerance_sec", timestamp_tolerance_sec);
  timestamp_tolerance_ns_ =
      static_cast<int64_t>(timestamp_tolerance_sec * kNanoSecondsInSecond);

  // MULTI_SENSOR_PLAN.md M6.
  param(
      *node_, "transform_queue_retention_sec", transform_queue_retention_sec_);

  // Transform settings.
  param(*node_, "use_tf_transforms", use_tf_transforms_);
  // If we use topic transforms, we have 2 parts: a dynamic transform from a
  // topic and a static transform from parameters (calibration).
  // Dynamic transform should be T_G_D (where D is whatever sensor the
  // dynamic coordinate frame is in) and the static should be T_D_C (where
  // C is the sensor frame that produces the depth data). It is possible to
  // specify T_C_D and set invert_static_tranform to true.
  // ROS 1 constructed tf::TransformListener unconditionally as a plain
  // member; keep that behavior identical here rather than making it
  // conditional on use_tf_transforms_.
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!use_tf_transforms_) {
    transform_sub_ =
        node_->create_subscription<geometry_msgs::msg::TransformStamped>(
            "transform", rclcpp::QoS(40),
            std::bind(
                &Transformer::transformCallback, this, std::placeholders::_1));
    // Retrieve T_D_C from params.
    getTransformationParam(*node_, "T_B_D", "invert_T_B_D", &T_B_D_);
    getTransformationParam(*node_, "T_B_C", "invert_T_B_C", &T_B_C_);
  }
  // Or we will use tf_transform, we do not need the calibration parameters
  // lookupTransformTf
  // Model transformation
  getTransformationParam(*node_, "T_C_CH", "invert_T_C_CH", &T_C_CH_);
}

void Transformer::transformCallback(
    const geometry_msgs::msg::TransformStamped::SharedPtr transform_msg) {
  transform_queue_.push_back(*transform_msg);
}

Transformation Transformer::getStaticTransform() {
  return T_B_C_;
}

Transformation Transformer::getModelTransform() {
  return T_C_CH_;
}

bool Transformer::lookupTransform(
    const std::string& from_frame, const std::string& to_frame,
    const rclcpp::Time& timestamp, Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (use_tf_transforms_) {
    return lookupTransformTf(from_frame, to_frame, timestamp, transform);
  } else {
    // Single-sensor callers get exactly today's behavior: the global
    // T_B_C_ (MULTI_SENSOR_PLAN.md M6).
    return lookupTransformQueue(timestamp, T_B_C_, transform);
  }
}

bool Transformer::lookupSensorTransform(
    const std::string& frame, const Transformation* T_B_C_or_null,
    const rclcpp::Time& stamp, Transformation* T_G_C) {
  CHECK_NOTNULL(T_G_C);
  if (use_tf_transforms_) {
    return lookupTransformTf(frame, world_frame_, stamp, T_G_C);
  }
  const Transformation& T_B_C =
      T_B_C_or_null != nullptr ? *T_B_C_or_null : T_B_C_;
  return lookupTransformQueue(stamp, T_B_C, T_G_C);
}

// Stolen from octomap_manager
bool Transformer::lookupTransformTf(
    const std::string& from_frame, const std::string& to_frame,
    const rclcpp::Time& timestamp, Transformation* transform) {
  CHECK_NOTNULL(transform);

  // Previous behavior was just to use the latest transform if the time is in
  // the future. Now we will just wait.
  if (!tf_buffer_->canTransform(to_frame, from_frame, timestamp)) {
    return false;
  }

  geometry_msgs::msg::TransformStamped tf_transform;
  try {
    tf_transform = tf_buffer_->lookupTransform(to_frame, from_frame, timestamp);
  } catch (tf2::TransformException& ex) {  // NOLINT
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Error getting TF transform from sensor data: " << ex.what());
    return false;
  }

  transformMsgToKindr(tf_transform.transform, transform);
  return true;
}

bool Transformer::lookupTransformQueue(
    const rclcpp::Time& timestamp, const Transformation& T_B_C,
    Transformation* T_G_C) {
  CHECK_NOTNULL(T_G_C);
  if (transform_queue_.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 30000,
        "No match found for transform timestamp: "
            << timestamp.nanoseconds() << " as transform queue is empty.");
    return false;
  }
  // Try to match the transforms in the queue.
  bool match_found = false;
  std::deque<geometry_msgs::msg::TransformStamped>::iterator it =
      transform_queue_.begin();
  for (; it != transform_queue_.end(); ++it) {
    const rclcpp::Time it_stamp(it->header.stamp, RCL_ROS_TIME);
    // If the current transform is newer than the requested timestamp, we need
    // to break.
    if (it_stamp > timestamp) {
      if ((it_stamp - timestamp).nanoseconds() < timestamp_tolerance_ns_) {
        match_found = true;
      }
      break;
    }

    if ((timestamp - it_stamp).nanoseconds() < timestamp_tolerance_ns_) {
      match_found = true;
      break;
    }
  }

  // Match found basically means an exact match.
  Transformation T_G_D;
  if (match_found) {
    transformMsgToKindr(it->transform, &T_G_D);
  } else {
    // If we think we have an inexact match, have to check that we're still
    // within bounds and interpolate.
    if (it == transform_queue_.begin() || it == transform_queue_.end()) {
      RCLCPP_WARN_STREAM_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 30000,
          "No match found for transform timestamp: "
              << timestamp.nanoseconds() << " Queue front: "
              << rclcpp::Time(
                     transform_queue_.front().header.stamp, RCL_ROS_TIME)
                     .nanoseconds()
              << " back: "
              << rclcpp::Time(
                     transform_queue_.back().header.stamp, RCL_ROS_TIME)
                     .nanoseconds());
      return false;
    }
    // Newest should be 1 past the requested timestamp, oldest should be one
    // before the requested timestamp.
    Transformation T_G_D_newest;
    transformMsgToKindr(it->transform, &T_G_D_newest);
    const rclcpp::Time newest_stamp(it->header.stamp, RCL_ROS_TIME);
    int64_t offset_newest_ns = (newest_stamp - timestamp).nanoseconds();
    // We already checked that this is not the beginning.
    it--;
    Transformation T_G_D_oldest;
    transformMsgToKindr(it->transform, &T_G_D_oldest);
    const rclcpp::Time oldest_stamp(it->header.stamp, RCL_ROS_TIME);
    int64_t offset_oldest_ns = (timestamp - oldest_stamp).nanoseconds();

    // Interpolate between the two transformations using the exponential map.
    FloatingPoint t_diff_ratio =
        static_cast<FloatingPoint>(offset_oldest_ns) /
        static_cast<FloatingPoint>(offset_newest_ns + offset_oldest_ns);

    Transformation::Vector6 diff_vector =
        (T_G_D_oldest.inverse() * T_G_D_newest).log();
    T_G_D = T_G_D_oldest * Transformation::exp(t_diff_ratio * diff_vector);
  }

  // If we have a static transform, apply it too.
  // Transform should actually be T_G_C. So need to take it through the full
  // chain. transform is T_G_C. T_B_D stays global; T_B_C is per call
  // (MULTI_SENSOR_PLAN.md M6).
  const Transformation T_D_C = T_B_D_.inverse() * T_B_C;
  *T_G_C = T_G_D * T_D_C;

  // MULTI_SENSOR_PLAN.md A8/M6: erase entries older than the retention
  // window, not "everything before this lookup's bracket" -- with >1
  // sensor, a cloud stamped slightly earlier than one already processed
  // would otherwise find its bracketing poses already erased. Single-sensor
  // output is unaffected except when two consecutive clouds are stamped
  // closer together than timestamp_tolerance_sec (1 ms default): see
  // docs/MULTI_SENSOR_NOTES.md.
  const int64_t retention_ns =
      static_cast<int64_t>(transform_queue_retention_sec_ * 1.0e9);
  const int64_t cutoff_ns = timestamp.nanoseconds() - retention_ns;
  while (!transform_queue_.empty()) {
    const rclcpp::Time front_stamp(
        transform_queue_.front().header.stamp, RCL_ROS_TIME);
    if (front_stamp.nanoseconds() >= cutoff_ns) {
      break;
    }
    transform_queue_.pop_front();
  }
  constexpr size_t kMaxQueueSize = 10000;
  while (transform_queue_.size() > kMaxQueueSize) {
    transform_queue_.pop_front();
  }
  return true;
}

}  // namespace voxfield
