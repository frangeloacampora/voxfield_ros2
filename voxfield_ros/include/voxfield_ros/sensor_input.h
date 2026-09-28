#ifndef VOXFIELD_ROS_SENSOR_INPUT_H_
#define VOXFIELD_ROS_SENSOR_INPUT_H_

#include <cstddef>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <voxfield/core/common.h>

#include "voxfield_ros/range_image_projector.h"

namespace voxfield {

// Per-sensor configuration (MULTI_SENSOR_PLAN.md M4). In legacy
// (single-sensor) mode, exactly one SensorConfig named "default" is built
// from the existing top-level params. In multi-sensor mode, one is built
// per entry in sensor_names, from sensors.<name>.* overrides on top of the
// top-level params (sensor_config_loader.h).
struct SensorConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::string name;                        // "default" in legacy mode
  std::string topic;                       // legacy: "pointcloud"
  std::string freespace_topic;             // "" = none
  std::string frame;                       // "" = use header.frame_id (M5)
  int queue_size = 1;                      // inherits pointcloud_queue_size
  bool input_qos_best_effort = false;      // inherits
  double min_time_between_msgs_sec = 0.0;  // inherits
  // Queue mode only (M6): if false, the sensor inherits Transformer's
  // global T_B_C_; Transformer::lookupSensorTransform() implements this
  // fallback (T_B_C_or_null == nullptr).
  bool has_T_B_C = false;
  Transformation T_B_C;
};

// The per-sensor frontend: owns its own subscription(s), queue(s), and
// integrator/projector instance(s), all integrating into the shared
// TsdfMap layer (M3). Servers hold these as
// std::vector<std::unique_ptr<SensorInput<...>>>; sensors_[0] is the
// primary sensor (M10/M14). Kept as unique_ptr rather than by value:
// subscription lambdas capture the raw pointer, which must stay stable
// (M4) -- the vector itself must never be resized after subscriptions
// exist.
template <typename IntegratorBaseT>
struct SensorInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SensorConfig config;
  std::unique_ptr<IntegratorBaseT> integrator;
  // NpTsdfServer only; stays nullptr for TsdfServer's SensorInput.
  std::unique_ptr<RangeImageProjector> projector;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr freespace_sub;
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> queue;
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> freespace_queue;
  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_freespace_msg_time{0, 0, RCL_ROS_TIME};

  // Stats (logged when verbose_, and in the dropped-cloud warning).
  size_t num_received = 0;
  size_t num_throttled = 0;
  size_t num_dropped = 0;
  size_t num_integrated = 0;
};

}  // namespace voxfield

#endif  // VOXFIELD_ROS_SENSOR_INPUT_H_
