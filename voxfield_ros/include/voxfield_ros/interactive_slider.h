#ifndef VOXFIELD_ROS_INTERACTIVE_SLIDER_H_
#define VOXFIELD_ROS_INTERACTIVE_SLIDER_H_

#include <functional>
#include <string>

#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

#include <voxfield/core/common.h>

namespace voxfield {

/// InteractiveSlider class which can be used for visualizing voxel map slices.
class InteractiveSlider {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  InteractiveSlider(
      rclcpp::Node::SharedPtr node, const std::string& slider_name,
      const std::function<void(const double& slice_level)>& slider_callback,
      const Point& initial_position, const unsigned int free_plane_index,
      const float marker_scale_meters);
  virtual ~InteractiveSlider() {}

 private:
  const unsigned int free_plane_index_;
  interactive_markers::InteractiveMarkerServer interactive_marker_server_;

  /// Processes the feedback after moving the slider.
  virtual void interactiveMarkerFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&
          feedback,
      const std::function<void(const double slice_level)>& slider_callback);
};

}  // namespace voxfield

#endif  // VOXFIELD_ROS_INTERACTIVE_SLIDER_H_
