#ifndef VOXFIELD_ROS_INTENSITY_SERVER_H_
#define VOXFIELD_ROS_INTENSITY_SERVER_H_

#include <memory>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <voxfield/core/voxel.h>
#include <voxfield/integrator/intensity_integrator.h>
#include <voxfield/utils/color_maps.h>

#include "voxfield_ros/intensity_vis.h"
#include "voxfield_ros/tsdf_server.h"

namespace voxfield {

class IntensityServer : public TsdfServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit IntensityServer(rclcpp::Node::SharedPtr node);
  virtual ~IntensityServer() {}

  virtual void updateMesh();
  virtual void publishPointclouds();

  void intensityImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr image);

 protected:
  /// Subscriber for intensity images.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr intensity_image_sub_;

  // Publish markers for visualization.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      intensity_pointcloud_pub_;
  rclcpp::Publisher<voxfield_msgs::msg::Mesh>::SharedPtr intensity_mesh_pub_;

  /// Parameters of the incoming UNDISTORTED intensity images.
  double focal_length_px_;

  /** How much to subsample the image by (not proper downsampling, just
   * subsampling).
   */
  int subsample_factor_;

  // Intensity layer, integrator, and color maps, all related to storing
  // and visualizing intensity data.
  std::shared_ptr<Layer<IntensityVoxel>> intensity_layer_;
  std::unique_ptr<IntensityIntegrator> intensity_integrator_;

  // Visualization tools.
  std::shared_ptr<ColorMap> color_map_;
};

}  // namespace voxfield

#endif  // VOXFIELD_ROS_INTENSITY_SERVER_H_
