#ifndef VOXFIELD_ROS_VOXFIELD_SERVER_H_
#define VOXFIELD_ROS_VOXFIELD_SERVER_H_

#include <memory>
#include <string>

#include <voxfield/core/esdf_map.h>
#include <voxfield/core/occupancy_map.h>
#include <voxfield/integrator/esdf_voxfield_integrator.h>
#include <voxfield/integrator/occupancy_tsdf_integrator.h>
#include <voxfield_msgs/msg/layer.hpp>

#include "voxfield_ros/np_tsdf_server.h"

namespace voxfield {

class VoxfieldServer : public NpTsdfServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit VoxfieldServer(rclcpp::Node::SharedPtr node);
  VoxfieldServer(
      rclcpp::Node::SharedPtr node, const EsdfMap::Config& esdf_config,
      const EsdfVoxfieldIntegrator::Config& esdf_integrator_config,
      const TsdfMap::Config& tsdf_config,
      const NpTsdfIntegratorBase::Config& tsdf_integrator_config,
      const MeshIntegratorConfig& mesh_config);
  virtual ~VoxfieldServer() {}

  // NOTE(ROS2 port): declared but never defined upstream (its body is
  // commented out in voxfield_server.cc) and never bound to a service in
  // setupRos(); generate_esdf_srv_ is likewise declared but unused. Kept
  // exactly as-is per the port's behavior-preservation rule.
  void generateEsdfCallback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void publishAllUpdatedEsdfVoxels();
  virtual void publishSlices();
  void publishTraversable();
  void publishOccupancyOccupiedNodes();

  virtual void publishPointclouds();
  virtual void newPoseCallback(const Transformation& T_G_C);
  virtual void publishMap(bool reset_remote_map = false);
  virtual bool saveMap(const std::string& file_path);
  virtual bool loadMap(const std::string& file_path);

  void updateEsdfEvent();

  /// Call this to update the ESDF based on latest state of the TSDF map,
  /// considering only the newly updated parts of the TSDF map (checked with
  /// the ESDF updated bit in Update::Status).
  void updateEsdf();
  /// Update the ESDF all at once; clear the existing map.
  void updateEsdfBatch(bool full_euclidean = false);

  // Overwrites the layer with what's coming from the topic!
  void esdfMapCallback(const voxfield_msgs::msg::Layer::SharedPtr layer_msg);

  void saveEsdfMapCallback(
      const std::shared_ptr<voxfield_msgs::srv::FilePath::Request> request,
      std::shared_ptr<voxfield_msgs::srv::FilePath::Response> response);

  inline std::shared_ptr<EsdfMap> getEsdfMapPtr() {
    return esdf_map_;
  }
  inline std::shared_ptr<const EsdfMap> getEsdfMapPtr() const {
    return esdf_map_;
  }

  bool getClearSphere() const {
    return clear_sphere_for_planning_;
  }
  void setClearSphere(bool clear_sphere_for_planning) {
    clear_sphere_for_planning_ = clear_sphere_for_planning;
  }
  float getEsdfMaxDistance() const;
  void setEsdfMaxDistance(float max_distance);
  float getTraversabilityRadius() const;
  void setTraversabilityRadius(float traversability_radius);

  /**
   * These are for enabling or disabling incremental update of the ESDF. Use
   * carefully.
   */
  void disableIncrementalUpdate() {
    incremental_update_ = false;
  }
  void enableIncrementalUpdate() {
    incremental_update_ = true;
  }

  virtual void clear();

  // py: added
  void updateOccFromTsdf();
  void evalEsdfEvent();
  void evalEsdfRefOcc();
  void visualizeEsdfError();

 protected:
  /// Sets up publishing and subscribing. Should only be called from
  /// constructor.
  void setupRos();

  /// Publish markers for visualization.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      esdf_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      esdf_slice_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      traversable_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      esdf_error_slice_pub_;  // py: added

  /// Publish the complete map for other nodes to consume.
  rclcpp::Publisher<voxfield_msgs::msg::Layer>::SharedPtr esdf_map_pub_;

  /// Subscriber to subscribe to another node generating the map.
  rclcpp::Subscription<voxfield_msgs::msg::Layer>::SharedPtr esdf_map_sub_;

  /// Services.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr generate_esdf_srv_;
  rclcpp::Service<voxfield_msgs::srv::FilePath>::SharedPtr save_esdf_map_srv_;

  /// Timers.
  rclcpp::TimerBase::SharedPtr update_esdf_timer_;
  rclcpp::TimerBase::SharedPtr eval_esdf_timer_;

  bool clear_sphere_for_planning_;
  bool publish_esdf_map_;
  bool publish_traversable_;
  float traversability_radius_;
  bool incremental_update_;
  int num_subscribers_esdf_map_;

  bool esdf_ready_;

  // default: not update according to the counter
  int update_esdf_every_n_ = 0;

  // ESDF maps.
  std::shared_ptr<EsdfMap> esdf_map_;
  std::unique_ptr<EsdfVoxfieldIntegrator> esdf_integrator_;

  // Occupancy maps.
  std::shared_ptr<OccupancyMap> occupancy_map_;
  std::unique_ptr<OccTsdfIntegrator> occupancy_integrator_;
};

}  // namespace voxfield

#endif  // VOXFIELD_ROS_VOXFIELD_SERVER_H_
