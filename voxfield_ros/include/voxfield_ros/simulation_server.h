#ifndef VOXFIELD_ROS_SIMULATION_SERVER_H_
#define VOXFIELD_ROS_SIMULATION_SERVER_H_

#include <memory>
#include <string>

#include <ros/ros.h>

${1}voxfield/core/esdf_map.h>
${1}voxfield/core/tsdf_map.h>
${1}voxfield/integrator/esdf_integrator.h>
${1}voxfield/integrator/esdf_occ_integrator.h>
${1}voxfield/integrator/occupancy_integrator.h>
${1}voxfield/integrator/tsdf_integrator.h>
${1}voxfield/mesh/mesh_integrator.h>
${1}voxfield/simulation/simulation_world.h>

#include "voxfield_ros/conversions.h"
#include "voxfield_ros/mesh_vis.h"
#include "voxfield_ros/ptcloud_vis.h"
#include "voxfield_ros/ros_params.h"

namespace voxfield {

class SimulationServer {
 public:
  SimulationServer(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  SimulationServer(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      const EsdfMap::Config& esdf_config,
      const EsdfIntegrator::Config& esdf_integrator_config,
      const TsdfMap::Config& tsdf_config,
      const TsdfIntegratorBase::Config& tsdf_integrator_config);

  virtual ~SimulationServer() {}

  /// Runs all of the below functions in the correct order:
  void run();

  /// Creates a new world, and prepares ground truth SDF(s).
  virtual void prepareWorld() = 0;

  /// Generates a SDF by generating random poses and putting them into an SDF.
  void generateSDF();

  /// Evaluate errors...
  void evaluate();

  /// Visualize results. :)
  void visualize();

 protected:
  void getServerConfigFromRosParam(const ros::NodeHandle& nh_private);

  /// Convenience function to generate valid viewpoints.
  bool generatePlausibleViewpoint(
      FloatingPoint min_distance, Point* ray_origin,
      Point* ray_direction) const;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // A bunch of publishers :)
  ros::Publisher sim_pub_;
  ros::Publisher tsdf_gt_pub_;
  ros::Publisher esdf_gt_pub_;
  ros::Publisher tsdf_gt_mesh_pub_;
  ros::Publisher tsdf_test_pub_;
  ros::Publisher esdf_test_pub_;
  ros::Publisher tsdf_test_mesh_pub_;
  ros::Publisher view_ptcloud_pub_;

  // Settings
  FloatingPoint voxel_size_;
  int voxels_per_side_;
  std::string world_frame_;
  bool generate_occupancy_;
  bool visualize_;
  FloatingPoint visualization_slice_level_;
  bool generate_mesh_;
  bool incremental_;
  bool add_robot_pose_;
  FloatingPoint truncation_distance_;
  FloatingPoint esdf_max_distance_;
  size_t max_attempts_to_generate_viewpoint_;

  // Camera settings
  Eigen::Vector2i depth_camera_resolution_;
  FloatingPoint fov_h_rad_;
  FloatingPoint max_dist_;
  FloatingPoint min_dist_;
  int num_viewpoints_;

  // Actual simulation server.
  std::unique_ptr<SimulationWorld> world_;

  // Maps (GT and generates from sensors) generated here.
  std::unique_ptr<Layer<TsdfVoxel> > tsdf_gt_;
  std::unique_ptr<Layer<EsdfVoxel> > esdf_gt_;

  // Generated maps:
  std::unique_ptr<Layer<TsdfVoxel> > tsdf_test_;
  std::unique_ptr<Layer<EsdfVoxel> > esdf_test_;
  std::unique_ptr<Layer<OccupancyVoxel> > occ_test_;

  // Integrators:
  std::unique_ptr<TsdfIntegratorBase> tsdf_integrator_;
  std::unique_ptr<EsdfIntegrator> esdf_integrator_;
  std::unique_ptr<OccupancyIntegrator> occ_integrator_;
  std::unique_ptr<EsdfOccIntegrator> esdf_occ_integrator_;
};

}  // namespace voxfield

#endif  // VOXFIELD_ROS_SIMULATION_SERVER_H_
