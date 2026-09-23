#ifndef VOXFIELD_ROS_TSDF_SERVER_H_
#define VOXFIELD_ROS_TSDF_SERVER_H_

#include <memory>
#include <queue>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <voxfield/alignment/icp.h>
#include <voxfield/core/tsdf_map.h>
#include <voxfield/integrator/tsdf_integrator.h>
#include <voxfield/io/layer_io.h>
#include <voxfield/io/mesh_ply.h>
#include <voxfield/mesh/mesh_integrator.h>
#include <voxfield/utils/color_maps.h>
#include <voxfield_msgs/msg/layer.hpp>
#include <voxfield_msgs/msg/mesh.hpp>
#include <voxfield_msgs/srv/file_path.hpp>

#include "voxfield_ros/mesh_vis.h"
#include "voxfield_ros/ptcloud_vis.h"
#include "voxfield_ros/transformer.h"

namespace voxfield {

constexpr float kDefaultMaxIntensity = 100.0;

class TsdfServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit TsdfServer(rclcpp::Node::SharedPtr node);
  TsdfServer(
      rclcpp::Node::SharedPtr node, const TsdfMap::Config& config,
      const TsdfIntegratorBase::Config& integrator_config,
      const MeshIntegratorConfig& mesh_config);
  virtual ~TsdfServer() {}

  void getServerConfigFromRosParam();

  void insertPointcloud(
      sensor_msgs::msg::PointCloud2::SharedPtr pointcloud);

  void insertFreespacePointcloud(
      sensor_msgs::msg::PointCloud2::SharedPtr pointcloud);

  virtual void processPointCloudMessageAndInsert(
      sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg,
      const Transformation& T_G_C, const bool is_freespace_pointcloud);

  void integratePointcloud(
      const Transformation& T_G_C, const Pointcloud& ptcloud_C,
      const Colors& colors, const bool is_freespace_pointcloud = false);
  virtual void newPoseCallback(const Transformation& /*new_pose*/) {
    // Do nothing.
  }

  void publishAllUpdatedTsdfVoxels();
  void publishTsdfSurfacePoints();
  void publishTsdfOccupiedNodes();

  virtual void publishSlices();
  /// Incremental update.
  virtual void updateMesh();
  /// Batch update.
  virtual bool generateMesh();
  // Publishes all available pointclouds.
  virtual void publishPointclouds();
  // Publishes the complete map
  virtual void publishMap(bool reset_remote_map = false);
  virtual bool saveMap(const std::string& file_path);
  virtual bool loadMap(const std::string& file_path);

  void clearMapCallback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void saveMapCallback(
      const std::shared_ptr<voxfield_msgs::srv::FilePath::Request> request,
      std::shared_ptr<voxfield_msgs::srv::FilePath::Response> response);
  void loadMapCallback(
      const std::shared_ptr<voxfield_msgs::srv::FilePath::Request> request,
      std::shared_ptr<voxfield_msgs::srv::FilePath::Response> response);
  void generateMeshCallback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void publishPointcloudsCallback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void publishTsdfMapCallback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void updateMeshEvent();
  void publishMapEvent();

  std::shared_ptr<TsdfMap> getTsdfMapPtr() {
    return tsdf_map_;
  }
  std::shared_ptr<const TsdfMap> getTsdfMapPtr() const {
    return tsdf_map_;
  }

  /// Accessors for setting and getting parameters.
  double getSliceLevel() const {
    return slice_level_;
  }
  void setSliceLevel(double slice_level) {
    slice_level_ = slice_level;
  }

  bool setPublishSlices() const {
    return publish_slices_;
  }
  void setPublishSlices(const bool publish_slices) {
    publish_slices_ = publish_slices;
  }

  void setWorldFrame(const std::string& world_frame) {
    world_frame_ = world_frame;
  }
  std::string getWorldFrame() const {
    return world_frame_;
  }

  /// CLEARS THE ENTIRE MAP!
  virtual void clear();

  // Visualize the robot model in the map
  void publishRobotMesh(const Transformation& T_G_C);

  /// Overwrites the layer with what's coming from the topic!
  void tsdfMapCallback(const voxfield_msgs::msg::Layer::SharedPtr layer_msg);

 protected:
  /**
   * Gets the next pointcloud that has an available transform to process from
   * the queue.
   */
  bool getNextPointcloudFromQueue(
      std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>* queue,
      sensor_msgs::msg::PointCloud2::SharedPtr* pointcloud_msg,
      Transformation* T_G_C);

  rclcpp::Node::SharedPtr node_;

  /// Data subscribers.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      freespace_pointcloud_sub_;

  /// Publish markers for visualization.
  rclcpp::Publisher<voxfield_msgs::msg::Mesh>::SharedPtr mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      tsdf_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      surface_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      tsdf_slice_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      occupancy_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr
      icp_transform_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      robot_model_pub_;

  /// Publish the complete map for other nodes to consume.
  rclcpp::Publisher<voxfield_msgs::msg::Layer>::SharedPtr tsdf_map_pub_;

  /// Subscriber to subscribe to another node generating the map.
  rclcpp::Subscription<voxfield_msgs::msg::Layer>::SharedPtr tsdf_map_sub_;

  // Services.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr generate_mesh_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_map_srv_;
  rclcpp::Service<voxfield_msgs::srv::FilePath>::SharedPtr save_map_srv_;
  rclcpp::Service<voxfield_msgs::srv::FilePath>::SharedPtr load_map_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr publish_pointclouds_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr publish_tsdf_map_srv_;

  /// Tools for broadcasting TFs.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timers.
  rclcpp::TimerBase::SharedPtr update_mesh_timer_;
  rclcpp::TimerBase::SharedPtr publish_map_timer_;

  // output detailed log or not
  bool verbose_;
  // output timing record or not
  // NOTE(ROS2 port): upstream ROS 1 read this parameter via
  // `nh_private.param("timing", timing_, timing_)` without ever
  // initializing timing_ first, i.e. it used an indeterminate bool as its
  // own default (undefined behavior). Initialized to false here to remove
  // the UB while keeping the same effective default. See
  // docs/ROS2_PORT_NOTES.md.
  bool timing_ = false;

  /**
   * Global/map coordinate frame. Will always look up TF transforms to this
   * frame.
   */
  std::string world_frame_;
  std::string sensor_frame_;

  // Robot model related
  std::string robot_model_file_;
  float robot_model_scale_ = 1.0;

  // Mesh reconstruction interval counter
  int update_mesh_every_n_ = 0;

  /**
   * Name of the ICP corrected frame. Publishes TF and transform topic to this
   * if ICP on.
   */
  std::string icp_corrected_frame_;
  /// Name of the pose in the ICP correct Frame.
  std::string pose_corrected_frame_;

  /// Delete blocks that are far from the system to help manage memory
  double max_block_distance_from_body_;

  /// Pointcloud visualization settings.
  double slice_level_;

  /// If the system should subscribe to a pointcloud giving points in freespace
  bool use_freespace_pointcloud_;

  /**
   * Mesh output settings. Mesh is only written to file if mesh_filename_ is
   * not empty.
   */
  std::string mesh_filename_;
  /// How to color the mesh.
  ColorMode color_mode_;

  /// Colormap to use for intensity pointclouds.
  std::shared_ptr<ColorMap> color_map_;

  /// Will throttle to this message rate.
  rclcpp::Duration min_time_between_msgs_ = rclcpp::Duration(0, 0);

  /// What output information to publish
  bool publish_pointclouds_on_update_;
  bool publish_slices_;
  bool publish_pointclouds_;
  bool publish_tsdf_map_;
  // NOTE(ROS2 port): same uninitialized-default issue as timing_ above.
  bool publish_robot_model_ = false;

  /// Whether to save the latest mesh message sent (for inheriting classes).
  bool cache_mesh_;

  /**
   *Whether to enable ICP corrections. Every pointcloud coming in will attempt
   * to be matched up to the existing structure using ICP. Requires the initial
   * guess from odometry to already be very good.
   */
  bool enable_icp_;
  /**
   * If using ICP corrections, whether to store accumulate the corrected
   * transform. If this is set to false, the transform will reset every
   * iteration.
   */
  bool accumulate_icp_corrections_;

  /// Subscriber settings.
  int pointcloud_queue_size_;
  int num_subscribers_tsdf_map_;

  // Maps and integrators.
  std::shared_ptr<TsdfMap> tsdf_map_;
  std::unique_ptr<TsdfIntegratorBase> tsdf_integrator_;

  /// ICP matcher
  std::shared_ptr<ICP> icp_;

  // Mesh accessories.
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::unique_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator_;
  /// Optionally cached mesh message.
  voxfield_msgs::msg::Mesh cached_mesh_msg_;

  /**
   * Transformer object to keep track of either TF transforms or messages from
   * a transform topic.
   */
  Transformer transformer_;
  /**
   * Queue of incoming pointclouds, in case the transforms can't be immediately
   * resolved.
   */
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointcloud_queue_;
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>
      freespace_pointcloud_queue_;

  // Last message times for throttling input.
  rclcpp::Time last_msg_time_ptcloud_;
  rclcpp::Time last_msg_time_freespace_ptcloud_;

  /// Current transform corrections from ICP.
  Transformation icp_corrected_transform_;

  size_t frame_count_ = 0;
};

}  // namespace voxfield

#endif  // VOXFIELD_ROS_TSDF_SERVER_H_
