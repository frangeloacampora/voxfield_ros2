#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <voxfield/core/tsdf_map.h>
#include <voxfield/io/layer_io.h>
#include <voxfield/io/mesh_ply.h>
#include <voxfield/mesh/mesh_integrator.h>

#include "voxfield_ros/mesh_pcl.h"
#include "voxfield_ros/mesh_vis.h"
#include "voxfield_ros/node_main.h"
#include "voxfield_ros/param_utils.h"
#include "voxfield_ros/ptcloud_vis.h"

namespace voxfield {
class SimpleTsdfVisualizer {
 public:
  explicit SimpleTsdfVisualizer(rclcpp::Node::SharedPtr node)
      : node_(node),
        tsdf_surface_distance_threshold_factor_(2.0),
        tsdf_world_frame_("world"),
        tsdf_mesh_color_mode_(ColorMode::kColor),
        tsdf_voxel_ply_output_path_("") {
    RCLCPP_DEBUG(node_->get_logger(), "\tSetting up ROS publishers...");

    const rclcpp::QoS kLatchedQos =
        rclcpp::QoS(1).transient_local().reliable();
    surface_pointcloud_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/tsdf_voxels_near_surface", kLatchedQos);

    tsdf_pointcloud_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/all_tsdf_voxels", kLatchedQos);

    mesh_pub_ = node_->create_publisher<voxfield_msgs::msg::Mesh>(
        "~/mesh", kLatchedQos);

    mesh_pointcloud_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/mesh_as_pointcloud", kLatchedQos);

    mesh_pcl_mesh_pub_ = node_->create_publisher<voxfield_msgs::msg::Mesh>(
        "~/mesh_pcl", kLatchedQos);

    RCLCPP_DEBUG(node_->get_logger(), "\tRetreiving ROS parameters...");

    param(
        *node_, "tsdf_surface_distance_threshold_factor",
        tsdf_surface_distance_threshold_factor_);
    param(*node_, "tsdf_world_frame", tsdf_world_frame_);
    param(
        *node_, "tsdf_voxel_ply_output_path", tsdf_voxel_ply_output_path_);
    param(*node_, "tsdf_mesh_output_path", tsdf_mesh_output_path_);

    std::string color_mode = "color";
    param(*node_, "tsdf_mesh_color_mode", color_mode);
    if (color_mode == "color") {
      tsdf_mesh_color_mode_ = ColorMode::kColor;
    } else if (color_mode == "height") {
      tsdf_mesh_color_mode_ = ColorMode::kHeight;
    } else if (color_mode == "normals") {
      tsdf_mesh_color_mode_ = ColorMode::kNormals;
    } else if (color_mode == "lambert") {
      tsdf_mesh_color_mode_ = ColorMode::kLambert;
    } else if (color_mode == "gray") {
      tsdf_mesh_color_mode_ = ColorMode::kGray;
    } else {
      RCLCPP_FATAL_STREAM(
          node_->get_logger(), "Undefined mesh coloring mode: " << color_mode);
      rclcpp::shutdown();
    }

    rclcpp::spin_some(node_);
  }

  void run(const Layer<TsdfVoxel>& tsdf_layer);

 private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      surface_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      tsdf_pointcloud_pub_;
  rclcpp::Publisher<voxfield_msgs::msg::Mesh>::SharedPtr mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      mesh_pointcloud_pub_;
  // NOTE(ROS2 port): upstream published pcl_msgs/PolygonMesh here, but the
  // publish call actually sent a voxfield_msgs::Mesh (mesh_msg), not a
  // pcl_msgs::PolygonMesh -- a pre-existing upstream bug (the constructed
  // pcl_mesh_msg is unused). Kept as-is; the publisher's declared type is
  // adjusted to match what's actually published so this still compiles
  // under ROS 2's static typed publishers. See docs/ROS2_PORT_NOTES.md.
  rclcpp::Publisher<voxfield_msgs::msg::Mesh>::SharedPtr mesh_pcl_mesh_pub_;

  // Settings
  double tsdf_surface_distance_threshold_factor_;
  std::string tsdf_world_frame_;
  ColorMode tsdf_mesh_color_mode_;
  std::string tsdf_voxel_ply_output_path_;
  std::string tsdf_mesh_output_path_;
};

void SimpleTsdfVisualizer::run(const Layer<TsdfVoxel>& tsdf_layer) {
  RCLCPP_INFO_STREAM(
      node_->get_logger(),
      "\nTSDF Layer info:\n"
      << "\tVoxel size:\t\t " << tsdf_layer.voxel_size() << "\n"
      << "\t# Voxels per side:\t " << tsdf_layer.voxels_per_side() << "\n"
      << "\tMemory size:\t\t " << tsdf_layer.getMemorySize() / 1024 / 1024
      << "MB\n"
      << "\t# Allocated blocks:\t " << tsdf_layer.getNumberOfAllocatedBlocks()
      << "\n");

  RCLCPP_DEBUG(node_->get_logger(), "\tVisualize voxels near surface...");
  {
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    const FloatingPoint surface_distance_thresh_m =
        tsdf_layer.voxel_size() * tsdf_surface_distance_threshold_factor_;
    voxfield::createSurfaceDistancePointcloudFromTsdfLayer(
        tsdf_layer, surface_distance_thresh_m, &pointcloud);

    publishPclCloud(
        surface_pointcloud_pub_, pointcloud, tsdf_world_frame_, node_->now());
  }

  RCLCPP_DEBUG(node_->get_logger(), "\tVisualize all voxels...");
  {
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    voxfield::createDistancePointcloudFromTsdfLayer(tsdf_layer, &pointcloud);

    publishPclCloud(
        tsdf_pointcloud_pub_, pointcloud, tsdf_world_frame_, node_->now());

    if (!tsdf_voxel_ply_output_path_.empty()) {
      pcl::PLYWriter writer;
      constexpr bool kUseBinary = true;
      writer.write(tsdf_voxel_ply_output_path_, pointcloud, kUseBinary);
    }
  }

  RCLCPP_DEBUG(node_->get_logger(), "\tVisualize mesh...");
  {
    std::shared_ptr<MeshLayer> mesh_layer;
    mesh_layer.reset(new MeshLayer(tsdf_layer.block_size()));
    MeshIntegratorConfig mesh_config;
    std::shared_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator;
    mesh_integrator.reset(new MeshIntegrator<TsdfVoxel>(
        mesh_config, tsdf_layer, mesh_layer.get()));

    constexpr bool kOnlyMeshUpdatedBlocks = false;
    constexpr bool kClearUpdatedFlag = false;
    mesh_integrator->generateMesh(kOnlyMeshUpdatedBlocks, kClearUpdatedFlag);

    // Output as native voxblox mesh.
    voxfield_msgs::msg::Mesh mesh_msg;
    generateVoxbloxMeshMsg(
        mesh_layer, tsdf_mesh_color_mode_, &mesh_msg, node_->now());
    mesh_msg.header.frame_id = tsdf_world_frame_;
    mesh_pub_->publish(mesh_msg);

    // Output as point cloud.
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    fillPointcloudWithMesh(mesh_layer, tsdf_mesh_color_mode_, &pointcloud);
    publishPclCloud(
        mesh_pointcloud_pub_, pointcloud, tsdf_world_frame_, node_->now());

    // Output as pcl mesh.
    pcl::PolygonMesh polygon_mesh;
    toConnectedPCLPolygonMesh(*mesh_layer, tsdf_world_frame_, &polygon_mesh);
    pcl_msgs::msg::PolygonMesh pcl_mesh_msg;
    pcl_conversions::fromPCL(polygon_mesh, pcl_mesh_msg);
    mesh_msg.header.stamp = node_->now();
    mesh_pcl_mesh_pub_->publish(mesh_msg);

    if (!tsdf_mesh_output_path_.empty()) {
      if (voxfield::outputMeshLayerAsPly(tsdf_mesh_output_path_, *mesh_layer)) {
        RCLCPP_INFO_STREAM(
            node_->get_logger(),
            "Output mesh PLY file to " << tsdf_mesh_output_path_);
      }
    }
  }
}

}  // namespace voxfield

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  voxfield::initGflagsAndGlog(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("visualize_tsdf_node");

  std::string tsdf_proto_path = "";
  voxfield::param(*node, "tsdf_proto_path", tsdf_proto_path);
  if (tsdf_proto_path.empty()) {
    RCLCPP_FATAL_STREAM(
        node->get_logger(),
        "Please provide a TSDF proto file to visualize using the ros "
        << "parameter: tsdf_proto_path");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO_STREAM(
      node->get_logger(), "Visualize TSDF grid from " << tsdf_proto_path);

  RCLCPP_INFO_STREAM(node->get_logger(), "Loading...");
  voxfield::Layer<voxfield::TsdfVoxel>::Ptr tsdf_layer;
  if (!voxfield::io::LoadLayer<voxfield::TsdfVoxel>(
          tsdf_proto_path, &tsdf_layer)) {
    RCLCPP_FATAL_STREAM(
        node->get_logger(),
        "Unable to load a TSDF grid from: " << tsdf_proto_path);
    rclcpp::shutdown();
    return 1;
  }
  CHECK(tsdf_layer);
  RCLCPP_INFO_STREAM(node->get_logger(), "Done.");

  RCLCPP_INFO_STREAM(node->get_logger(), "Visualizing...");
  voxfield::SimpleTsdfVisualizer visualizer(node);
  visualizer.run(*tsdf_layer);
  RCLCPP_INFO_STREAM(node->get_logger(), "Done.");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
