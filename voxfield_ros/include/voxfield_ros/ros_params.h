#ifndef VOXFIELD_ROS_ROS_PARAMS_H_
#define VOXFIELD_ROS_ROS_PARAMS_H_

#include <rclcpp/rclcpp.hpp>
#include <voxfield/alignment/icp.h>
#include <voxfield/core/esdf_map.h>
#include <voxfield/core/occupancy_map.h>
#include <voxfield/core/tsdf_map.h>
#include <voxfield/integrator/esdf_integrator.h>
#include <voxfield/integrator/esdf_occ_edt_integrator.h>
#include <voxfield/integrator/esdf_occ_fiesta_integrator.h>
#include <voxfield/integrator/esdf_voxfield_integrator.h>
#include <voxfield/integrator/np_tsdf_integrator.h>
#include <voxfield/integrator/occupancy_integrator.h>
#include <voxfield/integrator/occupancy_tsdf_integrator.h>
#include <voxfield/integrator/tsdf_integrator.h>
#include <voxfield/mesh/mesh_integrator.h>

#include "voxfield_ros/param_utils.h"

namespace voxfield {

inline TsdfMap::Config getTsdfMapConfigFromRosParam(rclcpp::Node& node) {
  TsdfMap::Config tsdf_config;

  /**
   * Workaround for OS X on mac mini not having specializations for float
   * for some reason.
   */
  double voxel_size = tsdf_config.tsdf_voxel_size;
  int voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  param(node, "tsdf_voxel_size", voxel_size);
  param(node, "tsdf_voxels_per_side", voxels_per_side);
  if (!isPowerOfTwo(voxels_per_side)) {
    RCLCPP_ERROR(
        node.get_logger(),
        "voxels_per_side must be a power of 2, setting to default value");
    voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  }

  tsdf_config.tsdf_voxel_size = static_cast<FloatingPoint>(voxel_size);
  tsdf_config.tsdf_voxels_per_side = voxels_per_side;

  return tsdf_config;
}

inline ICP::Config getICPConfigFromRosParam(rclcpp::Node& node) {
  ICP::Config icp_config;

  param(node, "icp_min_match_ratio", icp_config.min_match_ratio);
  param(node, "icp_subsample_keep_ratio", icp_config.subsample_keep_ratio);
  param(node, "icp_mini_batch_size", icp_config.mini_batch_size);
  param(node, "icp_refine_roll_pitch", icp_config.refine_roll_pitch);
  param(
      node, "icp_inital_translation_weighting",
      icp_config.inital_translation_weighting);
  param(
      node, "icp_inital_rotation_weighting",
      icp_config.inital_rotation_weighting);

  return icp_config;
}

inline TsdfIntegratorBase::Config getTsdfIntegratorConfigFromRosParam(
    rclcpp::Node& node) {
  TsdfIntegratorBase::Config integrator_config;

  integrator_config.voxel_carving_enabled = true;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(node);

  double max_weight = integrator_config.max_weight;
  // if negative, it means the multiplier of the voxel size
  float truncation_distance = -2.0;

  param(node, "truncation_distance", truncation_distance);

  integrator_config.default_truncation_distance =
      truncation_distance > 0
          ? truncation_distance
          : -truncation_distance * tsdf_config.tsdf_voxel_size;

  param(node, "voxel_carving_enabled", integrator_config.voxel_carving_enabled);

  param(node, "max_ray_length_m", integrator_config.max_ray_length_m);
  param(node, "min_ray_length_m", integrator_config.min_ray_length_m);
  param(node, "max_weight", max_weight);
  integrator_config.max_weight = static_cast<float>(max_weight);
  param(node, "use_const_weight", integrator_config.use_const_weight);
  // LiDAR point weighting (see TsdfIntegratorBase::Config::sensor_is_lidar).
  // Same keys the NP integrator and range-image projector read.
  param(node, "sensor_is_lidar", integrator_config.sensor_is_lidar);
  param(node, "weight_reduction_exp", integrator_config.weight_reduction_exp);
  param(node, "lidar_z_weighting", integrator_config.lidar_z_weighting);
  param(node, "use_weight_dropoff", integrator_config.use_weight_dropoff);
  param(node, "allow_clear", integrator_config.allow_clear);
  param(
      node, "start_voxel_subsampling_factor",
      integrator_config.start_voxel_subsampling_factor);
  param(
      node, "max_consecutive_ray_collisions",
      integrator_config.max_consecutive_ray_collisions);
  param(
      node, "clear_checks_every_n_frames",
      integrator_config.clear_checks_every_n_frames);
  param(
      node, "max_integration_time_s", integrator_config.max_integration_time_s);
  param(node, "anti_grazing", integrator_config.enable_anti_grazing);
  param(
      node, "use_sparsity_compensation_factor",
      integrator_config.use_sparsity_compensation_factor);
  param(
      node, "sparsity_compensation_factor",
      integrator_config.sparsity_compensation_factor);
  param(
      node, "integration_order_mode", integrator_config.integration_order_mode);
  float integrator_threads = std::thread::hardware_concurrency();
  param(node, "integrator_threads", integrator_threads);
  integrator_config.integrator_threads = static_cast<int>(integrator_threads);
  param(node, "merge_with_clear", integrator_config.merge_with_clear);

  return integrator_config;
}

inline NpTsdfIntegratorBase::Config getNpTsdfIntegratorConfigFromRosParam(
    rclcpp::Node& node) {
  NpTsdfIntegratorBase::Config integrator_config;

  integrator_config.voxel_carving_enabled = true;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(node);

  double max_weight = integrator_config.max_weight;
  // if negative, it means the multiplier of the voxel size
  float truncation_distance = -2.0;

  param(node, "truncation_distance", truncation_distance);

  integrator_config.default_truncation_distance =
      truncation_distance > 0
          ? truncation_distance
          : -truncation_distance * tsdf_config.tsdf_voxel_size;

  param(node, "voxel_carving_enabled", integrator_config.voxel_carving_enabled);

  param(node, "max_ray_length_m", integrator_config.max_ray_length_m);
  param(node, "min_ray_length_m", integrator_config.min_ray_length_m);
  param(node, "max_weight", max_weight);
  integrator_config.max_weight = static_cast<float>(max_weight);
  param(node, "use_const_weight", integrator_config.use_const_weight);
  param(node, "weight_reduction_exp", integrator_config.weight_reduction_exp);
  param(node, "use_weight_dropoff", integrator_config.use_weight_dropoff);
  param(
      node, "weight_dropoff_epsilon", integrator_config.weight_dropoff_epsilon);
  param(node, "allow_clear", integrator_config.allow_clear);
  param(
      node, "start_voxel_subsampling_factor",
      integrator_config.start_voxel_subsampling_factor);
  param(
      node, "max_consecutive_ray_collisions",
      integrator_config.max_consecutive_ray_collisions);
  param(
      node, "clear_checks_every_n_frames",
      integrator_config.clear_checks_every_n_frames);
  param(
      node, "max_integration_time_s", integrator_config.max_integration_time_s);
  param(node, "anti_grazing", integrator_config.enable_anti_grazing);
  param(
      node, "use_sparsity_compensation_factor",
      integrator_config.use_sparsity_compensation_factor);
  param(
      node, "sparsity_compensation_factor",
      integrator_config.sparsity_compensation_factor);
  param(
      node, "integration_order_mode", integrator_config.integration_order_mode);
  float integrator_threads = std::thread::hardware_concurrency();
  param(node, "integrator_threads", integrator_threads);
  integrator_config.integrator_threads = static_cast<int>(integrator_threads);
  param(node, "merge_with_clear", integrator_config.merge_with_clear);
  param(node, "normal_available", integrator_config.normal_available);
  param(node, "reliable_band_ratio", integrator_config.reliable_band_ratio);
  param(node, "curve_assumption", integrator_config.curve_assumption);
  param(
      node, "reliable_normal_ratio_thre",
      integrator_config.reliable_normal_ratio_thre);

  return integrator_config;
}

inline EsdfMap::Config getEsdfMapConfigFromRosParam(rclcpp::Node& node) {
  EsdfMap::Config esdf_config;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(node);
  esdf_config.esdf_voxel_size = tsdf_config.tsdf_voxel_size;
  esdf_config.esdf_voxels_per_side = tsdf_config.tsdf_voxels_per_side;

  return esdf_config;
}

inline EsdfIntegrator::Config getEsdfIntegratorConfigFromRosParam(
    rclcpp::Node& node) {
  EsdfIntegrator::Config esdf_integrator_config;

  TsdfIntegratorBase::Config tsdf_integrator_config =
      getTsdfIntegratorConfigFromRosParam(node);

  esdf_integrator_config.min_distance_m =
      tsdf_integrator_config.default_truncation_distance / 2.0;

  param(
      node, "esdf_euclidean_distance",
      esdf_integrator_config.full_euclidean_distance);
  param(node, "esdf_max_distance_m", esdf_integrator_config.max_distance_m);
  param(node, "esdf_min_distance_m", esdf_integrator_config.min_distance_m);
  param(
      node, "esdf_default_distance_m",
      esdf_integrator_config.default_distance_m);
  param(node, "esdf_min_diff_m", esdf_integrator_config.min_diff_m);
  param(
      node, "clear_sphere_radius", esdf_integrator_config.clear_sphere_radius);
  param(
      node, "occupied_sphere_radius",
      esdf_integrator_config.occupied_sphere_radius);
  param(
      node, "esdf_add_occupied_crust",
      esdf_integrator_config.add_occupied_crust);

  if (esdf_integrator_config.default_distance_m <
      esdf_integrator_config.max_distance_m) {
    esdf_integrator_config.default_distance_m =
        esdf_integrator_config.max_distance_m;
  }

  return esdf_integrator_config;
}

inline MeshIntegratorConfig getMeshIntegratorConfigFromRosParam(
    rclcpp::Node& node) {
  MeshIntegratorConfig mesh_integrator_config;

  param(node, "mesh_min_weight", mesh_integrator_config.min_weight);
  param(node, "mesh_use_color", mesh_integrator_config.use_color);

  return mesh_integrator_config;
}

inline OccupancyMap::Config getOccupancyMapConfigFromRosParam(
    rclcpp::Node& node) {
  OccupancyMap::Config occ_config;

  /**
   * Workaround for OS X on mac mini not having specializations for float
   * for some reason.
   */
  double voxel_size = occ_config.occupancy_voxel_size;
  int voxels_per_side = occ_config.occupancy_voxels_per_side;
  param(node, "occ_voxel_size", voxel_size);
  param(
      node, "occ_voxels_per_side",
      voxels_per_side);  // block size (unit: voxel)
  if (!isPowerOfTwo(voxels_per_side)) {
    RCLCPP_ERROR(
        node.get_logger(),
        "voxels_per_side must be a power of 2, setting to default value");
    voxels_per_side = occ_config.occupancy_voxels_per_side;
  }

  occ_config.occupancy_voxel_size = static_cast<FloatingPoint>(voxel_size);
  occ_config.occupancy_voxels_per_side = voxels_per_side;

  return occ_config;
}

inline OccTsdfIntegrator::Config getOccTsdfIntegratorConfigFromRosParam(
    rclcpp::Node& node) {
  OccTsdfIntegrator::Config integrator_config;

  param(node, "occ_min_weight", integrator_config.min_weight);

  param(node, "occ_voxel_size_ratio", integrator_config.occ_voxel_size_ratio);

  return integrator_config;
}

inline EsdfMap::Config getEsdfMapConfigFromOccMapRosParam(rclcpp::Node& node) {
  EsdfMap::Config esdf_config;

  const OccupancyMap::Config occ_config =
      getOccupancyMapConfigFromRosParam(node);
  esdf_config.esdf_voxel_size = occ_config.occupancy_voxel_size;
  esdf_config.esdf_voxels_per_side = occ_config.occupancy_voxels_per_side;

  return esdf_config;
}

inline TsdfMap::Config getTsdfMapConfigFromOccMapRosParam(rclcpp::Node& node) {
  TsdfMap::Config tsdf_config;

  const OccupancyMap::Config occ_config =
      getOccupancyMapConfigFromRosParam(node);
  tsdf_config.tsdf_voxel_size = occ_config.occupancy_voxel_size;
  tsdf_config.tsdf_voxels_per_side = occ_config.occupancy_voxels_per_side;

  return tsdf_config;
}

inline TsdfMap::Config getTsdfMapConfigFromEsdfMapRosParam(rclcpp::Node& node) {
  TsdfMap::Config tsdf_config;

  const EsdfMap::Config esdf_config = getEsdfMapConfigFromRosParam(node);
  tsdf_config.tsdf_voxel_size = esdf_config.esdf_voxel_size;
  tsdf_config.tsdf_voxels_per_side = esdf_config.esdf_voxels_per_side;

  return tsdf_config;
}

inline EsdfVoxfieldIntegrator::Config
getEsdfVoxfieldIntegratorConfigFromRosParam(rclcpp::Node& node) {
  EsdfVoxfieldIntegrator::Config esdf_integrator_config;

  int range_boundary_offset_x = esdf_integrator_config.range_boundary_offset(0);
  int range_boundary_offset_y = esdf_integrator_config.range_boundary_offset(1);
  int range_boundary_offset_z = esdf_integrator_config.range_boundary_offset(2);

  param(node, "local_range_offset_x", range_boundary_offset_x);

  param(node, "local_range_offset_y", range_boundary_offset_y);

  param(node, "local_range_offset_z", range_boundary_offset_z);

  param(node, "esdf_max_distance_m", esdf_integrator_config.max_distance_m);

  param(
      node, "esdf_default_distance_m",
      esdf_integrator_config.default_distance_m);

  param(node, "fix_band_distance_m", esdf_integrator_config.band_distance_m);

  param(
      node, "max_behind_surface_m",
      esdf_integrator_config.max_behind_surface_m);
  // max_behind_surface_m should be at least sqrt(3) * truncation_dist

  param(node, "occ_min_weight", esdf_integrator_config.min_weight);

  param(
      node, "occ_voxel_size_ratio",
      esdf_integrator_config.occ_voxel_size_ratio);

  param(node, "num_buckets", esdf_integrator_config.num_buckets);

  param(node, "patch_on", esdf_integrator_config.patch_on);

  param(node, "early_break", esdf_integrator_config.early_break);

  param(node, "finer_esdf_on", esdf_integrator_config.finer_esdf_on);

  esdf_integrator_config.range_boundary_offset(0) = range_boundary_offset_x;
  esdf_integrator_config.range_boundary_offset(1) = range_boundary_offset_y;
  esdf_integrator_config.range_boundary_offset(2) = range_boundary_offset_z;

  return esdf_integrator_config;
}

inline EsdfOccFiestaIntegrator::Config
getEsdfOccFiestaIntegratorConfigFromRosParam(rclcpp::Node& node) {  // NOLINT
  EsdfOccFiestaIntegrator::Config esdf_integrator_config;

  int range_boundary_offset_x = esdf_integrator_config.range_boundary_offset(0);
  int range_boundary_offset_y = esdf_integrator_config.range_boundary_offset(1);
  int range_boundary_offset_z = esdf_integrator_config.range_boundary_offset(2);

  // esdf_integrator_config.min_distance_m =
  //     tsdf_integrator_config.default_truncation_distance / 2.0;

  param(node, "local_range_offset_x", range_boundary_offset_x);

  param(node, "local_range_offset_y", range_boundary_offset_y);

  param(node, "local_range_offset_z", range_boundary_offset_z);

  param(node, "esdf_max_distance_m", esdf_integrator_config.max_distance_m);

  param(
      node, "esdf_default_distance_m",
      esdf_integrator_config.default_distance_m);

  param(
      node, "max_behind_surface_m",
      esdf_integrator_config.max_behind_surface_m);
  // max_behind_surface_m should be at least sqrt(3) * truncation_dist

  param(node, "num_buckets", esdf_integrator_config.num_buckets);

  param(node, "patch_on", esdf_integrator_config.patch_on);

  param(node, "early_break", esdf_integrator_config.early_break);

  esdf_integrator_config.range_boundary_offset(0) = range_boundary_offset_x;
  esdf_integrator_config.range_boundary_offset(1) = range_boundary_offset_y;
  esdf_integrator_config.range_boundary_offset(2) = range_boundary_offset_z;

  return esdf_integrator_config;
}

inline EsdfOccEdtIntegrator::Config getEsdfEdtIntegratorConfigFromRosParam(
    rclcpp::Node& node) {
  EsdfOccEdtIntegrator::Config esdf_integrator_config;

  int range_boundary_offset_x = esdf_integrator_config.range_boundary_offset(0);
  int range_boundary_offset_y = esdf_integrator_config.range_boundary_offset(1);
  int range_boundary_offset_z = esdf_integrator_config.range_boundary_offset(2);

  // esdf_integrator_config.min_distance_m =
  //     tsdf_integrator_config.default_truncation_distance / 2.0;

  param(node, "local_range_offset_x", range_boundary_offset_x);

  param(node, "local_range_offset_y", range_boundary_offset_y);

  param(node, "local_range_offset_z", range_boundary_offset_z);

  param(node, "esdf_max_distance_m", esdf_integrator_config.max_distance_m);

  param(
      node, "esdf_default_distance_m",
      esdf_integrator_config.default_distance_m);

  param(
      node, "max_behind_surface_m",
      esdf_integrator_config.max_behind_surface_m);
  // max_behind_surface_m should be at least sqrt(3) * truncation_dist

  param(node, "num_buckets", esdf_integrator_config.num_buckets);

  if (esdf_integrator_config.default_distance_m <
      esdf_integrator_config.max_distance_m) {
    esdf_integrator_config.default_distance_m =
        esdf_integrator_config.max_distance_m;
  }

  esdf_integrator_config.range_boundary_offset(0) = range_boundary_offset_x;
  esdf_integrator_config.range_boundary_offset(1) = range_boundary_offset_y;
  esdf_integrator_config.range_boundary_offset(2) = range_boundary_offset_z;

  return esdf_integrator_config;
}

}  // namespace voxfield

#endif  // VOXFIELD_ROS_ROS_PARAMS_H_
