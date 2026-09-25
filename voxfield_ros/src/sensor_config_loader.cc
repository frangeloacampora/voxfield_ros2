#include "voxfield_ros/sensor_config_loader.h"

#include <algorithm>
#include <cctype>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

#include "voxfield_ros/param_utils.h"

namespace voxfield {
namespace {

// MULTI_SENSOR_PLAN.md §7.1: the per-sensor key whitelist. Every key here
// maps 1:1 to a top-level param already read by ros_params.h (same name,
// same units), except `anti_grazing`, which maps to the config field
// `enable_anti_grazing`.
const std::unordered_set<std::string>& sensorConfigKeys() {
  static const std::unordered_set<std::string> keys = {
      "topic", "freespace_topic", "frame", "pointcloud_queue_size",
      "input_qos_best_effort", "min_time_between_msgs_sec", "T_B_C",
      "invert_T_B_C", "method"};
  return keys;
}

// Shared by TsdfIntegratorBase::Config and NpTsdfIntegratorBase::Config
// (both structs define these fields, with matching names/types/units).
const std::unordered_set<std::string>& integratorKeys() {
  static const std::unordered_set<std::string> keys = {
      "min_ray_length_m",
      "max_ray_length_m",
      "voxel_carving_enabled",
      "allow_clear",
      "use_const_weight",
      "use_weight_dropoff",
      "use_sparsity_compensation_factor",
      "sparsity_compensation_factor",
      "anti_grazing",
      "merge_with_clear",
      "start_voxel_subsampling_factor",
      "max_consecutive_ray_collisions",
      "clear_checks_every_n_frames",
      "max_integration_time_s"};
  return keys;
}

// NpTsdfIntegratorBase::Config only.
const std::unordered_set<std::string>& npIntegratorKeys() {
  static const std::unordered_set<std::string> keys = {
      "weight_reduction_exp", "normal_available", "reliable_band_ratio",
      "curve_assumption", "reliable_normal_ratio_thre"};
  return keys;
}

// RangeImageProjector::Config only.
const std::unordered_set<std::string>& projectorKeys() {
  static const std::unordered_set<std::string> keys = {
      "sensor_is_lidar", "width",  "height", "fov_up",
      "fov_down",        "vx",     "vy",     "fx",
      "fy",              "smooth_thre_ratio", "min_z", "min_dist"};
  return keys;
}

// MULTI_SENSOR_PLAN.md §7.2: map-global, forbidden per sensor.
const std::unordered_set<std::string>& forbiddenKeys() {
  static const std::unordered_set<std::string> keys = {
      "tsdf_voxel_size",  "tsdf_voxels_per_side", "voxel_size",
      "voxels_per_side_in_block", "truncation_distance", "max_weight",
      "integrator_threads", "integration_order_mode", "weight_dropoff_epsilon",
      "world_frame", "use_tf_transforms", "T_B_D", "invert_T_B_D", "T_C_CH",
      "invert_T_C_CH", "enable_icp", "body_frame"};
  return keys;
}

bool isForbiddenByPrefix(const std::string& key) {
  static const char* const kPrefixes[] = {
      "esdf_", "occ_", "mesh_", "publish_", "update_"};
  for (const char* prefix : kPrefixes) {
    if (key.compare(0, std::string(prefix).size(), prefix) == 0) {
      return true;
    }
  }
  return false;
}

bool isWhitelisted(const std::string& key) {
  return sensorConfigKeys().count(key) > 0 || integratorKeys().count(key) > 0 ||
      npIntegratorKeys().count(key) > 0 || projectorKeys().count(key) > 0;
}

bool isValidNameChar(char c) {
  return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
}

bool hasOverride(rclcpp::Node& node, const std::string& full_key) {
  const auto& overrides =
      node.get_node_parameters_interface()->get_parameter_overrides();
  return overrides.count(full_key) > 0;
}

// Applies the §7.1 whitelist's integrator/projector keys (categories B, C,
// D) to `sensor`, reading only the keys actually present under `prefix`
// (Phase 4 step 2: never declares the full ~30-key set per sensor).
void applyIntegratorAndProjectorOverrides(
    rclcpp::Node& node, const std::string& prefix, LoadedSensor* sensor) {
  // Category B: shared by both integrator Config structs.
  if (hasOverride(node, prefix + "min_ray_length_m")) {
    param(node, prefix + "min_ray_length_m", sensor->tsdf.min_ray_length_m);
    param(
        node, prefix + "min_ray_length_m", sensor->np_tsdf.min_ray_length_m);
  }
  if (hasOverride(node, prefix + "max_ray_length_m")) {
    param(node, prefix + "max_ray_length_m", sensor->tsdf.max_ray_length_m);
    param(
        node, prefix + "max_ray_length_m", sensor->np_tsdf.max_ray_length_m);
  }
  if (hasOverride(node, prefix + "voxel_carving_enabled")) {
    param(
        node, prefix + "voxel_carving_enabled",
        sensor->tsdf.voxel_carving_enabled);
    param(
        node, prefix + "voxel_carving_enabled",
        sensor->np_tsdf.voxel_carving_enabled);
  }
  if (hasOverride(node, prefix + "allow_clear")) {
    param(node, prefix + "allow_clear", sensor->tsdf.allow_clear);
    param(node, prefix + "allow_clear", sensor->np_tsdf.allow_clear);
  }
  if (hasOverride(node, prefix + "use_const_weight")) {
    param(node, prefix + "use_const_weight", sensor->tsdf.use_const_weight);
    param(
        node, prefix + "use_const_weight", sensor->np_tsdf.use_const_weight);
  }
  if (hasOverride(node, prefix + "use_weight_dropoff")) {
    param(
        node, prefix + "use_weight_dropoff", sensor->tsdf.use_weight_dropoff);
    param(
        node, prefix + "use_weight_dropoff",
        sensor->np_tsdf.use_weight_dropoff);
  }
  if (hasOverride(node, prefix + "use_sparsity_compensation_factor")) {
    param(
        node, prefix + "use_sparsity_compensation_factor",
        sensor->tsdf.use_sparsity_compensation_factor);
    param(
        node, prefix + "use_sparsity_compensation_factor",
        sensor->np_tsdf.use_sparsity_compensation_factor);
  }
  if (hasOverride(node, prefix + "sparsity_compensation_factor")) {
    param(
        node, prefix + "sparsity_compensation_factor",
        sensor->tsdf.sparsity_compensation_factor);
    param(
        node, prefix + "sparsity_compensation_factor",
        sensor->np_tsdf.sparsity_compensation_factor);
  }
  if (hasOverride(node, prefix + "anti_grazing")) {
    param(node, prefix + "anti_grazing", sensor->tsdf.enable_anti_grazing);
    param(
        node, prefix + "anti_grazing", sensor->np_tsdf.enable_anti_grazing);
  }
  if (hasOverride(node, prefix + "merge_with_clear")) {
    param(node, prefix + "merge_with_clear", sensor->tsdf.merge_with_clear);
    param(
        node, prefix + "merge_with_clear", sensor->np_tsdf.merge_with_clear);
  }
  if (hasOverride(node, prefix + "start_voxel_subsampling_factor")) {
    param(
        node, prefix + "start_voxel_subsampling_factor",
        sensor->tsdf.start_voxel_subsampling_factor);
    param(
        node, prefix + "start_voxel_subsampling_factor",
        sensor->np_tsdf.start_voxel_subsampling_factor);
  }
  if (hasOverride(node, prefix + "max_consecutive_ray_collisions")) {
    param(
        node, prefix + "max_consecutive_ray_collisions",
        sensor->tsdf.max_consecutive_ray_collisions);
    param(
        node, prefix + "max_consecutive_ray_collisions",
        sensor->np_tsdf.max_consecutive_ray_collisions);
  }
  if (hasOverride(node, prefix + "clear_checks_every_n_frames")) {
    param(
        node, prefix + "clear_checks_every_n_frames",
        sensor->tsdf.clear_checks_every_n_frames);
    param(
        node, prefix + "clear_checks_every_n_frames",
        sensor->np_tsdf.clear_checks_every_n_frames);
  }
  if (hasOverride(node, prefix + "max_integration_time_s")) {
    param(
        node, prefix + "max_integration_time_s",
        sensor->tsdf.max_integration_time_s);
    param(
        node, prefix + "max_integration_time_s",
        sensor->np_tsdf.max_integration_time_s);
  }

  // Category C: NP-integrator only.
  if (hasOverride(node, prefix + "weight_reduction_exp")) {
    param(
        node, prefix + "weight_reduction_exp",
        sensor->np_tsdf.weight_reduction_exp);
  }
  if (hasOverride(node, prefix + "normal_available")) {
    param(
        node, prefix + "normal_available", sensor->np_tsdf.normal_available);
  }
  if (hasOverride(node, prefix + "reliable_band_ratio")) {
    param(
        node, prefix + "reliable_band_ratio",
        sensor->np_tsdf.reliable_band_ratio);
  }
  if (hasOverride(node, prefix + "curve_assumption")) {
    param(
        node, prefix + "curve_assumption", sensor->np_tsdf.curve_assumption);
  }
  if (hasOverride(node, prefix + "reliable_normal_ratio_thre")) {
    param(
        node, prefix + "reliable_normal_ratio_thre",
        sensor->np_tsdf.reliable_normal_ratio_thre);
  }

  // Category D: range-image projection model.
  if (hasOverride(node, prefix + "sensor_is_lidar")) {
    param(
        node, prefix + "sensor_is_lidar", sensor->projector.sensor_is_lidar);
  }
  if (hasOverride(node, prefix + "width")) {
    param(node, prefix + "width", sensor->projector.width);
  }
  if (hasOverride(node, prefix + "height")) {
    param(node, prefix + "height", sensor->projector.height);
  }
  if (hasOverride(node, prefix + "fov_up")) {
    param(node, prefix + "fov_up", sensor->projector.fov_up);
  }
  if (hasOverride(node, prefix + "fov_down")) {
    param(node, prefix + "fov_down", sensor->projector.fov_down);
  }
  if (hasOverride(node, prefix + "vx")) {
    param(node, prefix + "vx", sensor->projector.vx);
  }
  if (hasOverride(node, prefix + "vy")) {
    param(node, prefix + "vy", sensor->projector.vy);
  }
  if (hasOverride(node, prefix + "fx")) {
    param(node, prefix + "fx", sensor->projector.fx);
  }
  if (hasOverride(node, prefix + "fy")) {
    param(node, prefix + "fy", sensor->projector.fy);
  }
  if (hasOverride(node, prefix + "smooth_thre_ratio")) {
    param(
        node, prefix + "smooth_thre_ratio",
        sensor->projector.smooth_thre_ratio);
  }
  if (hasOverride(node, prefix + "min_z")) {
    param(node, prefix + "min_z", sensor->projector.min_z);
  }
  if (hasOverride(node, prefix + "min_dist")) {
    param(node, prefix + "min_dist", sensor->projector.min_dist);
  }
}

bool sensorNamesPresent(rclcpp::Node& node) {
  const auto& overrides =
      node.get_node_parameters_interface()->get_parameter_overrides();
  return node.has_parameter("sensor_names") ||
      overrides.count("sensor_names") > 0;
}

}  // namespace

std::vector<LoadedSensor> loadSensors(
    rclcpp::Node& node, const TsdfIntegratorBase::Config* tsdf_base,
    const NpTsdfIntegratorBase::Config* np_base,
    const RangeImageProjector::Config* projector_base,
    const std::string& legacy_method, const SensorConfig& legacy_input) {
  // ---- Legacy mode (M1): sensor_names not set. ----
  if (!sensorNamesPresent(node)) {
    if (!listParameterOverrides(node, "sensors.").empty()) {
      RCLCPP_WARN(
          node.get_logger(),
          "sensor_names is not set (legacy single-sensor mode), so every "
          "'sensors.*' override is ignored.");
    }
    LoadedSensor sensor;
    sensor.input = legacy_input;
    sensor.tsdf = tsdf_base != nullptr ? *tsdf_base : TsdfIntegratorBase::Config();
    sensor.np_tsdf =
        np_base != nullptr ? *np_base : NpTsdfIntegratorBase::Config();
    sensor.projector =
        projector_base != nullptr ? *projector_base : RangeImageProjector::Config();
    sensor.method = legacy_method;
    return {sensor};
  }

  // ---- Multi-sensor mode (M1): fail-fast validation (M13). ----
  std::vector<std::string> errors;
  const std::vector<std::string> sensor_names =
      getParam<std::vector<std::string>>(node, "sensor_names", {});

  // M13.1: names non-empty, unique, [A-Za-z0-9_]+.
  std::set<std::string> valid_names;
  {
    std::set<std::string> seen;
    for (const std::string& name : sensor_names) {
      if (name.empty()) {
        errors.push_back("sensor_names contains an empty name");
        continue;
      }
      if (!std::all_of(name.begin(), name.end(), isValidNameChar)) {
        errors.push_back(
            "sensor name '" + name +
            "' is not a valid parameter-name segment (must match "
            "[A-Za-z0-9_]+)");
        continue;
      }
      if (!seen.insert(name).second) {
        errors.push_back("sensor name '" + name + "' is duplicated");
        continue;
      }
      valid_names.insert(name);
    }
  }

  // M13.3/M13.4: every "sensors.*" override key must be
  // "sensors.<name in sensor_names>.<key in the whitelist>", and not a
  // forbidden (map-global) key.
  for (const std::string& full_key : listParameterOverrides(node, "sensors.")) {
    const std::string rest = full_key.substr(std::string("sensors.").size());
    const size_t dot = rest.find('.');
    if (dot == std::string::npos) {
      errors.push_back(
          "'" + full_key +
          "' is not of the form sensors.<name>.<key>");
      continue;
    }
    const std::string name = rest.substr(0, dot);
    const std::string key = rest.substr(dot + 1);
    if (valid_names.count(name) == 0) {
      errors.push_back(
          "'" + full_key + "' names sensor '" + name +
          "', which is not in sensor_names");
      continue;
    }
    if (forbiddenKeys().count(key) > 0 || isForbiddenByPrefix(key)) {
      errors.push_back(
          "'" + full_key + "': '" + key +
          "' is map-global and cannot be set per sensor");
      continue;
    }
    if (!isWhitelisted(key)) {
      errors.push_back(
          "'" + full_key + "': unknown per-sensor key '" + key +
          "' (typo?)");
    }
  }

  // Build one LoadedSensor per valid name.
  std::vector<LoadedSensor> sensors;
  std::set<std::string> seen_topics;
  const bool is_np_server = np_base != nullptr;
  for (const std::string& name : sensor_names) {
    if (valid_names.count(name) == 0) {
      continue;  // already reported above
    }

    LoadedSensor sensor;
    // SensorConfig fields not explicitly listed inherit the top-level
    // value via this copy (queue_size, input_qos_best_effort,
    // min_time_between_msgs_sec); name/topic/freespace_topic/frame/T_B_C
    // are reset below -- they must not be shared across sensors.
    sensor.input = legacy_input;
    sensor.input.name = name;
    sensor.input.topic.clear();
    sensor.input.freespace_topic.clear();
    sensor.input.frame.clear();
    sensor.input.has_T_B_C = false;
    sensor.tsdf = tsdf_base != nullptr ? *tsdf_base : TsdfIntegratorBase::Config();
    sensor.np_tsdf =
        np_base != nullptr ? *np_base : NpTsdfIntegratorBase::Config();
    sensor.projector =
        projector_base != nullptr ? *projector_base : RangeImageProjector::Config();
    sensor.method = legacy_method;

    const std::string prefix = "sensors." + name + ".";

    if (hasOverride(node, prefix + "topic")) {
      param(node, prefix + "topic", sensor.input.topic);
    }
    if (sensor.input.topic.empty()) {
      errors.push_back("sensor '" + name + "' is missing required key 'topic'");
    } else if (!seen_topics.insert(sensor.input.topic).second) {
      errors.push_back(
          "sensor '" + name + "': topic '" + sensor.input.topic +
          "' is used by another sensor too (would double-integrate)");
    }

    if (hasOverride(node, prefix + "freespace_topic")) {
      param(node, prefix + "freespace_topic", sensor.input.freespace_topic);
    }
    if (hasOverride(node, prefix + "frame")) {
      param(node, prefix + "frame", sensor.input.frame);
    }
    if (hasOverride(node, prefix + "pointcloud_queue_size")) {
      param(node, prefix + "pointcloud_queue_size", sensor.input.queue_size);
    }
    if (hasOverride(node, prefix + "input_qos_best_effort")) {
      param(
          node, prefix + "input_qos_best_effort",
          sensor.input.input_qos_best_effort);
    }
    if (hasOverride(node, prefix + "min_time_between_msgs_sec")) {
      param(
          node, prefix + "min_time_between_msgs_sec",
          sensor.input.min_time_between_msgs_sec);
    }
    if (hasOverride(node, prefix + "method")) {
      param(node, prefix + "method", sensor.method);
    }
    sensor.input.has_T_B_C = getTransformationParam(
        node, prefix + "T_B_C", prefix + "invert_T_B_C", &sensor.input.T_B_C);

    applyIntegratorAndProjectorOverrides(node, prefix, &sensor);

    if (is_np_server) {
      std::string why;
      if (!sensor.projector.isValid(&why)) {
        errors.push_back("sensor '" + name + "': invalid projector config: " + why);
      }
    }

    sensors.push_back(std::move(sensor));
  }

  // M13.6: queue mode only -- warn if several sensors end up with the
  // identical T_B_C (almost always a config mistake). Sensors that didn't
  // override T_B_C all inherit the same global one, so 2+ of those already
  // qualify; also flag exact duplicate explicit overrides.
  const bool use_tf_transforms = getParam<bool>(node, "use_tf_transforms", true);
  if (!use_tf_transforms) {
    size_t inherited_count = 0;
    for (const LoadedSensor& sensor : sensors) {
      if (!sensor.input.has_T_B_C) {
        ++inherited_count;
      }
    }
    if (inherited_count >= 2) {
      RCLCPP_WARN(
          node.get_logger(),
          "%zu sensors have no per-sensor T_B_C override and all inherit "
          "the same global T_B_C; this is almost always a config mistake "
          "in multi-sensor mode.",
          inherited_count);
    }
    for (size_t i = 0; i < sensors.size(); ++i) {
      if (!sensors[i].input.has_T_B_C) {
        continue;
      }
      for (size_t j = i + 1; j < sensors.size(); ++j) {
        if (sensors[j].input.has_T_B_C &&
            sensors[i].input.T_B_C == sensors[j].input.T_B_C) {
          RCLCPP_WARN(
              node.get_logger(),
              "sensors '%s' and '%s' have identical explicit T_B_C "
              "overrides; this is almost always a config mistake.",
              sensors[i].input.name.c_str(), sensors[j].input.name.c_str());
        }
      }
    }
  }

  if (!errors.empty()) {
    std::ostringstream oss;
    oss << errors.size() << " multi-sensor config problem(s):";
    for (const std::string& error : errors) {
      oss << "\n  - " << error;
    }
    throw std::invalid_argument(oss.str());
  }

  for (const LoadedSensor& sensor : sensors) {
    const std::string frame_desc =
        sensor.input.frame.empty() ? "<header>" : sensor.input.frame;
    if (is_np_server) {
      std::string model_desc;
      if (sensor.projector.sensor_is_lidar) {
        model_desc = "lidar fov=[" + std::to_string(sensor.projector.fov_up) +
            ", " + std::to_string(sensor.projector.fov_down) + "]deg";
      } else {
        model_desc = "camera fx=" + std::to_string(sensor.projector.fx) +
            " fy=" + std::to_string(sensor.projector.fy);
      }
      RCLCPP_INFO(
          node.get_logger(),
          "sensor '%s': topic='%s' frame='%s' ray=[%.2f, %.2f]m %s",
          sensor.input.name.c_str(), sensor.input.topic.c_str(),
          frame_desc.c_str(), sensor.tsdf.min_ray_length_m,
          sensor.tsdf.max_ray_length_m, model_desc.c_str());
    } else {
      RCLCPP_INFO(
          node.get_logger(),
          "sensor '%s': topic='%s' frame='%s' ray=[%.2f, %.2f]m",
          sensor.input.name.c_str(), sensor.input.topic.c_str(),
          frame_desc.c_str(), sensor.tsdf.min_ray_length_m,
          sensor.tsdf.max_ray_length_m);
    }
  }

  return sensors;
}

}  // namespace voxfield
