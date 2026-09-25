#ifndef VOXFIELD_ROS_SENSOR_CONFIG_LOADER_H_
#define VOXFIELD_ROS_SENSOR_CONFIG_LOADER_H_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <voxfield/integrator/np_tsdf_integrator.h>
#include <voxfield/integrator/tsdf_integrator.h>

#include "voxfield_ros/range_image_projector.h"
#include "voxfield_ros/sensor_input.h"

namespace voxfield {

// One sensor's fully-resolved configuration: the per-sensor frontend config
// plus the integrator/projector configs it needs, with every whitelisted
// key's inheritance already applied (MULTI_SENSOR_PLAN.md M2). Both `tsdf`
// and `np_tsdf`/`projector` are always populated (from the caller's base
// config, or default-constructed); a TsdfServer caller only reads `tsdf`,
// an NpTsdfServer caller only reads `np_tsdf`/`projector`. `method` is
// separate because it selects *which* integrator class (Simple/Merged/Fast)
// to construct, not a field on any Config struct.
struct LoadedSensor {
  SensorConfig input;
  TsdfIntegratorBase::Config tsdf;
  NpTsdfIntegratorBase::Config np_tsdf;
  RangeImageProjector::Config projector;
  std::string method;
};

// Builds one SensorConfig per multi-sensor mode's `sensor_names` entry (or
// exactly one, named "default", in legacy mode -- unset `sensor_names`),
// applying inheritance (M2) and, in multi-sensor mode, strict fail-fast
// validation (M13).
//
// `tsdf_base`/`np_base`/`projector_base` are the already-read top-level
// configs; pass null for `np_base`/`projector_base` when called from
// TsdfServer (which never reads them) -- this also skips
// RangeImageProjector::Config::isValid() validation, since a TSDF-only
// config never sets width/height/fov_*. `legacy_input` is the SensorConfig
// legacy mode returns unchanged, and multi-sensor mode's per-sensor
// inheritance base for queue_size/input_qos_best_effort/
// min_time_between_msgs_sec (name/topic/freespace_topic/frame/T_B_C are
// reset per sensor, not inherited from it -- frame in particular must not
// force every sensor into legacy's single `sensor_frame`). `legacy_method`
// is the already-read top-level `method` param.
//
// Throws std::invalid_argument, with every problem found (not just the
// first) joined into its message, on a multi-sensor-mode validation
// failure. Never reads any `sensors.*` key in legacy mode; logs a
// RCLCPP_WARN if one is set anyway (it's ignored).
std::vector<LoadedSensor> loadSensors(
    rclcpp::Node& node, const TsdfIntegratorBase::Config* tsdf_base,
    const NpTsdfIntegratorBase::Config* np_base,
    const RangeImageProjector::Config* projector_base,
    const std::string& legacy_method, const SensorConfig& legacy_input);

}  // namespace voxfield

#endif  // VOXFIELD_ROS_SENSOR_CONFIG_LOADER_H_
