#ifndef VOXFIELD_ROS_PARAM_UTILS_H_
#define VOXFIELD_ROS_PARAM_UTILS_H_

// Tolerant declare-or-get parameter helper (ROS2_PORT_PLAN.md D6).
//
// ROS 1 code reads the same parameter name from several places and the
// shipped YAML files mix ints and floats freely, which plain
// declare_parameter<T>() can't tolerate (it throws on redeclaration and on
// a type mismatch). getParam()/param() below declare a parameter (with
// dynamic typing) the first time they see its name, then coerce whatever
// value is stored to the caller's C++ type -- never throwing, always
// falling back to the caller's default on a real type error.

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <type_traits>
#include <vector>

#include "voxfield/core/common.h"

namespace voxfield {

namespace param_utils_internal {

inline rclcpp::ParameterValue declareAndGet(
    rclcpp::Node& node, const std::string& name,
    const rclcpp::ParameterValue& default_value) {
  if (!node.has_parameter(name)) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    node.declare_parameter(name, default_value, descriptor);
  }
  return node.get_parameter(name).get_parameter_value();
}

}  // namespace param_utils_internal

template <typename T>
T getParam(
    rclcpp::Node& node, const std::string& name, const T& default_value) {
  using param_utils_internal::declareAndGet;

  if constexpr (std::is_same_v<T, bool>) {
    rclcpp::ParameterValue value =
        declareAndGet(node, name, rclcpp::ParameterValue(default_value));
    switch (value.get_type()) {
      case rclcpp::ParameterType::PARAMETER_NOT_SET:
        return default_value;
      case rclcpp::ParameterType::PARAMETER_BOOL:
        return value.get<bool>();
      default:
        RCLCPP_ERROR(
            node.get_logger(),
            "Parameter '%s': expected bool, got %s; using default",
            name.c_str(), rclcpp::to_string(value.get_type()).c_str());
        return default_value;
    }
  } else if constexpr (std::is_same_v<T, std::string>) {
    rclcpp::ParameterValue value =
        declareAndGet(node, name, rclcpp::ParameterValue(default_value));
    switch (value.get_type()) {
      case rclcpp::ParameterType::PARAMETER_NOT_SET:
        return default_value;
      case rclcpp::ParameterType::PARAMETER_STRING:
        return value.get<std::string>();
      default:
        RCLCPP_ERROR(
            node.get_logger(),
            "Parameter '%s': expected string, got %s; using default",
            name.c_str(), rclcpp::to_string(value.get_type()).c_str());
        return default_value;
    }
  } else if constexpr (std::is_same_v<T, std::vector<double>>) {
    rclcpp::ParameterValue value =
        declareAndGet(node, name, rclcpp::ParameterValue(default_value));
    switch (value.get_type()) {
      case rclcpp::ParameterType::PARAMETER_NOT_SET:
        return default_value;
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        return value.get<std::vector<double>>();
      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
        const std::vector<int64_t> ints = value.get<std::vector<int64_t>>();
        return std::vector<double>(ints.begin(), ints.end());
      }
      default:
        RCLCPP_ERROR(
            node.get_logger(),
            "Parameter '%s': expected a double array, got %s; using default",
            name.c_str(), rclcpp::to_string(value.get_type()).c_str());
        return default_value;
    }
  } else if constexpr (std::is_floating_point_v<T>) {
    // float and FloatingPoint targets go through double.
    rclcpp::ParameterValue value = declareAndGet(
        node, name, rclcpp::ParameterValue(static_cast<double>(default_value)));
    switch (value.get_type()) {
      case rclcpp::ParameterType::PARAMETER_NOT_SET:
        return default_value;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        return static_cast<T>(value.get<double>());
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return static_cast<T>(value.get<int64_t>());
      default:
        RCLCPP_ERROR(
            node.get_logger(),
            "Parameter '%s': expected a floating-point value, got %s; "
            "using default",
            name.c_str(), rclcpp::to_string(value.get_type()).c_str());
        return default_value;
    }
  } else if constexpr (std::is_integral_v<T>) {
    // int, size_t, and unsigned go through int64_t.
    rclcpp::ParameterValue value = declareAndGet(
        node, name,
        rclcpp::ParameterValue(static_cast<int64_t>(default_value)));
    switch (value.get_type()) {
      case rclcpp::ParameterType::PARAMETER_NOT_SET:
        return default_value;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return static_cast<T>(value.get<int64_t>());
      case rclcpp::ParameterType::PARAMETER_DOUBLE: {
        const double d = value.get<double>();
        if (std::floor(d) == d) {
          return static_cast<T>(d);
        }
        RCLCPP_ERROR(
            node.get_logger(),
            "Parameter '%s': expected an integer, got non-integral double "
            "%f; using default",
            name.c_str(), d);
        return default_value;
      }
      default:
        RCLCPP_ERROR(
            node.get_logger(),
            "Parameter '%s': expected an integer, got %s; using default",
            name.c_str(), rclcpp::to_string(value.get_type()).c_str());
        return default_value;
    }
  } else {
    static_assert(!sizeof(T*), "getParam<T>: unsupported parameter type T");
  }
}

// Mirrors ros::NodeHandle::param(name, value, default): declares once,
// returns the override if present, otherwise the default. Coerces
// int<->double.
template <typename T>
void param(rclcpp::Node& node, const std::string& name, T& value) {
  value = getParam(node, name, value);
}

// Reads a 4x4 row-major transform stored as a flat 16-double array, plus an
// optional "invert_<name>" bool. Returns false (and leaves T unchanged) if
// absent.
inline bool getTransformationParam(
    rclcpp::Node& node, const std::string& name, const std::string& invert_name,
    Transformation* T_out) {
  const auto& overrides =
      node.get_node_parameters_interface()->get_parameter_overrides();
  const bool present = node.has_parameter(name) || overrides.count(name) > 0;
  if (!present) {
    return false;
  }

  const std::vector<double> flat =
      getParam<std::vector<double>>(node, name, std::vector<double>());
  if (flat.size() != 16) {
    RCLCPP_ERROR(
        node.get_logger(),
        "Parameter '%s' must be a flat 16-element (4x4 row-major) "
        "transform, got %zu elements",
        name.c_str(), flat.size());
    return false;
  }

  Transformation::TransformationMatrix T_mat;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      T_mat(row, col) =
          static_cast<FloatingPoint>(flat[static_cast<size_t>(row * 4 + col)]);
    }
  }

  Transformation transformation =
      Transformation::constructAndRenormalizeRotation(T_mat);

  const bool invert = getParam<bool>(node, invert_name, false);
  *T_out = invert ? transformation.inverse() : transformation;
  return true;
}

}  // namespace voxfield

#endif  // VOXFIELD_ROS_PARAM_UTILS_H_
