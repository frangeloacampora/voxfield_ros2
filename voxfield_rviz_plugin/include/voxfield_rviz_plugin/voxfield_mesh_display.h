#ifndef VOXFIELD_RVIZ_PLUGIN_VOXFIELD_MESH_DISPLAY_H_
#define VOXFIELD_RVIZ_PLUGIN_VOXFIELD_MESH_DISPLAY_H_

#include <memory>

#include <rclcpp/time.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <voxfield_msgs/msg/mesh.hpp>

#include "voxfield_rviz_plugin/voxfield_mesh_visual.h"

namespace voxfield_rviz_plugin {

class VoxfieldMeshVisual;

class VoxfieldMeshDisplay
    : public rviz_common::MessageFilterDisplay<voxfield_msgs::msg::Mesh> {
  Q_OBJECT
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VoxfieldMeshDisplay();
  virtual ~VoxfieldMeshDisplay() = default;

 protected:
  void reset() override;
  void fixedFrameChanged() override;

 private:
  void processMessage(voxfield_msgs::msg::Mesh::ConstSharedPtr msg) override;
  bool updateTransformation(rclcpp::Time stamp);

  std::unique_ptr<VoxfieldMeshVisual> visual_;

  // Allows the user to still clear the mesh by clicking this property
  rviz_common::properties::BoolProperty visible_property_;
  Q_SLOT void visibleSLOT();
};

}  // namespace voxfield_rviz_plugin

#endif  // VOXFIELD_RVIZ_PLUGIN_VOXFIELD_MESH_DISPLAY_H_
