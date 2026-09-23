#ifndef VOXFIELD_RVIZ_PLUGIN_VOXFIELD_MESH_DISPLAY_H_
#define VOXFIELD_RVIZ_PLUGIN_VOXFIELD_MESH_DISPLAY_H_

#include <memory>

#include <rviz/message_filter_display.h>
#include <voxfield_msgs/Mesh.h>

#include "voxfield_rviz_plugin/voxfield_mesh_visual.h"

namespace voxfield_rviz_plugin {

class VoxfieldMeshVisual;

class VoxfieldMeshDisplay
    : public rviz::MessageFilterDisplay<voxfield_msgs::Mesh> {
  Q_OBJECT
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VoxfieldMeshDisplay();
  virtual ~VoxfieldMeshDisplay() = default;

 protected:
  void reset() override;
  void fixedFrameChanged() override;

 private:
  void processMessage(const voxfield_msgs::Mesh::ConstPtr& msg) override;
  bool updateTransformation(ros::Time stamp);

  std::unique_ptr<VoxfieldMeshVisual> visual_;

  // Allows the user to still clear the mesh by clicking this property
  rviz::BoolProperty visible_property_;
  Q_SLOT void visibleSLOT();
};

}  // namespace voxfield_rviz_plugin

#endif  // VOXFIELD_RVIZ_PLUGIN_VOXFIELD_MESH_DISPLAY_H_
