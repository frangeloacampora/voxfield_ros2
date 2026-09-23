#ifndef VOXFIELD_RVIZ_PLUGIN_VOXFIELD_MULTI_MESH_DISPLAY_H_
#define VOXFIELD_RVIZ_PLUGIN_VOXFIELD_MULTI_MESH_DISPLAY_H_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include <rviz/message_filter_display.h>
#include <voxfield_msgs/MultiMesh.h>

#include "voxfield_rviz_plugin/voxfield_mesh_visual.h"

namespace voxfield_rviz_plugin {

class VoxfieldMeshVisual;
class VisibilityField;

class VoxfieldMultiMeshDisplay
    : public rviz::MessageFilterDisplay<voxfield_msgs::MultiMesh> {
  Q_OBJECT

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VoxfieldMultiMeshDisplay();
  ~VoxfieldMultiMeshDisplay() override = default;
  void updateVisible();

 protected:
  void reset() override;
  void fixedFrameChanged() override;

  // Override subscribe to enable a custom queue size of more than 10.
  static constexpr uint32_t kSubscriberQueueLength = 1000;
  void subscribe() override;
  void onInitialize() override;

  // Automatically update the mesh poses based on their frame_ids.
  void update(float wall_dt, float ros_dt) override;

 private:
  void processMessage(const voxfield_msgs::MultiMesh::ConstPtr& msg) override;
  bool updateTransformation(VoxfieldMeshVisual* visual, ros::Time stamp);
  void updateAllTransformations();

  // The set of all visuals, identified by namespace.
  std::unordered_map<std::string, VoxfieldMeshVisual> visuals_;

  // The root of the visibility tree.
  std::unique_ptr<VisibilityField> visibility_fields_;
  Q_SLOT void visibleSlot();

  // Property to set visibility for all submaps.
  rviz::BoolProperty toggle_visibility_all_property_;
  Q_SLOT void toggleVisibilityAllSLOT();

  // Keep track of the time that elapsed since we last updated the submap poses,
  // such that we can throttle these updates to a reasonable rate.
  float dt_since_last_update_;
};

// Allow the user to show hide sets of submaps based on the name spaces.
class VisibilityField : public rviz::BoolProperty {
  Q_OBJECT
 public:
  VisibilityField(
      const std::string& name, rviz::BoolProperty* parent,
      VoxfieldMultiMeshDisplay* master);
  void addField(const std::string& field_name);
  void removeField(const std::string& field_name);
  bool isEnabled(const std::string& field_name);
  void setEnabledForAll(bool enabled);

 private:
  Q_SLOT void visibleSlot();
  VoxfieldMultiMeshDisplay* master_;
  std::unordered_map<std::string, std::unique_ptr<VisibilityField>> children_;
  bool hasNameSpace(
      const std::string& name, std::string* ns, std::string* sub_name);
};

}  // namespace voxfield_rviz_plugin

#endif  // VOXFIELD_RVIZ_PLUGIN_VOXFIELD_MULTI_MESH_DISPLAY_H_
