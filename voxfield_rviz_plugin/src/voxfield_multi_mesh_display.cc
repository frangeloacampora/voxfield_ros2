#include "voxfield_rviz_plugin/voxfield_multi_mesh_display.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/logging.hpp>

#include "voxfield_rviz_plugin/material_loader.h"

namespace voxfield_rviz_plugin {

VoxfieldMultiMeshDisplay::VoxfieldMultiMeshDisplay()
    : toggle_visibility_all_property_(
          "Toggle Visibility All", true, "Set the visibility for all meshes.",
          this, SLOT(toggleVisibilityAllSLOT())),
      dt_since_last_update_(0.f) {
  voxfield_rviz_plugin::MaterialLoader::loadMaterials();
  // Initialize the top level of the visibility hierarchy.
  visibility_fields_.reset(new VisibilityField("Visible", this, this));

  // Multi-mesh submaps can arrive in large bursts, so default both the
  // subscription QoS depth and the tf2 message-filter queue to a larger
  // value than rviz_common's defaults (5 / 10). Both stay user-editable
  // in the property panel afterwards.
  qos_profile = rclcpp::QoS(kSubscriberQueueLength);
  message_queue_property_->setInt(kSubscriberQueueLength);
}

void VoxfieldMultiMeshDisplay::reset() {
  MFDClass::reset();
  visuals_.clear();
}

void VoxfieldMultiMeshDisplay::visibleSlot() {
  updateVisible();
}

void VoxfieldMultiMeshDisplay::updateVisible() {
  // Set visibility of all visuals and update poses if visibility is turned on.
  for (auto& ns_visual_pair : visuals_) {
    bool visible = false;
    if (isEnabled()) {
      visible = visibility_fields_->isEnabled(ns_visual_pair.first);
    }
    ns_visual_pair.second.setEnabled(visible);
    if (visible) {
      updateTransformation(
          &(ns_visual_pair.second), context_->getFrameManager()->getTime());
    }
  }
}

void VoxfieldMultiMeshDisplay::toggleVisibilityAllSLOT() {
  // Toggle all visibility fields except for the root.
  const bool root_visible = visibility_fields_->getBool();
  visibility_fields_->setEnabledForAll(
      toggle_visibility_all_property_.getBool());
  visibility_fields_->setBool(root_visible);
  updateVisible();
}

void VoxfieldMultiMeshDisplay::processMessage(
    voxfield_msgs::msg::MultiMesh::ConstSharedPtr msg) {
  // Select the matching visual
  auto it = visuals_.find(msg->name_space);
  if (msg->mesh.mesh_blocks.empty()) {
    // if blocks are empty the visual is to be cleared.
    if (it != visuals_.end()) {
      visibility_fields_->removeField(it->first);
      visuals_.erase(it);
    }
  } else {
    // create a visual if it does not yet exist.
    if (it == visuals_.end()) {
      it = visuals_
               .insert(
                   std::make_pair(
                       msg->name_space,
                       VoxfieldMeshVisual(
                           context_->getSceneManager(), scene_node_)))
               .first;
      visibility_fields_->addField(msg->name_space);
      it->second.setEnabled(visibility_fields_->isEnabled(msg->name_space));
    }

    // update the frame, pose and mesh of the visual.
    it->second.setFrameId(msg->header.frame_id);
    if (updateTransformation(
            &(it->second), rclcpp::Time(msg->header.stamp, RCL_ROS_TIME))) {
      // here we use the multi-mesh msg header.
      // catch uninitialized alpha values, since nobody wants to display a
      // completely invisible mesh.
      uint8_t alpha = msg->alpha;
      if (alpha == 0) {
        alpha = std::numeric_limits<uint8_t>::max();
      }

      // convert to normal mesh msg for visual
      auto mesh = std::make_shared<voxfield_msgs::msg::Mesh>(msg->mesh);
      it->second.setMessage(mesh, alpha);
    }
  }
}

bool VoxfieldMultiMeshDisplay::updateTransformation(
    VoxfieldMeshVisual* visual, rclcpp::Time stamp) {
  // Look up the transform from tf. If it doesn't work we have to skip.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          visual->getFrameId(), stamp, position, orientation)) {
    RVIZ_COMMON_LOG_DEBUG_STREAM(
        "Error transforming from frame '"
        << visual->getFrameId() << "' to frame '" << fixed_frame_.toStdString()
        << "'");
    return false;
  }
  visual->setPose(position, orientation);
  return true;
}

void VoxfieldMultiMeshDisplay::update(float wall_dt, float /*ros_dt*/) {
  constexpr float kMinUpdateDt = 1e-1;
  dt_since_last_update_ += wall_dt;
  if (isEnabled() && kMinUpdateDt < dt_since_last_update_) {
    dt_since_last_update_ = 0;
    updateAllTransformations();
  }
}

void VoxfieldMultiMeshDisplay::updateAllTransformations() {
  for (auto& visual : visuals_) {
    updateTransformation(
        &(visual.second), context_->getFrameManager()->getTime());
  }
}

void VoxfieldMultiMeshDisplay::fixedFrameChanged() {
  tf_filter_->setTargetFrame(fixed_frame_.toStdString());
  // update the transformation of the visuals w.r.t fixed frame
  updateAllTransformations();
}

VisibilityField::VisibilityField(
    const std::string& name, rviz_common::properties::BoolProperty* parent,
    VoxfieldMultiMeshDisplay* master)
    : rviz_common::properties::BoolProperty(
          name.c_str(), true,
          "Show or hide the mesh. If the mesh is hidden but not disabled, it "
          "will persist and is incrementally built in the background.",
          parent, SLOT(visibleSlot())),
      master_(master) {
  setDisableChildrenIfFalse(true);
}

void VisibilityField::visibleSlot() {
  master_->updateVisible();
}

bool VisibilityField::hasNameSpace(
    const std::string& name, std::string* ns, std::string* sub_name) {
  std::size_t ns_indicator = name.find('/');
  if (ns_indicator != std::string::npos) {
    *sub_name = name.substr(ns_indicator + 1);
    *ns = name.substr(0, ns_indicator);
    return true;
  }
  *sub_name = name;
  return false;
}

void VisibilityField::addField(const std::string& field_name) {
  std::string sub_name;
  std::string ns;
  if (hasNameSpace(field_name, &ns, &sub_name)) {
    // If there is at least a namespace present resolve it first.
    auto it = children_.find(ns);
    if (it == children_.end()) {
      // Add the new namespace.
      it = children_
               .insert(std::make_pair(ns, std::unique_ptr<VisibilityField>()))
               .first;
      it->second.reset(new VisibilityField(ns, this, master_));
    }
    it->second->addField(sub_name);
  } else {
    auto it =
        children_
            .insert(
                std::make_pair(field_name, std::unique_ptr<VisibilityField>()))
            .first;
    it->second.reset(new VisibilityField(field_name, this, master_));
  }
}

void VisibilityField::removeField(const std::string& field_name) {
  std::string sub_name;
  std::string ns;
  if (hasNameSpace(field_name, &ns, &sub_name)) {
    // If there is at least a namespace present resolve it first.
    auto it = children_.find(ns);
    if (it != children_.end()) {
      it->second->removeField(sub_name);
      if (it->second->children_.empty()) {
        // If the namespace has no more members remove it.
        children_.erase(it);
      }
    }
  } else {
    children_.erase(field_name);
  }
}

bool VisibilityField::isEnabled(const std::string& field_name) {
  if (!getBool()) {
    // This property and therefore all children are disabled.
    return false;
  }
  std::string sub_name;
  std::string ns;
  if (hasNameSpace(field_name, &ns, &sub_name)) {
    // If there is at least a namespace present resolve it first.
    auto it = children_.find(ns);
    if (it == children_.end()) {
      return false;
    }
    return it->second->isEnabled(sub_name);
  } else {
    auto it = children_.find(field_name);
    if (it == children_.end()) {
      return false;
    }
    return it->second->getBool();
  }
}

void VisibilityField::setEnabledForAll(bool enabled) {
  // Recursively set all properties.
  setBool(enabled);
  for (auto& child : children_) {
    child.second->setEnabledForAll(enabled);
  }
}

}  // namespace voxfield_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    voxfield_rviz_plugin::VoxfieldMultiMeshDisplay, rviz_common::Display)
