#include "voxfield_rviz_plugin/voxfield_mesh_display.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/logging.hpp>

#include "voxfield_rviz_plugin/material_loader.h"

namespace voxfield_rviz_plugin {

VoxfieldMeshDisplay::VoxfieldMeshDisplay()
    : visible_property_(
          "Visible", true,
          "Show or hide the mesh. If the mesh is hidden but not disabled, it "
          "will persist and is incrementally built in the background.",
          this, SLOT(visibleSLOT())) {
  voxfield_rviz_plugin::MaterialLoader::loadMaterials();
}

void VoxfieldMeshDisplay::reset() {
  MFDClass::reset();
  visual_.reset();
}

void VoxfieldMeshDisplay::visibleSLOT() {
  if (visual_) {
    // Set visibility and update the pose if visibility is turned on.
    visual_->setEnabled(visible_property_.getBool());
    if (visible_property_.getBool()) {
      updateTransformation(context_->getFrameManager()->getTime());
    }
  }
}

void VoxfieldMeshDisplay::processMessage(
    voxfield_msgs::msg::Mesh::ConstSharedPtr msg) {
  if (!visual_) {
    visual_.reset(
        new VoxfieldMeshVisual(context_->getSceneManager(), scene_node_));
    visual_->setEnabled(visible_property_.getBool());
  }

  // update the frame, pose and mesh of the visual
  visual_->setFrameId(msg->header.frame_id);
  if (updateTransformation(rclcpp::Time(msg->header.stamp, RCL_ROS_TIME))) {
    visual_->setMessage(msg);
  }
}

bool VoxfieldMeshDisplay::updateTransformation(rclcpp::Time stamp) {
  if (!visual_) {
    // can not get the transform if we don't have a visual
    return false;
  }
  // Look up the transform from tf. If it doesn't work we have to skip.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          visual_->getFrameId(), stamp, position, orientation)) {
    RVIZ_COMMON_LOG_DEBUG_STREAM(
        "Error transforming from frame '"
        << visual_->getFrameId() << "' to frame '" << fixed_frame_.toStdString()
        << "'");
    return false;
  }
  visual_->setPose(position, orientation);
  return true;
}

void VoxfieldMeshDisplay::fixedFrameChanged() {
  tf_filter_->setTargetFrame(fixed_frame_.toStdString());
  // update the transformation of the visuals w.r.t fixed frame
  updateTransformation(context_->getFrameManager()->getTime());
}

}  // namespace voxfield_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    voxfield_rviz_plugin::VoxfieldMeshDisplay, rviz_common::Display)
