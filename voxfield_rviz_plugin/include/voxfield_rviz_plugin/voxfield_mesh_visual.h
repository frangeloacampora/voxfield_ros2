#ifndef VOXFIELD_RVIZ_PLUGIN_VOXFIELD_MESH_VISUAL_H_
#define VOXFIELD_RVIZ_PLUGIN_VOXFIELD_MESH_VISUAL_H_

#include <limits>
#include <map>
#include <string>

#include <OgreManualObject.h>

#include <voxfield/core/block_hash.h>
#include <voxfield_msgs/msg/mesh.hpp>
#include <voxfield_msgs/msg/multi_mesh.hpp>

namespace voxfield_rviz_plugin {

/// Visualizes a single voxfield_msgs::Mesh message.
class VoxfieldMeshVisual {
 public:
  VoxfieldMeshVisual(
      Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
      std::string name_space = "");
  virtual ~VoxfieldMeshVisual();

  void setMessage(
      voxfield_msgs::msg::Mesh::ConstSharedPtr msg,
      uint8_t alpha = std::numeric_limits<uint8_t>::max());

  // enable / disable visibility
  void setEnabled(bool enabled);

  /// Set the coordinate frame pose.
  void setPose(
      const Ogre::Vector3& position, const Ogre::Quaternion& orientation);

  void setFrameId(const std::string& frame_id) {
    frame_id_ = frame_id;
  }
  const std::string& getFrameId() {
    return frame_id_;
  }

 private:
  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;

  unsigned int instance_number_;
  static unsigned int instance_counter_;
  std::string name_space_;  // this is the id used by multi-mesh messages
  bool is_enabled_;
  std::string frame_id_;  // the frame this mesh is in, newer messages will
                          // overwrite this

  voxfield::AnyIndexHashMapType<Ogre::ManualObject*>::type object_map_;
};

}  // namespace voxfield_rviz_plugin

#endif  // VOXFIELD_RVIZ_PLUGIN_VOXFIELD_MESH_VISUAL_H_
