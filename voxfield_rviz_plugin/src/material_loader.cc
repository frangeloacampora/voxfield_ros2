#include "voxfield_rviz_plugin/material_loader.h"

#include <OgreResourceGroupManager.h>
#include <ros/package.h>

namespace voxfield_rviz_plugin {

bool MaterialLoader::materials_loaded_ = false;

void MaterialLoader::loadMaterials() {
  if (materials_loaded_) {
    return;
  }
  // first instance loads a custom ogre material that supports transparent
  // colors.
  std::string path =
      ros::package::getPath("voxfield_rviz_plugin") + "/content/materials";
  Ogre::ResourceGroupManager::getSingletonPtr()->createResourceGroup(
      "VoxfieldMaterials");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      path, "FileSystem", "VoxfieldMaterials", true);
  Ogre::ResourceGroupManager::getSingletonPtr()->initialiseResourceGroup(
      "VoxfieldMaterials");
  Ogre::ResourceGroupManager::getSingletonPtr()->loadResourceGroup(
      "VoxfieldMaterials");
  materials_loaded_ = true;
}

}  // namespace voxfield_rviz_plugin
