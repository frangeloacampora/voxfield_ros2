#ifndef VOXFIELD_RVIZ_PLUGIN_MATERIAL_LOADER_H_
#define VOXFIELD_RVIZ_PLUGIN_MATERIAL_LOADER_H_

namespace voxfield_rviz_plugin {

/**
 * This class simply loads custom ogre materials for visualization upon startup.
 */
class MaterialLoader {
 public:
  static void loadMaterials();

 private:
  MaterialLoader() = default;

  static bool materials_loaded_;
};

}  // namespace voxfield_rviz_plugin

#endif  // VOXFIELD_RVIZ_PLUGIN_MATERIAL_LOADER_H_
