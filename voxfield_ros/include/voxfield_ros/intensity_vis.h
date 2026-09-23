#ifndef VOXFIELD_ROS_INTENSITY_VIS_H_
#define VOXFIELD_ROS_INTENSITY_VIS_H_

#include <memory>

${1}voxfield/utils/color_maps.h>
#include <voxfield_msgs/Mesh.h>

#include "voxfield_ros/conversions.h"
#include "voxfield_ros/mesh_vis.h"

namespace voxfield {

inline void recolorVoxbloxMeshMsgByIntensity(
    const Layer<IntensityVoxel>& intensity_layer,
    const std::shared_ptr<ColorMap>& color_map, voxfield_msgs::Mesh* mesh_msg) {
  CHECK_NOTNULL(mesh_msg);
  CHECK(color_map);

  // Go over all the blocks in the mesh.
  for (voxfield_msgs::MeshBlock& mesh_block : mesh_msg->mesh_blocks) {
    // Look up verticies in the thermal layer.
    for (size_t vert_idx = 0u; vert_idx < mesh_block.x.size(); ++vert_idx) {
      // only needed if color information was originally missing
      mesh_block.r.resize(mesh_block.x.size());
      mesh_block.g.resize(mesh_block.x.size());
      mesh_block.b.resize(mesh_block.x.size());

      const IntensityVoxel* voxel =
          intensity_layer.getVoxelPtrByCoordinates(Point(
              mesh_block.x[vert_idx], mesh_block.y[vert_idx],
              mesh_block.z[vert_idx]));
      if (voxel != nullptr && voxel->weight > 0.0) {
        float intensity = voxel->intensity;
        Color new_color = color_map->colorLookup(intensity);
        mesh_block.r[vert_idx] = new_color.r;
        mesh_block.g[vert_idx] = new_color.g;
        mesh_block.b[vert_idx] = new_color.b;
      }
    }
  }
}

}  // namespace voxfield

#endif  // VOXFIELD_ROS_INTENSITY_VIS_H_
