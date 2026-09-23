#ifndef VOXFIELD_SIMULATION_SIMULATION_WORLD_INL_H_
#define VOXFIELD_SIMULATION_SIMULATION_WORLD_INL_H_

#include <algorithm>
#include <iostream>
#include <memory>

#include "voxfield/core/block.h"
#include "voxfield/utils/timing.h"

namespace voxfield {

template <typename VoxelType>
void SimulationWorld::generateSdfFromWorld(
    FloatingPoint max_dist, Layer<VoxelType>* layer) const {
  timing::Timer sim_timer("sim/generate_sdf");

  CHECK_NOTNULL(layer);
  // Iterate over every voxel in the layer and compute its distance to all
  // objects.

  // Get all blocks within bounds. For now, only respect bounds approximately:
  // that is, up to block boundaries.
  FloatingPoint block_size = layer->block_size();
  FloatingPoint half_block_size = block_size / 2.0;

  BlockIndexList blocks;
  for (FloatingPoint x = min_bound_.x() - half_block_size;
       x <= max_bound_.x() + half_block_size; x += block_size) {
    for (FloatingPoint y = min_bound_.y() - half_block_size;
         y <= max_bound_.y() + half_block_size; y += block_size) {
      for (FloatingPoint z = min_bound_.z() - half_block_size;
           z <= max_bound_.z() + half_block_size; z += block_size) {
        blocks.push_back(
            layer->computeBlockIndexFromCoordinates(Point(x, y, z)));
      }
    }
  }

  // Iterate over all blocks filling this stuff in.
  for (const BlockIndex& block_index : blocks) {
    typename Block<VoxelType>::Ptr block =
        layer->allocateBlockPtrByIndex(block_index);
    for (size_t i = 0; i < block->num_voxels(); ++i) {
      VoxelType& voxel = block->getVoxelByLinearIndex(i);
      Point coords = block->computeCoordinatesFromLinearIndex(i);
      // Check that it's in bounds, otherwise skip it.
      if (!(coords.x() >= min_bound_.x() && coords.x() <= max_bound_.x() &&
            coords.y() >= min_bound_.y() && coords.y() <= max_bound_.y() &&
            coords.z() >= min_bound_.z() && coords.z() <= max_bound_.z())) {
        continue;
      }

      // Iterate over all objects and get distances to this thing.
      FloatingPoint voxel_dist = max_dist;
      Color color;
      for (const std::unique_ptr<Object>& object : objects_) {
        FloatingPoint object_dist = object->getDistanceToPoint(coords);
        if (object_dist < voxel_dist) {
          voxel_dist = object_dist;
          color = object->getColor();
        }
      }

      // Then update the thing.
      voxel_dist = std::max(voxel_dist, -max_dist);
      setVoxel(voxel_dist, color, &voxel);
    }
  }
}

// NOTE(ROS2 port): these full specializations are defined in this header,
// so they need `inline` -- without it, any two translation units that both
// include this header (directly or transitively) and get linked into the
// same binary hit an ODR "multiple definition" link error. This was latent
// since Phase 3 (nothing had linked two such TUs together until Phase 7's
// simulation_eval, which links simulation_server.cc's object code together
// with simulation_eval.cc, both of which include this header).
template <>
inline void SimulationWorld::setVoxel(
    FloatingPoint dist, const Color& color, TsdfVoxel* voxel) const {
  voxel->distance = static_cast<float>(dist);
  voxel->color = color;
  voxel->weight = 1.0f;  // Just to make sure it gets visualized/meshed/etc.
}

// Color ignored.
template <>
inline void SimulationWorld::setVoxel(
    FloatingPoint dist, const Color& /*color*/, EsdfVoxel* voxel) const {
  voxel->distance = static_cast<float>(dist);
  voxel->observed = true;
}

}  // namespace voxfield

#endif  // VOXFIELD_SIMULATION_SIMULATION_WORLD_INL_H_
