#ifndef VOXFIELD_UTILS_VOXEL_UTILS_H_
#define VOXFIELD_UTILS_VOXEL_UTILS_H_

#include "voxfield/core/color.h"
#include "voxfield/core/common.h"
#include "voxfield/core/voxel.h"

namespace voxfield {
template <typename VoxelType>
void mergeVoxelAIntoVoxelB(const VoxelType& voxel_A, VoxelType* voxel_B);

template <>
void mergeVoxelAIntoVoxelB(const TsdfVoxel& voxel_A, TsdfVoxel* voxel_B);

template <>
void mergeVoxelAIntoVoxelB(const EsdfVoxel& voxel_A, EsdfVoxel* voxel_B);

template <>
void mergeVoxelAIntoVoxelB(
    const OccupancyVoxel& voxel_A, OccupancyVoxel* voxel_B);

}  // namespace voxfield

#endif  // VOXFIELD_UTILS_VOXEL_UTILS_H_
