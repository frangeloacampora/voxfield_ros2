#ifndef VOXFIELD_UTILS_PLANNING_UTILS_H_
#define VOXFIELD_UTILS_PLANNING_UTILS_H_

${1}voxfield/core/layer.h"
${1}voxfield/core/voxel.h"

namespace voxfield {

namespace utils {

/// Gets the indices of all points within the sphere.
template <typename VoxelType>
void getSphereAroundPoint(
    const Layer<VoxelType>& layer, const Point& center, FloatingPoint radius,
    HierarchicalIndexMap* block_voxel_list);

/**
 * Gets the indices of all points around a sphere, and also allocates any
 * blocks that don't already exist.
 */
template <typename VoxelType>
void getAndAllocateSphereAroundPoint(
    const Point& center, FloatingPoint radius, Layer<VoxelType>* layer,
    HierarchicalIndexMap* block_voxel_list);

/**
 * Tools for manually editing a set of voxels. Sets the values around a sphere
 * to be artifically free or occupied, and marks them as hallucinated.
 */
template <typename VoxelType>
void fillSphereAroundPoint(
    const Point& center, const FloatingPoint radius,
    const FloatingPoint max_distance_m, Layer<VoxelType>* layer);
template <typename VoxelType>
void clearSphereAroundPoint(
    const Point& center, const FloatingPoint radius,
    const FloatingPoint max_distance_m, Layer<VoxelType>* layer);

/**
 * Utility function to get map bounds from an arbitrary layer.
 * Only accurate to block level (i.e., outer bounds of allocated blocks).
 */
template <typename VoxelType>
void computeMapBoundsFromLayer(
    const voxfield::Layer<VoxelType>& layer, Eigen::Vector3d* lower_bound,
    Eigen::Vector3d* upper_bound);

}  // namespace utils
}  // namespace voxfield

${1}voxfield/utils/planning_utils_inl.h"

#endif  // VOXFIELD_UTILS_PLANNING_UTILS_H_
