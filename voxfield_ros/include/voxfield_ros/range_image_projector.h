#ifndef VOXFIELD_ROS_RANGE_IMAGE_PROJECTOR_H_
#define VOXFIELD_ROS_RANGE_IMAGE_PROJECTOR_H_

#include <opencv2/core/mat.hpp>
#include <string>
#include <voxfield/core/common.h>

namespace voxfield {

// Projects an unorganized point cloud onto a spherical (LiDAR) or pinhole
// (camera) range image and back, for non-projective ("NP") TSDF
// integration: computes per-pixel normals and returns the reprojected
// points/normals/colors (nearest point per pixel; pixels with no valid
// neighbor dropped).
//
// Extracted verbatim (MULTI_SENSOR_PLAN.md M8, Phase 2) from what used to be
// NpTsdfServer's own methods, so each sensor in multi-sensor mode can later
// own its own projection model; behavior is unchanged from before the
// extraction.
class RangeImageProjector {
 public:
  struct Config {
    bool sensor_is_lidar = false;
    int width = 0;
    int height = 0;

    // LiDAR model (degrees).
    float fov_up = 0.f;
    float fov_down = 0.f;

    // Camera model (pixels).
    float vx = 0.f;
    float vy = 0.f;
    float fx = 0.f;
    float fy = 0.f;

    float smooth_thre_ratio = 1.0f;

    // Noise filter, used by process() only (the individual methods below
    // take min_z/min_d explicitly instead).
    float min_z = -1000.0f;
    float min_dist = 0.1f;

    // width/height must both be positive; a camera model needs fx, fy > 0;
    // a LiDAR model needs fov_up != fov_down. On failure, if `why` is not
    // null it's set to a message naming the problem.
    bool isValid(std::string* why) const;
  };

  explicit RangeImageProjector(const Config& config);

  const Config& config() const {
    return config_;
  }

  // Full preprocessing pipeline used by
  // NpTsdfServer::processPointCloudMessageAndInsert(): project points_in
  // onto the range image using config_.min_z/min_dist, compute normals, and
  // extract the reprojected points/normals/colors back out.
  void process(
      const Pointcloud& points_in, const Colors& colors_in,
      Pointcloud* points_out, Pointcloud* normals_out,
      Colors* colors_out) const;

  // Below: verbatim from the original NpTsdfServer methods, unchanged
  // behavior. Kept individually callable (not just through process())
  // because test_range_image_projector.cc and NpTsdfServer's compatibility
  // wrappers call them directly, with explicit min_z/min_d independent of
  // config_'s.
  bool projectPointCloudToImage(
      const Pointcloud& points_C, const Colors& colors,
      cv::Mat& vertex_map,   // NOLINT
      cv::Mat& depth_image,  // NOLINT
      cv::Mat& color_image,  // NOLINT
      float min_z,           // NOLINT
      float min_d) const;    // NOLINT
  // point should be in the sensor's coordinate system
  float projectPointToImageLiDAR(const Point& p_C, int* u, int* v) const;
  // Returns the point's range (Euclidean distance from the sensor origin,
  // same convention as projectPointToImageLiDAR()) if it projects inside the
  // image, or -1 if it does not (behind the camera, non-finite, or outside
  // the image bounds).
  float projectPointToImageCamera(const Point& p_C, int* u, int* v) const;
  cv::Mat computeNormalImage(
      const cv::Mat& vertex_map, const cv::Mat& depth_image) const;
  Pointcloud extractPointCloud(
      const cv::Mat& vertex_map,
      const cv::Mat& depth_image) const;  // NOLINT
  Pointcloud extractNormals(
      const cv::Mat& normal_image,
      const cv::Mat& depth_image) const;  // NOLINT
  Colors extractColors(
      const cv::Mat& color_image,
      const cv::Mat& depth_image) const;  // NOLINT

 private:
  Config config_;
  // Precomputed in the constructor exactly as
  // NpTsdfServer::getServerConfigFromRosParam() used to (LiDAR only; left at
  // 0 for a camera model, matching the original uninitialized-but-unused
  // behavior since projectPointToImageCamera() never reads them).
  float fov_down_rad_ = 0.f;
  float fov_rad_ = 0.f;
};

}  // namespace voxfield

#endif  // VOXFIELD_ROS_RANGE_IMAGE_PROJECTOR_H_
