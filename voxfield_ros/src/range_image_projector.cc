#include "voxfield_ros/range_image_projector.h"

#include <cmath>
#include <string>

namespace voxfield {

bool RangeImageProjector::Config::isValid(std::string* why) const {
  if (width <= 0 || height <= 0) {
    if (why != nullptr) {
      *why = "width and height must both be positive (got width=" +
          std::to_string(width) + ", height=" + std::to_string(height) + ")";
    }
    return false;
  }
  if (sensor_is_lidar) {
    if (fov_up == fov_down) {
      if (why != nullptr) {
        *why = "fov_up and fov_down must differ for a LiDAR model (got " +
            std::to_string(fov_up) + ")";
      }
      return false;
    }
  } else {
    if (!(fx > 0.f) || !(fy > 0.f)) {
      if (why != nullptr) {
        *why =
            "fx and fy must both be positive for a camera model (got fx=" +
            std::to_string(fx) + ", fy=" + std::to_string(fy) + ")";
      }
      return false;
    }
  }
  return true;
}

RangeImageProjector::RangeImageProjector(const Config& config)
    : config_(config) {
  if (config_.sensor_is_lidar) {
    const float fov = std::abs(config_.fov_down) + std::abs(config_.fov_up);
    fov_down_rad_ = config_.fov_down / 180.0f * M_PI;
    fov_rad_ = fov / 180.0f * M_PI;
  }
}

void RangeImageProjector::process(
    const Pointcloud& points_in, const Colors& colors_in,
    Pointcloud* points_out, Pointcloud* normals_out,
    Colors* colors_out) const {
  CHECK_NOTNULL(points_out);
  CHECK_NOTNULL(normals_out);
  CHECK_NOTNULL(colors_out);

  cv::Mat vertex_map =
      cv::Mat::zeros(config_.height, config_.width, CV_32FC3);
  cv::Mat depth_image(vertex_map.size(), CV_32FC1, -1.0);
  cv::Mat color_image = cv::Mat::zeros(vertex_map.size(), CV_8UC3);
  projectPointCloudToImage(
      points_in, colors_in, vertex_map, depth_image, color_image,
      config_.min_z, config_.min_dist);
  const cv::Mat normal_image = computeNormalImage(vertex_map, depth_image);

  *points_out = extractPointCloud(vertex_map, depth_image);
  *normals_out = extractNormals(normal_image, depth_image);
  *colors_out = extractColors(color_image, depth_image);
}

bool RangeImageProjector::projectPointCloudToImage(
    const Pointcloud& points_C, const Colors& colors,
    cv::Mat& vertex_map,   // corresponding point // NOLINT
    cv::Mat& depth_image,  // Float depth image (CV_32FC1). // NOLINT
    cv::Mat& color_image, float min_z, float min_d) const {
  // TODO(py): consider to calculate in parallel to speed up
  for (size_t i = 0; i < points_C.size(); i++) {
    int u, v;
    float depth;
    if (config_.sensor_is_lidar)
      depth = projectPointToImageLiDAR(points_C[i], &u, &v);
    else
      depth = projectPointToImageCamera(points_C[i], &u, &v);
    if (depth > min_d && points_C[i].z() > min_z) {
      float old_depth = depth_image.at<float>(v, u);
      // save only nearest point for each pixel
      if (old_depth <= 0.0 || old_depth > depth) {
        for (int k = 0; k <= 2; k++) {
          vertex_map.at<cv::Vec3f>(v, u)[k] = points_C[i](k);
        }
        depth_image.at<float>(v, u) = depth;
        // BGR default order
        color_image.at<cv::Vec3b>(v, u)[0] = colors[i].b;
        color_image.at<cv::Vec3b>(v, u)[1] = colors[i].g;
        color_image.at<cv::Vec3b>(v, u)[2] = colors[i].r;
      }
    }
  }
  return false;
}

// point should be in the LiDAR's coordinate system
float RangeImageProjector::projectPointToImageLiDAR(
    const Point& p_C, int* u, int* v) const {
  // All values are ceiled and floored to guarantee that the resulting points
  // will be valid for any integer conversion.
  float depth =
      std::sqrt(p_C.x() * p_C.x() + p_C.y() * p_C.y() + p_C.z() * p_C.z());
  float yaw = std::atan2(p_C.y(), p_C.x());
  float pitch = std::asin(p_C.z() / depth);
  // projections in image coordinates (percentage)
  float proj_x = 0.5 * (yaw / M_PI + 1.0);
  float proj_y = 1.0 - (pitch - fov_down_rad_) / fov_rad_;
  // scale to image size
  proj_x *= config_.width;
  proj_y *= config_.height;
  // round for integer index
  CHECK_NOTNULL(u);
  *u = std::round(proj_x);
  if (*u == config_.width)
    *u = 0;

  CHECK_NOTNULL(v);
  *v = std::round(proj_y);
  if (std::ceil(proj_y) > config_.height - 1 || std::floor(proj_y) < 0) {
    return (-1.0);
  }
  return depth;
}

// ROS2_PORT deviation (docs/ROS2_PORT_NOTES.md "Known upstream issues" #2):
// upstream returned `bool` (in bounds or not) into the caller's `float
// depth`, so every in-image camera point got depth 1.0 and
// projectPointCloudToImage() kept the *first* point per pixel instead of the
// nearest one, and computeNormalImage()'s depth-discontinuity check never
// fired. This now returns the real range, like projectPointToImageLiDAR(),
// and -1 (always rejected by the caller's `depth > min_d`) for points that
// don't project into the image. Points behind the camera (z <= 0) and
// non-finite points are rejected too: upstream projected them through the
// pinhole model anyway (mirrored into the image, or an undefined
// float -> int conversion).
float RangeImageProjector::projectPointToImageCamera(
    const Point& p_C, int* u, int* v) const {
  CHECK_NOTNULL(u);
  CHECK_NOTNULL(v);
  if (!(p_C.z() > 0.0f)) {  // also catches NaN
    return -1.0f;
  }
  const float proj_u = p_C.x() * config_.fx / p_C.z() + config_.vx;
  const float proj_v = p_C.y() * config_.fy / p_C.z() + config_.vy;
  // std::round() rounds halfway cases away from zero, so round(-0.5) == -1,
  // not 0: the lower bound below must be an open one (unlike every other
  // integer boundary, which rounds towards +infinity on a tie). Checked on
  // the float before converting so out-of-range or non-finite values never
  // reach the float -> int conversion.
  if (!(proj_u > -0.5f && proj_u < config_.width - 0.5f) ||
      !(proj_v > -0.5f && proj_v < config_.height - 0.5f)) {
    return -1.0f;
  }
  *u = static_cast<int>(std::round(proj_u));
  *v = static_cast<int>(std::round(proj_v));
  return p_C.norm();
}

cv::Mat RangeImageProjector::computeNormalImage(
    const cv::Mat& vertex_map, const cv::Mat& depth_image) const {
  cv::Mat normal_image(depth_image.size(), CV_32FC3, 0.0);
  // Every normal needs a neighboring column and row.
  if (config_.width < 2 || config_.height < 2) {
    return normal_image;
  }
  for (int u = 0; u < config_.width; u++) {
    for (int v = 0; v < config_.height; v++) {
      Point p;
      p << vertex_map.at<cv::Vec3f>(v, u)[0], vertex_map.at<cv::Vec3f>(v, u)[1],
          vertex_map.at<cv::Vec3f>(v, u)[2];

      float d_p = depth_image.at<float>(v, u);
      // sign of the normal vector
      float sign = 1.0;

      if (d_p > 0) {
        // neighbor x: the next column. A LiDAR range image is a 360-degree
        // ring, so its last column wraps to column 0. A camera image does
        // not wrap: use the previous column and flip the normal's sign
        // instead (ROS2_PORT deviation, see the row case below).
        int n_x_u;
        if (u == config_.width - 1) {
          if (config_.sensor_is_lidar) {
            n_x_u = 0;
          } else {
            n_x_u = u - 1;
            sign *= -1.0;
          }
        } else {
          n_x_u = u + 1;
        }
        Point n_x;
        n_x << vertex_map.at<cv::Vec3f>(v, n_x_u)[0],
            vertex_map.at<cv::Vec3f>(v, n_x_u)[1],
            vertex_map.at<cv::Vec3f>(v, n_x_u)[2];
        float d_n_x = depth_image.at<float>(v, n_x_u);
        if (d_n_x < 0)
          continue;
        // on the boundary, not continous
        if (std::abs(d_n_x - d_p) > config_.smooth_thre_ratio * d_p)
          continue;

        // neighbor y: the next row; the last row uses the previous row and
        // flips the normal's sign (dy then points the other way).
        // ROS2_PORT deviation (docs/ROS2_PORT_NOTES.md "Known upstream
        // issues" #1): upstream tested `v == height_`, which can never be
        // true inside `for (v = 0; v < height_; ...)`, so the last row read
        // one row past the end of vertex_map/depth_image.
        int n_y_v;
        if (v == config_.height - 1) {
          n_y_v = v - 1;
          sign *= -1.0;
        } else {
          n_y_v = v + 1;
        }
        Point n_y;
        n_y << vertex_map.at<cv::Vec3f>(n_y_v, u)[0],
            vertex_map.at<cv::Vec3f>(n_y_v, u)[1],
            vertex_map.at<cv::Vec3f>(n_y_v, u)[2];

        float d_n_y = depth_image.at<float>(n_y_v, u);
        if (d_n_y < 0)
          continue;
        // on the boundary, not continous
        if (std::abs(d_n_y - d_p) > config_.smooth_thre_ratio * d_p)
          continue;
        Point dx = n_x - p;
        Point dy = n_y - p;

        Point normal = (dx.cross(dy)).normalized() * sign;
        cv::Vec3f& normals = normal_image.at<cv::Vec3f>(v, u);
        for (int k = 0; k <= 2; k++)
          normals[k] = normal(k);
      }
    }
  }
  return normal_image;
}

Pointcloud RangeImageProjector::extractPointCloud(
    const cv::Mat& vertex_map, const cv::Mat& depth_image) const {
  Pointcloud points_C;
  for (int v = 0; v < vertex_map.rows; v++) {
    for (int u = 0; u < vertex_map.cols; u++) {
      cv::Vec3f vertex = vertex_map.at<cv::Vec3f>(v, u);
      if (depth_image.at<float>(v, u) > 0) {
        Point p_C(vertex[0], vertex[1], vertex[2]);
        points_C.push_back(p_C);
      }
    }
  }
  return points_C;
}

Colors RangeImageProjector::extractColors(
    const cv::Mat& color_image, const cv::Mat& depth_image) const {
  Colors colors;
  for (int v = 0; v < color_image.rows; v++) {
    for (int u = 0; u < color_image.cols; u++) {
      // BGR
      cv::Vec3b color = color_image.at<cv::Vec3b>(v, u);
      if (depth_image.at<float>(v, u) > 0) {
        // RGB
        Color c_C(color[2], color[1], color[0]);
        colors.push_back(c_C);
      }
    }
  }
  return colors;
}

Pointcloud RangeImageProjector::extractNormals(
    const cv::Mat& normal_image, const cv::Mat& depth_image) const {
  Pointcloud normals_C;
  for (int v = 0; v < normal_image.rows; v++) {
    for (int u = 0; u < normal_image.cols; u++) {
      cv::Vec3f vertex = normal_image.at<cv::Vec3f>(v, u);
      if (depth_image.at<float>(v, u) > 0) {
        Ray n_C(vertex[0], vertex[1], vertex[2]);
        normals_C.push_back(n_C);
      }
    }
  }
  return normals_C;
}

}  // namespace voxfield
