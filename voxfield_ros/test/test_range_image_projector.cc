// Direct unit tests for RangeImageProjector, extracted from NpTsdfServer
// (MULTI_SENSOR_PLAN.md M8/Phase 2). Ports test_np_tsdf_server.cc's
// camera-model regression cases 1:1 (same assertions; no ROS node needed
// here since RangeImageProjector has no ROS dependency), plus a new LiDAR
// round-trip case (Phase 2 step 4).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/core.hpp>

#include "voxfield_ros/range_image_projector.h"

namespace voxfield {
namespace {

RangeImageProjector makeCameraProjector(
    int width, int height, double fx, double fy, double vx, double vy,
    double smooth_thre_ratio = 1.0) {
  RangeImageProjector::Config config;
  config.sensor_is_lidar = false;
  config.width = width;
  config.height = height;
  config.fx = fx;
  config.fy = fy;
  config.vx = vx;
  config.vy = vy;
  config.smooth_thre_ratio = smooth_thre_ratio;
  return RangeImageProjector(config);
}

RangeImageProjector makeLidarProjector(
    int width, int height, double fov_up_deg, double fov_down_deg) {
  RangeImageProjector::Config config;
  config.sensor_is_lidar = true;
  config.width = width;
  config.height = height;
  config.fov_up = fov_up_deg;
  config.fov_down = fov_down_deg;
  return RangeImageProjector(config);
}

struct RangeImage {
  cv::Mat vertex_map;
  cv::Mat depth_image;
  cv::Mat color_image;
};

RangeImage project(
    const RangeImageProjector& projector, int width, int height,
    const Pointcloud& pts) {
  RangeImage img;
  img.vertex_map = cv::Mat::zeros(height, width, CV_32FC3);
  img.depth_image = cv::Mat(img.vertex_map.size(), CV_32FC1, -1.0);
  img.color_image = cv::Mat::zeros(img.vertex_map.size(), CV_8UC3);
  const Colors colors(pts.size(), Color(10, 20, 30));
  constexpr float kMinZ = -1000.0f;
  constexpr float kMinDist = 0.1f;
  projector.projectPointCloudToImage(
      pts, colors, img.vertex_map, img.depth_image, img.color_image, kMinZ,
      kMinDist);
  return img;
}

// ---- #2: projectPointToImageCamera() returned bool into a float depth ----

TEST(
    RangeImageProjectorCamera, ProjectionReturnsRangeAndRejectsInvalidPoints) {
  const RangeImageProjector projector =
      makeCameraProjector(640, 480, 500, 500, 320, 240);
  int u = -1, v = -1;
  const Point p(0.3f, -0.4f, 1.2f);
  EXPECT_NEAR(projector.projectPointToImageCamera(p, &u, &v), p.norm(), 1e-6);
  EXPECT_EQ(u, static_cast<int>(std::round(0.3 * 500 / 1.2 + 320)));
  EXPECT_EQ(v, static_cast<int>(std::round(-0.4 * 500 / 1.2 + 240)));

  // Outside the image, behind the camera, on the image plane, non-finite.
  EXPECT_LT(projector.projectPointToImageCamera(Point(5, 0, 1), &u, &v), 0);
  EXPECT_LT(projector.projectPointToImageCamera(Point(0, 0, -1), &u, &v), 0);
  EXPECT_LT(projector.projectPointToImageCamera(Point(0, 0, 0), &u, &v), 0);
  EXPECT_LT(
      projector.projectPointToImageCamera(Point(NAN, NAN, NAN), &u, &v), 0);
  EXPECT_LT(projector.projectPointToImageCamera(Point(NAN, 0, 1), &u, &v), 0);
}

// std::round() rounds -0.5 away from zero (to -1, not 0), unlike every
// other integer boundary in this image, so a point that projects to
// exactly column/row -0.5 must be rejected, not rounded into column/row
// -1 (an out-of-bounds vertex_map/depth_image access one row before the
// first element).
TEST(RangeImageProjectorCamera, ExactNegativeHalfBoundaryIsRejected) {
  const RangeImageProjector projector =
      makeCameraProjector(10, 10, 100, 100, -0.5, -0.5);
  int u = -1, v = -1;
  EXPECT_LT(projector.projectPointToImageCamera(Point(0, 0, 1), &u, &v), 0);
}

TEST(RangeImageProjectorCamera, NearestPointPerPixelWins) {
  constexpr int kW = 640, kH = 480;
  const RangeImageProjector projector =
      makeCameraProjector(kW, kH, 500, 500, 320, 240);
  const Point far(0.2f, 0.1f, 4.0f);
  const Point near = far * 0.25f;  // same ray -> same pixel
  int u_far, v_far, u_near, v_near;
  ASSERT_GT(projector.projectPointToImageCamera(far, &u_far, &v_far), 0);
  ASSERT_GT(projector.projectPointToImageCamera(near, &u_near, &v_near), 0);
  ASSERT_EQ(u_far, u_near);
  ASSERT_EQ(v_far, v_near);

  // Nearest must win regardless of input order (upstream kept the first).
  for (const Pointcloud& pts :
       {Pointcloud{far, near}, Pointcloud{near, far}}) {
    const RangeImage img = project(projector, kW, kH, pts);
    EXPECT_NEAR(img.depth_image.at<float>(v_near, u_near), near.norm(), 1e-6);
    const cv::Vec3f vertex = img.vertex_map.at<cv::Vec3f>(v_near, u_near);
    EXPECT_NEAR(vertex[2], near.z(), 1e-6);

    const Pointcloud extracted =
        projector.extractPointCloud(img.vertex_map, img.depth_image);
    ASSERT_EQ(extracted.size(), 1u);
    EXPECT_NEAR((extracted[0] - near).norm(), 0.0, 1e-6);
  }
}

TEST(RangeImageProjectorCamera, PointsBehindCameraAreDropped) {
  constexpr int kW = 640, kH = 480;
  const RangeImageProjector projector =
      makeCameraProjector(kW, kH, 500, 500, 320, 240);
  const RangeImage img =
      project(projector, kW, kH, {Point(0.1f, 0.1f, -1.0f)});
  EXPECT_TRUE(
      projector.extractPointCloud(img.vertex_map, img.depth_image).empty());
}

// ---- #3: camera intrinsics were `int`, truncating calibrations ----

TEST(RangeImageProjectorCamera, NonIntegerIntrinsicsAreNotTruncated) {
  // u = round(x * fx / z + vx) = round(200.6 + 100.6) = 301; truncated
  // intrinsics (200, 100) would give 300. Same for v with fy/vy.
  const RangeImageProjector projector =
      makeCameraProjector(640, 480, 200.6, 150.7, 100.6, 50.7);
  int u, v;
  ASSERT_GT(projector.projectPointToImageCamera(Point(1, 1, 1), &u, &v), 0);
  EXPECT_EQ(u, 301);
  EXPECT_EQ(v, 201);  // round(150.7 + 50.7) = 201; truncated -> 200
}

TEST(RangeImageProjectorCamera, IntegerIntrinsicsStillAccepted) {
  // The shipped calib YAMLs write intrinsics as YAML integers (fx: 580).
  const RangeImageProjector projector =
      makeCameraProjector(640, 480, 200, 200, 100, 50);
  int u, v;
  ASSERT_GT(projector.projectPointToImageCamera(Point(1, 1, 1), &u, &v), 0);
  EXPECT_EQ(u, 300);
  EXPECT_EQ(v, 250);
}

// ---- #1: computeNormalImage() read one row out of bounds ----

// A fronto-parallel plane z = 2 filling every pixel of a small image: every
// normal, including the last row and last column, must be +z.
TEST(RangeImageProjectorCamera, PlaneNormalsIncludingLastRowAndColumn) {
  constexpr int kW = 8, kH = 6;
  constexpr double kF = 10.0, kCx = 4.0, kCy = 3.0;
  const RangeImageProjector projector = makeCameraProjector(kW, kH, kF, kF, kCx, kCy);
  Pointcloud pts;
  constexpr float kZ = 2.0f;
  for (int v = 0; v < kH; ++v) {
    for (int u = 0; u < kW; ++u) {
      pts.emplace_back((u - kCx) * kZ / kF, (v - kCy) * kZ / kF, kZ);
    }
  }
  const RangeImage img = project(projector, kW, kH, pts);
  for (int v = 0; v < kH; ++v) {
    for (int u = 0; u < kW; ++u) {
      ASSERT_GT(img.depth_image.at<float>(v, u), 0) << u << "," << v;
    }
  }
  const cv::Mat normals =
      projector.computeNormalImage(img.vertex_map, img.depth_image);
  for (int v = 0; v < kH; ++v) {
    for (int u = 0; u < kW; ++u) {
      const cv::Vec3f n = normals.at<cv::Vec3f>(v, u);
      EXPECT_NEAR(n[0], 0.0f, 1e-5) << "pixel (" << u << "," << v << ")";
      EXPECT_NEAR(n[1], 0.0f, 1e-5) << "pixel (" << u << "," << v << ")";
      EXPECT_NEAR(n[2], 1.0f, 1e-5) << "pixel (" << u << "," << v << ")";
    }
  }
}

// With real depths (#2), the smoothness check now rejects normals across a
// depth discontinuity instead of blending the two surfaces.
TEST(RangeImageProjectorCamera, NoNormalAcrossDepthDiscontinuity) {
  constexpr int kW = 8, kH = 6;
  constexpr double kF = 10.0, kCx = 4.0, kCy = 3.0;
  const RangeImageProjector projector =
      makeCameraProjector(kW, kH, kF, kF, kCx, kCy, 0.05);
  Pointcloud pts;
  for (int v = 0; v < kH; ++v) {
    for (int u = 0; u < kW; ++u) {
      const float z = u < kW / 2 ? 1.0f : 3.0f;
      pts.emplace_back((u - kCx) * z / kF, (v - kCy) * z / kF, z);
    }
  }
  const RangeImage img = project(projector, kW, kH, pts);
  const cv::Mat normals =
      projector.computeNormalImage(img.vertex_map, img.depth_image);
  for (int v = 0; v < kH; ++v) {
    // Column kW/2 - 1 sits on the near side of the step: its x-neighbor is
    // on the far plane, so it must get no normal.
    const cv::Vec3f edge = normals.at<cv::Vec3f>(v, kW / 2 - 1);
    EXPECT_EQ(cv::norm(edge), 0.0) << "row " << v;
    // Interior pixels still get the plane normal.
    EXPECT_NEAR(normals.at<cv::Vec3f>(v, 0)[2], 1.0f, 1e-5) << "row " << v;
  }
}

// ---- New (Phase 2 step 4): LiDAR round-trip ----

// Points sampled exactly on the LiDAR range image's own (row, col) grid, on
// a cylinder around the sensor origin, so every pixel gets exactly one
// point and the round trip is lossless up to projection/back-projection
// rounding. The per-(row, col) angle formulas below are the exact inverse
// of RangeImageProjector::projectPointToImageLiDAR()'s own
// yaw/pitch -> proj_x/proj_y mapping (not, e.g., a plain linspace over
// [fov_up, fov_down]), so that projecting each point lands back on the same
// pixel it was generated from.
TEST(RangeImageProjectorLidar, CylinderRoundTrip) {
  constexpr int kWidth = 360, kHeight = 8;
  constexpr double kFovUp = 10.0, kFovDown = -10.0;
  constexpr double kRadius = 5.0;
  const RangeImageProjector projector =
      makeLidarProjector(kWidth, kHeight, kFovUp, kFovDown);

  const double fov = std::abs(kFovDown) + std::abs(kFovUp);
  const double fov_down_rad = kFovDown * M_PI / 180.0;
  const double fov_rad = fov * M_PI / 180.0;

  Pointcloud pts;
  for (int row = 0; row < kHeight; ++row) {
    const double pitch =
        fov_down_rad + fov_rad * (1.0 - static_cast<double>(row) / kHeight);
    const double cos_p = std::cos(pitch);
    const double sin_p = std::sin(pitch);
    for (int col = 0; col < kWidth; ++col) {
      const double yaw = M_PI * (2.0 * col / kWidth - 1.0);
      pts.emplace_back(
          kRadius * cos_p * std::cos(yaw), kRadius * cos_p * std::sin(yaw),
          kRadius * sin_p);
    }
  }

  const RangeImage img = project(projector, kWidth, kHeight, pts);

  const Pointcloud extracted =
      projector.extractPointCloud(img.vertex_map, img.depth_image);
  ASSERT_EQ(extracted.size(), pts.size());

  // Match by nearest neighbor (pixel rounding can permute array order):
  // every input point must have a corresponding extracted point at (nearly)
  // the same location.
  for (const Point& p : pts) {
    double best = std::numeric_limits<double>::infinity();
    for (const Point& q : extracted) {
      best = std::min(best, static_cast<double>((p - q).norm()));
    }
    EXPECT_LT(best, 1e-5);
  }
}

}  // namespace
}  // namespace voxfield
