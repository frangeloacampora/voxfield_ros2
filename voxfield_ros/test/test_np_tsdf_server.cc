// Regression tests for NpTsdfServer's range-image preprocessing
// (docs/ROS2_PORT_NOTES.md "Known upstream issues" #1, #2, #3).
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "voxfield_ros/np_tsdf_server.h"

namespace voxfield {
namespace {

std::unique_ptr<NpTsdfServer> makeCameraServer(
    const std::string& name, int width, int height, double fx, double fy,
    double vx, double vy, double smooth_thre_ratio = 1.0) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("sensor_is_lidar", false),
      rclcpp::Parameter("width", width),
      rclcpp::Parameter("height", height),
      rclcpp::Parameter("fx", fx),
      rclcpp::Parameter("fy", fy),
      rclcpp::Parameter("vx", vx),
      rclcpp::Parameter("vy", vy),
      rclcpp::Parameter("smooth_thre_ratio", smooth_thre_ratio),
      // No timers needed for these tests.
      rclcpp::Parameter("update_mesh_every_n_sec", 0.0),
      rclcpp::Parameter("publish_map_every_n_sec", 0.0),
  });
  auto node = std::make_shared<rclcpp::Node>(name, options);
  return std::make_unique<NpTsdfServer>(node);
}

struct RangeImage {
  cv::Mat vertex_map;
  cv::Mat depth_image;
  cv::Mat color_image;
};

RangeImage project(
    const NpTsdfServer& server, int width, int height, const Pointcloud& pts) {
  RangeImage img;
  img.vertex_map = cv::Mat::zeros(height, width, CV_32FC3);
  img.depth_image = cv::Mat(img.vertex_map.size(), CV_32FC1, -1.0);
  img.color_image = cv::Mat::zeros(img.vertex_map.size(), CV_8UC3);
  const Colors colors(pts.size(), Color(10, 20, 30));
  constexpr float kMinZ = -1000.0f;
  constexpr float kMinDist = 0.1f;
  server.projectPointCloudToImage(
      pts, colors, img.vertex_map, img.depth_image, img.color_image, kMinZ,
      kMinDist);
  return img;
}

// ---- #2: projectPointToImageCamera() returned bool into a float depth ----

TEST(NpTsdfServerCamera, ProjectionReturnsRangeAndRejectsInvalidPoints) {
  auto server = makeCameraServer("proj_range", 640, 480, 500, 500, 320, 240);
  int u = -1, v = -1;
  const Point p(0.3f, -0.4f, 1.2f);
  EXPECT_NEAR(server->projectPointToImageCamera(p, &u, &v), p.norm(), 1e-6);
  EXPECT_EQ(u, static_cast<int>(std::round(0.3 * 500 / 1.2 + 320)));
  EXPECT_EQ(v, static_cast<int>(std::round(-0.4 * 500 / 1.2 + 240)));

  // Outside the image, behind the camera, on the image plane, non-finite.
  EXPECT_LT(server->projectPointToImageCamera(Point(5, 0, 1), &u, &v), 0);
  EXPECT_LT(server->projectPointToImageCamera(Point(0, 0, -1), &u, &v), 0);
  EXPECT_LT(server->projectPointToImageCamera(Point(0, 0, 0), &u, &v), 0);
  EXPECT_LT(
      server->projectPointToImageCamera(Point(NAN, NAN, NAN), &u, &v), 0);
  EXPECT_LT(server->projectPointToImageCamera(Point(NAN, 0, 1), &u, &v), 0);
}

// std::round() rounds -0.5 away from zero (to -1, not 0), unlike every
// other integer boundary in this image, so a point that projects to
// exactly column/row -0.5 must be rejected, not rounded into column/row
// -1 (an out-of-bounds vertex_map/depth_image access one row before the
// first element).
TEST(NpTsdfServerCamera, ExactNegativeHalfBoundaryIsRejected) {
  auto server = makeCameraServer("proj_boundary", 10, 10, 100, 100, -0.5, -0.5);
  int u = -1, v = -1;
  EXPECT_LT(server->projectPointToImageCamera(Point(0, 0, 1), &u, &v), 0);
}

TEST(NpTsdfServerCamera, NearestPointPerPixelWins) {
  constexpr int kW = 640, kH = 480;
  auto server = makeCameraServer("nearest_wins", kW, kH, 500, 500, 320, 240);
  const Point far(0.2f, 0.1f, 4.0f);
  const Point near = far * 0.25f;  // same ray -> same pixel
  int u_far, v_far, u_near, v_near;
  ASSERT_GT(server->projectPointToImageCamera(far, &u_far, &v_far), 0);
  ASSERT_GT(server->projectPointToImageCamera(near, &u_near, &v_near), 0);
  ASSERT_EQ(u_far, u_near);
  ASSERT_EQ(v_far, v_near);

  // Nearest must win regardless of input order (upstream kept the first).
  for (const Pointcloud& pts :
       {Pointcloud{far, near}, Pointcloud{near, far}}) {
    const RangeImage img = project(*server, kW, kH, pts);
    EXPECT_NEAR(img.depth_image.at<float>(v_near, u_near), near.norm(), 1e-6);
    const cv::Vec3f vertex = img.vertex_map.at<cv::Vec3f>(v_near, u_near);
    EXPECT_NEAR(vertex[2], near.z(), 1e-6);

    const Pointcloud extracted =
        server->extractPointCloud(img.vertex_map, img.depth_image);
    ASSERT_EQ(extracted.size(), 1u);
    EXPECT_NEAR((extracted[0] - near).norm(), 0.0, 1e-6);
  }
}

TEST(NpTsdfServerCamera, PointsBehindCameraAreDropped) {
  constexpr int kW = 640, kH = 480;
  auto server = makeCameraServer("behind_dropped", kW, kH, 500, 500, 320, 240);
  // Upstream projected (x, y, -z) through the pinhole model into the
  // mirrored pixel and kept it.
  const RangeImage img = project(*server, kW, kH, {Point(0.1f, 0.1f, -1.0f)});
  EXPECT_TRUE(
      server->extractPointCloud(img.vertex_map, img.depth_image).empty());
}

// ---- #3: camera intrinsics were `int`, truncating calibrations ----

TEST(NpTsdfServerCamera, NonIntegerIntrinsicsAreNotTruncated) {
  // u = round(x * fx / z + vx) = round(200.6 + 100.6) = 301; truncated
  // intrinsics (200, 100) would give 300. Same for v with fy/vy.
  auto server =
      makeCameraServer("float_intrinsics", 640, 480, 200.6, 150.7, 100.6, 50.7);
  int u, v;
  ASSERT_GT(server->projectPointToImageCamera(Point(1, 1, 1), &u, &v), 0);
  EXPECT_EQ(u, 301);
  EXPECT_EQ(v, 201);  // round(150.7 + 50.7) = 201; truncated -> 200
}

TEST(NpTsdfServerCamera, IntegerIntrinsicsStillAccepted) {
  // The shipped calib YAMLs write intrinsics as YAML integers (fx: 580).
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("sensor_is_lidar", false),
      rclcpp::Parameter("width", 640), rclcpp::Parameter("height", 480),
      rclcpp::Parameter("fx", 200), rclcpp::Parameter("fy", 200),
      rclcpp::Parameter("vx", 100), rclcpp::Parameter("vy", 50),
      rclcpp::Parameter("update_mesh_every_n_sec", 0.0),
      rclcpp::Parameter("publish_map_every_n_sec", 0.0),
  });
  NpTsdfServer server(std::make_shared<rclcpp::Node>("int_intrinsics", options));
  int u, v;
  ASSERT_GT(server.projectPointToImageCamera(Point(1, 1, 1), &u, &v), 0);
  EXPECT_EQ(u, 300);
  EXPECT_EQ(v, 250);
}

// ---- #1: computeNormalImage() read one row out of bounds ----

// A fronto-parallel plane z = 2 filling every pixel of a small image: every
// normal, including the last row and last column, must be +z.
TEST(NpTsdfServerCamera, PlaneNormalsIncludingLastRowAndColumn) {
  constexpr int kW = 8, kH = 6;
  constexpr double kF = 10.0, kCx = 4.0, kCy = 3.0;
  auto server = makeCameraServer("plane_normals", kW, kH, kF, kF, kCx, kCy);
  Pointcloud pts;
  constexpr float kZ = 2.0f;
  for (int v = 0; v < kH; ++v) {
    for (int u = 0; u < kW; ++u) {
      pts.emplace_back((u - kCx) * kZ / kF, (v - kCy) * kZ / kF, kZ);
    }
  }
  const RangeImage img = project(*server, kW, kH, pts);
  for (int v = 0; v < kH; ++v) {
    for (int u = 0; u < kW; ++u) {
      ASSERT_GT(img.depth_image.at<float>(v, u), 0) << u << "," << v;
    }
  }
  const cv::Mat normals =
      server->computeNormalImage(img.vertex_map, img.depth_image);
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
TEST(NpTsdfServerCamera, NoNormalAcrossDepthDiscontinuity) {
  constexpr int kW = 8, kH = 6;
  constexpr double kF = 10.0, kCx = 4.0, kCy = 3.0;
  auto server =
      makeCameraServer("discontinuity", kW, kH, kF, kF, kCx, kCy, 0.05);
  Pointcloud pts;
  for (int v = 0; v < kH; ++v) {
    for (int u = 0; u < kW; ++u) {
      const float z = u < kW / 2 ? 1.0f : 3.0f;
      pts.emplace_back((u - kCx) * z / kF, (v - kCy) * z / kF, z);
    }
  }
  const RangeImage img = project(*server, kW, kH, pts);
  const cv::Mat normals =
      server->computeNormalImage(img.vertex_map, img.depth_image);
  for (int v = 0; v < kH; ++v) {
    // Column kW/2 - 1 sits on the near side of the step: its x-neighbor is
    // on the far plane, so it must get no normal.
    const cv::Vec3f edge = normals.at<cv::Vec3f>(v, kW / 2 - 1);
    EXPECT_EQ(cv::norm(edge), 0.0) << "row " << v;
    // Interior pixels still get the plane normal.
    EXPECT_NEAR(normals.at<cv::Vec3f>(v, 0)[2], 1.0f, 1e-5) << "row " << v;
  }
}

}  // namespace
}  // namespace voxfield

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
