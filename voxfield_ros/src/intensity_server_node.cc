#include "voxfield_ros/intensity_server.h"
#include "voxfield_ros/node_main.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  voxfield::initGflagsAndGlog(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("intensity");
  voxfield::IntensityServer server(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
