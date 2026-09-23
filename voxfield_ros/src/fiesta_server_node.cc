#include "voxfield_ros/fiesta_server.h"
#include "voxfield_ros/node_main.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  voxfield::initGflagsAndGlog(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("fiesta");
  voxfield::FiestaServer server(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
