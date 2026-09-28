#include "voxfield_ros/node_main.h"
#include "voxfield_ros/tsdf_server.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  voxfield::initGflagsAndGlog(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("tsdf");
  voxfield::TsdfServer server(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
