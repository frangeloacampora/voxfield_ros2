#include "voxfield_ros/np_tsdf_server.h"

#include "voxfield_ros/node_main.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  voxfield::initGflagsAndGlog(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("voxfield");
  voxfield::NpTsdfServer server(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
