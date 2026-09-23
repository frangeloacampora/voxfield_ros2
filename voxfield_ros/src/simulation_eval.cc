#include "voxfield_ros/node_main.h"
#include "voxfield_ros/simulation_server.h"

namespace voxfield {
class SimulationServerImpl : public voxfield::SimulationServer {
 public:
  explicit SimulationServerImpl(rclcpp::Node::SharedPtr node)
      : SimulationServer(node) {}

  void prepareWorld() {
    CHECK_NOTNULL(world_);
    world_->addObject(
        std::unique_ptr<Object>(
            new Sphere(Point(0.0, 0.0, 2.0), 2.0, Color::Red())));

    world_->addObject(
        std::unique_ptr<Object>(new PlaneObject(
            Point(-2.0, -4.0, 2.0), Point(0, 1, 0), Color::White())));

    world_->addObject(
        std::unique_ptr<Object>(new PlaneObject(
            Point(4.0, 0.0, 0.0), Point(-1, 0, 0), Color::Pink())));

    world_->addObject(
        std::unique_ptr<Object>(
            new Cube(Point(-4.0, 4.0, 2.0), Point(4, 4, 4), Color::Green())));

    world_->addGroundLevel(0.03);

    world_->generateSdfFromWorld(truncation_distance_, tsdf_gt_.get());
    world_->generateSdfFromWorld(esdf_max_distance_, esdf_gt_.get());
  }
};

}  // namespace voxfield

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  voxfield::initGflagsAndGlog(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("voxblox_sim");
  voxfield::SimulationServerImpl sim_eval(node);

  sim_eval.run();

  RCLCPP_INFO(node->get_logger(), "Done.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
