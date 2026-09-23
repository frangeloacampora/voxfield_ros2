#ifndef VOXFIELD_ROS_NODE_MAIN_H_
#define VOXFIELD_ROS_NODE_MAIN_H_

// Shared main()-startup helper (ROS2_PORT_PLAN.md D16). ROS 2 appends
// "--ros-args ..." to argv, which gflags would abort on as an unknown flag,
// so every server main strips the ROS args before handing argv to gflags.
// Also installs the glog failure signal handler and replaces the old
// `args="-alsologtostderr"` launch-file convention with a default flag
// value (still overridable on the command line).

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace voxfield {

// Call once at the very top of main(), after rclcpp::init(argc, argv).
inline void initGflagsAndGlog(int argc, char** argv) {
  std::vector<std::string> non_ros_args =
      rclcpp::remove_ros_arguments(argc, argv);

  std::vector<char*> nargv;
  nargv.reserve(non_ros_args.size());
  for (std::string& arg : non_ros_args) {
    nargv.push_back(&arg[0]);
  }
  int nargc = static_cast<int>(nargv.size());
  char** nargv_data = nargv.empty() ? nullptr : nargv.data();

  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::ParseCommandLineFlags(&nargc, &nargv_data, /*remove_flags=*/true);
  google::InstallFailureSignalHandler();
}

}  // namespace voxfield

#endif  // VOXFIELD_ROS_NODE_MAIN_H_
