cmake_minimum_required(VERSION 3.8)
project(voxfield_ros)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(voxfield REQUIRED)
find_package(voxfield_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)

set(VOXFIELD_ROS_DEPENDENCIES
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  nav_msgs
  visualization_msgs
  pcl_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
  cv_bridge
  interactive_markers
  ament_index_cpp
  voxfield_msgs
)

#############
# LIBRARIES #
#############
# Phase 5: infra only (param_utils.h, kindr_conversions.h, ros_params.h,
# conversions.h, the *_vis.h headers, node_main.h are header-only; the two
# infra .cc files below are the only sources so far). Server sources are
# added in Phases 6-7.
add_library(${PROJECT_NAME}
  src/interactive_slider.cc
  src/transformer.cc
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${VOXFIELD_ROS_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} PUBLIC
  voxfield::voxfield
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  glog::glog
  ${GFLAGS_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

#########
# TESTS #
#########
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Both test files define their own main() (test_param_utils needs
  # rclcpp::init/shutdown around the tests), so skip gtest_main.
  ament_add_gtest(test_param_utils test/test_param_utils.cc
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_param_utils ${PROJECT_NAME} gtest)

  ament_add_gtest(test_kindr_conversions test/test_kindr_conversions.cc
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_kindr_conversions ${PROJECT_NAME} gtest)
endif()

##########
# EXPORT #
##########
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${VOXFIELD_ROS_DEPENDENCIES} voxfield PCL OpenCV glog gflags)
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
