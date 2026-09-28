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
# Phase 5: infra (param_utils.h, kindr_conversions.h, ros_params.h,
# conversions.h, the *_vis.h headers, node_main.h are header-only).
# Phase 6: TsdfServer + NpTsdfServer. Remaining derived servers/tools are
# added in Phase 7.
add_library(${PROJECT_NAME}
  src/interactive_slider.cc
  src/transformer.cc
  src/range_image_projector.cc
  src/sensor_config_loader.cc
  src/tsdf_server.cc
  src/np_tsdf_server.cc
  src/voxfield_server.cc
  src/voxblox_server.cc
  src/fiesta_server.cc
  src/voxedt_server.cc
  src/intensity_server.cc
  src/simulation_server.cc
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

##############
# EXECUTABLES #
##############
# Phase 6 + 7 mains.
add_executable(tsdf_server src/tsdf_server_node.cc)
target_link_libraries(tsdf_server ${PROJECT_NAME})

add_executable(np_tsdf_server src/np_tsdf_server_node.cc)
target_link_libraries(np_tsdf_server ${PROJECT_NAME})

add_executable(voxfield_server src/voxfield_server_node.cc)
target_link_libraries(voxfield_server ${PROJECT_NAME})

add_executable(voxblox_server src/voxblox_server_node.cc)
target_link_libraries(voxblox_server ${PROJECT_NAME})

add_executable(fiesta_server src/fiesta_server_node.cc)
target_link_libraries(fiesta_server ${PROJECT_NAME})

add_executable(voxedt_server src/voxedt_server_node.cc)
target_link_libraries(voxedt_server ${PROJECT_NAME})

add_executable(intensity_server src/intensity_server_node.cc)
target_link_libraries(intensity_server ${PROJECT_NAME})

add_executable(simulation_eval src/simulation_eval.cc)
target_link_libraries(simulation_eval ${PROJECT_NAME})

add_executable(voxblox_eval src/voxblox_eval.cc)
target_link_libraries(voxblox_eval ${PROJECT_NAME})

add_executable(visualize_tsdf src/visualize_tsdf.cc)
target_link_libraries(visualize_tsdf ${PROJECT_NAME})

install(
  TARGETS
    tsdf_server
    np_tsdf_server
    voxfield_server
    voxblox_server
    fiesta_server
    voxedt_server
    intensity_server
    simulation_eval
    voxblox_eval
    visualize_tsdf
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

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
  target_compile_definitions(test_param_utils PRIVATE
    VOXFIELD_ROS_CFG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/cfg")

  ament_add_gtest(test_kindr_conversions test/test_kindr_conversions.cc
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_kindr_conversions ${PROJECT_NAME} gtest)

  # Transformer multi-sensor support (MULTI_SENSOR_PLAN.md Phase 3, M5/M6).
  ament_add_gtest(test_transformer_multi test/test_transformer_multi.cc
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_transformer_multi ${PROJECT_NAME} gtest)

  # Sensor config loading and validation (MULTI_SENSOR_PLAN.md Phase 4,
  # M2/M13).
  ament_add_gtest(test_sensor_config test/test_sensor_config.cc
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_sensor_config ${PROJECT_NAME} gtest)

  # TsdfServer / NpTsdfServer multi-sensor frontends (MULTI_SENSOR_PLAN.md
  # Phase 5/6). Same TsdfServer/NpTsdfServer-can't-share-a-translation-unit
  # split as test_server_map_io and test_legacy_golden above.
  ament_add_gtest(test_multi_sensor_server test/test_multi_sensor_server.cc
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_multi_sensor_server ${PROJECT_NAME} gtest)

  ament_add_gtest(
    test_multi_sensor_server_np_tsdf test/test_multi_sensor_server.cc
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(
    test_multi_sensor_server_np_tsdf ${PROJECT_NAME} gtest)
  target_compile_definitions(
    test_multi_sensor_server_np_tsdf PRIVATE TEST_NP_TSDF_SERVER)

  # NpTsdfServer range-image preprocessing regressions ("Known upstream
  # issues" #1-#3 in docs/ROS2_PORT_NOTES.md).
  ament_add_gtest(test_np_tsdf_server test/test_np_tsdf_server.cc
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_np_tsdf_server ${PROJECT_NAME} gtest)

  # Direct RangeImageProjector unit tests (MULTI_SENSOR_PLAN.md M8/Phase 2).
  # No ROS node needed, so this links the default gtest_main.
  ament_add_gtest(
    test_range_image_projector test/test_range_image_projector.cc)
  target_link_libraries(test_range_image_projector ${PROJECT_NAME} gtest)

  # save_map/load_map/save_esdf_map file handling ("Known upstream issues"
  # #11).
  foreach(_map_io_server IN ITEMS voxfield voxblox fiesta voxedt)
    ament_add_gtest(test_${_map_io_server}_server_map_io
      test/test_server_map_io.cc
      SKIP_LINKING_MAIN_LIBRARIES)
    target_link_libraries(test_${_map_io_server}_server_map_io
      ${PROJECT_NAME} gtest)
  endforeach()
  target_compile_definitions(test_voxblox_server_map_io PRIVATE
    TEST_VOXBLOX_SERVER)
  target_compile_definitions(test_fiesta_server_map_io PRIVATE
    TEST_FIESTA_SERVER)
  target_compile_definitions(test_voxedt_server_map_io PRIVATE
    TEST_VOXEDT_SERVER)

  # Golden regression tests pinning today's (legacy, single-sensor) TSDF
  # integration output, added before the multi-sensor refactor begins
  # (MULTI_SENSOR_PLAN.md Phase 1). Same TsdfServer/NpTsdfServer-can't-share-
  # a-translation-unit split as test_server_map_io above.
  ament_add_gtest(test_legacy_golden_tsdf test/test_legacy_golden.cc
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_legacy_golden_tsdf ${PROJECT_NAME} gtest)
  target_compile_definitions(test_legacy_golden_tsdf PRIVATE
    VOXFIELD_ROS_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/test_data")

  ament_add_gtest(test_legacy_golden_np_tsdf test/test_legacy_golden.cc
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_legacy_golden_np_tsdf ${PROJECT_NAME} gtest)
  target_compile_definitions(test_legacy_golden_np_tsdf PRIVATE
    TEST_NP_TSDF_SERVER
    VOXFIELD_ROS_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/test_data")

  # Dataset-free smoke test (plan §6 Phase 12 step 1): one launch_test per
  # server executable, sharing test/test_smoke.launch.py via the `method`
  # launch argument. Every method gets the full mesh/slice/save+load check
  # (see the launch file's own docstring).
  find_package(launch_testing_ament_cmake REQUIRED)
  foreach(_smoke_method IN ITEMS voxfield np_tsdf voxblox fiesta voxedt)
    add_launch_test(test/test_smoke.launch.py
      TARGET test_smoke_${_smoke_method}
      ARGS "method:=${_smoke_method}"
      TIMEOUT 60
    )
  endforeach()

  # Dataset-free multi-sensor smoke test (MULTI_SENSOR_PLAN.md §8 Phase 8
  # step 2): one launch_test per integration path -- voxfield (projective/
  # NP) and voxblox (non-projective/ray-casting).
  foreach(_multi_smoke_method IN ITEMS voxfield voxblox)
    add_launch_test(test/test_multi_sensor_smoke.launch.py
      TARGET test_multi_sensor_smoke_${_multi_smoke_method}
      ARGS "method:=${_multi_smoke_method}"
      TIMEOUT 60
    )
  endforeach()
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
install(
  DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "bak" EXCLUDE
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${VOXFIELD_ROS_DEPENDENCIES} voxfield PCL OpenCV glog gflags)
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
