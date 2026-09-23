cmake_minimum_required(VERSION 3.8)
project(voxfield_rviz_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(voxfield REQUIRED)
find_package(voxfield_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Qt's MOC generation happens automatically for headers with Q_OBJECT that
# are listed as target sources.
set(CMAKE_AUTOMOC ON)

# Avoid Qt signals and slots defining "emit", "slots", etc.
add_definitions(-DQT_NO_KEYWORDS)

set(HEADER_FILES
  include/voxfield_rviz_plugin/voxfield_mesh_display.h
  include/voxfield_rviz_plugin/voxfield_multi_mesh_display.h
  include/voxfield_rviz_plugin/voxfield_mesh_visual.h
  include/voxfield_rviz_plugin/material_loader.h
)

set(SRC_FILES
  src/voxfield_mesh_display.cc
  src/voxfield_multi_mesh_display.cc
  src/voxfield_mesh_visual.cc
  src/material_loader.cc
)

add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
  ${HEADER_FILES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  rviz_common
  rviz_rendering
  pluginlib
  ament_index_cpp
  voxfield_msgs
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  voxfield::voxfield
  rviz_ogre_vendor::OgreMain
  Qt5::Widgets
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

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
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons
)
install(
  DIRECTORY content/
  DESTINATION share/${PROJECT_NAME}/content
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  pluginlib
  ament_index_cpp
  voxfield
  voxfield_msgs
  Qt5
)
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
