cmake_minimum_required(VERSION 3.8)
project(voxfield_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Block.msg"
  "msg/Layer.msg"
  "msg/LayerWithTrajectory.msg"
  "msg/Mesh.msg"
  "msg/MeshBlock.msg"
  "msg/MultiMesh.msg"
  "msg/VoxelEvaluationDetails.msg"
  "srv/FilePath.srv"
  DEPENDENCIES std_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
