cmake_minimum_required(VERSION 3.8)
project(voxfield)

# Match the ROS 1 catkin workspace's usual default: unlike catkin, plain
# ament_cmake leaves CMAKE_BUILD_TYPE unset (no -DNDEBUG, no optimization)
# unless colcon/the user sets one. Default to Release so glog's DCHECK()s
# compile out as they do in a normal Release build -- see
# docs/ROS2_PORT_NOTES.md for a DCHECK that fires in unoptimized builds.
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
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)

############
# PROTOBUF #
############
# Invoke protoc directly (rather than FindProtobuf's protobuf_generate())
# so we control the -I root: protobuf_generate() always tries
# "-I ${CMAKE_CURRENT_SOURCE_DIR}" (i.e. voxfield/) before any IMPORT_DIRS,
# and since that's also a valid prefix of proto/voxfield/*.proto, protoc
# picks it first and names the message "proto.voxfield.Block", generating
# "proto/voxfield/Block.pb.h" -- not the "voxfield/Block.pb.h" the plan
# calls for. Passing only "-I proto" ourselves makes protoc treat
# "voxfield/Block.proto" as the canonical path, so the generated header is
# "voxfield/Block.pb.h" and PROTO_GEN_DIR can be used as-is for both the
# BUILD_INTERFACE include dir and the install destination. See
# docs/ROS2_PORT_NOTES.md.
set(PROTO_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto_gen)
file(MAKE_DIRECTORY ${PROTO_GEN_DIR}/voxfield)

set(PROTO_SRCS)
set(PROTO_HDRS)
foreach(_proto_name Block Layer)
  set(_proto_src ${CMAKE_CURRENT_SOURCE_DIR}/proto/voxfield/${_proto_name}.proto)
  set(_pb_cc ${PROTO_GEN_DIR}/voxfield/${_proto_name}.pb.cc)
  set(_pb_h ${PROTO_GEN_DIR}/voxfield/${_proto_name}.pb.h)
  add_custom_command(
    OUTPUT ${_pb_cc} ${_pb_h}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --cpp_out=${PROTO_GEN_DIR}
         -I ${CMAKE_CURRENT_SOURCE_DIR}/proto
         ${_proto_src}
    DEPENDS ${_proto_src}
    COMMENT "Running cpp protocol buffer compiler on proto/voxfield/${_proto_name}.proto"
    VERBATIM
  )
  list(APPEND PROTO_SRCS ${_pb_cc})
  list(APPEND PROTO_HDRS ${_pb_h})
endforeach()

add_library(voxfield_proto SHARED ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(voxfield_proto PUBLIC
  $<BUILD_INTERFACE:${PROTO_GEN_DIR}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(voxfield_proto PUBLIC protobuf::libprotobuf)

####################
# SET SOURCE FILES #
####################
set(VOXFIELD_SRCS
  src/alignment/icp.cc
  src/core/block.cc
  src/core/esdf_map.cc
  src/core/tsdf_map.cc
  src/integrator/esdf_integrator.cc
  src/integrator/esdf_occ_integrator.cc
  src/integrator/integrator_utils.cc
  src/integrator/intensity_integrator.cc
  src/integrator/tsdf_integrator.cc
  src/integrator/np_tsdf_integrator.cc
  src/integrator/esdf_voxfield_integrator.cc
  src/integrator/esdf_occ_fiesta_integrator.cc
  src/integrator/esdf_occ_edt_integrator.cc
  src/io/mesh_ply.cc
  src/io/sdf_ply.cc
  src/mesh/marching_cubes.cc
  src/simulation/objects.cc
  src/simulation/simulation_world.cc
  src/utils/camera_model.cc
  src/utils/evaluation_utils.cc
  src/utils/layer_utils.cc
  src/utils/neighbor_tools.cc
  src/utils/protobuf_utils.cc
  src/utils/timing.cc
  src/utils/voxel_utils.cc
  src/utils/neighbor_tools_ex.cc
)

#############
# LIBRARIES #
#############
add_library(voxfield SHARED ${VOXFIELD_SRCS})
target_include_directories(voxfield PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/minkindr/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(voxfield PUBLIC
  voxfield_proto
  Eigen3::Eigen
  glog::glog
  ${GFLAGS_LIBRARIES}
)

############
# BINARIES #
############
add_executable(tsdf_to_esdf test/tsdf_to_esdf.cc)
target_link_libraries(tsdf_to_esdf voxfield)

add_executable(test_load_esdf test/test_load_esdf.cc)
target_link_libraries(test_load_esdf voxfield)

install(TARGETS tsdf_to_esdf test_load_esdf
  DESTINATION lib/${PROJECT_NAME}
)

#########
# TESTS #
#########
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(EIGEN_CHECKS_SHIM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test/include)

  set(VOXFIELD_TEST_NAMES
    test_approx_hash_array
    test_tsdf_map
    test_protobuf
    test_tsdf_interpolator
    test_layer
    test_merge_integration
    test_layer_utils
    test_sdf_integrators
    test_bucket_queue
    test_clear_spheres
  )
  foreach(test_name ${VOXFIELD_TEST_NAMES})
    ament_add_gtest(${test_name} test/${test_name}.cc
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
    target_include_directories(${test_name} PRIVATE ${EIGEN_CHECKS_SHIM_DIR})
    target_link_libraries(${test_name} voxfield)
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
  DIRECTORY third_party/minkindr/include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY ${PROTO_GEN_DIR}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.pb.h"
)

install(
  TARGETS voxfield voxfield_proto
  EXPORT voxfieldTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(voxfieldTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 Protobuf glog gflags)
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
