cmake_minimum_required(VERSION 2.8.3)
project(traj_planner)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(OpenMP REQUIRED)
find_package(common REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin_simple REQUIRED)
find_package(ompl REQUIRED)
# find_package(SUITESPARSE REQUIRED)
# find_package(G2O REQUIRED)


set(THIRD_PARTY_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
    cv_bridge
    decomp_ros_utils
    # costmap_2d
    # base_local_planner
    # move_base_msgs
    # nav_core
    # pluginlib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${DECOMP_UTIL_INCLUDE_DIRS}
    ${THIRD_PARTY_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    

)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
    proto/minco_config.proto
)


add_message_files(
  FILES
  DataDisp.msg
  PolyTraj.msg
  )
  

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

#if this catkin packge's header is used by other packages, use catkin_package to
#declare the include directories of this package.
catkin_package(
    INCLUDE_DIRS include ${THIRD_PARTY_INCLUDE_DIRS} 
    ${CMAKE_CURRENT_BINARY_DIR} 
    CATKIN_DEPENDS decomp_ros_utils
    LIBRARIES traj_planner traj_server_ros 
)

add_library(traj_planner
    src/traj_optimizer.cpp
    src/traj_manager.cpp
    src/map_adapter.cpp
    src/kino_astar.cpp
    ${PROTO_SRCS}
)
target_link_libraries(traj_planner PRIVATE
    hkust_pl_common
    hkust_pl_smm
    ompl
    "${OpenMP_CXX_FLAGS}"
    ${catkin_LIBRARIES}
)

target_compile_options(traj_planner PRIVATE "${OpenMP_CXX_FLAGS}")

add_library(traj_server_ros
    src/traj_server_ros.cpp
    src/traj_visualizer.cpp
)
target_link_libraries(traj_server_ros PRIVATE
    traj_planner
    hkust_smm_ros
    ${OpenMP_CXX_FLAGS}
)