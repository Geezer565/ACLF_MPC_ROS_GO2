cmake_minimum_required(VERSION 3.0.2)
project(ocs2_legged_robot_adaptive_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roslib
  cmake_modules
  tf
  urdf
  kdl_parser
  robot_state_publisher
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_assets
  ocs2_msgs
  ocs2_ros_interfaces
  ocs2_legged_robot
  ocs2_legged_robot_ros
  ocs2_legged_robot_adaptive
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
    pinocchio
)

include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

add_executable(legged_robot_adaptive_ddp_mpc
  src/LeggedRobotAdaptiveDdpMpcNode.cpp
)
add_dependencies(legged_robot_adaptive_ddp_mpc
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_adaptive_ddp_mpc
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_options(legged_robot_adaptive_ddp_mpc PRIVATE ${OCS2_CXX_FLAGS})

install(
  TARGETS
    legged_robot_adaptive_ddp_mpc
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
