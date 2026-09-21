cmake_minimum_required(VERSION 3.22)
project(franka_motion_app)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(
  cartesian_pose_goal
  src/cartesian_pose_goal.cpp
)

ament_target_dependencies(
  cartesian_pose_goal
  rclcpp
  geometry_msgs
  tf2
  moveit_ros_planning_interface
)

add_executable(
  reachability_sweep
  src/reachability_sweep.cpp
)

ament_target_dependencies(
  reachability_sweep
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(
  collision_aware_pose
  src/collision_aware_pose.cpp
)

ament_target_dependencies(
  collision_aware_pose
  rclcpp
  geometry_msgs
  moveit_core
  moveit_msgs
  shape_msgs
  moveit_ros_planning_interface
)

add_executable(
  cartesian_approach
  src/cartesian_approach.cpp
)

ament_target_dependencies(
  cartesian_approach
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

add_executable(
  gripper_cycle
  src/gripper_cycle.cpp
)

ament_target_dependencies(
  gripper_cycle
  rclcpp
  moveit_core
  moveit_ros_planning_interface
)

add_executable(
  spawn_cube
  src/spawn_cube.cpp
)

ament_target_dependencies(
  spawn_cube
  rclcpp
  moveit_msgs
  shape_msgs
  moveit_ros_planning_interface
)

add_executable(
  grasp_attach
  src/grasp_attach.cpp
)

ament_target_dependencies(
  grasp_attach
  rclcpp
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
)

add_executable(
  grasp_approach
  src/grasp_approach.cpp
)

ament_target_dependencies(
  grasp_approach
  rclcpp
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
)

add_executable(
  lift_attached
  src/lift_attached.cpp
)

ament_target_dependencies(
  lift_attached
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

add_executable(
  day2_cleanup
  src/day2_cleanup.cpp
)

ament_target_dependencies(
  day2_cleanup
  rclcpp
  moveit_msgs
  moveit_ros_planning_interface
)

install(
  TARGETS
    cartesian_pose_goal
    reachability_sweep
    collision_aware_pose
    cartesian_approach
    gripper_cycle
    spawn_cube
    grasp_attach
    grasp_approach
    lift_attached
    day2_cleanup
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
