#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>


double positionError(
    const geometry_msgs::msg::Pose& target,
    const geometry_msgs::msg::Pose& actual)
{
  const double dx = target.position.x - actual.position.x;
  const double dy = target.position.y - actual.position.y;
  const double dz = target.position.z - actual.position.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}


double orientationError(
    const geometry_msgs::msg::Pose& target,
    const geometry_msgs::msg::Pose& actual)
{
  const auto& a = target.orientation;
  const auto& b = actual.orientation;

  double dot =
      a.x*b.x +
      a.y*b.y +
      a.z*b.z +
      a.w*b.w;

  dot = std::abs(dot);
  dot = std::clamp(dot, 0.0, 1.0);

  return 2.0 * std::acos(dot);
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "cartesian_approach",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  const auto logger = node->get_logger();

  constexpr const char* GROUP = "panda_arm";
  constexpr const char* EE_LINK = "panda_link8";

  moveit::planning_interface::MoveGroupInterface move_group(
      node, GROUP);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  move_group.setStartStateToCurrentState();

  move_group.setMaxVelocityScalingFactor(0.20);
  move_group.setMaxAccelerationScalingFactor(0.20);

  move_group.setPoseReferenceFrame(
      move_group.getPlanningFrame());

  const auto start =
      move_group.getCurrentPose(EE_LINK);

  geometry_msgs::msg::Pose target =
      start.pose;

  // Straight-line Cartesian approach:
  // +5 cm in panda_link0 X.
  target.position.z -= 0.10;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target);

  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::MoveItErrorCodes error_code;

  constexpr double EEF_STEP = 0.005;  // 5 mm sampling

  RCLCPP_INFO(
      logger,
      "Planning frame: %s",
      move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(
      logger,
      "START xyz: [%.6f %.6f %.6f]",
      start.pose.position.x,
      start.pose.position.y,
      start.pose.position.z);

  RCLCPP_INFO(
      logger,
      "TARGET xyz: [%.6f %.6f %.6f]",
      target.position.x,
      target.position.y,
      target.position.z);

  const double fraction =
      move_group.computeCartesianPath(
          waypoints,
          EEF_STEP,
          trajectory,
          true,
          &error_code);

  RCLCPP_INFO(
      logger,
      "Cartesian path fraction: %.4f",
      fraction);

  RCLCPP_INFO(
      logger,
      "Trajectory points: %zu",
      trajectory.joint_trajectory.points.size());

  if (fraction < 0.999)
  {
    RCLCPP_ERROR(
        logger,
        "FULL CARTESIAN PATH NOT FOUND");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();

    return 1;
  }

  if (trajectory.joint_trajectory.points.empty())
  {
    RCLCPP_ERROR(
        logger,
        "Cartesian trajectory is empty");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();

    return 1;
  }

  RCLCPP_INFO(
      logger,
      "CARTESIAN PATH GENERATION: PASS");

  const auto execution_result =
      move_group.execute(trajectory);

  if (execution_result !=
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
        logger,
        "CARTESIAN EXECUTION: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();

    return 1;
  }

  RCLCPP_INFO(
      logger,
      "CARTESIAN EXECUTION: PASS");

  std::this_thread::sleep_for(
      std::chrono::milliseconds(1000));

  const auto final_pose =
      move_group.getCurrentPose(EE_LINK);

  const double pos_error =
      positionError(
          target,
          final_pose.pose);

  const double ori_error =
      orientationError(
          target,
          final_pose.pose);

  RCLCPP_INFO(
      logger,
      "FINAL xyz: [%.6f %.6f %.6f]",
      final_pose.pose.position.x,
      final_pose.pose.position.y,
      final_pose.pose.position.z);

  RCLCPP_INFO(
      logger,
      "POSITION ERROR: %.6f m",
      pos_error);

  RCLCPP_INFO(
      logger,
      "ORIENTATION ERROR: %.6f rad",
      ori_error);

  const bool pass =
      pos_error <= 0.005 &&
      ori_error <= 0.01;

  if (pass)
  {
    RCLCPP_INFO(
        logger,
        "D2.2 CARTESIAN APPROACH: PASS");
  }
  else
  {
    RCLCPP_ERROR(
        logger,
        "D2.2 CARTESIAN APPROACH: FAIL");
  }

  executor.cancel();

  if (spinner.joinable())
    spinner.join();

  rclcpp::shutdown();

  return pass ? 0 : 1;
}
