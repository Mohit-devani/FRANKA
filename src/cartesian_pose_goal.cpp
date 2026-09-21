#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

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

  const double norm_a =
      std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w);

  const double norm_b =
      std::sqrt(b.x*b.x + b.y*b.y + b.z*b.z + b.w*b.w);

  if (norm_a < 1e-12 || norm_b < 1e-12)
    return M_PI;

  double dot =
      (a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w) /
      (norm_a * norm_b);

  dot = std::abs(dot);
  dot = std::clamp(dot, 0.0, 1.0);

  return 2.0 * std::acos(dot);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<rclcpp::Node>(
      "cartesian_pose_goal",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  const auto logger = node->get_logger();

  static const std::string PLANNING_GROUP = "panda_arm";
  static const std::string END_EFFECTOR_LINK = "panda_link8";

  constexpr double PLANNING_POSITION_TOLERANCE = 0.0025;
  constexpr double PLANNING_ORIENTATION_TOLERANCE = 0.005;

  constexpr double VALIDATION_POSITION_TOLERANCE = 0.005;
  constexpr double VALIDATION_ORIENTATION_TOLERANCE = 0.01;

  constexpr double DEG_TO_RAD =
      3.14159265358979323846 / 180.0;

  std::string mode = "relative";

  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;

  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double yaw_deg = 0.0;

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;

  node->get_parameter_or("mode", mode, mode);

  node->get_parameter_or("dx", dx, dx);
  node->get_parameter_or("dy", dy, dy);
  node->get_parameter_or("dz", dz, dz);

  node->get_parameter_or("roll_deg", roll_deg, roll_deg);
  node->get_parameter_or("pitch_deg", pitch_deg, pitch_deg);
  node->get_parameter_or("yaw_deg", yaw_deg, yaw_deg);

  node->get_parameter_or("x", x, x);
  node->get_parameter_or("y", y, y);
  node->get_parameter_or("z", z, z);

  node->get_parameter_or("qx", qx, qx);
  node->get_parameter_or("qy", qy, qy);
  node->get_parameter_or("qz", qz, qz);
  node->get_parameter_or("qw", qw, qw);

  moveit::planning_interface::MoveGroupInterface move_group(
      node, PLANNING_GROUP);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  move_group.setStartStateToCurrentState();
  move_group.setPlanningTime(5.0);

  move_group.setMaxVelocityScalingFactor(0.20);
  move_group.setMaxAccelerationScalingFactor(0.20);

  move_group.setGoalPositionTolerance(
      PLANNING_POSITION_TOLERANCE);

  move_group.setGoalOrientationTolerance(
      PLANNING_ORIENTATION_TOLERANCE);

  RCLCPP_INFO(
      logger,
      "Planning frame: %s",
      move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(
      logger,
      "End-effector link: %s",
      END_EFFECTOR_LINK.c_str());

  const auto start =
      move_group.getCurrentPose(END_EFFECTOR_LINK);

  geometry_msgs::msg::Pose target;

  if (mode == "relative")
  {
    target = start.pose;

    target.position.x += dx;
    target.position.y += dy;
    target.position.z += dz;

    tf2::Quaternion q_current(
        start.pose.orientation.x,
        start.pose.orientation.y,
        start.pose.orientation.z,
        start.pose.orientation.w);

    tf2::Quaternion q_delta;

    q_delta.setRPY(
        roll_deg * DEG_TO_RAD,
        pitch_deg * DEG_TO_RAD,
        yaw_deg * DEG_TO_RAD);

    tf2::Quaternion q_target =
        q_current * q_delta;

    q_target.normalize();

    target.orientation.x = q_target.x();
    target.orientation.y = q_target.y();
    target.orientation.z = q_target.z();
    target.orientation.w = q_target.w();

    RCLCPP_INFO(logger, "MODE: relative");

    RCLCPP_INFO(
        logger,
        "COMMAND delta xyz: [%.4f, %.4f, %.4f] m",
        dx, dy, dz);

    RCLCPP_INFO(
        logger,
        "COMMAND local delta RPY: [%.2f, %.2f, %.2f] deg",
        roll_deg, pitch_deg, yaw_deg);
  }
  else if (mode == "absolute")
  {
    target.position.x = x;
    target.position.y = y;
    target.position.z = z;

    tf2::Quaternion q_target(qx, qy, qz, qw);

    if (q_target.length2() < 1e-12)
    {
      RCLCPP_ERROR(logger, "Invalid zero-length quaternion.");

      executor.cancel();
      spinner.join();
      rclcpp::shutdown();

      return 1;
    }

    q_target.normalize();

    target.orientation.x = q_target.x();
    target.orientation.y = q_target.y();
    target.orientation.z = q_target.z();
    target.orientation.w = q_target.w();

    RCLCPP_INFO(logger, "MODE: absolute");
  }
  else
  {
    RCLCPP_ERROR(
        logger,
        "Unknown mode '%s'. Use relative or absolute.",
        mode.c_str());

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();

    return 1;
  }

  RCLCPP_INFO(
      logger,
      "START xyz: [%.6f, %.6f, %.6f]",
      start.pose.position.x,
      start.pose.position.y,
      start.pose.position.z);

  RCLCPP_INFO(
      logger,
      "TARGET xyz: [%.6f, %.6f, %.6f]",
      target.position.x,
      target.position.y,
      target.position.z);

  RCLCPP_INFO(
      logger,
      "TARGET quaternion xyzw: [%.6f, %.6f, %.6f, %.6f]",
      target.orientation.x,
      target.orientation.y,
      target.orientation.z,
      target.orientation.w);

  if (!move_group.setPoseTarget(target, END_EFFECTOR_LINK))
  {
    RCLCPP_ERROR(logger, "TARGET REJECTED");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();

    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  const auto planning_result =
      move_group.plan(plan);

  if (planning_result !=
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "PLANNING FAILED");

    move_group.clearPoseTargets();

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();

    return 1;
  }

  RCLCPP_INFO(logger, "PLANNING PASSED");

  const auto execution_result =
      move_group.execute(plan);

  if (execution_result !=
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "EXECUTION FAILED");

    move_group.clearPoseTargets();

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();

    return 1;
  }

  RCLCPP_INFO(logger, "EXECUTION PASSED");

  std::this_thread::sleep_for(
      std::chrono::milliseconds(1000));

  const auto final_pose =
      move_group.getCurrentPose(END_EFFECTOR_LINK);

  const double pos_error =
      positionError(target, final_pose.pose);

  const double ori_error =
      orientationError(target, final_pose.pose);

  RCLCPP_INFO(
      logger,
      "FINAL xyz: [%.6f, %.6f, %.6f]",
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
      pos_error <= VALIDATION_POSITION_TOLERANCE &&
      ori_error <= VALIDATION_ORIENTATION_TOLERANCE;

  if (pass)
    RCLCPP_INFO(logger, "CARTESIAN POSE VALIDATION: PASS");
  else
    RCLCPP_ERROR(logger, "CARTESIAN POSE VALIDATION: FAIL");

  move_group.clearPoseTargets();

  executor.cancel();

  if (spinner.joinable())
    spinner.join();

  rclcpp::shutdown();

  return pass ? 0 : 1;
}
