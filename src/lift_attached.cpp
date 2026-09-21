#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <rclcpp/rclcpp.hpp>


double positionError(
    const geometry_msgs::msg::Pose& a,
    const geometry_msgs::msg::Pose& b)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  const double dz = a.position.z - b.position.z;

  return std::sqrt(dx*dx + dy*dy + dz*dz);
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "lift_attached",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  const auto logger = node->get_logger();

  constexpr const char* GROUP = "panda_arm";
  constexpr const char* EE_LINK = "panda_link8";
  constexpr const char* OBJECT = "pick_cube";

  constexpr double LIFT_DISTANCE = 0.10;
  constexpr double EEF_STEP = 0.005;

  moveit::planning_interface::MoveGroupInterface arm(
      node, GROUP);

  moveit::planning_interface::PlanningSceneInterface scene;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  // --------------------------------------------------
  // 1. Verify cube is attached before lifting.
  // --------------------------------------------------

  const bool attached_before =
      scene.getAttachedObjects({OBJECT})
          .count(OBJECT) == 1;

  if (!attached_before)
  {
    RCLCPP_ERROR(
        logger,
        "CUBE ATTACHED BEFORE LIFT: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "CUBE ATTACHED BEFORE LIFT: PASS");

  // --------------------------------------------------
  // 2. Generate +Z Cartesian lift.
  // --------------------------------------------------

  arm.setStartStateToCurrentState();

  const auto start =
      arm.getCurrentPose(EE_LINK);

  geometry_msgs::msg::Pose target =
      start.pose;

  target.position.z += LIFT_DISTANCE;

  std::vector<geometry_msgs::msg::Pose> waypoints{
      target
  };

  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::MoveItErrorCodes error_code;

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
      arm.computeCartesianPath(
          waypoints,
          EEF_STEP,
          trajectory,
          true,
          &error_code);

  RCLCPP_INFO(
      logger,
      "Cartesian lift fraction: %.4f",
      fraction);

  RCLCPP_INFO(
      logger,
      "Lift trajectory points: %zu",
      trajectory.joint_trajectory.points.size());

  if (fraction < 0.999)
  {
    RCLCPP_ERROR(
        logger,
        "FULL CARTESIAN LIFT NOT FOUND");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "CARTESIAN LIFT PATH: PASS");

  // --------------------------------------------------
  // 3. Execute.
  // --------------------------------------------------

  const auto execution_result =
      arm.execute(trajectory);

  if (execution_result !=
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
        logger,
        "LIFT EXECUTION: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "LIFT EXECUTION: PASS");

  std::this_thread::sleep_for(
      std::chrono::milliseconds(1000));

  // --------------------------------------------------
  // 4. Validate final EE pose.
  // --------------------------------------------------

  const auto final_pose =
      arm.getCurrentPose(EE_LINK);

  const double pos_error =
      positionError(
          target,
          final_pose.pose);

  const double measured_lift =
      final_pose.pose.position.z -
      start.pose.position.z;

  RCLCPP_INFO(
      logger,
      "FINAL xyz: [%.6f %.6f %.6f]",
      final_pose.pose.position.x,
      final_pose.pose.position.y,
      final_pose.pose.position.z);

  RCLCPP_INFO(
      logger,
      "MEASURED LIFT: %.6f m",
      measured_lift);

  RCLCPP_INFO(
      logger,
      "POSITION ERROR: %.6f m",
      pos_error);

  // --------------------------------------------------
  // 5. Verify cube is still attached after lifting.
  // --------------------------------------------------

  const bool attached_after =
      scene.getAttachedObjects({OBJECT})
          .count(OBJECT) == 1;

  const bool absent_from_world =
      scene.getObjects({OBJECT})
          .count(OBJECT) == 0;

  RCLCPP_INFO(
      logger,
      "CUBE STILL ATTACHED AFTER LIFT: %s",
      attached_after ? "PASS" : "FAIL");

  RCLCPP_INFO(
      logger,
      "CUBE ABSENT FROM WORLD LIST: %s",
      absent_from_world ? "PASS" : "FAIL");

  const bool lift_distance_ok =
      std::abs(measured_lift - LIFT_DISTANCE)
      <= 0.005;

  const bool pass =
      fraction >= 0.999 &&
      pos_error <= 0.005 &&
      lift_distance_ok &&
      attached_after &&
      absent_from_world;

  if (pass)
  {
    RCLCPP_INFO(
        logger,
        "D2.4E ATTACHED CUBE LIFT: PASS");
  }
  else
  {
    RCLCPP_ERROR(
        logger,
        "D2.4E ATTACHED CUBE LIFT: FAIL");
  }

  executor.cancel();

  if (spinner.joinable())
    spinner.join();

  rclcpp::shutdown();

  return pass ? 0 : 1;
}
