#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

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
      "grasp_approach",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  const auto logger = node->get_logger();

  constexpr const char* GROUP = "panda_arm";
  constexpr const char* EE_LINK = "panda_link8";
  constexpr const char* OBJECT = "pick_cube";

  moveit::planning_interface::MoveGroupInterface arm(
      node, GROUP);

  moveit::planning_interface::PlanningSceneInterface scene;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  // --------------------------------------------------
  // Verify cube exists.
  // --------------------------------------------------

  if (scene.getObjects({OBJECT}).count(OBJECT) == 0)
  {
    RCLCPP_ERROR(logger, "pick_cube not found");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "CUBE PRESENT: PASS");

  // --------------------------------------------------
  // Retrieve current Allowed Collision Matrix.
  // --------------------------------------------------

  using GetPlanningScene =
      moveit_msgs::srv::GetPlanningScene;

  auto client =
      node->create_client<GetPlanningScene>(
          "/get_planning_scene");

  if (!client->wait_for_service(
          std::chrono::seconds(3)))
  {
    RCLCPP_ERROR(
        logger,
        "get_planning_scene unavailable");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  auto request =
      std::make_shared<GetPlanningScene::Request>();

  request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::
          ALLOWED_COLLISION_MATRIX;

  auto future =
      client->async_send_request(request);

  if (future.wait_for(std::chrono::seconds(3)) !=
      std::future_status::ready)
  {
    RCLCPP_ERROR(logger, "ACM request failed");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  collision_detection::AllowedCollisionMatrix acm(
      future.get()->scene.allowed_collision_matrix);

  // Allow ONLY finger/cube contact.
  const std::vector<std::string> finger_links{
      "panda_leftfinger",
      "panda_rightfinger"
  };

  acm.setEntry(
      OBJECT,
      finger_links,
      true);

  moveit_msgs::msg::PlanningScene diff;

  diff.is_diff = true;
  diff.robot_state.is_diff = true;

  acm.getMessage(
      diff.allowed_collision_matrix);

  if (!scene.applyPlanningScene(diff))
  {
    RCLCPP_ERROR(
        logger,
        "ALLOW FINGER-CUBE CONTACT: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "ALLOW FINGER-CUBE CONTACT: PASS");

  // --------------------------------------------------
  // Cartesian approach to canonical grasp depth.
  // --------------------------------------------------

  arm.setStartStateToCurrentState();

  const auto start =
      arm.getCurrentPose(EE_LINK);

  geometry_msgs::msg::Pose target =
      start.pose;

  // From ~0.60 m down to ~0.50 m.
  target.position.z -= 0.10;

  std::vector<geometry_msgs::msg::Pose> waypoints{
      target
  };

  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::MoveItErrorCodes error_code;

  constexpr double EEF_STEP = 0.005;

  const double fraction =
      arm.computeCartesianPath(
          waypoints,
          EEF_STEP,
          trajectory,
          true,
          &error_code);

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

  RCLCPP_INFO(
      logger,
      "Cartesian fraction: %.4f",
      fraction);

  if (fraction < 0.999)
  {
    RCLCPP_ERROR(
        logger,
        "GRASP APPROACH STILL COLLISION-BLOCKED");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();

    return 1;
  }

  RCLCPP_INFO(
      logger,
      "GRASP CARTESIAN PATH: PASS");

  const auto result =
      arm.execute(trajectory);

  if (result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
        logger,
        "GRASP APPROACH EXECUTION: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();

    return 1;
  }

  RCLCPP_INFO(
      logger,
      "GRASP APPROACH EXECUTION: PASS");

  std::this_thread::sleep_for(
      std::chrono::milliseconds(1000));

  const auto final_pose =
      arm.getCurrentPose(EE_LINK);

  const double error =
      positionError(
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
      error);

  const bool pass =
      error <= 0.005;

  if (pass)
    RCLCPP_INFO(
        logger,
        "D2.4C2 CONTACT-AWARE GRASP APPROACH: PASS");
  else
    RCLCPP_ERROR(
        logger,
        "D2.4C2 CONTACT-AWARE GRASP APPROACH: FAIL");

  executor.cancel();

  if (spinner.joinable())
    spinner.join();

  rclcpp::shutdown();

  return pass ? 0 : 1;
}
