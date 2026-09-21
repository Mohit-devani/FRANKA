#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "grasp_attach",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  const auto logger = node->get_logger();

  constexpr const char* OBJECT = "pick_cube";
  constexpr const char* HAND_GROUP = "hand";
  constexpr const char* ATTACH_LINK = "panda_hand";

  constexpr double GRASP_Q = 0.020;
  constexpr double GRASP_TOLERANCE = 0.001;

  const std::vector<std::string> finger_links{
      "panda_leftfinger",
      "panda_rightfinger"
  };

  moveit::planning_interface::MoveGroupInterface hand(
      node, HAND_GROUP);

  moveit::planning_interface::PlanningSceneInterface scene;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  // --------------------------------------------------
  // 1. Verify cube is still a world object.
  // --------------------------------------------------

  if (scene.getObjects({OBJECT}).count(OBJECT) == 0)
  {
    RCLCPP_ERROR(
        logger,
        "pick_cube not present in world");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "CUBE WORLD OBJECT PRESENT: PASS");

  // --------------------------------------------------
  // 2. Explicitly allow ONLY finger-cube contact.
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
    RCLCPP_ERROR(
        logger,
        "ACM retrieval failed");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  collision_detection::AllowedCollisionMatrix acm(
      future.get()->scene.allowed_collision_matrix);

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
  // 3. Command geometric grasp width.
  // --------------------------------------------------

  hand.setStartStateToCurrentState();
  hand.setPlanningTime(2.0);

  hand.setMaxVelocityScalingFactor(0.10);
  hand.setMaxAccelerationScalingFactor(0.10);

  if (!hand.setJointValueTarget(
          "panda_finger_joint1",
          GRASP_Q))
  {
    RCLCPP_ERROR(
        logger,
        "GRASP JOINT TARGET REJECTED");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan
      grasp_plan;

  if (hand.plan(grasp_plan) !=
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
        logger,
        "GEOMETRIC GRASP PLANNING: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "GEOMETRIC GRASP PLANNING: PASS");

  if (hand.execute(grasp_plan) !=
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
        logger,
        "GEOMETRIC GRASP EXECUTION: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  std::this_thread::sleep_for(
      std::chrono::milliseconds(500));

  const auto state =
      hand.getCurrentState(2.0);

  if (!state)
  {
    RCLCPP_ERROR(
        logger,
        "Unable to read hand state");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  const double q =
      state->getVariablePosition(
          "panda_finger_joint1");

  const double inferred_opening =
      2.0 * q;

  RCLCPP_INFO(
      logger,
      "panda_finger_joint1: %.6f m",
      q);

  RCLCPP_INFO(
      logger,
      "Approx geometric opening: %.6f m",
      inferred_opening);

  const bool grasp_position_ok =
      std::abs(q - GRASP_Q) <=
      GRASP_TOLERANCE;

  if (!grasp_position_ok)
  {
    RCLCPP_ERROR(
        logger,
        "GEOMETRIC GRASP POSITION: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "GEOMETRIC GRASP POSITION: PASS");

  // --------------------------------------------------
  // 4. Convert cube from world object to attached body.
  // --------------------------------------------------

  moveit_msgs::msg::AttachedCollisionObject attached;

  attached.link_name = ATTACH_LINK;

  attached.object.id = OBJECT;

  attached.object.operation =
      moveit_msgs::msg::CollisionObject::ADD;

  attached.touch_links =
      finger_links;

  if (!scene.applyAttachedCollisionObject(
          attached))
  {
    RCLCPP_ERROR(
        logger,
        "ATTACH CUBE: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  // --------------------------------------------------
  // 5. Verify transition.
  // --------------------------------------------------

  const bool attached_ok =
      scene.getAttachedObjects({OBJECT})
          .count(OBJECT) == 1;

  const bool removed_from_world =
      scene.getObjects({OBJECT})
          .count(OBJECT) == 0;

  RCLCPP_INFO(
      logger,
      "CUBE ATTACHED TO panda_hand: %s",
      attached_ok ? "PASS" : "FAIL");

  RCLCPP_INFO(
      logger,
      "CUBE REMOVED FROM WORLD LIST: %s",
      removed_from_world ? "PASS" : "FAIL");

  const bool pass =
      grasp_position_ok &&
      attached_ok &&
      removed_from_world;

  if (pass)
  {
    RCLCPP_INFO(
        logger,
        "D2.4D GEOMETRIC GRASP + ATTACH: PASS");
  }
  else
  {
    RCLCPP_ERROR(
        logger,
        "D2.4D GEOMETRIC GRASP + ATTACH: FAIL");
  }

  executor.cancel();

  if (spinner.joinable())
    spinner.join();

  rclcpp::shutdown();

  return pass ? 0 : 1;
}
