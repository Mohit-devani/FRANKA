#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "day2_cleanup",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  const auto logger = node->get_logger();

  constexpr const char* OBJECT = "pick_cube";

  moveit::planning_interface::MoveGroupInterface arm(
      node, "panda_arm");

  moveit::planning_interface::MoveGroupInterface hand(
      node, "hand");

  moveit::planning_interface::PlanningSceneInterface scene;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  // --------------------------------------------------
  // 1. Verify Day-2 final state.
  // --------------------------------------------------

  if (scene.getAttachedObjects({OBJECT}).count(OBJECT) != 1)
  {
    RCLCPP_ERROR(
        logger,
        "CUBE ATTACHED AT START: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "CUBE ATTACHED AT START: PASS");

  // --------------------------------------------------
  // 2. Detach object.
  // --------------------------------------------------

  if (!arm.detachObject(OBJECT))
  {
    RCLCPP_ERROR(
        logger,
        "DETACH REQUEST: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  bool detached = false;
  bool in_world = false;

  for (int i = 0; i < 30; ++i)
  {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(100));

    detached =
        scene.getAttachedObjects({OBJECT}).count(OBJECT) == 0;

    in_world =
        scene.getObjects({OBJECT}).count(OBJECT) == 1;

    if (detached && in_world)
      break;
  }

  RCLCPP_INFO(
      logger,
      "CUBE DETACHED: %s",
      detached ? "PASS" : "FAIL");

  RCLCPP_INFO(
      logger,
      "CUBE RETURNED TO WORLD: %s",
      in_world ? "PASS" : "FAIL");

  if (!detached || !in_world)
  {
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  // --------------------------------------------------
  // 3. Open hand.
  // --------------------------------------------------

  hand.setStartStateToCurrentState();
  hand.setPlanningTime(2.0);

  if (!hand.setNamedTarget("open"))
  {
    RCLCPP_ERROR(
        logger,
        "OPEN TARGET: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan
      open_plan;

  if (hand.plan(open_plan) !=
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
        logger,
        "OPEN HAND PLANNING: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  if (hand.execute(open_plan) !=
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
        logger,
        "OPEN HAND EXECUTION: FAIL");

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
        "HAND STATE READ: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  const double q =
      state->getVariablePosition(
          "panda_finger_joint1");

  RCLCPP_INFO(
      logger,
      "panda_finger_joint1: %.6f m",
      q);

  const bool hand_open =
      q > 0.033;

  RCLCPP_INFO(
      logger,
      "HAND OPEN: %s",
      hand_open ? "PASS" : "FAIL");

  if (!hand_open)
  {
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  // --------------------------------------------------
  // 4. Remove cube from world completely.
  // --------------------------------------------------

  moveit_msgs::msg::CollisionObject remove;

  remove.header.frame_id =
      arm.getPlanningFrame();

  remove.id = OBJECT;
  remove.operation =
      moveit_msgs::msg::CollisionObject::REMOVE;

  if (!scene.applyCollisionObject(remove))
  {
    RCLCPP_ERROR(
        logger,
        "REMOVE WORLD CUBE: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  const bool no_world_cube =
      scene.getObjects({OBJECT}).count(OBJECT) == 0;

  const bool no_attached_cube =
      scene.getAttachedObjects({OBJECT}).count(OBJECT) == 0;

  RCLCPP_INFO(
      logger,
      "WORLD CUBE REMOVED: %s",
      no_world_cube ? "PASS" : "FAIL");

  RCLCPP_INFO(
      logger,
      "NO ATTACHED CUBE: %s",
      no_attached_cube ? "PASS" : "FAIL");

  const bool pass =
      hand_open &&
      no_world_cube &&
      no_attached_cube;

  if (pass)
  {
    RCLCPP_INFO(logger, "================================");
    RCLCPP_INFO(logger, "DAY 2 MANIPULATION PIPELINE: PASS");
    RCLCPP_INFO(logger, "================================");
  }
  else
  {
    RCLCPP_ERROR(
        logger,
        "DAY 2 MANIPULATION PIPELINE: FAIL");
  }

  executor.cancel();

  if (spinner.joinable())
    spinner.join();

  rclcpp::shutdown();

  return pass ? 0 : 1;
}
