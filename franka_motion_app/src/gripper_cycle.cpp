#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "gripper_cycle",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  const auto logger = node->get_logger();

  moveit::planning_interface::MoveGroupInterface hand(
      node, "hand");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  hand.setPlanningTime(2.0);

  auto run_named_target =
      [&](const std::string& name) -> bool
  {
    if (!hand.setNamedTarget(name))
    {
      RCLCPP_ERROR(
          logger,
          "Named hand target '%s' not found",
          name.c_str());

      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    const auto planning_result =
        hand.plan(plan);

    if (planning_result !=
        moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
          logger,
          "%s planning: FAIL",
          name.c_str());

      return false;
    }

    RCLCPP_INFO(
        logger,
        "%s planning: PASS",
        name.c_str());

    const auto execution_result =
        hand.execute(plan);

    if (execution_result !=
        moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
          logger,
          "%s execution: FAIL",
          name.c_str());

      return false;
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(500));

    auto state = hand.getCurrentState(2.0);

    if (!state)
    {
      RCLCPP_ERROR(
          logger,
          "Unable to read hand state");

      return false;
    }

    const double finger =
        state->getVariablePosition(
            "panda_finger_joint1");

    RCLCPP_INFO(
        logger,
        "%s execution: PASS",
        name.c_str());

    RCLCPP_INFO(
        logger,
        "panda_finger_joint1: %.6f m",
        finger);

    return true;
  };

  RCLCPP_INFO(
      logger,
      "D2.3 PROGRAMMATIC GRIPPER TEST");

  // Safe known initial state.
  if (!run_named_target("open"))
  {
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  // Close completely.
  if (!run_named_target("close"))
  {
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  // Leave robot open for the next grasp test.
  if (!run_named_target("open"))
  {
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "D2.3 GRIPPER OPEN/CLOSE CYCLE: PASS");

  executor.cancel();

  if (spinner.joinable())
    spinner.join();

  rclcpp::shutdown();

  return 0;
}
