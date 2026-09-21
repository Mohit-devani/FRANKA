#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>

#include <rclcpp/rclcpp.hpp>


using StateValidity =
    moveit_msgs::srv::GetStateValidity;


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

  const double na =
      std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w);

  const double nb =
      std::sqrt(b.x*b.x + b.y*b.y + b.z*b.z + b.w*b.w);

  double dot =
      (a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w) /
      (na * nb);

  dot = std::abs(dot);
  dot = std::clamp(dot, 0.0, 1.0);

  return 2.0 * std::acos(dot);
}


bool checkState(
    const rclcpp::Client<StateValidity>::SharedPtr& client,
    const std::vector<std::string>& names,
    const std::vector<double>& positions,
    const std::string& group,
    bool& valid)
{
  auto request =
      std::make_shared<StateValidity::Request>();

  request->group_name = group;

  request->robot_state.joint_state.name = names;
  request->robot_state.joint_state.position = positions;

  auto future =
      client->async_send_request(request);

  if (future.wait_for(std::chrono::seconds(2)) !=
      std::future_status::ready)
  {
    return false;
  }

  valid = future.get()->valid;
  return true;
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "collision_aware_pose",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  const auto logger = node->get_logger();

  const std::string GROUP = "panda_arm";
  const std::string EE_LINK = "panda_link8";
  const std::string COLLISION_LINK = "panda_hand";
  const std::string OBSTACLE_ID = "day2_path_blocker";

  moveit::planning_interface::MoveGroupInterface move_group(
      node, GROUP);

  moveit::planning_interface::PlanningSceneInterface
      planning_scene_interface;

  auto validity_client =
      node->create_client<StateValidity>(
          "/check_state_validity");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  if (!validity_client->wait_for_service(
          std::chrono::seconds(3)))
  {
    RCLCPP_ERROR(
        logger,
        "check_state_validity service unavailable");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  // Remove stale object if a previous run left one.
  moveit_msgs::msg::CollisionObject cleanup;
  cleanup.header.frame_id = move_group.getPlanningFrame();
  cleanup.id = OBSTACLE_ID;
  cleanup.operation = cleanup.REMOVE;

  planning_scene_interface.applyCollisionObject(cleanup);

  move_group.setPlanningTime(5.0);

  move_group.setMaxVelocityScalingFactor(0.20);
  move_group.setMaxAccelerationScalingFactor(0.20);

  move_group.setGoalPositionTolerance(0.0025);
  move_group.setGoalOrientationTolerance(0.005);

  const auto start_pose =
      move_group.getCurrentPose(EE_LINK);

  geometry_msgs::msg::Pose target =
      start_pose.pose;

  // Same target already proven reachable in the control test.
  target.position.x += 0.25;

  RCLCPP_INFO(
      logger,
      "Planning frame: %s",
      move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(
      logger,
      "START xyz: [%.4f %.4f %.4f]",
      start_pose.pose.position.x,
      start_pose.pose.position.y,
      start_pose.pose.position.z);

  RCLCPP_INFO(
      logger,
      "TARGET xyz: [%.4f %.4f %.4f]",
      target.position.x,
      target.position.y,
      target.position.z);

  // ======================================================
  // A — BASELINE PLAN WITH NO OBSTACLE
  // ======================================================

  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target, EE_LINK);

  moveit::planning_interface::MoveGroupInterface::Plan baseline;

  const auto baseline_result =
      move_group.plan(baseline);

  if (baseline_result !=
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
        logger,
        "BASELINE PLAN WITHOUT OBSTACLE: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "BASELINE PLAN WITHOUT OBSTACLE: PASS");

  const auto& baseline_joint_trajectory =
      baseline.trajectory.joint_trajectory;

  if (baseline_joint_trajectory.points.size() < 2)
  {
    RCLCPP_ERROR(
        logger,
        "Baseline trajectory has fewer than 2 points");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "Baseline trajectory points: %zu",
      baseline_joint_trajectory.points.size());

  // ======================================================
  // Find a state around the middle of the baseline path.
  // ======================================================

  std::vector<double> midpoint_positions;

  if (baseline_joint_trajectory.points.size() > 2)
  {
    const std::size_t mid =
        baseline_joint_trajectory.points.size() / 2;

    midpoint_positions =
        baseline_joint_trajectory.points[mid].positions;
  }
  else
  {
    const auto& a =
        baseline_joint_trajectory.points.front().positions;

    const auto& b =
        baseline_joint_trajectory.points.back().positions;

    midpoint_positions.resize(a.size());

    for (std::size_t i = 0; i < a.size(); ++i)
      midpoint_positions[i] =
          0.5 * (a[i] + b[i]);
  }

  auto midpoint_state =
      move_group.getCurrentState(2.0);

  if (!midpoint_state)
  {
    RCLCPP_ERROR(
        logger,
        "Unable to obtain RobotState");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  midpoint_state->setVariablePositions(
      baseline_joint_trajectory.joint_names,
      midpoint_positions);

  midpoint_state->update();

  const Eigen::Isometry3d hand_transform =
      midpoint_state->getGlobalLinkTransform(
          COLLISION_LINK);

  // ======================================================
  // B — INSERT OBSTACLE ON THE OLD PATH
  // ======================================================

  moveit_msgs::msg::CollisionObject obstacle;

  obstacle.header.frame_id =
      move_group.getPlanningFrame();

  obstacle.id = OBSTACLE_ID;

  shape_msgs::msg::SolidPrimitive box;

  box.type = box.BOX;
  box.dimensions.resize(3);

  // Small blocker: enough to invalidate the old hand path,
  // but small enough that the 7-DOF Panda should have room
  // to route around it.
  box.dimensions[box.BOX_X] = 0.06;
  box.dimensions[box.BOX_Y] = 0.06;
  box.dimensions[box.BOX_Z] = 0.06;

  geometry_msgs::msg::Pose obstacle_pose;

  obstacle_pose.orientation.w = 1.0;

  obstacle_pose.position.x =
      hand_transform.translation().x();

  obstacle_pose.position.y =
      hand_transform.translation().y();

  obstacle_pose.position.z =
      hand_transform.translation().z();

  obstacle.primitives.push_back(box);
  obstacle.primitive_poses.push_back(obstacle_pose);
  obstacle.operation = obstacle.ADD;

  RCLCPP_INFO(
      logger,
      "Placing blocker at baseline hand-path midpoint:");
  RCLCPP_INFO(
      logger,
      "OBSTACLE xyz: [%.4f %.4f %.4f]",
      obstacle_pose.position.x,
      obstacle_pose.position.y,
      obstacle_pose.position.z);

  if (!planning_scene_interface.applyCollisionObject(
          obstacle))
  {
    RCLCPP_ERROR(
        logger,
        "FAILED TO APPLY OBSTACLE");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  const auto names =
      planning_scene_interface.getKnownObjectNames();

  const bool object_present =
      std::find(
          names.begin(),
          names.end(),
          OBSTACLE_ID) != names.end();

  if (!object_present)
  {
    RCLCPP_ERROR(
        logger,
        "OBSTACLE NOT PRESENT IN PLANNING SCENE");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "PLANNING SCENE OBJECT: PASS");

  // ======================================================
  // Prove obstacle actually blocks the OLD trajectory.
  // ======================================================

  bool midpoint_valid = true;

  if (!checkState(
          validity_client,
          baseline_joint_trajectory.joint_names,
          midpoint_positions,
          GROUP,
          midpoint_valid))
  {
    RCLCPP_ERROR(
        logger,
        "State validity service call failed");

    obstacle.operation = obstacle.REMOVE;
    planning_scene_interface.applyCollisionObject(obstacle);

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  if (midpoint_valid)
  {
    RCLCPP_ERROR(
        logger,
        "OLD PATH WAS NOT BLOCKED BY THE OBSTACLE");

    obstacle.operation = obstacle.REMOVE;
    planning_scene_interface.applyCollisionObject(obstacle);

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "OLD BASELINE PATH BLOCKED: PASS");

  // ======================================================
  // C — REPLAN SAME TARGET WITH OBSTACLE PRESENT
  // ======================================================

  move_group.clearPoseTargets();

  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target, EE_LINK);

  moveit::planning_interface::MoveGroupInterface::Plan
      collision_plan;

  const auto collision_result =
      move_group.plan(collision_plan);

  if (collision_result !=
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
        logger,
        "REPLAN AROUND OBSTACLE: FAIL");

    obstacle.operation = obstacle.REMOVE;
    planning_scene_interface.applyCollisionObject(obstacle);

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "REPLAN AROUND OBSTACLE: PASS");

  const auto& safe_trajectory =
      collision_plan.trajectory.joint_trajectory;

  RCLCPP_INFO(
      logger,
      "Collision-aware trajectory points: %zu",
      safe_trajectory.points.size());

  // ======================================================
  // Validate every output trajectory state against
  // MoveIt's current planning scene.
  // ======================================================

  int invalid_states = 0;

  for (std::size_t i = 0;
       i < safe_trajectory.points.size();
       ++i)
  {
    bool state_valid = false;

    if (!checkState(
            validity_client,
            safe_trajectory.joint_names,
            safe_trajectory.points[i].positions,
            GROUP,
            state_valid))
    {
      RCLCPP_ERROR(
          logger,
          "State validity service failed at trajectory point %zu",
          i);

      obstacle.operation = obstacle.REMOVE;
      planning_scene_interface.applyCollisionObject(obstacle);

      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }

    if (!state_valid)
      ++invalid_states;
  }

  RCLCPP_INFO(
      logger,
      "Collision-aware trajectory invalid states: %d",
      invalid_states);

  if (invalid_states != 0)
  {
    RCLCPP_ERROR(
        logger,
        "COLLISION VALIDATION: FAIL");

    obstacle.operation = obstacle.REMOVE;
    planning_scene_interface.applyCollisionObject(obstacle);

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "COLLISION VALIDATION: PASS");

  // ======================================================
  // D — EXECUTE ONLY THE NEW SAFE PLAN
  // ======================================================

  const auto execution_result =
      move_group.execute(collision_plan);

  if (execution_result !=
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
        logger,
        "EXECUTION FAILED");

    obstacle.operation = obstacle.REMOVE;
    planning_scene_interface.applyCollisionObject(obstacle);

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "EXECUTION PASSED");

  std::this_thread::sleep_for(
      std::chrono::milliseconds(1000));

  const auto final_pose =
      move_group.getCurrentPose(EE_LINK);

  const double pos_error =
      positionError(target, final_pose.pose);

  const double ori_error =
      orientationError(target, final_pose.pose);

  RCLCPP_INFO(
      logger,
      "FINAL xyz: [%.4f %.4f %.4f]",
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

  const bool pose_pass =
      pos_error <= 0.005 &&
      ori_error <= 0.01;

  // Cleanup.
  obstacle.operation = obstacle.REMOVE;

  const bool cleanup_ok =
      planning_scene_interface.applyCollisionObject(
          obstacle);

  RCLCPP_INFO(
      logger,
      "Obstacle cleanup: %s",
      cleanup_ok ? "PASS" : "FAIL");

  move_group.clearPoseTargets();

  if (pose_pass && cleanup_ok)
  {
    RCLCPP_INFO(
        logger,
        "D2.1 COLLISION-AWARE REPLANNING: PASS");
  }
  else
  {
    RCLCPP_ERROR(
        logger,
        "D2.1 COLLISION-AWARE REPLANNING: FAIL");
  }

  executor.cancel();

  if (spinner.joinable())
    spinner.join();

  rclcpp::shutdown();

  return (pose_pass && cleanup_ok) ? 0 : 1;
}
