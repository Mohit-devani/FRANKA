#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("spawn_cube");
  const auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  moveit::planning_interface::PlanningSceneInterface scene;

  constexpr const char* CUBE_ID = "pick_cube";
  constexpr const char* FRAME = "panda_link0";

  moveit_msgs::msg::CollisionObject cube;

  cube.header.frame_id = FRAME;
  cube.id = CUBE_ID;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);

  primitive.dimensions[primitive.BOX_X] = 0.04;
  primitive.dimensions[primitive.BOX_Y] = 0.04;
  primitive.dimensions[primitive.BOX_Z] = 0.04;

  geometry_msgs::msg::Pose pose;

  pose.orientation.w = 1.0;

  // Intentionally floating for now.
  // Physics is NOT the purpose of this phase.
  pose.position.x = 0.45;
  pose.position.y = 0.00;
  pose.position.z = 0.40;

  cube.primitives.push_back(primitive);
  cube.primitive_poses.push_back(pose);
  cube.operation = cube.ADD;

  if (!scene.applyCollisionObject(cube))
  {
    RCLCPP_ERROR(logger, "CUBE APPLY: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  const auto names = scene.getKnownObjectNames();

  const bool found =
      std::find(
          names.begin(),
          names.end(),
          CUBE_ID) != names.end();

  if (!found)
  {
    RCLCPP_ERROR(logger, "CUBE NOT FOUND IN PLANNING SCENE");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  const auto objects = scene.getObjects({CUBE_ID});

  if (objects.find(CUBE_ID) == objects.end())
  {
    RCLCPP_ERROR(logger, "CUBE READBACK: FAIL");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  const auto& stored =
      objects.at(CUBE_ID);

  const auto& stored_pose =
      stored.primitive_poses.front();

  RCLCPP_INFO(logger, "CUBE PLANNING SCENE: PASS");

  RCLCPP_INFO(
      logger,
      "cube xyz: [%.3f %.3f %.3f]",
      stored_pose.position.x,
      stored_pose.position.y,
      stored_pose.position.z);

  RCLCPP_INFO(
      logger,
      "cube dimensions: [0.040 0.040 0.040] m");

  RCLCPP_INFO(
      logger,
      "Leave RViz open and inspect object 'pick_cube'.");

  RCLCPP_INFO(
      logger,
      "D2.4A CUBE WORLD OBJECT: PASS");

  // Keep node alive briefly so the user can inspect output.
  std::this_thread::sleep_for(
      std::chrono::seconds(2));

  executor.cancel();

  if (spinner.joinable())
    spinner.join();

  rclcpp::shutdown();
  return 0;
}
