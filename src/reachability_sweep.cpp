#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "reachability_sweep",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  const auto logger = node->get_logger();

  constexpr const char* GROUP = "panda_arm";
  constexpr const char* EE_LINK = "panda_link8";

  moveit::planning_interface::MoveGroupInterface move_group(node, GROUP);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  move_group.setPoseReferenceFrame(move_group.getPlanningFrame());

  move_group.setPlanningTime(0.40);

  move_group.setGoalPositionTolerance(0.0025);
  move_group.setGoalOrientationTolerance(0.005);

  // Make sure the state monitor has received a real robot state.
  const auto start_pose = move_group.getCurrentPose(EE_LINK);

  RCLCPP_INFO(
      logger,
      "Planning frame: %s",
      move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(
      logger,
      "Sweep start xyz: [%.4f, %.4f, %.4f]",
      start_pose.pose.position.x,
      start_pose.pose.position.y,
      start_pose.pose.position.z);

  // 4 × 5 × 5 = 100 absolute Cartesian targets.
  const std::vector<double> xs{
      0.20, 0.35, 0.50, 0.65};

  const std::vector<double> ys{
      -0.30, -0.15, 0.00, 0.15, 0.30};

  const std::vector<double> zs{
      0.25, 0.40, 0.55, 0.70, 0.85};

  // Fixed orientation from our validated ready pose.
  constexpr double QX = 0.923956;
  constexpr double QY = -0.382499;
  constexpr double QZ = 0.0;
  constexpr double QW = 0.0;

  const char* home = std::getenv("HOME");

  if (home == nullptr)
  {
    RCLCPP_ERROR(logger, "HOME environment variable not available.");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();

    return 1;
  }

  const std::filesystem::path artifact_dir =
      std::filesystem::path(home) /
      "ws_moveit/artifacts/day1";

  std::filesystem::create_directories(artifact_dir);

  const auto csv_path =
      artifact_dir / "reachability_sweep.csv";

  std::ofstream csv(csv_path);

  if (!csv)
  {
    RCLCPP_ERROR(
        logger,
        "Unable to create CSV: %s",
        csv_path.c_str());

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();

    return 1;
  }

  csv
      << "x,y,z,"
      << "qx,qy,qz,qw,"
      << "ik_success,"
      << "q1,q2,q3,q4,q5,q6,q7,"
      << "plan_success,"
      << "planning_time_ms\n";

  csv << std::fixed << std::setprecision(6);

  int total = 0;
  int ik_success_count = 0;
  int plan_success_count = 0;

  int ik_yes_plan_no = 0;
  int ik_no_plan_yes = 0;

  for (double z : zs)
  {
    for (double y : ys)
    {
      for (double x : xs)
      {
        ++total;

        geometry_msgs::msg::Pose target;

        target.position.x = x;
        target.position.y = y;
        target.position.z = z;

        target.orientation.x = QX;
        target.orientation.y = QY;
        target.orientation.z = QZ;
        target.orientation.w = QW;

        move_group.setStartStateToCurrentState();

        // -------------------------------------------------
        // Layer 1: Direct IK
        // -------------------------------------------------

        const bool ik_success =
            move_group.setJointValueTarget(
                target,
                EE_LINK);

        std::vector<double> q;

        if (ik_success)
        {
          ++ik_success_count;
          move_group.getJointValueTarget(q);
        }

        // -------------------------------------------------
        // Layer 2: Full pose-goal motion planning
        //
        // IMPORTANT:
        // setPoseTarget() deliberately overwrites the
        // previous joint target, including any partial
        // result left behind by failed IK.
        // -------------------------------------------------

        move_group.setPoseTarget(
            target,
            EE_LINK);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        const auto t0 =
            std::chrono::steady_clock::now();

        const auto result =
            move_group.plan(plan);

        const auto t1 =
            std::chrono::steady_clock::now();

        const double planning_time_ms =
            std::chrono::duration<double, std::milli>(
                t1 - t0).count();

        const bool plan_success =
            result ==
            moveit::core::MoveItErrorCode::SUCCESS;

        if (plan_success)
          ++plan_success_count;

        if (ik_success && !plan_success)
          ++ik_yes_plan_no;

        if (!ik_success && plan_success)
          ++ik_no_plan_yes;

        csv
            << x << ","
            << y << ","
            << z << ","
            << QX << ","
            << QY << ","
            << QZ << ","
            << QW << ","
            << (ik_success ? 1 : 0) << ",";

        for (std::size_t i = 0; i < 7; ++i)
        {
          if (ik_success && i < q.size())
            csv << q[i];

          csv << ",";
        }

        csv
            << (plan_success ? 1 : 0) << ","
            << planning_time_ms
            << "\n";

        RCLCPP_INFO(
            logger,
            "[%03d/100] xyz=[%.2f %.2f %.2f] IK=%s PLAN=%s",
            total,
            x, y, z,
            ik_success ? "YES" : "NO",
            plan_success ? "YES" : "NO");

        move_group.clearPoseTargets();
      }
    }
  }

  csv.close();

  RCLCPP_INFO(logger, "==================================");
  RCLCPP_INFO(logger, "D1.9 REACHABILITY SWEEP COMPLETE");
  RCLCPP_INFO(logger, "==================================");

  RCLCPP_INFO(logger, "Total targets: %d", total);

  RCLCPP_INFO(
      logger,
      "Direct IK success: %d / %d",
      ik_success_count,
      total);

  RCLCPP_INFO(
      logger,
      "Pose planning success: %d / %d",
      plan_success_count,
      total);

  RCLCPP_INFO(
      logger,
      "IK YES / PLAN NO: %d",
      ik_yes_plan_no);

  RCLCPP_INFO(
      logger,
      "IK NO / PLAN YES: %d",
      ik_no_plan_yes);

  RCLCPP_INFO(
      logger,
      "CSV: %s",
      csv_path.c_str());

  RCLCPP_INFO(
      logger,
      "NO TRAJECTORIES WERE EXECUTED.");

  executor.cancel();

  if (spinner.joinable())
    spinner.join();

  rclcpp::shutdown();

  return 0;
}
