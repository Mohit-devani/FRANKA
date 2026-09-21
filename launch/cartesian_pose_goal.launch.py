from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="panda",
            package_name="panda_moveit_config",
        )
        .to_moveit_configs()
    )

    args = [
        DeclareLaunchArgument("mode", default_value="relative"),

        DeclareLaunchArgument("dx", default_value="0.0"),
        DeclareLaunchArgument("dy", default_value="0.0"),
        DeclareLaunchArgument("dz", default_value="0.0"),

        DeclareLaunchArgument("roll_deg", default_value="0.0"),
        DeclareLaunchArgument("pitch_deg", default_value="0.0"),
        DeclareLaunchArgument("yaw_deg", default_value="0.0"),

        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.0"),

        DeclareLaunchArgument("qx", default_value="0.0"),
        DeclareLaunchArgument("qy", default_value="0.0"),
        DeclareLaunchArgument("qz", default_value="0.0"),
        DeclareLaunchArgument("qw", default_value="1.0"),
    ]

    node = Node(
        package="franka_motion_app",
        executable="cartesian_pose_goal",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "mode": LaunchConfiguration("mode"),

                "dx": ParameterValue(LaunchConfiguration("dx"), value_type=float),
                "dy": ParameterValue(LaunchConfiguration("dy"), value_type=float),
                "dz": ParameterValue(LaunchConfiguration("dz"), value_type=float),

                "roll_deg": ParameterValue(LaunchConfiguration("roll_deg"), value_type=float),
                "pitch_deg": ParameterValue(LaunchConfiguration("pitch_deg"), value_type=float),
                "yaw_deg": ParameterValue(LaunchConfiguration("yaw_deg"), value_type=float),

                "x": ParameterValue(LaunchConfiguration("x"), value_type=float),
                "y": ParameterValue(LaunchConfiguration("y"), value_type=float),
                "z": ParameterValue(LaunchConfiguration("z"), value_type=float),

                "qx": ParameterValue(LaunchConfiguration("qx"), value_type=float),
                "qy": ParameterValue(LaunchConfiguration("qy"), value_type=float),
                "qz": ParameterValue(LaunchConfiguration("qz"), value_type=float),
                "qw": ParameterValue(LaunchConfiguration("qw"), value_type=float),
            },
        ],
    )

    return LaunchDescription(args + [node])
