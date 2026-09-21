from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="panda",
            package_name="panda_moveit_config",
        )
        .to_moveit_configs()
    )

    node = Node(
        package="franka_motion_app",
        executable="gripper_cycle",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([node])
