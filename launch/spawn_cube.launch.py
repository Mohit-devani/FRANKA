from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    node = Node(
        package="franka_motion_app",
        executable="spawn_cube",
        output="screen",
    )

    return LaunchDescription([node])
