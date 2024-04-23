from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "drone_cmd"

    mainController = Node(
        package=package_name,
        executable="control",
        output="screen",
        parameters=[],
    )

    sendVelocity = Node(
        package=package_name,
        executable="vel",
        output="screen",
        parameters=[],
    )

    return LaunchDescription(
        [
            mainController,
            sendVelocity
        ]
    )