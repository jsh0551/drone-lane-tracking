from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "learning_topic"

    node_streaming_pub = Node(
        package=package_name,
        executable="streaming_pub",
        parameters=[],
        output='screen'
    )

    node_webcam_sub = Node(
        package=package_name,
        executable="topic_webcam_sub",
        parameters=[],
        output='screen'
    )

    return LaunchDescription(
        [
            node_webcam_sub,
            node_streaming_pub
        ]
    )