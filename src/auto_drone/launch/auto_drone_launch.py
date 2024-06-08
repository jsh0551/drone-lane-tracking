from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node_server = Node(
        package='auto_drone',
        executable='server_node',
        output='screen'
    )
    node_cam = Node(
        package='auto_drone',
        executable='node_camera',
        name='dual_camera_node',
        output='screen'
    )
    node_detection = Node(
        package='auto_drone',
        executable='node_detection_line',
        parameters=[],
        output='screen'
    )
    node_control = Node(
        package='auto_drone',
        namespace='control',
        executable='node_control_direction',
        name='control',
        output='screen'
    )
    return LaunchDescription([
        node_server,
        node_cam,
        node_detection,
        node_control
    ])