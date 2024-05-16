from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cam_node = Node(
            package='auto_drone',
            executable='camera_node',
            name='dual_camera_node',
            output='screen'
        )
    detection_node = Node(
            package='auto_drone',
            executable='detection_node',
            parameters=[],
            output='screen'
        )
    control_node = Node(
            package='auto_drone',
            namespace='control',
            executable='control_direction_node',
            name='control'
        )
    return LaunchDescription([
        cam_node,
        detection_node,
        control_node
    ])