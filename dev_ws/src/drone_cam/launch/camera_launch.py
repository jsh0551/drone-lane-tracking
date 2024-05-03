from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cam_node = Node(
            package='drone_cam',
            executable='camera_node',
            name='dual_camera_node',
            output='screen'
        )
    detection_node = Node(
            package='drone_cam',
            executable='detection_node',
            parameters=[],
            output='screen'
        )
    
    return LaunchDescription([
        cam_node,
        detection_node
    ])