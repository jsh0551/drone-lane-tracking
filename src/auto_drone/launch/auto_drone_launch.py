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
        executable='camera_node',
        name='dual_camera_node',
        output='screen'
    )
    node_detection = Node(
        package='auto_drone',
        executable='line_detection_node',
        parameters=[],
        output='screen'
    )
    node_control = Node(
        package='auto_drone',
        namespace='control',
        executable='control_velocity_node',
        name='control',
        output='screen'
    )
    node_record_video = Node(
        package='auto_drone',
        executable='record_video_node',
        output='screen'
    )
    return LaunchDescription([
        node_server,
        node_control,
        node_cam,
        node_detection,
        node_record_video
    ])