from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node_server = Node(
        package='auto_drone',
        executable='server_node',
        output='screen'
    )
    node_cam1 = Node(
        package='auto_drone',
        executable='airsim_cam1_node',
        name='dual_camera_node',
        output='screen'
    )
    node_cam2 = Node(
        package='auto_drone',
        executable='airsim_cam2_node',
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
        executable='control_velocity_node',
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
        node_cam1,
        node_cam2,
        node_detection,
        node_record_video
    ])