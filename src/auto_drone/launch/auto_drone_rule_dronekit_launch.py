from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node_server = Node(
        package='auto_drone',
        executable='server_node_dronekit',
        output='screen'
    )
    node_vel = Node(
        package='auto_drone',
        executable='vel_node_dronekit',
        output='screen'
    )
    node_cam = Node(
        package='auto_drone',
        executable='camera_node',
        name='dual_camera_node',
        output='screen'
    )
    node_record_video = Node(
        package='auto_drone',
        executable='record_video_node',
        output='screen'
    )

    return LaunchDescription([
        node_server,
        node_vel,
        # node_cam,
        node_record_video
    ])