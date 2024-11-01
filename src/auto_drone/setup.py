import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'auto_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'rclpy', 'mavros_msgs'],
    zip_safe=True,
    maintainer='admin957-2',
    maintainer_email='jaewhoon_cho@957.co.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server_node = auto_drone.node_controller_main:main',
            'vel_node = auto_drone.send_velocity:main',
            'vel_circle_node = auto_drone.send_velocity_circle:main',
            'camera_node = auto_drone.node_camera:main',
            'airsim_cam1_node = auto_drone.node_camera1_airsim:main',
            'airsim_cam2_node = auto_drone.node_camera2_airsim:main',
            'line_detection_node = auto_drone.node_detection_line:main',
            'runner_detection_node = auto_drone.node_detection_runner:main',
            'control_direction_node = auto_drone.node_control_direction:main',
            'control_velocity_node = auto_drone.node_control_velocity:main',
            'takeoff = auto_drone.cmd_takeoff:main',
            'land = auto_drone.cmd_land:main',
            'drive = auto_drone.cmd_drive:main',
            'drive_auto = auto_drone.cmd_drive_auto:main',
            'record = auto_drone.cmd_record:main',
            'drive_termination = auto_drone.cmd_drive_termination:main',
            'record_video_node = auto_drone.node_record_video:main',
        ],
    },
)
