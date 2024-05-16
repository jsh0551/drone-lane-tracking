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
            'camera_node = auto_drone.camera_node:main',
            'detection_node = auto_drone.detection_node:main',
            'control_direction_node = auto_drone.control_direction_node:main'
        ],
    },
)