from setuptools import find_packages, setup

package_name = 'drone_cam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cto2',
    maintainer_email='cto2@todo.todo',
    description='Example package with two camera nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = drone_cam.camera_node:main',
            'detection_node = drone_cam.detection_node:main'
        ],
    },
)