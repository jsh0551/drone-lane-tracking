from glob import glob
from setuptools import find_packages, setup

package_name = 'drone_cmd'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='base',
    maintainer_email='base@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = drone_cmd.main_controller:main',
            'test = drone_cmd.test:main',
            'vel = drone_cmd.send_velocity:main',
            'takeoff = drone_cmd.cmd_takeoff:main',
            'drive = drone_cmd.cmd_drive:main',
            'land = drone_cmd.cmd_land:main',
        ],
    },
)
