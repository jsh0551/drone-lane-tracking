from setuptools import setup

package_name = 'python_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_rng = python_test.pub_rng:main',
            'pub_point = python_test.pub_point:main',
            'turtle_run = python_test.turtle_run:main',
            'turtle_goal = python_test.turtle_goal:main',
            'turtle_goal_lidar = python_test.turtle_goal_lidar:main',
            'pub_twist_datmo = python_test.pub_twist_datmo:main',
            'lidar_point = python_test.lidar_point:main',
            'another_turtle_goal = python_test.another_turtle_goal:main',

        ],
    },
)
