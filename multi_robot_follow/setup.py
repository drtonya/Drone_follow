from setuptools import find_packages, setup

package_name = 'multi_robot_follow'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_robot_launch.py']),
        ('share/' + package_name + '/maps', ['maps/warehouse_map.yaml', 'maps/warehouse_map.pgm']),
        ('share/' + package_name + '/rviz', ['rviz/warehouse_rviz_config.rviz']),
        ('share/' + package_name + '/worlds', ['worlds/warehouse.sdf']),
        ('share/' + package_name + '/models/drone', ['models/drone/model.sdf']),
        ('share/' + package_name + '/models/ground_robot', ['models/ground_robot/model.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='A multi-robot simulation package with modern Gazebo (gz-sim) integration, drone following, and object detection.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'drone_follower = multi_robot_follow.drone_follower:main',
            'object_detection_node = multi_robot_follow.object_detection_node:main',
            'obstacle_comm_node = multi_robot_follow.obstacle_comm_node:main',
            'ground_robot_nav = multi_robot_follow.ground_robot_nav:main',
        ],
    },
)
