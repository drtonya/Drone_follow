#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from pathlib import Path

ARGUMENTS = []

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))
    
def generate_launch_description():
    pkg_share = get_package_share_directory('multi_robot_follow')
    turtle_gz_share = get_package_share_directory('turtlebot4_gz_bringup')
    
    # Paths to resources
    world_file = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')
    warehouse_map = os.path.join(pkg_share, 'maps', 'warehouse_map.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'warehouse_rviz_config.rviz')
    drone_model = os.path.join(pkg_share, 'models', 'drone', 'model.sdf')
    ground_robot_model = os.path.join(pkg_share, 'models', 'ground_robot', 'model.sdf')
    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')
    
    robot_spawn_launch = PathJoinSubstitution(
        [turtle_gz_share, 'launch', 'turtlebot4_spawn.launch.py'])
    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim')

    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_share, 'worlds'),
            str(Path(drone_model).parent.resolve()),
            str(Path(pkg_turtlebot4_description).parent.resolve())        ])
    )

    # Paths
    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    # Gazebo harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [
                'warehouse.sdf',
                ' -r',
                ' -v 4'
            ])
        ]
    )

    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge',
                        executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock'
                        ])

    
    # Spawn ground robot using ros_gz_sim's "create" node
    spawn_ground_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', 'tbot')
            # ('rviz', LaunchConfiguration('rviz')),
            # ('x', LaunchConfiguration('x')),
            # ('y', LaunchConfiguration('y')),
            # ('z', LaunchConfiguration('z')),
            # ('yaw', LaunchConfiguration('yaw'))
            ]
    )
    
    # Spawn drone using ros_gz_sim's "create" node
    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-entity', 'drone', '-file', drone_model, '-x', '2.0', '-y', '0.0', '-z', '1.0'],
        output='screen'
    )
    
    # Multi-robot nodes
    drone_follower = Node(
        package='multi_robot_follow',
        executable='drone_follower',
        name='drone_follower'
    )
    object_detection_node = Node(
        package='multi_robot_follow',
        executable='object_detection_node',
        name='object_detection_node'
    )
    obstacle_comm_node = Node(
        package='multi_robot_follow',
        executable='obstacle_comm_node',
        name='obstacle_comm_node'
    )
    ground_robot_nav = Node(
        package='multi_robot_follow',
        executable='ground_robot_nav',
        name='ground_robot_nav'
    )
    
    # Nav2 Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': warehouse_map}]
    )
    
    # RViz2 visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config]
    )
    
    # Use TimerAction for a staggered startup sequence
    ld = LaunchDescription()
    ld.add_action(gz_resource_path)
    ld.add_action(gazebo)
    ld.add_action(clock_bridge)
    ld.add_action(spawn_ground_robot)
    ld.add_action(spawn_drone)
    ld.add_action(drone_follower)
    ld.add_action(object_detection_node)
    ld.add_action(obstacle_comm_node)
    ld.add_action(ground_robot_nav)
    ld.add_action(map_server)
    ld.add_action(rviz2)
    return ld
