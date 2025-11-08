#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    slam = LaunchConfiguration('slam')
    publish_initial_pose = LaunchConfiguration('publish_initial_pose')
    initial_x = LaunchConfiguration('initial_pose_x')
    initial_y = LaunchConfiguration('initial_pose_y')
    initial_yaw = LaunchConfiguration('initial_pose_yaw')

    declare_sim = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('tennisbuddy_planning'),
                                   'configs', 'nav2_params.yaml'))
    declare_map = DeclareLaunchArgument('map', default_value='')
    declare_slam = DeclareLaunchArgument('slam', default_value='False')
    declare_publish_initial_pose = DeclareLaunchArgument('publish_initial_pose', default_value='False')
    declare_initial_x = DeclareLaunchArgument('initial_pose_x', default_value='-5.0')
    declare_initial_y = DeclareLaunchArgument('initial_pose_y', default_value='0.0')
    declare_initial_yaw = DeclareLaunchArgument('initial_pose_yaw', default_value='0.0')

    pkg_plan = get_package_share_directory('tennisbuddy_planning')
    nav_frontend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_plan, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_yaml,
            'slam': slam,
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items()
    )

    sources_yaml = os.path.join(pkg_plan, 'configs', 'sources.yaml')

    goal_pusher = Node(
        package='tennisbuddy_planning',
        executable='nav2_goal_pusher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': '/odometry/filtered',
            'frame_id': 'map',
            'pickup_distance': 0.2,
        }],
    )

    initial_pose_node = Node(
        condition=IfCondition(publish_initial_pose),
        package='tennisbuddy_planning',
        executable='publish_initial_pose',
        name='publish_initial_pose_once',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'x': initial_x,
                'y': initial_y,
                'yaw': initial_yaw,
                'frame_id': 'map',
                'publish_delay': 1.0,
            }
        ],
    )

    return LaunchDescription([
        declare_sim, 
        declare_params, 
        declare_map, 
        declare_slam,
        declare_publish_initial_pose, 
        declare_initial_x, 
        declare_initial_y, 
        declare_initial_yaw,
        nav_frontend,
        initial_pose_node,
        goal_pusher,
    ])
