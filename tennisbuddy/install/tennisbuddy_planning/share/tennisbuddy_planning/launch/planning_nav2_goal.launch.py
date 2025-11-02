#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    slam = LaunchConfiguration('slam')

    declare_sim = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('tennisbuddy_planning'),
                                   'configs', 'nav2_params.yaml'))
    declare_map = DeclareLaunchArgument('map', default_value='')
    declare_slam = DeclareLaunchArgument('slam', default_value='False')

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

    sources_yaml = os.path.join(pkg_plan, 'config', 'sources.yaml')

    goal_pusher = Node(
        package='tennisbuddy_planning',
        executable='nav2_goal_pusher',
        output='screen',
        parameters=[sources_yaml, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_sim, declare_params, declare_map, declare_slam,
        nav_frontend,
        goal_pusher,
    ])
