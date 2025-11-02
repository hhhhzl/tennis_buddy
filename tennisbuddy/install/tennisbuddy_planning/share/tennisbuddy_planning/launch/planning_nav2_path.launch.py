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

    declare_sim = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('tennisbuddy_planning'),
                                   'configs', 'nav2_params.yaml'))

    pkg_plan = get_package_share_directory('tennisbuddy_planning')

    nav_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_plan, 'launch', 'nav2_backend.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
            'container_name': 'nav2_container'
        }.items()
    )

    planner_params = os.path.join(pkg_plan, 'configs', 'planner_params.yaml')
    sources_yaml = os.path.join(pkg_plan, 'configs', 'sources.yaml')

    planner = Node(
        package='tennisbuddy_planning',
        executable='planner_node',
        output='screen',
        parameters=[planner_params, sources_yaml, {'use_sim_time': use_sim_time}],
    )

    path_bridge = Node(
        package='tennisbuddy_planning',
        executable='nav2_path_bridge',
        output='screen',
        parameters=[sources_yaml, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_sim, declare_params,
        nav_backend,
        planner,
        path_bridge,
    ])
