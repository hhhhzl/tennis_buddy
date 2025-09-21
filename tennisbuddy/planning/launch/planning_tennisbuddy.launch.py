#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use sim time')

    pkg = get_package_share_directory('tennisbuddy_planning')
    planner_params = os.path.join(pkg, 'configs', 'planner_params.yaml')
    tracker_params = os.path.join(pkg, 'configs', 'tracker_params.yaml')
    sources_yaml = os.path.join(pkg, 'configs', 'sources.yaml')

    planner = Node(
        package='tennisbuddy_planning',
        executable='planner_node',
        output='screen',
        parameters=[planner_params, sources_yaml, {'use_sim_time': use_sim_time}],
    )

    tracker = Node(
        package='tennisbuddy_planning',
        executable='trajectory_tracker',
        output='screen',
        parameters=[tracker_params, sources_yaml, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        planner,
        tracker,
    ])
