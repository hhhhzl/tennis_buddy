#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='default'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        ExecuteProcess(cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            f'/world/{ {"default": None} and ""}'
        ], shell=True, output='screen'),

        Node(
            package='tennisbuddy_perception',
            executable='ball_groundtruth',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'world': world,
                'in_topic': ['/world/', world, '/pose/info'][0],
                'out_topic': '/ball_positions',
                'target_frame': 'map',
                'z_max': 0.25,
                'xmin': -4.0, 'xmax': 4.0, 'ymin': -7.0, 'ymax': 7.0,
                'max_balls': 60,
            }]
        )
    ])
