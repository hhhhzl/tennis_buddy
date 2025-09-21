#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='default'),
        DeclareLaunchArgument('count', default_value='15'),
        DeclareLaunchArgument('xmin', default_value='-3.5'),
        DeclareLaunchArgument('xmax', default_value='3.5'),
        DeclareLaunchArgument('ymin', default_value='-6.0'),
        DeclareLaunchArgument('ymax', default_value='6.0'),
        DeclareLaunchArgument('z', default_value='0.065'),
        DeclareLaunchArgument('radius', default_value='0.065'),
        DeclareLaunchArgument('mass', default_value='0.057'),
        DeclareLaunchArgument('name_prefix', default_value='tennis_ball_'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('allow_renaming', default_value='false'),
        DeclareLaunchArgument('color', default_value='[1.0,1.0,0.0]'),
        Node(
            package='tennisbuddy_perception',
            executable='ball_spawner',
            output='screen',
            parameters=[{
                'world': LaunchConfiguration('world'),
                'count': LaunchConfiguration('count'),
                'xmin': LaunchConfiguration('xmin'),
                'xmax': LaunchConfiguration('xmax'),
                'ymin': LaunchConfiguration('ymin'),
                'ymax': LaunchConfiguration('ymax'),
                'z': LaunchConfiguration('z'),
                'radius': LaunchConfiguration('radius'),
                'mass': LaunchConfiguration('mass'),
                'name_prefix': LaunchConfiguration('name_prefix'),
                'seed': LaunchConfiguration('seed'),
                'allow_renaming': LaunchConfiguration('allow_renaming'),
                'color': LaunchConfiguration('color'),
            }]
        )
    ])
