#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_world = DeclareLaunchArgument('world', default_value='tennis_world')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    gz_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/tennis_world/pose/info@ros_gz_interfaces/msg/Pose_V[gz.msgs.Pose_V',
        ],
        output='screen'
    )

    ball_groundtruth_node = Node(
        package='tennisbuddy_perception',
        executable='ball_groundtruth',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'world': world,  
            'in_topic': '/world/tennis_world/pose/info',
            'ball_name_prefix': 'tennis_ball_',
            'out_topic': '/ball_positions',
            'target_frame': 'map',
            'z_max': 0.25,
            'xmin': -4.0, 'xmax': 4.0, 'ymin': -7.0, 'ymax': 7.0,
            'max_balls': 15,
        }]
    )

    return LaunchDescription([
        declare_world,
        declare_use_sim_time,
        gz_pose_bridge,
        ball_groundtruth_node,
    ])