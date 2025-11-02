#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_world = DeclareLaunchArgument('world', default_value='default')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    # Bridge for pose info from Gazebo
    gz_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            f'/world/{world}/pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
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
            'in_topic': f'/world/{world}/pose/info',
            'out_topic': '/ball_positions',
            'target_frame': 'map',
            'z_max': 0.25,
            'xmin': -4.0, 'xmax': 4.0, 'ymin': -7.0, 'ymax': 7.0,
            'max_balls': 60,
        }]
    )

    return LaunchDescription([
        declare_world,
        declare_use_sim_time,
        gz_pose_bridge,
        ball_groundtruth_node,
    ])
