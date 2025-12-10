#!/usr/bin/env python3
"""
Perception launch file for Isaac Sim - compatible with Gazebo version.
Provides the same interface as perception_gazebo_gt.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch perception nodes for Isaac Sim.
    Maintains compatibility with Gazebo version by using the same topics and parameters.
    """
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_world = DeclareLaunchArgument('world', default_value='tennis_world')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    # Note: Isaac Sim uses different topics than Gazebo for pose information
    # This bridge node translates Isaac Sim topics to match Gazebo interface
    # For now, we'll use the same ball_groundtruth node with Isaac Sim-compatible topic
    
    # Isaac Sim pose bridge (if using Isaac ROS Bridge)
    # This would bridge Isaac Sim topics to ROS2 topics
    # For example: /isaac/pose_array -> /world/tennis_world/pose/info
    
    isaac_pose_bridge = Node(
        package='isaac_ros_bridge',
        executable='isaac_ros_bridge_node',
        name='isaac_pose_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'ros_topic': '/world/tennis_world/pose/info',
            'isaac_topic': '/isaac/pose_array',
            'topic_type': 'geometry_msgs/msg/PoseArray'
        }],
        output='screen',
        condition=lambda context: False  # Disabled by default, enable if using Isaac ROS Bridge
    )

    # Ball groundtruth node (same as Gazebo version)
    # This node works with both simulators since it uses ROS2 topics
    ball_groundtruth_node = Node(
        package='tennisbuddy_perception',
        executable='ball_groundtruth',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'world': world,
            'in_topic': '/world/tennis_world/pose/info',  # Same topic as Gazebo
            'out_topic': '/ball_positions',
            'target_frame': 'map',
            'z_max': 0.25,
            'xmin': -4.0, 'xmax': 4.0, 'ymin': -7.0, 'ymax': 7.0,
            'max_balls': 15,
            'robot_odom_topic': '/odometry/filtered',
            'robot_exclusion_radius': 0.6,
        }]
    )

    return LaunchDescription([
        declare_world,
        declare_use_sim_time,
        # isaac_pose_bridge,  # Enable if using Isaac ROS Bridge
        ball_groundtruth_node,
    ])
