#!/usr/bin/env python3
from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    # ---------- Launch args ----------
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    start_ekf = LaunchConfiguration('start_ekf')
    start_driver = LaunchConfiguration('start_driver')  # real：rover launch；Simulation False
    model = LaunchConfiguration('model')  # URDF/Xacro
    ros2_ctrl_config = LaunchConfiguration('ros2_ctrl_config')  # ros2_control yaml
    ekf_config = LaunchConfiguration('ekf_config')
    start_accessories = LaunchConfiguration('start_accessories')  # ）

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use Gazebo/Sim time'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Start RViz for TF/odom'),
        DeclareLaunchArgument('start_ekf', default_value='true', description='Start robot_localization EKF'),
        DeclareLaunchArgument('start_driver', default_value='false', description='Start rover launch (real HW)'),
        DeclareLaunchArgument('start_accessories', default_value='false', description='Start accessories.launch.py'),
        DeclareLaunchArgument(
            'model',
            default_value=str(get_package_share_path('tennisbuddy_description') / 'urdf/miti_65.urdf'),
            description='Absolute path to robot URDF/Xacro'),
        DeclareLaunchArgument(
            'ros2_ctrl_config',
            default_value=str(get_package_share_directory('tennisbuddy_ros2_control') + '/configs/miti_65_config.yaml'),
            description='ros2_control controllers YAML'),
        DeclareLaunchArgument(
            'ekf_config',
            default_value=str(get_package_share_directory('tennisbuddy_perception') + '/configs/robot_localization.yaml'),
            description='robot_localization EKF YAML'),
    ]

    # ---------- Robot description (URDF/Xacro) ----------
    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)

    # ---------- Nodes ----------
    # 1) robot_state_publisher + joint_state_publisher_gui/cli
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen'
    )
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'use_sim_time': use_sim_time}, ros2_ctrl_config, {'robot_description': robot_description}],
        output='screen'
    )
    js_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['base_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    hardware_config = Path(get_package_share_directory('tennisbuddy_ros2_control'), 'configs', 'miti_65_config.yaml')
    assert hardware_config.is_file()

    robot_driver = Node(
        package='ros2_control',
        name='ros2_control',
        executable='ros2_control',
        parameters=[hardware_config],
        output='screen',
        respawn=True,
        respawn_delay=1
    )

    # EKF
    ekf = Node(
        condition=IfCondition(start_ekf),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    # 5) accessories
    accessories_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tennisbuddy_perception'), 'launch', 'accessories.launch.py')
        ),
        condition=IfCondition(start_accessories)
    )

    # 6) RViz
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription(
        declare_args + [
            rsp,
            jsp,
            ros2_control_node,
            js_broadcaster_spawner,
            base_controller_spawner,
            robot_driver,
            ekf,
            accessories_launch,
            rviz
        ]
    )
