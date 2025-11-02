import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time')
    declare_world = DeclareLaunchArgument(
        'world', default_value='court.sdf', description='World file name')

    # Launch Gazebo with robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('tennisbuddy_gazebo'),
            '/launch/miti_65_gazebo.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
        }.items()
    )

    # Spawn balls in Gazebo
    ball_spawner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('tennisbuddy_perception'),
            '/launch/ball_spawner.launch.py'
        ]),
        launch_arguments={
            'world': world,
            'count': '15',
            'xmin': '-3.5', 'xmax': '3.5',
            'ymin': '-6.0', 'ymax': '6.0',
        }.items()
    )

    # Ball groundtruth (simulate depth camera data)
    perception_gt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('tennisbuddy_perception'),
            '/launch/perception_gazebo_gt.launch.py'
        ]),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Navigation stack (Nav2)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('tennisbuddy_planning'),
            '/launch/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'False',  # Set to True if you want SLAM
            'map': '',  # Provide map path if using localization
        }.items()
    )

    # Planning with Nav2 Goal Pusher (uses Nav2's NavigateToPose)
    planning_goal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('tennisbuddy_planning'),
            '/launch/planning_nav2_goal.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Planning with Nav2 Path Bridge (uses custom planner + Nav2's FollowPath)
    # Uncomment this and comment out planning_goal_launch if you want to use custom planner
    # planning_path_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('tennisbuddy_planning'),
    #         '/launch/planning_nav2_path.launch.py'
    #     ]),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #     }.items()
    # )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        gazebo_launch,
        ball_spawner_launch,
        perception_gt_launch,
        nav_launch,
        planning_goal_launch,
    ])