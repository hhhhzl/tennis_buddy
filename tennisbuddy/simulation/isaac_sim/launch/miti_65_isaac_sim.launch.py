import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Launch Isaac Sim with miti_65 robot.
    This maintains the same interface as the Gazebo launch file for compatibility.
    """
    # Create the launch configuration variables (same as Gazebo version)
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = os.path.join(get_package_share_directory(
        'tennisbuddy_description'), 'urdf', 'miti_65.urdf')
    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')

    robot_desc = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    # Launch arguments (same names as Gazebo for compatibility)
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='court.usd',
        description='USD world file to use in Isaac Sim (supports .sdf or .usd, auto-converts)'
    )

    declare_spawn_x_cmd = DeclareLaunchArgument(
        'spawn_x',
        default_value='-5.0',
        description='Robot spawn X offset in Isaac Sim world coordinates'
    )

    declare_spawn_y_cmd = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Robot spawn Y offset in Isaac Sim world coordinates'
    )

    # World file path
    world_path = PathJoinSubstitution([
        get_package_share_directory('tennisbuddy_isaac_sim'), 'worlds', world
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # Note: Isaac Sim needs to be launched separately or via Isaac ROS Bridge
    # This launch file sets up the ROS2 side. Users should launch Isaac Sim with:
    # - The ROS2 bridge extension enabled
    # - The world file loaded
    # - The robot spawned via URDF importer

    # Create the launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_spawn_x_cmd)
    ld.add_action(declare_spawn_y_cmd)

    # Add nodes
    ld.add_action(robot_state_publisher)

    return ld
