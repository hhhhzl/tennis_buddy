import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    """
    Unified launch file that allows choosing between Gazebo and Isaac Sim.
    Maintains compatibility with existing launch files.
    """
    # Launch arguments
    simulator = LaunchConfiguration('simulator')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')

    declare_simulator = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='Simulator to use: "gazebo" or "isaac_sim"',
        choices=['gazebo', 'isaac_sim']
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='court',
        description='World file name (without extension, or with .sdf/.usd extension)'
    )

    declare_spawn_x = DeclareLaunchArgument(
        'spawn_x',
        default_value='-5.0',
        description='Robot spawn X position'
    )

    declare_spawn_y = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Robot spawn Y position'
    )

    # Convert world file name based on simulator
    # Helper functions to map world file extensions
    world_gazebo = PythonExpression([
        "world + '.sdf' if not (world.endswith('.sdf') or world.endswith('.usd')) else ",
        "world.replace('.usd', '.sdf') if world.endswith('.usd') else world"
    ])
    
    world_isaac = PythonExpression([
        "world + '.usd' if not (world.endswith('.sdf') or world.endswith('.usd')) else ",
        "world.replace('.sdf', '.usd') if world.endswith('.sdf') else world"
    ])

    # Gazebo launch (only if simulator == 'gazebo')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('tennisbuddy_gazebo'),
            '/launch/miti_65_gazebo.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world_gazebo,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
        }.items(),
        condition=IfCondition(PythonExpression(["simulator == 'gazebo'"]))
    )

    # Isaac Sim launch (only if simulator == 'isaac_sim')
    isaac_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('tennisbuddy_isaac_sim'),
            '/launch/miti_65_isaac_sim.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world_isaac,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
        }.items(),
        condition=IfCondition(PythonExpression(["simulator == 'isaac_sim'"]))
    )

    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_simulator)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world)
    ld.add_action(declare_spawn_x)
    ld.add_action(declare_spawn_y)

    # Add simulator launches (only one will execute based on condition)
    ld.add_action(gazebo_launch)
    ld.add_action(isaac_sim_launch)

    return ld
