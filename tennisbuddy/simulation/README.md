# Simulation Packages

This directory contains simulation packages for the Tennisbuddy robot, supporting both **Gazebo** and **Isaac Sim**.

## Overview

The simulation system is designed to support both simulators simultaneously, allowing you to choose which one to use based on your needs:

- **Gazebo**: Open-source, cross-platform, CPU-friendly
- **Isaac Sim**: NVIDIA GPU-accelerated, high-performance

Both simulators use the same ROS2 interface and topic names, ensuring full compatibility.

## Available Simulators

### Gazebo Sim

**Package**: `tennisbuddy_gazebo`

**Usage**:
```bash
ros2 launch tennisbuddy_gazebo miti_65_gazebo.launch.py world:=court.sdf
```

**Features**:
- ✅ Open-source
- ✅ Cross-platform (Linux, macOS, Windows)
- ✅ CPU-based physics
- ✅ Easy setup

See [gazebo/](gazebo/) directory for Gazebo-specific files.

### Isaac Sim

**Package**: `tennisbuddy_isaac_sim`

**Usage**:
```bash
ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py world:=court.usd
```

**Features**:
- ✅ GPU-accelerated physics
- ✅ High-performance simulation
- ✅ Advanced graphics and rendering
- ✅ NVIDIA GPU recommended

See [isaac_sim/README.md](isaac_sim/README.md) for detailed installation and usage instructions.

### Unified Launch (Choose Simulator)

**Location**: `simulation/launch/miti_65_sim.launch.py`

**Usage**:
```bash
# Use Gazebo (default)
ros2 launch tennisbuddy_simulation miti_65_sim.launch.py simulator:=gazebo world:=court

# Use Isaac Sim
ros2 launch tennisbuddy_simulation miti_65_sim.launch.py simulator:=isaac_sim world:=court
```

The unified launch file automatically:
- Maps world file extensions (`.sdf` ↔ `.usd`)
- Selects the appropriate simulator package
- Maintains consistent launch arguments

## Compatibility

### Topic Compatibility

Both simulators expose the same ROS2 topics:
- `/cmd_vel` - Robot velocity commands
- `/odometry/wheels` - Wheel odometry
- `/joint_states` - Joint states
- `/scan` - Laser scan data
- `/imu/data` - IMU data
- `/clock` - Simulation clock

### Launch Arguments

Both simulators support the same launch arguments:
- `use_sim_time` - Use simulation time (default: `true`)
- `world` - World file name (auto-converts between formats)
- `spawn_x` - Robot spawn X position (default: `-5.0`)
- `spawn_y` - Robot spawn Y position (default: `0.0`)

### World Files

World files are provided in both formats:
- **Gazebo**: `.sdf` files in `tennisbuddy_gazebo/worlds/`
- **Isaac Sim**: `.usd` files in `tennisbuddy_isaac_sim/worlds/`

World file names automatically map between formats:
- `court.sdf` ↔ `court.usd`
- `maze.sdf` ↔ `maze.usd`
- `warehouse.sdf` ↔ `warehouse.usd`
- `depot.sdf` ↔ `depot.usd`

Use the world file mapper script to convert:
```bash
python3 isaac_sim/scripts/world_file_mapper.py to_isaac court.sdf
```

## Installation

### Gazebo

Gazebo is typically installed as part of ROS2:
```bash
sudo apt-get install ros-humble-gazebo-ros-pkgs
```

### Isaac Sim

1. Install Isaac Sim from NVIDIA Omniverse
2. Install Isaac ROS Bridge
3. Generate USD world files

See [isaac_sim/README.md](isaac_sim/README.md) for detailed instructions.

## Choosing a Simulator

### Use Gazebo if:
- ✅ You need cross-platform compatibility
- ✅ You're using CPU-only systems
- ✅ You prefer open-source solutions
- ✅ You want easier setup
- ✅ You're doing basic testing

### Use Isaac Sim if:
- ✅ You have NVIDIA GPUs available
- ✅ You need better performance
- ✅ You want GPU-accelerated features
- ✅ You're doing ML/AI research
- ✅ You need high-fidelity graphics

### Both Can Coexist

- Both simulators can be installed simultaneously
- Only launch one at a time
- Switch between them using launch arguments
- All ROS2 code remains the same

## Migration Guide

To switch between simulators:

### Option 1: Use Unified Launch File
```bash
# Switch from Gazebo to Isaac Sim
ros2 launch tennisbuddy_simulation miti_65_sim.launch.py simulator:=isaac_sim world:=court
```

### Option 2: Use Simulator-Specific Launch Files
```bash
# Gazebo
ros2 launch tennisbuddy_gazebo miti_65_gazebo.launch.py world:=court.sdf

# Isaac Sim  
ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py world:=court.usd
```

### Option 3: Update Existing Launch Files
Change the package name in your launch files:
```python
# From:
get_package_share_directory('tennisbuddy_gazebo')
# To:
get_package_share_directory('tennisbuddy_isaac_sim')
```

And update world file extensions:
```python
# From:
world:='court.sdf'
# To:
world:='court.usd'
```

**Important**: All ROS2 nodes and topics remain unchanged!

## Examples

### Full Stack with Gazebo

```bash
# Terminal 1: Launch Gazebo
ros2 launch tennisbuddy_gazebo miti_65_gazebo.launch.py world:=court.sdf

# Terminal 2: Spawn balls
ros2 launch tennisbuddy_perception ball_spawner.launch.py world:=tennis_world count:=15

# Terminal 3: Perception
ros2 launch tennisbuddy_perception perception_gazebo_gt.launch.py world:=tennis_world

# Terminal 4: Navigation
ros2 launch tennisbuddy_planning navigation_launch.py use_sim_time:=true slam:=true
```

### Full Stack with Isaac Sim

```bash
# Terminal 1: Launch Isaac Sim
ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py world:=court.usd

# Terminal 2: Spawn balls (same as Gazebo!)
ros2 launch tennisbuddy_perception ball_spawner.launch.py world:=tennis_world count:=15

# Terminal 3: Perception (same as Gazebo!)
ros2 launch tennisbuddy_perception perception_gazebo_gt.launch.py world:=tennis_world

# Terminal 4: Navigation (same as Gazebo!)
ros2 launch tennisbuddy_planning navigation_launch.py use_sim_time:=true slam:=true
```

Notice: The only difference is the simulator launch file!

## Troubleshooting

### World file not found

- **Gazebo**: Check that `.sdf` files exist in `tennisbuddy_gazebo/worlds/`
- **Isaac Sim**: Generate `.usd` files using the provided scripts
- **Both**: Use the unified launch file for automatic conversion

### Topics not appearing

- Verify the simulator is running
- Check bridge nodes are active (Isaac Sim requires ROS2 bridge)
- Ensure `use_sim_time:=true` is set
- Use `ros2 topic list` to verify topics

### Switching between simulators

- Both can be installed simultaneously ✅
- Only launch one at a time
- Close one before launching the other
- Restart ROS2 nodes when switching

### Performance issues

- **Gazebo**: Reduce physics update rate, simplify world
- **Isaac Sim**: Use GPU acceleration, reduce graphics quality in headless mode

## Package Structure

```
simulation/
├── gazebo/              # Gazebo simulation package
│   ├── launch/          # Gazebo launch files
│   ├── worlds/          # SDF world files
│   └── ...
├── isaac_sim/           # Isaac Sim simulation package
│   ├── launch/          # Isaac Sim launch files
│   ├── worlds/          # USD world files
│   ├── scripts/         # World generation scripts
│   ├── config/          # Bridge configuration
│   └── README.md        # Detailed Isaac Sim docs
├── launch/              # Unified launch files
│   └── miti_65_sim.launch.py  # Simulator selector
└── README.md            # This file
```

## Additional Resources

- [Gazebo Documentation](https://gazebosim.org/docs)
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/)
- [Isaac ROS Bridge](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_bridge)
- [Isaac ROS Common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common)

## Contributing

When adding new features:
1. Ensure compatibility with both simulators
2. Create world files in both formats (SDF and USD)
3. Update launch files for both simulators
4. Test with both Gazebo and Isaac Sim
5. Update documentation
