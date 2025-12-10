# Tennisbuddy Isaac Sim Integration

This package provides Isaac Sim simulation support for the Tennisbuddy robot, maintaining full compatibility with the existing Gazebo setup.

## Overview

The Isaac Sim integration provides:
- ✅ Equivalent world environments in USD format
- ✅ ROS2 bridge configuration for seamless integration
- ✅ Launch files compatible with existing Gazebo launch file interfaces
- ✅ URDF robot support through Isaac Sim's URDF importer
- ✅ **Coexistence with Gazebo** - both simulators can be installed simultaneously

## Requirements

- **Isaac Sim 2023.1 or later** (NVIDIA Omniverse)
- **Isaac ROS Bridge package** (for ROS2 communication)
- **ROS2 Humble or later**
- **NVIDIA GPU** (recommended, but not strictly required for basic usage)

## Installation

### 1. Install Isaac Sim

Follow the [Isaac Sim installation guide](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_basic.html).

**Quick setup:**
```bash
# Download Isaac Sim from NVIDIA Omniverse
# Follow the installation instructions for your platform
```

### 2. Install Isaac ROS Bridge

```bash
# Create workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src

# Clone Isaac ROS packages
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_bridge.git

# Install dependencies
cd ..
./src/isaac_ros_common/scripts/run_dev.sh
# This will prompt for Docker or native setup

# Build
colcon build --symlink-install
source install/setup.bash
```

For detailed instructions, see: [Isaac ROS Bridge Installation](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_bridge)

### 3. Build Tennisbuddy Isaac Sim Package

```bash
cd <tennisbuddy_workspace>
colcon build --packages-select tennisbuddy_isaac_sim
source install/setup.bash
```

### 4. Generate USD World Files

```bash
cd tennisbuddy/simulation/isaac_sim/scripts
python3 generate_tennis_court_usd.py

# This will create court.usd in the worlds/ directory
```

## Usage

### Basic Launch

Launch Isaac Sim with the miti_65 robot:

```bash
ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py world:=court.usd
```

**Note**: You need to have Isaac Sim running with:
1. ROS2 bridge extension enabled
2. The world file loaded
3. The robot spawned via URDF importer

### Launch Arguments

- `world`: USD world file name (default: `court.usd`)
  - Supports both `.usd` and `.sdf` extensions (auto-converts)
- `spawn_x`: Robot spawn X position (default: `-5.0`)
- `spawn_y`: Robot spawn Y position (default: `0.0`)
- `use_sim_time`: Use simulation time (default: `true`)

### Unified Launch (Choose Simulator)

Use the unified launch file to choose between Gazebo and Isaac Sim:

```bash
# Use Gazebo (default)
ros2 launch tennisbuddy_simulation miti_65_sim.launch.py simulator:=gazebo world:=court

# Use Isaac Sim
ros2 launch tennisbuddy_simulation miti_65_sim.launch.py simulator:=isaac_sim world:=court
```

### Example: Full Stack Launch

```bash
# Terminal 1: Launch Isaac Sim
ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py world:=court.usd use_sim_time:=true

# Terminal 2: Spawn balls
ros2 launch tennisbuddy_perception ball_spawner.launch.py world:=tennis_world count:=15

# Terminal 3: Perception
ros2 launch tennisbuddy_perception perception_gazebo_gt.launch.py world:=tennis_world

# Terminal 4: Navigation
ros2 launch tennisbuddy_planning navigation_launch.py use_sim_time:=true slam:=true
```

## Compatibility with Gazebo

### Topic Compatibility

All ROS2 topics remain the same between Gazebo and Isaac Sim:
- `/cmd_vel` - Robot velocity commands
- `/odometry/wheels` - Wheel odometry
- `/joint_states` - Joint states
- `/scan` - Laser scan data
- `/imu/data` - IMU data
- `/clock` - Simulation clock

### Launch File Compatibility

The launch files use the same arguments as Gazebo:
- Same parameter names
- Same default values
- Same robot spawn positions
- Auto-conversion of world file extensions

### World Files

World files are converted from SDF (Gazebo) to USD (Isaac Sim):
- `court.sdf` ↔ `court.usd`
- `maze.sdf` ↔ `maze.usd`
- `warehouse.sdf` ↔ `warehouse.usd`
- `depot.sdf` ↔ `depot.usd`

Use the `world_file_mapper.py` script to convert between formats:
```bash
python3 scripts/world_file_mapper.py to_isaac court.sdf
# Output: court.usd
```

## Differences from Gazebo

| Feature | Gazebo | Isaac Sim |
|---------|--------|-----------|
| World Format | SDF | USD |
| Bridge | ros_gz_bridge | isaac_ros_bridge |
| Physics Engine | ODE/Bullet | PhysX |
| Performance | CPU-based | GPU-accelerated |
| Setup | Easier | More complex |

## Isaac Sim Setup Steps

1. **Launch Isaac Sim**
   ```bash
   # Launch Isaac Sim from Omniverse Launcher
   # Or use command line:
   /isaac-sim/isaac-sim.sh
   ```

2. **Enable ROS2 Bridge**
   - In Isaac Sim, go to Extensions
   - Enable "omni.isaac.ros2_bridge"
   - Restart Isaac Sim if needed

3. **Load World**
   - File → Open → Select `court.usd`
   - Or use the script: `python3 scripts/generate_tennis_court_usd.py`

4. **Spawn Robot**
   - Extensions → Isaac Utils → URDF Importer
   - Load your robot URDF
   - Position at spawn location

5. **Launch ROS2 Nodes**
   ```bash
   ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py
   ```

## Troubleshooting

### Isaac Sim not launching

- Ensure Isaac Sim is properly installed from NVIDIA Omniverse
- Check GPU drivers are up to date: `nvidia-smi`
- Verify NVIDIA GPU is available

### Topics not bridging

- Check Isaac ROS Bridge is properly installed
- Verify Isaac Sim is running with ROS2 bridge extension enabled
- Check topic names match between Isaac Sim and ROS2
- Use `ros2 topic list` to verify topics are available

### World file not found

- Generate USD world files using the provided script
- Verify world file path in launch arguments
- Check `worlds/` directory exists in the package

### URDF import issues

- Ensure URDF file is valid
- Check mesh file paths are correct
- Verify Isaac Sim URDF importer extension is enabled

### Performance issues

- Use GPU acceleration (requires NVIDIA GPU)
- Reduce simulation quality settings in Isaac Sim
- Consider headless mode for better performance

## Migration from Gazebo

To migrate existing workflows from Gazebo to Isaac Sim:

1. **No code changes needed** - all ROS2 topics are identical
2. **Update launch files**: Change package name
   ```python
   # From:
   tennisbuddy_gazebo
   # To:
   tennisbuddy_isaac_sim
   ```
3. **Update world files**: Generate USD versions
   ```bash
   python3 scripts/generate_tennis_court_usd.py
   ```
4. **Test compatibility**: Run the same ROS2 nodes with Isaac Sim

## Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html)
- [Isaac ROS Bridge](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_bridge)
- [URDF Import in Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros_import_urdf.html)
- [Isaac ROS Common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common)

## Contributing

When adding new world files or features:
1. Create both SDF and USD versions
2. Update `world_file_mapper.py` if needed
3. Test compatibility with both simulators
4. Update documentation
