# Isaac Sim Quick Start Guide

Get up and running with Isaac Sim in 5 minutes!

## Prerequisites

- NVIDIA GPU (recommended)
- Isaac Sim installed from NVIDIA Omniverse
- ROS2 Humble or later

## Installation

### 1. Install Isaac Sim
Download and install from [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/isaac-sim/)

### 2. Install Isaac ROS Bridge
```bash
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_bridge.git
cd ~/isaac_ros_ws
./src/isaac_ros_common/scripts/run_dev.sh
```

### 3. Build Tennisbuddy Isaac Sim Package
```bash
cd <tennisbuddy_workspace>
colcon build --packages-select tennisbuddy_isaac_sim
source install/setup.bash
```

### 4. Generate World Files
```bash
cd tennisbuddy/simulation/isaac_sim/scripts
python3 generate_all_worlds.py
```

## Usage

### Quick Start (3 Steps)

1. **Launch Isaac Sim** (manually or via script)
   - Open Isaac Sim from Omniverse Launcher
   - Enable ROS2 bridge extension
   - Load your world file

2. **Launch ROS2 Nodes**
   ```bash
   ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py world:=court.usd
   ```

3. **Run Your Workflow** (same as Gazebo!)
   ```bash
   ros2 launch tennisbuddy_perception ball_spawner.launch.py world:=tennis_world count:=15
   ros2 launch tennisbuddy_planning navigation_launch.py use_sim_time:=true
   ```

### Using Unified Launch (Easier!)

```bash
# Automatically handles world file conversion and simulator selection
ros2 launch tennisbuddy_simulation miti_65_sim.launch.py simulator:=isaac_sim world:=court
```

## Switching from Gazebo

If you're currently using Gazebo, switching is easy:

```bash
# Before (Gazebo)
ros2 launch tennisbuddy_gazebo miti_65_gazebo.launch.py world:=court.sdf

# After (Isaac Sim) - just change the package and extension!
ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py world:=court.usd
```

That's it! All your ROS2 nodes work without changes.

## Common Commands

```bash
# List available launch files
ros2 launch tennisbuddy_isaac_sim

# Launch with custom spawn position
ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py \
    world:=court.usd spawn_x:=0.0 spawn_y:=0.0

# Launch in unified mode
ros2 launch tennisbuddy_simulation miti_65_sim.launch.py \
    simulator:=isaac_sim world:=court use_sim_time:=true
```

## Troubleshooting

**Problem**: Topics not appearing
- **Solution**: Check Isaac Sim has ROS2 bridge extension enabled

**Problem**: World file not found
- **Solution**: Run `python3 generate_all_worlds.py` in scripts directory

**Problem**: Robot not spawning
- **Solution**: Ensure URDF importer extension is enabled in Isaac Sim

## Next Steps

- Read [README.md](README.md) for detailed documentation
- Check [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) for migrating existing workflows
- See [../README.md](../README.md) for general simulation information

## Need Help?

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/)
- [Isaac ROS Bridge](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_bridge)
- Check existing issues or create a new one
