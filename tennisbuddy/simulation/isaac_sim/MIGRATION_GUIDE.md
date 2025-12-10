# Migration Guide: Gazebo to Isaac Sim

This guide helps you migrate from Gazebo to Isaac Sim or use both simulators interchangeably.

## Quick Start

### Option 1: Use Unified Launch (Recommended)

The easiest way to switch between simulators is using the unified launch file:

```bash
# Use Gazebo
ros2 launch tennisbuddy_simulation miti_65_sim.launch.py simulator:=gazebo world:=court

# Switch to Isaac Sim
ros2 launch tennisbuddy_simulation miti_65_sim.launch.py simulator:=isaac_sim world:=court
```

That's it! The unified launch handles all conversions automatically.

### Option 2: Direct Launch Files

```bash
# Gazebo
ros2 launch tennisbuddy_gazebo miti_65_gazebo.launch.py world:=court.sdf

# Isaac Sim
ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py world:=court.usd
```

## What Changes?

### ✅ No Changes Needed

- **ROS2 topics**: All topics remain identical
  - `/cmd_vel`
  - `/odometry/wheels`
  - `/joint_states`
  - `/scan`
  - `/imu/data`
  - `/clock`
- **ROS2 nodes**: All your nodes work without modification
- **Launch arguments**: Same parameter names and defaults
- **Robot spawn positions**: Identical coordinates

### 📝 What You Need to Update

1. **Launch file package name**:
   ```python
   # From:
   get_package_share_directory('tennisbuddy_gazebo')
   
   # To:
   get_package_share_directory('tennisbuddy_isaac_sim')
   ```

2. **World file extension** (if not using unified launch):
   ```python
   # From:
   world:='court.sdf'
   
   # To:
   world:='court.usd'
   ```

3. **Isaac Sim setup**: Install Isaac Sim and Isaac ROS Bridge (one-time setup)

## Step-by-Step Migration

### Step 1: Install Isaac Sim

See [README.md](README.md) for detailed installation instructions.

### Step 2: Generate USD World Files

```bash
cd tennisbuddy/simulation/isaac_sim/scripts
python3 generate_all_worlds.py
```

This creates USD equivalents of all SDF world files.

### Step 3: Update Your Launch Files

If you have custom launch files that include the simulator:

```python
# Before (Gazebo)
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('tennisbuddy_gazebo'),
        '/launch/miti_65_gazebo.launch.py'
    ]),
    launch_arguments={
        'world': 'court.sdf',
        'use_sim_time': 'true',
    }.items()
)
```

```python
# After (Isaac Sim)
isaac_sim_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('tennisbuddy_isaac_sim'),
        '/launch/miti_65_isaac_sim.launch.py'
    ]),
    launch_arguments={
        'world': 'court.usd',  # Changed extension
        'use_sim_time': 'true',
    }.items()
)
```

Or use the unified launch:

```python
# Unified (works with both)
unified_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('tennisbuddy_simulation'),
        '/launch/miti_65_sim.launch.py'
    ]),
    launch_arguments={
        'simulator': 'isaac_sim',  # or 'gazebo'
        'world': 'court',  # No extension needed!
        'use_sim_time': 'true',
    }.items()
)
```

### Step 4: Test Compatibility

Run your full stack with Isaac Sim:

```bash
# Terminal 1: Launch Isaac Sim
ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py world:=court.usd

# Terminal 2-4: Launch your existing nodes (no changes!)
ros2 launch tennisbuddy_perception ball_spawner.launch.py world:=tennis_world count:=15
ros2 launch tennisbuddy_perception perception_gazebo_gt.launch.py world:=tennis_world
ros2 launch tennisbuddy_planning navigation_launch.py use_sim_time:=true
```

Everything should work identically!

## Common Migration Scenarios

### Scenario 1: Switching Existing Workflow

**Before (Gazebo)**:
```bash
ros2 launch tennisbuddy_gazebo miti_65_gazebo.launch.py world:=court.sdf
# ... rest of your workflow
```

**After (Isaac Sim)**:
```bash
ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py world:=court.usd
# ... same workflow, no other changes!
```

### Scenario 2: Adding Simulator Choice to Scripts

If you have bash scripts or CI/CD pipelines:

```bash
# Add simulator selection
SIMULATOR=${1:-gazebo}  # Default to gazebo

if [ "$SIMULATOR" = "isaac_sim" ]; then
    ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py world:=court.usd
else
    ros2 launch tennisbuddy_gazebo miti_65_gazebo.launch.py world:=court.sdf
fi
```

Or use unified launch:

```bash
SIMULATOR=${1:-gazebo}
ros2 launch tennisbuddy_simulation miti_65_sim.launch.py simulator:=$SIMULATOR world:=court
```

### Scenario 3: Docker/Container Migration

Update your Docker setup to support both:

```dockerfile
# Install both simulators (optional)
RUN apt-get install -y ros-humble-gazebo-ros-pkgs
# Isaac Sim installation would be separate due to NVIDIA requirements
```

In your entrypoint:
```bash
if [ "$SIMULATOR" = "isaac_sim" ]; then
    # Launch Isaac Sim setup
    ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py world:=court.usd
else
    # Launch Gazebo
    ros2 launch tennisbuddy_gazebo miti_65_gazebo.launch.py world:=court.sdf
fi
```

## Troubleshooting

### World file not found

- **Solution**: Generate USD files using `generate_all_worlds.py`
- **Check**: Verify world files exist in `tennisbuddy_isaac_sim/worlds/`

### Topics not appearing

- **Gazebo**: Check `ros_gz_bridge` is running
- **Isaac Sim**: Check Isaac ROS Bridge is installed and Isaac Sim has ROS2 bridge extension enabled

### Robot not spawning

- **Gazebo**: Check robot description URDF is valid
- **Isaac Sim**: Ensure URDF importer extension is enabled in Isaac Sim

### Performance issues

- **Gazebo**: Reduce physics update rate, simplify world
- **Isaac Sim**: Use GPU acceleration, reduce graphics quality

## Rollback

To go back to Gazebo:

1. Simply use the Gazebo launch file:
   ```bash
   ros2 launch tennisbuddy_gazebo miti_65_gazebo.launch.py world:=court.sdf
   ```

2. No code changes needed - everything is compatible!

## Best Practices

1. **Use unified launch** when possible for easier switching
2. **Test with both simulators** to ensure compatibility
3. **Generate all USD files** upfront to avoid runtime issues
4. **Document simulator requirements** in your project README
5. **Use environment variables** for simulator selection in scripts

## Need Help?

- Check [Isaac Sim README](README.md) for detailed setup
- Check [Simulation README](../README.md) for general information
- Verify both simulators can coexist (they can!)
