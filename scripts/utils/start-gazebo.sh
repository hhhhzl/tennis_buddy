#!/usr/bin/env bash
# Start Gazebo simulator inside Docker container
# This script is used by supervisord in Docker containers
# Sets up OpenGL software rendering and launches Gazebo

set -e

# Set up software rendering (for headless/VNC environments)
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

# Source ROS2
source /opt/ros/humble/setup.bash

# Check if workspace is built
if [ -f /workspace/tennisbuddy/install/setup.bash ]; then
    source /workspace/tennisbuddy/install/setup.bash
fi

# Default values
WORLD=${WORLD:-court}
USE_SIM_TIME=${USE_SIM_TIME:-true}

echo "======================================"
echo "Starting Gazebo Simulator"
echo "======================================"
echo "World: $WORLD"
echo "Use Sim Time: $USE_SIM_TIME"
echo "DISPLAY: $DISPLAY"
echo "LIBGL_ALWAYS_SOFTWARE: $LIBGL_ALWAYS_SOFTWARE"
echo ""

# Launch Gazebo via ROS2 launch
exec ros2 launch tennisbuddy_gazebo miti_65_gazebo.launch.py \
    world:="${WORLD}.sdf" \
    use_sim_time:="$USE_SIM_TIME"
