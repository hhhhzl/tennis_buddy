# Isaac Sim Docker Support

This document provides detailed information about running Isaac Sim in Docker containers.

## Overview

Isaac Sim Docker support allows you to:
- Run Isaac Sim in a containerized environment
- Maintain consistency across different systems
- Use GPU acceleration with NVIDIA Docker runtime
- Access GUI via VNC/noVNC web interface

## Prerequisites

### Required

1. **NVIDIA GPU** (required for Isaac Sim)
2. **NVIDIA Docker Runtime** (`nvidia-container-toolkit`)
3. **Docker 20.10+** with GPU support

### Install NVIDIA Container Toolkit

```bash
# Ubuntu/Debian
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# Verify installation
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

## Building Docker Images

### Option 1: Using Main Dockerfile

```bash
docker build -f docker/Dockerfile \
  --build-arg ROS_DISTRO=humble \
  --build-arg SIM=isaac_sim \
  --build-arg BASE_IMAGE=nvcr.io/nvidia/isaac-sim:2023.1.1 \
  -t tennisbuddy:isaac_sim .
```

### Option 2: Using Isaac Sim Specific Dockerfile

```bash
docker build -f docker/Dockerfile.isaac-sim \
  --build-arg ISAAC_SIM_VERSION=2023.1.1 \
  -t tennisbuddy:isaac_sim .
```

### Option 3: Using Docker Compose

```bash
# Build Isaac Sim service
docker-compose -f docker/docker-compose.yml build isaac_sim

# Or build both
docker-compose -f docker/docker-compose.yml build
```

## Running Containers

### Basic Run

```bash
docker run -it --rm \
  --gpus all \
  -p 5901:5901 \
  -p 6080:6080 \
  tennisbuddy:isaac_sim
```

### With Volume Mounting

```bash
docker run -it --rm \
  --gpus all \
  -p 5901:5901 \
  -p 6080:6080 \
  -v $(pwd):/workspace/tennisbuddy \
  -e SIMULATOR=isaac_sim \
  tennisbuddy:isaac_sim
```

### Using Docker Compose

```bash
# Start Isaac Sim
docker-compose -f docker/docker-compose.yml up isaac_sim

# Or in detached mode
docker-compose -f docker/docker-compose.yml up -d isaac_sim
```

## Accessing the GUI

1. **Web Browser** (Recommended):
   - Open: http://localhost:6080/vnc.html
   - Password: `tennisbuddy`

2. **VNC Client**:
   - Connect to `localhost:5901`
   - Password: `tennisbuddy`

## Usage Inside Container

### Initial Setup

```bash
# Inside container
source /opt/ros/humble/setup.bash
cd /workspace/tennisbuddy

# Build workspace
colcon build
source install/setup.bash

# Generate USD world files (if not already done)
cd tennisbuddy/simulation/isaac_sim/scripts
python3 generate_all_worlds.py
```

### Launch Isaac Sim

```bash
# Method 1: Manual launch
# Isaac Sim should already be running via supervisord
# If not, launch manually:
/isaac-sim/isaac-sim.sh

# Method 2: Using ROS2 launch
ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py world:=court.usd
```

### Full Stack Launch

```bash
# Terminal 1: Isaac Sim (already running)
# Terminal 2: Ball spawner
ros2 launch tennisbuddy_perception ball_spawner.launch.py world:=tennis_world count:=15

# Terminal 3: Perception
ros2 launch tennisbuddy_perception perception_gazebo_gt.launch.py world:=tennis_world

# Terminal 4: Navigation
ros2 launch tennisbuddy_planning navigation_launch.py use_sim_time:=true
```

## Environment Variables

Control container behavior:

```bash
docker run -it --rm \
  --gpus all \
  -e SIMULATOR=isaac_sim \
  -e WORLD=court \
  -e HEADLESS=false \
  -e VNC_PASSWORD=mypassword \
  tennisbuddy:isaac_sim
```

Available variables:
- `SIMULATOR`: `gazebo` or `isaac_sim` (default: `gazebo`)
- `WORLD`: World file name without extension
- `HEADLESS`: Run without GUI (`true`/`false`)
- `VNC_PASSWORD`: VNC access password
- `DISPLAY`: X11 display (default: `:1`)

## Troubleshooting

### GPU Not Accessible

**Problem**: `--gpus all` not working

**Solution**:
```bash
# Verify NVIDIA Docker runtime
docker info | grep -i runtime

# Test GPU access
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi

# If not working, reinstall nvidia-container-toolkit
```

### Isaac Sim Not Launching

**Problem**: Isaac Sim executable not found

**Solution**:
- Verify base image includes Isaac Sim: `nvcr.io/nvidia/isaac-sim:2023.1.1`
- Check Isaac Sim path: `ls -la /isaac-sim/`
- Check container logs: `docker logs <container_id>`

### ROS2 Bridge Not Working

**Problem**: Topics not appearing

**Solution**:
```bash
# Inside container, check topics
ros2 topic list

# Check Isaac Sim ROS2 bridge extension is enabled
# (This needs to be done in Isaac Sim GUI)

# Verify bridge is running
ps aux | grep bridge
```

### Performance Issues

**Problem**: Slow simulation

**Solution**:
- Ensure GPU is being used: `nvidia-smi` inside container
- Use headless mode if GUI not needed: `-e HEADLESS=true`
- Reduce graphics quality in Isaac Sim settings

### Port Conflicts

**Problem**: Ports already in use

**Solution**:
```bash
# Use different ports
docker run -it --rm \
  --gpus all \
  -p 5902:5901 \
  -p 6081:6080 \
  tennisbuddy:isaac_sim
```

## Advanced Configuration

### Custom Isaac Sim Version

```bash
docker build -f docker/Dockerfile \
  --build-arg BASE_IMAGE=nvcr.io/nvidia/isaac-sim:2023.2.0 \
  -t tennisbuddy:isaac_sim_2023.2 .
```

### Multiple Containers

Run both Gazebo and Isaac Sim simultaneously:

```bash
# Terminal 1: Gazebo
docker run -it --rm -p 5901:5901 -p 6080:6080 tennisbuddy:gazebo

# Terminal 2: Isaac Sim (different ports)
docker run -it --rm --gpus all -p 5902:5901 -p 6081:6080 tennisbuddy:isaac_sim
```

### Network Configuration

For inter-container communication:

```bash
# Create network
docker network create tennisbuddy_net

# Run with network
docker run -it --rm \
  --gpus all \
  --network tennisbuddy_net \
  --name tennisbuddy_isaac \
  tennisbuddy:isaac_sim
```

### Resource Limits

Limit GPU and memory usage:

```bash
docker run -it --rm \
  --gpus '"device=0"' \
  --memory="8g" \
  --cpus="4" \
  tennisbuddy:isaac_sim
```

## Best Practices

1. **Use Docker Compose** for easier management
2. **Mount workspace** for development: `-v $(pwd):/workspace/tennisbuddy`
3. **Use environment variables** for configuration
4. **Separate images** for production vs development
5. **Keep base images updated** for security

## See Also

- [Main Docker README](../../docker/README.md)
- [Isaac Sim README](README.md)
- [Migration Guide](MIGRATION_GUIDE.md)
