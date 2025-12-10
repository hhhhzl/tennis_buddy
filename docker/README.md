# Docker Usage Guide

Complete guide for using Docker with Tennisbuddy, supporting both **Gazebo** and **Isaac Sim** simulators, including distributed deployment options.

## Table of Contents

- [Overview](#overview)
- [Requirements](#requirements)
- [Quick Start](#quick-start)
- [Distributed Setup](#distributed-setup)
- [Gazebo Simulator](#gazebo-simulator)
- [Isaac Sim](#isaac-sim)
- [Building Images](#building-images)
- [Running Containers](#running-containers)
- [Component Distribution](#component-distribution)
- [Troubleshooting](#troubleshooting)
- [Advanced Usage](#advanced-usage)

---

## Overview

Tennisbuddy provides Docker support for:
- **Gazebo**: Cross-platform simulator (MacOS, Linux, Windows)
- **Isaac Sim**: GPU-accelerated simulator (Linux with NVIDIA GPU)
- **Distributed Deployment**: Run Gazebo on cloud devices and nodes on lightweight devices (e.g., Nano)

### Architecture

```
┌─────────────────────────────────────┐
│      Cloud Device (Cloud)          │
│  ┌───────────────────────────────┐  │
│  │  Docker: tennisbuddy:gazebo   │  │
│  │  - Gazebo Simulator           │  │
│  │  - ROS2 DDS Server            │  │
│  │  - Port: 11811 (DDS)          │  │
│  │  - Port: 6080 (VNC Web)       │  │
│  └───────────────────────────────┘  │
│           ↓ ROS2 Topics              │
│      (Network via DDS)            │
│           ↑ ROS2 Topics              │
└─────────────────────────────────────┘
               │
               │ Internet/VPN/内网
               │
┌──────────────┴───────────────────────┐
│      Nano Device (Nano)              │
│  ┌───────────────────────────────┐  │
│  │  Docker: tennisbuddy:nodes    │  │
│  │  - Driver                     │  │
│  │  - Planning                   │  │
│  │  - Perception                 │  │
│  │  - ROS2 DDS Client            │  │
│  └───────────────────────────────┘  │
└─────────────────────────────────────┘
```

---

## Requirements

### Basic Requirements
- Docker 20.10+
- For Gazebo: Standard Docker (works on MacOS, Linux, Windows)
- For Isaac Sim: NVIDIA Docker runtime (`nvidia-container-toolkit`) and NVIDIA GPU
- For Distributed: Network connectivity between devices

### Isaac Sim Prerequisites

1. **Install NVIDIA Container Toolkit**:
```bash
# Ubuntu/Debian
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

2. **Verify GPU access**:
```bash
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

---

## Quick Start

### Standard Setup (Gazebo)

```bash
# Build image
docker build -f docker/Dockerfile \
  --build-arg ROS_DISTRO=humble \
  --build-arg SIM=gazebo \
  -t tennisbuddy:gazebo .

# Run container
docker run -it --rm \
  -p 5901:5901 \
  -p 6080:6080 \
  tennisbuddy:gazebo

# Access GUI: http://localhost:6080/vnc.html
# Password: tennisbuddy
```

### Distributed Setup

```bash
# 1. Build images
./scripts/build/build-distributed.sh

# 2. Start Gazebo on cloud device
./scripts/start/gazebo/start-gazebo-cloud.sh

# 3. Start components distributed:
#    - Driver on Nano
./scripts/start/driver/start-driver-nano.sh <cloud_ip>
#    - Navigation on Cloud
./scripts/start/nav/start-nav-cloud.sh
#    - CV/Perception on Cloud
./scripts/start/cv/start-cv-cloud.sh
```

---

## Distributed Setup

### Overview

Run Gazebo on a powerful cloud device and ROS2 nodes on lightweight Nano devices.

**Benefits**:
- ✅ Gazebo runs on powerful cloud (no CPU load on Nano)
- ✅ Nano only runs lightweight driver
- ✅ Navigation and CV run on cloud (compute-intensive)
- ✅ Works across different networks
- ✅ Easy to scale and deploy

### Architecture

- **Cloud Device**: 
  - `tennisbuddy:gazebo` - Gazebo Simulator
  - Navigation stack
  - CV/Perception nodes
- **Nano Device**: 
  - `tennisbuddy:nodes` - Driver only
- **Communication**: ROS2 DDS over network (port 11811)

### Step 1: Build Docker Images

#### Build Base Image (Required for all)

```bash
cd /path/to/tennisbuddy
docker build -f docker/Dockerfile.base -t tennisbuddy:base .
```

**Estimated time**: 10-20 minutes  
**Image size**: ~2-3 GB

#### Build Gazebo Image (Cloud Device)

```bash
docker build -f docker/Dockerfile.gazebo -t tennisbuddy:gazebo .
```

**Estimated time**: 5-10 minutes  
**Image size**: ~3-4 GB  
**Requires**: `tennisbuddy:base`

#### Build Nodes Image (Nano)

```bash
docker build -f docker/Dockerfile.nodes -t tennisbuddy:nodes .
```

**Estimated time**: 1-2 minutes  
**Image size**: ~2.5-3 GB  
**Requires**: `tennisbuddy:base`

#### Quick Build Script

```bash
./scripts/build/build-distributed.sh
```

#### Export/Import Images to Nano

```bash
# Export on build machine
docker save tennisbuddy:base tennisbuddy:nodes | gzip > tennisbuddy-nodes.tar.gz

# Transfer to Nano
scp tennisbuddy-nodes.tar.gz user@nano:/path/

# Import on Nano
docker load < tennisbuddy-nodes.tar.gz
```

### Step 2: Configure Network

#### Get Cloud Device IP

```bash
# Public IP
CLOUD_IP=$(curl -s ifconfig.me)
echo $CLOUD_IP

# Or local network IP
CLOUD_IP=$(hostname -I | awk '{print $1}')
echo $CLOUD_IP
```

#### Configure Firewall (Cloud Device)

```bash
# Ubuntu/Debian - Open ROS2 DDS port
sudo ufw allow 11811/udp
sudo ufw allow 11811/tcp

# Open VNC Web port (optional)
sudo ufw allow 6080/tcp

# Check firewall status
sudo ufw status
```

### Step 3: Start Gazebo on Cloud Device

#### Using Script (Recommended)

```bash
./scripts/start/gazebo/start-gazebo-cloud.sh
```

#### Manual Start

```bash
CLOUD_IP=$(hostname -I | awk '{print $1}')

docker run -d --name tennisbuddy_gazebo \
  --network host \
  --restart unless-stopped \
  -e ROS_DOMAIN_ID=0 \
  -e ROS_DISCOVERY_SERVER=";" \
  -e DISPLAY=:1 \
  -e VNC_PASSWORD=tennisbuddy \
  -v $(pwd):/workspace/tennisbuddy \
  tennisbuddy:gazebo

echo "Gazebo started. Access: http://$CLOUD_IP:6080/vnc.html"
echo "Discovery Server: $CLOUD_IP:11811"
```

#### Access Gazebo GUI

Open in browser: `http://<cloud_ip>:6080/vnc.html`
- **Password**: `tennisbuddy`

### Step 4: Start Nodes on Nano

#### Using Script (Recommended)

```bash
./scripts/start/driver/start-driver-nano.sh <cloud_ip>
```

#### Manual Start

```bash
CLOUD_IP="<your_cloud_ip>"  # Replace with actual IP

docker run -it --rm --name tennisbuddy_nodes \
  --network host \
  -e ROS_DOMAIN_ID=0 \
  -e ROS_DISCOVERY_SERVER="${CLOUD_IP}:11811" \
  -v $(pwd):/workspace/tennisbuddy \
  tennisbuddy:nodes
```

#### Inside Container

```bash
# Enter container
source /opt/ros/humble/setup.bash
source /workspace/tennisbuddy/install/setup.bash

# Test connection
ros2 topic list  # Should see Gazebo topics

# Start driver
ros2 launch tennisbuddy_ros2_control miti_65.launch.py \
  use_sim_time:=true \
  start_driver:=true \
  start_accessories:=true
```

### Step 5: Verify Communication

#### In Cloud Device Container

```bash
docker exec -it tennisbuddy_gazebo bash
source /opt/ros/humble/setup.bash
source /workspace/tennisbuddy/install/setup.bash

# View topics
ros2 topic list

# Should see topics from Nano, e.g.:
# /cmd_vel
# /joint_states
# /odometry/filtered
```

#### In Nano Container

```bash
# View topics
ros2 topic list

# Should see topics from Gazebo, e.g.:
# /scan
# /imu/data
# /clock
# /odometry/wheels
# /tf
# /tf_static
```

#### Test Topic Communication

```bash
# Publish test on Nano
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"

# Listen on cloud device
ros2 topic echo /cmd_vel
```

---

## Gazebo Simulator

### Supported Platforms
- ✅ MacOS (Apple Silicon)
- ✅ Linux
- ✅ Windows

### Build Image

```bash
docker build -f docker/Dockerfile \
  --build-arg ROS_DISTRO=humble \
  --build-arg SIM=gazebo \
  -t tennisbuddy:gazebo .
```

### Run Container

```bash
docker run -it --rm \
  -p 5901:5901 \
  -p 6080:6080 \
  tennisbuddy:gazebo
```

### Access GUI

Visit http://localhost:6080/vnc.html to access the Gazebo GUI.

**Password**: `tennisbuddy`

### Usage Inside Container

```bash
# Inside container
source /opt/ros/humble/setup.bash
cd /workspace/tennisbuddy
colcon build
source install/setup.bash

# Launch simulation
ros2 launch tennisbuddy_gazebo miti_65_gazebo.launch.py world:=court.sdf
```

---

## Isaac Sim

### Supported Platforms
- ✅ Linux (with NVIDIA GPU)
- ⚠️ MacOS (not supported - Isaac Sim requires NVIDIA GPU)
- ⚠️ Windows (not tested)

### Build Image

**Important**: Use NVIDIA Isaac Sim base image

```bash
docker build -f docker/Dockerfile \
  --build-arg ROS_DISTRO=humble \
  --build-arg SIM=isaac_sim \
  --build-arg BASE_IMAGE=nvcr.io/nvidia/isaac-sim:2023.1.1 \
  -t tennisbuddy:isaac_sim .
```

### Run Container

**Must use `--gpus all` for GPU access**

```bash
docker run -it --rm \
  --gpus all \
  -p 5901:5901 \
  -p 6080:6080 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  tennisbuddy:isaac_sim
```

### Access GUI

Visit http://localhost:6080/vnc.html to access the Isaac Sim GUI.

**Password**: `tennisbuddy`

### Usage Inside Container

```bash
# Inside container
source /opt/ros/humble/setup.bash
cd /workspace/tennisbuddy
colcon build
source install/setup.bash

# Note: Isaac Sim should be running with ROS2 bridge extension enabled
# Launch ROS2 nodes
ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py world:=court.usd
```

---

## Building Images

### Image Hierarchy

```
tennisbuddy:base (基础)
    ├── tennisbuddy:gazebo (Gazebo)
    └── tennisbuddy:nodes (Nano节点)
```

### Build All Images

```bash
./scripts/build/build-distributed.sh
```

### Individual Builds

```bash
# Base image
docker build -f docker/Dockerfile.base -t tennisbuddy:base .

# Gazebo image
docker build -f docker/Dockerfile.gazebo -t tennisbuddy:gazebo .

# Nodes image
docker build -f docker/Dockerfile.nodes -t tennisbuddy:nodes .
```

### Image Size Optimization

```bash
# View image sizes
docker images | grep tennisbuddy

# Optimization tips:
# 1. Use multi-stage builds (already in Dockerfile)
# 2. Clean apt cache (already in Dockerfile)
# 3. Remove unnecessary packages
# 4. Compress images when exporting (use gzip)
```

### Verify Build

```bash
# Check images exist
docker images | grep tennisbuddy

# Test base image
docker run --rm tennisbuddy:base ros2 --version

# Test Gazebo image
docker run --rm tennisbuddy:gazebo ros2 --version

# Test nodes image
docker run --rm tennisbuddy:nodes ros2 --version
```

---

## Running Containers

### Volume Mounting

Mount workspace for development:

```bash
docker run -it --rm \
  -p 5901:5901 -p 6080:6080 \
  -v $(pwd):/workspace/tennisbuddy \
  tennisbuddy:gazebo
```

### Custom Environment Variables

```bash
docker run -it --rm \
  -p 5901:5901 -p 6080:6080 \
  -e SIMULATOR=isaac_sim \
  -e WORLD=court \
  -e SPAWN_X=0.0 \
  -e SPAWN_Y=0.0 \
  tennisbuddy:isaac_sim
```

### Headless Mode (No GUI)

For Isaac Sim headless execution:

```bash
docker run -it --rm \
  --gpus all \
  -e HEADLESS=true \
  tennisbuddy:isaac_sim \
  ros2 launch tennisbuddy_isaac_sim miti_65_isaac_sim.launch.py \
    world:=court.usd headless:=true
```

---

## Component Distribution

### Flexible Component Deployment

All components (Gazebo, Driver, Nav, CV) can run on **any device** (cloud or Nano), allowing free combination.

### Universal Component Script

```bash
./scripts/start/component/start-component.sh <component> <gazebo_ip> [args...]
```

**Components**:
- `gazebo` - Gazebo simulator
- `driver` - Robot driver
- `nav` - Navigation stack
- `cv` - Computer vision/perception

### Convenience Scripts

```bash
# Gazebo (can run on any device)
./scripts/start/gazebo/start-gazebo-universal.sh [this_device_ip]

# Driver (can run on any device)
./scripts/start/driver/start-driver-universal.sh <gazebo_ip>

# Navigation (can run on any device)
./scripts/start/nav/start-nav-universal.sh <gazebo_ip> [map_file] [slam]

# CV/Perception (can run on any device)
./scripts/start/cv/start-cv-universal.sh <gazebo_ip> [world] [ball_count] [spawn_balls]
```

### Component Resource Requirements

| Component | CPU | Memory | Recommended Location |
|-----------|-----|--------|---------------------|
| Gazebo | High | 2-4GB | Cloud (needs GUI) |
| Driver | Low | 500MB-1GB | Nano (hardware connection) |
| Navigation | Medium-High | 1-2GB | Cloud (compute-intensive) |
| CV | Medium | 500MB-1GB | Cloud or Nano |

### Example: Standard Configuration

```bash
# Terminal 1 - Cloud: Gazebo
./scripts/start/gazebo/start-gazebo-universal.sh

# Terminal 2 - Nano: Driver
./scripts/start/driver/start-driver-universal.sh <cloud_ip>

# Terminal 3 - Nano: Navigation
./scripts/start/nav/start-nav-universal.sh <cloud_ip> '' true

# Terminal 4 - Nano: CV/Perception
./scripts/start/cv/start-cv-universal.sh <cloud_ip>
```

### Example: All on Cloud

```bash
# Cloud - All terminals
./scripts/start/gazebo/start-gazebo-universal.sh
./scripts/start/driver/start-driver-universal.sh <cloud_ip>
./scripts/start/nav/start-nav-universal.sh <cloud_ip> '' true
./scripts/start/cv/start-cv-universal.sh <cloud_ip>
```

---

## Troubleshooting

### Gazebo Issues

**Problem**: GUI not accessible
- **Solution**: Check ports are not blocked: `netstat -an | grep 6080`
- **Solution**: Try different port: `-p 6081:6080`

**Problem**: X11 display errors
- **Solution**: Container uses VNC, not X11. Access via web browser at http://localhost:6080/vnc.html

### Isaac Sim Issues

**Problem**: `--gpus all` not working
- **Solution**: Install nvidia-container-toolkit (see Prerequisites)
- **Solution**: Verify: `docker run --rm --gpus all nvidia/cuda:11.8.0-base nvidia-smi`

**Problem**: Isaac Sim not launching
- **Solution**: Verify NVIDIA base image is correct version
- **Solution**: Check GPU is accessible: `nvidia-smi`

**Problem**: ROS2 bridge not working
- **Solution**: Ensure Isaac Sim ROS2 bridge extension is enabled
- **Solution**: Check topics: `ros2 topic list` inside container

### Distributed Setup Issues

**Problem**: Nodes cannot communicate
- **Check**: Test network connection: `ping <cloud_ip>`
- **Check**: Test port: `telnet <cloud_ip> 11811` or `nc -zv <cloud_ip> 11811`
- **Check**: Firewall: `sudo ufw status`
- **Solution**: Ensure firewall allows port 11811 (UDP/TCP)
- **Solution**: Check ROS_DISCOVERY_SERVER environment variable is set correctly
- **Solution**: Use `--network host` mode (recommended)

**Problem**: Topic list is empty
- **Check**: Verify ROS_DOMAIN_ID is same: `echo $ROS_DOMAIN_ID`
- **Check**: Check environment variable: `echo $ROS_DISCOVERY_SERVER`
- **Solution**: Restart ROS2 daemon: `ros2 daemon stop && ros2 daemon start`

**Problem**: High latency
- **Optimization**: Use local network IP (if on same network)
- **Optimization**: Use VPN to reduce latency
- **Optimization**: Check network bandwidth: `iperf3 -c <cloud_ip>`

**Problem**: Nano resource insufficient
- **Optimization**: Use `tennisbuddy:nodes` image (lightweight, no GUI)
- **Optimization**: Don't run unnecessary nodes
- **Optimization**: Limit Docker resources:
```bash
docker run -it --rm \
  --memory="2g" --cpus="2" \
  -e ROS_DISCOVERY_SERVER="${CLOUD_IP}:11811" \
  tennisbuddy:nodes
```

### General Issues

**Problem**: Permission denied
- **Solution**: Use `--privileged` flag (not recommended for production)
- **Solution**: Check file permissions in mounted volumes

**Problem**: Network issues
- **Solution**: Use `--network host` for better network access (Linux only)

---

## Advanced Usage

### Docker Compose

Create `docker/docker-compose.yml` for easier management:

```yaml
version: '3.8'

services:
  gazebo:
    build:
      context: .
      dockerfile: docker/Dockerfile
      args:
        ROS_DISTRO: humble
        SIM: gazebo
    image: tennisbuddy:gazebo
    ports:
      - "5901:5901"
      - "6080:6080"
    volumes:
      - ./:/workspace/tennisbuddy
    environment:
      - SIMULATOR=gazebo

  isaac_sim:
    build:
      context: .
      dockerfile: docker/Dockerfile
      args:
        ROS_DISTRO: humble
        SIM: isaac_sim
        BASE_IMAGE: nvcr.io/nvidia/isaac-sim:2023.1.1
    image: tennisbuddy:isaac_sim
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
    ports:
      - "5902:5901"
      - "6081:6080"
    volumes:
      - ./:/workspace/tennisbuddy
    environment:
      - SIMULATOR=isaac_sim
      - DISPLAY=${DISPLAY}
```

Usage:
```bash
# Start Gazebo
docker-compose up gazebo

# Start Isaac Sim
docker-compose up isaac_sim
```

### Multi-stage Build (Development)

For faster development builds:

```bash
docker build \
  --target base \
  -f docker/Dockerfile \
  --build-arg ROS_DISTRO=humble \
  --build-arg SIM=gazebo \
  -t tennisbuddy:dev .
```

### Environment Variables

All scripts support the following environment variables:

```bash
# ROS domain ID (must be same on all devices)
export ROS_DOMAIN_ID=0

# Custom container name (avoid conflicts)
export CONTAINER_NAME=my_custom_name

# Override Discovery Server (advanced)
export ROS_DISCOVERY_SERVER=<custom_ip>:11811
```

### Monitoring and Debugging

```bash
# View network connections
netstat -an | grep 11811

# View ROS2 nodes
ros2 node list

# View topic bandwidth
ros2 topic hz /scan
ros2 topic bw /scan

# View topic details
ros2 topic info /cmd_vel
ros2 topic echo /cmd_vel --once

# View container resource usage
docker stats tennisbuddy_gazebo tennisbuddy_driver tennisbuddy_nav tennisbuddy_cv
```

### Security Recommendations

1. **Use VPN**: Don't expose ports directly to public network (recommended)
2. **Firewall**: Only open necessary ports
3. **Authentication**: Consider using ROS2 security features
4. **Encryption**: Use TLS/SSL to protect communication

---

## Notes

- Isaac Sim Docker images are large (~20GB+) - ensure sufficient disk space
- Isaac Sim requires NVIDIA GPU - won't work on CPU-only systems
- Gazebo works everywhere but is slower than GPU-accelerated Isaac Sim
- Both simulators can be installed in the same Dockerfile but requires conditional logic
- For production, consider separate Dockerfiles per simulator

---

## Additional Resources

- ROS2 Discovery Servers: https://docs.ros.org/en/humble/Concepts/About-Discovery-Servers.html
- ROS2 Security: https://docs.ros.org/en/humble/Concepts/Security.html
- Docker Documentation: https://docs.docker.com/
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/
