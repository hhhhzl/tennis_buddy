# Scripts Guide

Complete guide for all scripts in the Tennisbuddy project.

## Directory Structure

```
scripts/
├── build/              # Build scripts
│   └── build-distributed.sh
├── start/              # Component startup scripts
│   ├── component/      # Universal component launcher
│   │   └── start-component.sh
│   ├── distributed/    # Distributed deployment
│   │   └── start-all-distributed.sh
│   ├── gazebo/         # Gazebo simulator
│   │   ├── start-gazebo.sh
│   │   ├── start-gazebo-cloud.sh
│   │   └── start-gazebo-universal.sh
│   ├── driver/         # Robot driver
│   │   ├── start-driver-nano.sh
│   │   └── start-driver-universal.sh
│   ├── nav/            # Navigation stack
│   │   ├── start-nav-cloud.sh
│   │   └── start-nav-universal.sh
│   ├── cv/             # Computer vision/perception
│   │   ├── start-cv-cloud.sh
│   │   └── start-cv-universal.sh
│   └── nodes/          # Node containers
│       └── start-nodes-nano.sh
└── utils/              # Utility scripts
    ├── setup.sh
    ├── start-ros2.sh
    ├── start-vnc.sh
    └── start-isaac-sim.sh
```

## Build Scripts

### build-distributed.sh

Build all Docker images for distributed deployment.

```bash
./scripts/build/build-distributed.sh
```

Builds:
1. `tennisbuddy:base` - Base image
2. `tennisbuddy:gazebo` - Gazebo simulator image
3. `tennisbuddy:nodes` - Nodes image for Nano

## Component Startup Scripts

### Universal Component Launcher

**Location**: `scripts/start/component/start-component.sh`

Universal script to start any component on any device.

```bash
./scripts/start/component/start-component.sh <component> <gazebo_ip> [args...]
```

**Components**:
- `gazebo` - Gazebo simulator
- `driver` - Robot driver
- `nav` - Navigation stack
- `cv` - Computer vision/perception

**Examples**:
```bash
# Start Gazebo on any device
./scripts/start/component/start-component.sh gazebo $(hostname -I | awk '{print $1}')

# Start Driver on any device
./scripts/start/component/start-component.sh driver 192.168.1.100

# Start Navigation with SLAM
./scripts/start/component/start-component.sh nav 192.168.1.100 '' true

# Start CV/Perception
./scripts/start/component/start-component.sh cv 192.168.1.100 tennis_world 15
```

### Gazebo Scripts

#### start-gazebo-universal.sh

Start Gazebo on any device (auto-detects IP).

```bash
./scripts/start/gazebo/start-gazebo-universal.sh [this_device_ip]
```

#### start-gazebo-cloud.sh

Start Gazebo on cloud device (legacy script, still works).

```bash
./scripts/start/gazebo/start-gazebo-cloud.sh
```

### Driver Scripts

#### start-driver-universal.sh

Start driver on any device.

```bash
./scripts/start/driver/start-driver-universal.sh <gazebo_ip>
```

#### start-driver-nano.sh

Start driver on Nano device (legacy script, still works).

```bash
./scripts/start/driver/start-driver-nano.sh <cloud_ip>
```

### Navigation Scripts

#### start-nav-universal.sh

Start navigation stack on any device.

```bash
./scripts/start/nav/start-nav-universal.sh <gazebo_ip> [map_file] [slam]
```

**Parameters**:
- `gazebo_ip`: IP of device running Gazebo (required)
- `map_file`: Map file path (optional, empty for SLAM)
- `slam`: `true`/`false` (default: `false`)

**Examples**:
```bash
# Use SLAM (no map)
./scripts/start/nav/start-nav-universal.sh 192.168.1.100 '' true

# Use existing map
./scripts/start/nav/start-nav-universal.sh 192.168.1.100 /path/to/map.yaml false
```

#### start-nav-cloud.sh

Start navigation on cloud device (legacy script, still works).

```bash
./scripts/start/nav/start-nav-cloud.sh [map_file] [slam]
```

### CV/Perception Scripts

#### start-cv-universal.sh

Start CV/perception on any device.

```bash
./scripts/start/cv/start-cv-universal.sh <gazebo_ip> [world] [ball_count] [spawn_balls]
```

**Parameters**:
- `gazebo_ip`: IP of device running Gazebo (required)
- `world`: World name (default: `tennis_world`)
- `ball_count`: Number of balls (default: `15`)
- `spawn_balls`: `true`/`false` (default: `true`)

**Examples**:
```bash
# Default configuration
./scripts/start/cv/start-cv-universal.sh 192.168.1.100

# Custom configuration
./scripts/start/cv/start-cv-universal.sh 192.168.1.100 tennis_world 20 true
```

#### start-cv-cloud.sh

Start CV on cloud device (legacy script, still works).

```bash
./scripts/start/cv/start-cv-cloud.sh [world] [ball_count] [spawn_balls]
```

### Distributed Scripts

#### start-all-distributed.sh

Start all components in distributed mode.

```bash
./scripts/start/distributed/start-all-distributed.sh <cloud_ip>
```

## Utility Scripts

### setup.sh

Setup Python dependencies and install packages.

```bash
./scripts/utils/setup.sh
```

### start-ros2.sh

Start ROS2 environment setup.

```bash
./scripts/utils/start-ros2.sh
```

### start-vnc.sh

Start VNC server for GUI access.

```bash
./scripts/utils/start-vnc.sh
```

### start-isaac-sim.sh

Start Isaac Sim simulator.

```bash
./scripts/utils/start-isaac-sim.sh
```

## Common Usage Patterns

### Standard Configuration (Gazebo on Cloud, Others on Nano)

```bash
# Terminal 1 - Cloud: Gazebo
./scripts/start/gazebo/start-gazebo-universal.sh
# Note the IP displayed, e.g., 192.168.1.100

# Terminal 2 - Nano: Driver
./scripts/start/driver/start-driver-universal.sh 192.168.1.100

# Terminal 3 - Nano: Navigation
./scripts/start/nav/start-nav-universal.sh 192.168.1.100 '' true

# Terminal 4 - Nano: CV/Perception
./scripts/start/cv/start-cv-universal.sh 192.168.1.100
```

### All Components on Cloud

```bash
# Cloud - All terminals
CLOUD_IP=192.168.1.100
./scripts/start/gazebo/start-gazebo-universal.sh
./scripts/start/driver/start-driver-universal.sh $CLOUD_IP
./scripts/start/nav/start-nav-universal.sh $CLOUD_IP '' true
./scripts/start/cv/start-cv-universal.sh $CLOUD_IP
```

### Mixed Deployment

```bash
# Cloud: Gazebo + Navigation
./scripts/start/gazebo/start-gazebo-universal.sh
./scripts/start/nav/start-nav-universal.sh <cloud_ip> '' true

# Nano: Driver + CV
./scripts/start/driver/start-driver-universal.sh <cloud_ip>
./scripts/start/cv/start-cv-universal.sh <cloud_ip>
```

## Environment Variables

All scripts support the following environment variables:

```bash
# ROS domain ID (must be same on all devices)
export ROS_DOMAIN_ID=0

# Custom container name (avoid conflicts)
export CONTAINER_NAME=my_custom_name

# Override Discovery Server (advanced)
export ROS_DISCOVERY_SERVER=<custom_ip>:11811
```

**Example**:
```bash
# Run multiple Nav instances on same device
CONTAINER_NAME=nav1 ./scripts/start/nav/start-nav-universal.sh <gazebo_ip>
CONTAINER_NAME=nav2 ./scripts/start/nav/start-nav-universal.sh <gazebo_ip>
```

## Verification

### Check Running Containers

```bash
docker ps | grep tennisbuddy
```

### Check ROS2 Topics

```bash
# In any container
ros2 topic list

# Should see topics from all running components:
# /cmd_vel          # From Navigation
# /odometry/filtered # From Driver
# /scan              # From Gazebo
# /ball_positions    # From CV
# /tf, /tf_static    # From various components
```

### Check ROS2 Nodes

```bash
ros2 node list
```

### Test Communication

```bash
# Publish test command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# Echo topic
ros2 topic echo /cmd_vel
```

## Troubleshooting

### Script Not Found

Make sure you're in the project root directory:
```bash
cd /path/to/tennisbuddy
```

### Permission Denied

Make scripts executable:
```bash
chmod +x scripts/**/*.sh
```

### Container Name Conflicts

Use `CONTAINER_NAME` environment variable:
```bash
CONTAINER_NAME=my_custom_name ./scripts/start/gazebo/start-gazebo-universal.sh
```

### Network Issues

Check connectivity:
```bash
ping <gazebo_ip>
telnet <gazebo_ip> 11811
```

### ROS2 Communication Issues

Check environment variables:
```bash
echo $ROS_DOMAIN_ID
echo $ROS_DISCOVERY_SERVER
```

Restart ROS2 daemon:
```bash
ros2 daemon stop
ros2 daemon start
```

## Additional Resources

- [Docker README](../docker/README.md) - Complete Docker documentation
- [Main README](../README.md) - Project overview
