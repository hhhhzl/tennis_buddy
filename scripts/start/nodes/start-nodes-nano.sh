#!/usr/bin/env bash
# Start ROS2 nodes on Nano device, connecting to cloud Gazebo

set -e

# Cloud IP is required
CLOUD_IP=${1}
if [ -z "$CLOUD_IP" ]; then
    echo "======================================"
    echo "Usage: $0 <cloud_ip>"
    echo "======================================"
    echo "Example: $0 192.168.1.100"
    echo "         $0 cloud.example.com"
    echo "         $0 123.45.67.89"
    exit 1
fi

ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

echo "======================================"
echo "Starting ROS2 Nodes on Nano"
echo "======================================"
echo "Cloud IP: $CLOUD_IP"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Connecting to Discovery Server: $CLOUD_IP:11811"
echo ""

# Test connectivity
echo "[INFO] Testing connection to cloud..."
if command -v ping > /dev/null 2>&1; then
    if ping -c 1 -W 2 "$CLOUD_IP" > /dev/null 2>&1; then
        echo "[INFO] ✓ Network connectivity OK"
    else
        echo "[WARNING] ✗ Cannot ping $CLOUD_IP"
        echo "[WARNING] Please check network connectivity"
        read -p "Continue anyway? (y/N) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

# Check if container exists
if docker ps -a | grep -q tennisbuddy_nodes; then
    echo "[INFO] Container exists, removing old one..."
    docker rm -f tennisbuddy_nodes
fi

# Start container
echo "[INFO] Starting Docker container..."
docker run -it --rm --name tennisbuddy_nodes \
  --network host \
  -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  -e ROS_DISCOVERY_SERVER="${CLOUD_IP}:11811" \
  -v $(pwd):/workspace/tennisbuddy \
  tennisbuddy:nodes \
  bash -c "
    source /opt/ros/humble/setup.bash
    [ -f /workspace/tennisbuddy/install/setup.bash ] && source /workspace/tennisbuddy/install/setup.bash
    echo ''
    echo '======================================'
    echo 'Container Ready!'
    echo '======================================'
    echo 'ROS_DOMAIN_ID: $ROS_DOMAIN_ID'
    echo 'ROS_DISCOVERY_SERVER: ${CLOUD_IP}:11811'
    echo ''
    echo 'Test connection:'
    echo '  ros2 topic list          # Should see Gazebo topics'
    echo '  ros2 node list           # Should see Gazebo nodes'
    echo ''
    echo 'Start driver:'
    echo '  ros2 launch tennisbuddy_ros2_control miti_65.launch.py use_sim_time:=true'
    echo ''
    echo 'Start planning:'
    echo '  ros2 launch tennisbuddy_planning navigation_launch.py use_sim_time:=true'
    echo ''
    echo 'Start perception:'
    echo '  ros2 launch tennisbuddy_perception ball_spawner.launch.py world:=tennis_world count:=15'
    echo '  ros2 launch tennisbuddy_perception perception_gazebo_gt.launch.py world:=tennis_world'
    echo '======================================'
    exec bash
  "
