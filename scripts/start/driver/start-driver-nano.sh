#!/usr/bin/env bash
# Start driver on Nano device, connecting to cloud Gazebo

set -e

# Cloud IP is required
CLOUD_IP=${1}
if [ -z "$CLOUD_IP" ]; then
    echo "======================================"
    echo "Usage: $0 <cloud_ip>"
    echo "======================================"
    echo "Example: $0 192.168.1.100"
    exit 1
fi

ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
CONTAINER_NAME=${CONTAINER_NAME:-tennisbuddy_driver}

echo "======================================"
echo "Starting Driver on Nano"
echo "======================================"
echo "Cloud IP: $CLOUD_IP"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Container: $CONTAINER_NAME"
echo ""

# Test connectivity
if command -v ping > /dev/null 2>&1; then
    if ping -c 1 -W 2 "$CLOUD_IP" > /dev/null 2>&1; then
        echo "[INFO] ✓ Network connectivity OK"
    else
        echo "[WARNING] ✗ Cannot ping $CLOUD_IP"
    fi
fi

# Check if container exists
if docker ps -a | grep -q "$CONTAINER_NAME"; then
    echo "[INFO] Stopping existing container..."
    docker stop "$CONTAINER_NAME" > /dev/null 2>&1 || true
    docker rm "$CONTAINER_NAME" > /dev/null 2>&1 || true
fi

# Start container with driver
echo "[INFO] Starting driver container..."
docker run -it --rm --name "$CONTAINER_NAME" \
  --network host \
  --device=/dev/ttyUSB0 --device=/dev/ttyUSB1 \
  --device=/dev/ttyACM0 --device=/dev/ttyACM1 \
  -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  -e ROS_DISCOVERY_SERVER="${CLOUD_IP}:11811" \
  -v $(pwd):/workspace/tennisbuddy \
  tennisbuddy:nodes \
  bash -c "
    source /opt/ros/humble/setup.bash
    [ -f /workspace/tennisbuddy/install/setup.bash ] && source /workspace/tennisbuddy/install/setup.bash
    echo ''
    echo '======================================'
    echo 'Driver Container Ready!'
    echo '======================================'
    echo 'ROS_DISCOVERY_SERVER: ${CLOUD_IP}:11811'
    echo ''
    echo 'Starting driver...'
    echo ''
    exec ros2 launch tennisbuddy_ros2_control miti_65.launch.py use_sim_time:=true
  "
