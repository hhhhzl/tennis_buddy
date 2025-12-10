#!/usr/bin/env bash
# Start navigation on cloud device, connecting to cloud Gazebo

set -e

ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
CONTAINER_NAME=${CONTAINER_NAME:-tennisbuddy_nav}
CLOUD_IP=${CLOUD_IP:-$(hostname -I | awk '{print $1}')}

echo "======================================"
echo "Starting Navigation on Cloud"
echo "======================================"
echo "Cloud IP: $CLOUD_IP"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Container: $CONTAINER_NAME"
echo ""

# Parse arguments
MAP_FILE=${1:-}
SLAM=${2:-false}

# Check if container exists
if docker ps -a | grep -q "$CONTAINER_NAME"; then
    echo "[INFO] Stopping existing container..."
    docker stop "$CONTAINER_NAME" > /dev/null 2>&1 || true
    docker rm "$CONTAINER_NAME" > /dev/null 2>&1 || true
fi

# Build launch command
LAUNCH_CMD="ros2 launch tennisbuddy_planning navigation_launch.py use_sim_time:=true slam:=$SLAM"
if [ -n "$MAP_FILE" ]; then
    LAUNCH_CMD="$LAUNCH_CMD map:=$MAP_FILE"
fi

# Start container with navigation
echo "[INFO] Starting navigation container..."
docker run -it --rm --name "$CONTAINER_NAME" \
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
    echo 'Navigation Container Ready!'
    echo '======================================'
    echo 'ROS_DISCOVERY_SERVER: ${CLOUD_IP}:11811'
    echo 'SLAM: $SLAM'
    [ -n '$MAP_FILE' ] && echo 'Map: $MAP_FILE'
    echo ''
    echo 'Starting navigation...'
    echo ''
    exec $LAUNCH_CMD
  "
