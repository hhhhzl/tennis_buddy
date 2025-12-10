#!/usr/bin/env bash
# Start CV/Perception on cloud device, connecting to cloud Gazebo

set -e

ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
CONTAINER_NAME=${CONTAINER_NAME:-tennisbuddy_cv}
CLOUD_IP=${CLOUD_IP:-$(hostname -I | awk '{print $1}')}

echo "======================================"
echo "Starting CV/Perception on Cloud"
echo "======================================"
echo "Cloud IP: $CLOUD_IP"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Container: $CONTAINER_NAME"
echo ""

# Parse arguments
WORLD=${1:-tennis_world}
BALL_COUNT=${2:-15}
SPAWN_BALLS=${3:-true}

# Check if container exists
if docker ps -a | grep -q "$CONTAINER_NAME"; then
    echo "[INFO] Stopping existing container..."
    docker stop "$CONTAINER_NAME" > /dev/null 2>&1 || true
    docker rm "$CONTAINER_NAME" > /dev/null 2>&1 || true
fi

# Start container with CV/Perception
echo "[INFO] Starting CV container..."
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
    echo 'CV/Perception Container Ready!'
    echo '======================================'
    echo 'ROS_DISCOVERY_SERVER: ${CLOUD_IP}:11811'
    echo 'World: $WORLD'
    echo 'Ball Count: $BALL_COUNT'
    echo ''
    
    # Spawn balls if requested
    if [ '$SPAWN_BALLS' = 'true' ]; then
        echo 'Spawning balls...'
        ros2 launch tennisbuddy_perception ball_spawner.launch.py world:=$WORLD count:=$BALL_COUNT &
        sleep 2
    fi
    
    echo 'Starting perception...'
    echo ''
    exec ros2 launch tennisbuddy_perception perception_gazebo_gt.launch.py world:=$WORLD
  "
