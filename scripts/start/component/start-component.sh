#!/usr/bin/env bash
# Universal component launcher - can run any component on any device
# Usage: ./scripts/start-component.sh <component> <cloud_ip> [component_args...]

set -e

COMPONENT=${1}
CLOUD_IP=${2}

if [ -z "$COMPONENT" ] || [ -z "$CLOUD_IP" ]; then
    echo "======================================"
    echo "Universal Component Launcher"
    echo "======================================"
    echo "Usage: $0 <component> <cloud_ip> [args...]"
    echo ""
    echo "Components:"
    echo "  gazebo    - Gazebo simulator"
    echo "  driver    - Robot driver"
    echo "  nav       - Navigation stack"
    echo "  cv        - CV/Perception"
    echo ""
    echo "Examples:"
    echo "  $0 gazebo <cloud_ip>                    # Start Gazebo on cloud"
    echo "  $0 driver <cloud_ip>                    # Start driver anywhere"
    echo "  $0 nav <cloud_ip> '' true               # Start nav with SLAM"
    echo "  $0 cv <cloud_ip> tennis_world 15        # Start CV with args"
    echo ""
    echo "Environment variables:"
    echo "  ROS_DOMAIN_ID          - ROS2 domain (default: 0)"
    echo "  CONTAINER_NAME         - Docker container name"
    echo "  ROS_DISCOVERY_SERVER   - Override discovery server"
    exit 1
fi

ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
CONTAINER_NAME=${CONTAINER_NAME:-tennisbuddy_${COMPONENT}}
DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-${CLOUD_IP}:11811}

echo "======================================"
echo "Starting Component: $COMPONENT"
echo "======================================"
echo "Cloud IP: $CLOUD_IP"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Container: $CONTAINER_NAME"
echo "Discovery Server: $DISCOVERY_SERVER"
echo ""

# Shift to get component-specific arguments
shift 2
COMPONENT_ARGS="$@"

# Test connectivity (unless it's Gazebo itself)
if [ "$COMPONENT" != "gazebo" ]; then
    if command -v ping > /dev/null 2>&1; then
        if ping -c 1 -W 2 "$CLOUD_IP" > /dev/null 2>&1; then
            echo "[INFO] ✓ Network connectivity OK"
        else
            echo "[WARNING] ✗ Cannot ping $CLOUD_IP"
            read -p "Continue anyway? (y/N) " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                exit 1
            fi
        fi
    fi
fi

# Check if container exists
if docker ps -a | grep -q "$CONTAINER_NAME"; then
    echo "[INFO] Stopping existing container..."
    docker stop "$CONTAINER_NAME" > /dev/null 2>&1 || true
    docker rm "$CONTAINER_NAME" > /dev/null 2>&1 || true
fi

# Build Docker command based on component
case $COMPONENT in
    gazebo)
        # Gazebo runs as discovery server
        DISCOVERY_SERVER=";"
        DOCKER_IMAGE="tennisbuddy:gazebo"
        LAUNCH_CMD="ros2 launch tennisbuddy_gazebo miti_65_gazebo.launch.py world:=court.sdf use_sim_time:=true"
        DOCKER_OPTS="--network host"
        ;;
    driver)
        DOCKER_IMAGE="tennisbuddy:nodes"
        LAUNCH_CMD="ros2 launch tennisbuddy_ros2_control miti_65.launch.py use_sim_time:=true"
        DOCKER_OPTS="--network host --device=/dev/ttyUSB0 --device=/dev/ttyUSB1 --device=/dev/ttyACM0 --device=/dev/ttyACM1"
        ;;
    nav)
        # Parse nav arguments: [map_file] [slam]
        # Split by space, handle empty map_file
        if [ -z "$COMPONENT_ARGS" ]; then
            MAP_FILE=""
            SLAM="false"
        else
            # Use read to properly parse arguments
            read -r MAP_FILE SLAM <<< "$COMPONENT_ARGS"
            MAP_FILE=${MAP_FILE:-}
            SLAM=${SLAM:-false}
        fi
        
        DOCKER_IMAGE="tennisbuddy:nodes"
        LAUNCH_CMD="ros2 launch tennisbuddy_planning navigation_launch.py use_sim_time:=true slam:=$SLAM"
        if [ -n "$MAP_FILE" ] && [ "$MAP_FILE" != "''" ] && [ "$MAP_FILE" != '""' ]; then
            LAUNCH_CMD="$LAUNCH_CMD map:=$MAP_FILE"
        fi
        DOCKER_OPTS="--network host"
        ;;
    cv)
        # Parse cv arguments: [world] [ball_count] [spawn_balls]
        # Use read to properly parse multiple arguments
        if [ -z "$COMPONENT_ARGS" ]; then
            WORLD="tennis_world"
            BALL_COUNT="15"
            SPAWN_BALLS="true"
        else
            read -r WORLD BALL_COUNT SPAWN_BALLS <<< "$COMPONENT_ARGS"
            WORLD=${WORLD:-tennis_world}
            BALL_COUNT=${BALL_COUNT:-15}
            SPAWN_BALLS=${SPAWN_BALLS:-true}
        fi
        
        DOCKER_IMAGE="tennisbuddy:nodes"
        if [ "$SPAWN_BALLS" = "true" ]; then
            # Spawn balls first, then start perception
            LAUNCH_CMD="ros2 launch tennisbuddy_perception ball_spawner.launch.py world:=$WORLD count:=$BALL_COUNT & sleep 2 && ros2 launch tennisbuddy_perception perception_gazebo_gt.launch.py world:=$WORLD"
        else
            LAUNCH_CMD="ros2 launch tennisbuddy_perception perception_gazebo_gt.launch.py world:=$WORLD"
        fi
        DOCKER_OPTS="--network host"
        ;;
    *)
        echo "[ERROR] Unknown component: $COMPONENT"
        echo "Available: gazebo, driver, nav, cv"
        exit 1
        ;;
esac

# Start container
echo "[INFO] Starting $COMPONENT container..."
docker run -it --rm --name "$CONTAINER_NAME" \
  $DOCKER_OPTS \
  -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  -e ROS_DISCOVERY_SERVER="$DISCOVERY_SERVER" \
  -v $(pwd):/workspace/tennisbuddy \
  $DOCKER_IMAGE \
  bash -c "
    source /opt/ros/humble/setup.bash
    [ -f /workspace/tennisbuddy/install/setup.bash ] && source /workspace/tennisbuddy/install/setup.bash
    echo ''
    echo '======================================'
    echo 'Component: $COMPONENT'
    echo '======================================'
    echo 'ROS_DOMAIN_ID: $ROS_DOMAIN_ID'
    echo 'ROS_DISCOVERY_SERVER: $DISCOVERY_SERVER'
    echo 'Container: $CONTAINER_NAME'
    [ -n '$COMPONENT_ARGS' ] && echo 'Args: $COMPONENT_ARGS'
    echo ''
    echo 'Starting $COMPONENT...'
    echo ''
    exec $LAUNCH_CMD
  "
