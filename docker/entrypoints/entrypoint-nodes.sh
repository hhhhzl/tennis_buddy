#!/usr/bin/env bash
set -euo pipefail

set +u
[ -f "/opt/ros/humble/setup.bash" ] && source /opt/ros/humble/setup.bash
[ -f "/workspace/tennisbuddy/install/setup.bash" ] && source /workspace/tennisbuddy/install/setup.bash
set -u

# ROS2 Discovery Server configuration for distributed communication
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_DISCOVERY_SERVER="${ROS_DISCOVERY_SERVER:-}"

# Get container IP for debugging
CONTAINER_IP=$(hostname -I | awk '{print $1}') 2>/dev/null || echo "unknown"

echo "========================================"
echo "ROS2 Node Container"
echo "========================================"
echo "Container IP: $CONTAINER_IP"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_DISCOVERY_SERVER: ${ROS_DISCOVERY_SERVER:-<not set>}"
echo ""

# Validate ROS_DISCOVERY_SERVER is set
if [ -z "$ROS_DISCOVERY_SERVER" ]; then
    echo "[WARNING] ROS_DISCOVERY_SERVER not set!"
    echo "[INFO] For distributed communication, set:"
    echo "  export ROS_DISCOVERY_SERVER=<cloud_ip>:11811"
    echo "[INFO] Connecting to default DDS domain (local network only)"
    echo "[INFO] If you want to connect to cloud Gazebo, set ROS_DISCOVERY_SERVER"
else
    echo "[INFO] Connecting to ROS2 Discovery Server: $ROS_DISCOVERY_SERVER"
    # Parse IP and port from ROS_DISCOVERY_SERVER
    # Format: <ip>:<port>
    IFS=':' read -r DISCOVERY_IP DISCOVERY_PORT <<< "$ROS_DISCOVERY_SERVER"
    DISCOVERY_PORT=${DISCOVERY_PORT:-11811}
    echo "[INFO] Discovery Server: $DISCOVERY_IP:$DISCOVERY_PORT"
    
    # Test connectivity
    if command -v ping > /dev/null 2>&1; then
        if ping -c 1 -W 2 "$DISCOVERY_IP" > /dev/null 2>&1; then
            echo "[INFO] ✓ Network connectivity OK"
        else
            echo "[WARNING] ✗ Cannot ping discovery server IP"
        fi
    fi
fi

echo "========================================"
echo ""

if [[ "${1:-}" == "-"* ]]; then
  exec "$@"
fi

if [[ $# -gt 0 && "$1" != "/bin/bash" && "$1" != "bash" ]]; then
  exec "$@"
fi

exec /bin/bash
