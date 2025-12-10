#!/usr/bin/env bash
set -euo pipefail

set +u
[ -f "/opt/ros/humble/setup.bash" ] && source /opt/ros/humble/setup.bash
[ -f "/workspace/tennisbuddy/install/setup.bash" ] && source /workspace/tennisbuddy/install/setup.bash
set -u

export DISPLAY="${DISPLAY:-:1}"
export SUP_DIR="/root/.supervisor"
export SUP_CONF="/etc/supervisor/conf.d/supervisord.conf"
mkdir -p "${SUP_DIR}"

# ROS2 Discovery Server configuration
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_DISCOVERY_SERVER="${ROS_DISCOVERY_SERVER:-;}"

# Start ROS2 Discovery Server for distributed communication
if [ -z "$ROS_DISCOVERY_SERVER" ] || [ "$ROS_DISCOVERY_SERVER" = ";" ]; then
    echo "[INFO] Gazebo container will act as ROS2 Discovery Server"
    echo "[INFO] ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    echo "[INFO] Discovery Server will be accessible on this container's network interface"
    echo "[INFO] Clients should connect to: <cloud_ip>:11811"
fi

# Start supervisor (VNC + Gazebo)
if [[ "${1:-}" == "-"* ]]; then
  exec /usr/bin/supervisord -n -c "${SUP_CONF}" "$@"
fi

if [[ $# -gt 0 && "$1" != "/bin/bash" && "$1" != "bash" ]]; then
  exec "$@"
fi

exec /usr/bin/supervisord -n -c "${SUP_CONF}"
