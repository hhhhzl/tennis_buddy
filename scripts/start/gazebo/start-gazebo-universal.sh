#!/usr/bin/env bash
# Start Gazebo - can run on any device
# Usage: ./scripts/start/gazebo/start-gazebo-universal.sh [cloud_ip]

set -e

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

CLOUD_IP=${1:-$(hostname -I | awk '{print $1}')}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

echo "======================================"
echo "Starting Gazebo (Universal)"
echo "======================================"
echo "Device IP: $CLOUD_IP"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS2 Discovery Server: $CLOUD_IP:11811"
echo ""

cd "$PROJECT_ROOT"
"$PROJECT_ROOT/scripts/start/component/start-component.sh" gazebo "$CLOUD_IP"
