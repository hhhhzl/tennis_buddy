#!/usr/bin/env bash
# Start Driver - can run on any device
# Usage: ./scripts/start-driver-universal.sh <cloud_ip>

set -e

CLOUD_IP=${1}
if [ -z "$CLOUD_IP" ]; then
    echo "Usage: $0 <cloud_ip>"
    echo "  cloud_ip: IP address of device running Gazebo"
    exit 1
fi

./scripts/start-component.sh driver "$CLOUD_IP"
