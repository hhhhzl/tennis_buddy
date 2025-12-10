#!/usr/bin/env bash
# Start Driver - can run on any device
# Usage: ./scripts/start/driver/start-driver-universal.sh <cloud_ip>

set -e

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

CLOUD_IP=${1}
if [ -z "$CLOUD_IP" ]; then
    echo "Usage: $0 <cloud_ip>"
    echo "  cloud_ip: IP address of device running Gazebo"
    exit 1
fi

cd "$PROJECT_ROOT"
"$PROJECT_ROOT/scripts/start/component/start-component.sh" driver "$CLOUD_IP"
