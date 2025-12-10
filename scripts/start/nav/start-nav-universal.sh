#!/usr/bin/env bash
# Start Navigation - can run on any device
# Usage: ./scripts/start/nav/start-nav-universal.sh <cloud_ip> [map_file] [slam=true/false]

set -e

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

CLOUD_IP=${1}
if [ -z "$CLOUD_IP" ]; then
    echo "Usage: $0 <cloud_ip> [map_file] [slam]"
    echo "  cloud_ip: IP address of device running Gazebo"
    echo "  map_file: Map file path (optional, empty for SLAM)"
    echo "  slam: true/false (default: false)"
    echo ""
    echo "Examples:"
    echo "  $0 192.168.1.100 '' true     # Use SLAM"
    echo "  $0 192.168.1.100 map.yaml false  # Use map"
    exit 1
fi

shift  # Remove cloud_ip from args
cd "$PROJECT_ROOT"
"$PROJECT_ROOT/scripts/start/component/start-component.sh" nav "$CLOUD_IP" "$@"
