#!/usr/bin/env bash
# Start CV/Perception - can run on any device
# Usage: ./scripts/start/cv/start-cv-universal.sh <cloud_ip> [world] [ball_count] [spawn_balls]

set -e

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

CLOUD_IP=${1}
if [ -z "$CLOUD_IP" ]; then
    echo "Usage: $0 <cloud_ip> [world] [ball_count] [spawn_balls]"
    echo "  cloud_ip: IP address of device running Gazebo"
    echo "  world: World name (default: tennis_world)"
    echo "  ball_count: Number of balls (default: 15)"
    echo "  spawn_balls: true/false (default: true)"
    echo ""
    echo "Examples:"
    echo "  $0 192.168.1.100"
    echo "  $0 192.168.1.100 tennis_world 20"
    exit 1
fi

shift  # Remove cloud_ip from args
cd "$PROJECT_ROOT"
"$PROJECT_ROOT/scripts/start/component/start-component.sh" cv "$CLOUD_IP" "$@"
