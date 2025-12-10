#!/usr/bin/env bash
# Start all components distributed across cloud and Nano
# This script helps coordinate the distributed setup

set -e

CLOUD_IP=${1}
if [ -z "$CLOUD_IP" ]; then
    echo "======================================"
    echo "Usage: $0 <cloud_ip> [component]"
    echo "======================================"
    echo "Components:"
    echo "  gazebo   - Start Gazebo on cloud (default)"
    echo "  driver   - Start driver on Nano"
    echo "  nav      - Start navigation on cloud"
    echo "  cv       - Start CV/perception on cloud"
    echo "  all      - Start all components"
    echo ""
    echo "Examples:"
    echo "  $0 192.168.1.100 gazebo  # Start Gazebo"
    echo "  $0 192.168.1.100 driver  # Start driver (run on Nano)"
    echo "  $0 192.168.1.100 nav     # Start nav (run on cloud)"
    echo "  $0 192.168.1.100 cv      # Start CV (run on cloud)"
    echo "  $0 192.168.1.100 all     # Show instructions for all"
    exit 1
fi

COMPONENT=${2:-all}

echo "======================================"
echo "Distributed Tennisbuddy Setup"
echo "======================================"
echo "Cloud IP: $CLOUD_IP"
echo "Component: $COMPONENT"
echo ""

case $COMPONENT in
    gazebo)
        echo "[INFO] Starting Gazebo on cloud device..."
        ./scripts/start/gazebo/start-gazebo-cloud.sh
        ;;
    driver)
        echo "[INFO] Starting driver on Nano..."
        echo "[INFO] Run this command on Nano device!"
        ./scripts/start/driver/start-driver-nano.sh "$CLOUD_IP"
        ;;
    nav)
        echo "[INFO] Starting navigation on cloud..."
        echo "[INFO] Run this command on cloud device!"
        ./scripts/start/nav/start-nav-cloud.sh
        ;;
    cv)
        echo "[INFO] Starting CV/perception on cloud..."
        echo "[INFO] Run this command on cloud device!"
        ./scripts/start/cv/start-cv-cloud.sh
        ;;
    all)
        echo "======================================"
        echo "Distributed Setup Instructions"
        echo "======================================"
        echo ""
        echo "STEP 1: Start Gazebo on Cloud Device"
        echo "--------------------------------------"
        echo "On cloud device, run:"
        echo "  ./scripts/start/gazebo/start-gazebo-cloud.sh"
        echo "  # Access GUI at: http://$CLOUD_IP:6080/vnc.html"
        echo ""
        echo "STEP 2: Start Driver on Nano"
        echo "--------------------------------------"
        echo "On Nano device, run:"
        echo "  ./scripts/start/driver/start-driver-nano.sh $CLOUD_IP"
        echo ""
        echo "STEP 3: Start Navigation on Cloud (Optional)"
        echo "--------------------------------------"
        echo "On cloud device, run:"
        echo "  ./scripts/start/nav/start-nav-cloud.sh [map_file] [slam=true/false]"
        echo "  # Example: ./scripts/start/nav/start-nav-cloud.sh '' true"
        echo ""
        echo "STEP 4: Start CV/Perception on Cloud (Optional)"
        echo "--------------------------------------"
        echo "On cloud device, run:"
        echo "  ./scripts/start/cv/start-cv-cloud.sh [world] [ball_count]"
        echo "  # Example: ./scripts/start/cv/start-cv-cloud.sh tennis_world 15"
        echo ""
        echo "======================================"
        echo "Quick Start (All in Separate Terminals)"
        echo "======================================"
        echo ""
        echo "# Terminal 1 - Cloud: Gazebo"
        echo "./scripts/start/gazebo/start-gazebo-cloud.sh"
        echo ""
        echo "# Terminal 2 - Nano: Driver"
        echo "./scripts/start/driver/start-driver-nano.sh $CLOUD_IP"
        echo ""
        echo "# Terminal 3 - Cloud: Navigation"
        echo "./scripts/start/nav/start-nav-cloud.sh '' true"
        echo ""
        echo "# Terminal 4 - Cloud: CV/Perception"
        echo "./scripts/start/cv/start-cv-cloud.sh tennis_world 15"
        echo ""
        echo "======================================"
        ;;
    *)
        echo "[ERROR] Unknown component: $COMPONENT"
        echo "Available: gazebo, driver, nav, cv, all"
        exit 1
        ;;
esac
