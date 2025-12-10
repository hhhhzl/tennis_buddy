#!/usr/bin/env bash
# Start Gazebo on cloud device for distributed ROS2 setup

set -e

# Get cloud IP (can be overridden)
CLOUD_IP=${CLOUD_IP:-$(hostname -I | awk '{print $1}')}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

echo "======================================"
echo "Starting Gazebo on Cloud Device"
echo "======================================"
echo "Cloud IP: $CLOUD_IP"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS2 Discovery Server will listen on: $CLOUD_IP:11811"
echo ""

# Check if container already exists
if docker ps -a | grep -q tennisbuddy_gazebo; then
    echo "[INFO] Container exists, removing old one..."
    docker rm -f tennisbuddy_gazebo
fi

# Start container
echo "[INFO] Starting Docker container..."
docker run -d --name tennisbuddy_gazebo \
  --network host \
  --restart unless-stopped \
  -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  -e ROS_DISCOVERY_SERVER=";" \
  -e DISPLAY=:1 \
  -e VNC_PASSWORD=tennisbuddy \
  -v $(pwd):/workspace/tennisbuddy \
  tennisbuddy:gazebo

echo ""
echo "======================================"
echo "Gazebo Started Successfully!"
echo "======================================"
echo "Access Gazebo GUI: http://$CLOUD_IP:6080/vnc.html"
echo "Password: tennisbuddy"
echo ""
echo "ROS2 Discovery Server: $CLOUD_IP:11811"
echo "Share this IP with Nano devices"
echo ""
echo "View logs: docker logs -f tennisbuddy_gazebo"
echo "Stop: docker stop tennisbuddy_gazebo"
echo "======================================"
