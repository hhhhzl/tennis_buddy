#!/usr/bin/env bash
# Build Docker images for distributed deployment
# Builds: base, gazebo, nodes images

set -e

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$PROJECT_ROOT"

echo "======================================"
echo "Building Distributed Docker Images"
echo "======================================"
echo "Project root: $PROJECT_ROOT"
echo ""

# Step 1: Build base image
echo "[1/3] Building base image..."
echo "--------------------------------------"
docker build -f docker/Dockerfile.base -t tennisbuddy:base .
echo ""

# Step 2: Build Gazebo image
echo "[2/3] Building Gazebo image..."
echo "--------------------------------------"
docker build -f docker/Dockerfile.gazebo -t tennisbuddy:gazebo .
echo ""

# Step 3: Build Nodes image
echo "[3/3] Building Nodes image..."
echo "--------------------------------------"
docker build -f docker/Dockerfile.nodes -t tennisbuddy:nodes .
echo ""

echo "======================================"
echo "Build Complete!"
echo "======================================"
echo ""
echo "Built images:"
docker images | grep tennisbuddy | head -3
echo ""
echo "To export images for transfer:"
echo "  docker save tennisbuddy:base tennisbuddy:nodes | gzip > tennisbuddy-nodes.tar.gz"
echo ""
echo "To import on another device:"
echo "  docker load < tennisbuddy-nodes.tar.gz"
echo ""
