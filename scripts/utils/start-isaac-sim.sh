#!/usr/bin/env bash
# Start Isaac Sim in Docker container
# This script assumes Isaac Sim is available in the container

export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

# Check if Isaac Sim is available
if command -v isaac-sim &> /dev/null; then
    # Launch Isaac Sim
    exec isaac-sim "$@"
elif [ -d "/isaac-sim" ]; then
    # Launch from standard Isaac Sim installation path
    exec /isaac-sim/isaac-sim.sh "$@"
else
    echo "[ERROR] Isaac Sim not found in container"
    echo "[INFO] Please ensure Isaac Sim base image is used: nvcr.io/nvidia/isaac-sim:<tag>"
    echo "[INFO] Or install Isaac Sim in the container"
    exit 1
fi
