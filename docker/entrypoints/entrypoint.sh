#!/usr/bin/env bash
set -euo pipefail

set +u
[ -f "/opt/ros/humble/setup.bash" ] && source /opt/ros/humble/setup.bash
set -u

#export USER="${USER:-developer}"

export DISPLAY="${DISPLAY:-:1}"
export SUP_DIR="/root/.supervisor"
export SUP_CONF="/etc/supervisor/conf.d/supervisord.conf"
mkdir -p "${SUP_DIR}"

# Set simulator based on environment variable or build arg
export SIMULATOR="${SIMULATOR:-${SIM:-gazebo}}"

# Enable appropriate simulator in supervisord based on SIMULATOR env var
if [[ "${SIMULATOR}" == "isaac_sim" ]] || [[ "${SIMULATOR}" == "isaac" ]]; then
  # Enable Isaac Sim, disable Gazebo
  supervisorctl -c "${SUP_CONF}" update gazebo &
  supervisorctl -c "${SUP_CONF}" start isaac_sim || true
elif [[ "${SIMULATOR}" == "gazebo" ]]; then
  # Enable Gazebo, disable Isaac Sim
  supervisorctl -c "${SUP_CONF}" update isaac_sim &
  supervisorctl -c "${SUP_CONF}" start gazebo || true
fi

if [[ "${1:-}" == "-"* ]]; then
  exec /usr/bin/supervisord -n -c "${SUP_CONF}" "$@"
fi

if [[ $# -gt 0 && "$1" != "/bin/bash" && "$1" != "bash" ]]; then
  exec "$@"
fi

exec /usr/bin/supervisord -n -c "${SUP_CONF}"