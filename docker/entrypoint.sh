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

if [[ "${1:-}" == "-"* ]]; then
  exec /usr/bin/supervisord -n -c "${SUP_CONF}" "$@"
fi

if [[ $# -gt 0 && "$1" != "/bin/bash" && "$1" != "bash" ]]; then
  exec "$@"
fi

exec /usr/bin/supervisord -n -c "${SUP_CONF}"