#!/usr/bin/env bash
set -euo pipefail

export USER="${USER:-developer}"
export HOME="${HOME:-/home/${USER}}"
export DISPLAY="${DISPLAY:-:1}"

export SUP_DIR="${HOME}/.supervisor"
export SUP_CONF="/etc/supervisor/conf.d/supervisord.conf"
mkdir -p "${SUP_DIR}"

if [[ "${1:-}" == "-"* ]]; then
  exec /usr/bin/supervisord -n -c "${SUP_CONF}" "$@"
fi

if [[ $# -gt 0 && "$1" != "/bin/bash" && "$1" != "bash" ]]; then
  exec "$@"
fi

exec /usr/bin/supervisord -n -c "${SUP_CONF}"