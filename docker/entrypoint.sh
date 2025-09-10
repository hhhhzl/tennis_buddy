#!/usr/bin/env bash
set -euo pipefail

# Defaults
export USER="${USER:-developer}"
export HOME="${HOME:-/home/${USER}}"
export DISPLAY="${DISPLAY:-:1}"

# Supervisor files go under the user's home (no root perms needed)
export SUP_DIR="${HOME}/.supervisor"
export SUP_CONF="/etc/supervisor/conf.d/supervisord.conf"
mkdir -p "${SUP_DIR}"

# If the first arg looks like an option, pass it to supervisord.
if [[ "${1:-}" == "-"* ]]; then
  exec /usr/bin/supervisord -n -c "${SUP_CONF}" "$@"
fi

# If a custom command is provided (not the default shell), run it.
if [[ $# -gt 0 && "$1" != "/bin/bash" && "$1" != "bash" ]]; then
  exec "$@"
fi

# Default: start supervisord in foreground
exec /usr/bin/supervisord -n -c "${SUP_CONF}"