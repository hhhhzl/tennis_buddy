#!/usr/bin/env bash
set -euo pipefail

DISPLAY=${DISPLAY:-:1}
VNC_PASSWORD=${VNC_PASSWORD:-tennisbuddy}
GEOMETRY=${GEOMETRY:-1600x900}
DEPTH=${DEPTH:-24}

mkdir -p "$HOME/.vnc"

# Create password
echo "$VNC_PASSWORD" | vncpasswd -f > "$HOME/.vnc/passwd"
chmod 600 "$HOME/.vnc/passwd"

# Ultra-safe xstartup: never crashes even if no WM/xterm installed
cat > "$HOME/.vnc/xstartup" <<'EOF'
#!/usr/bin/env bash
# Minimal desktop: solid background + never exit
( command -v xsetroot >/dev/null 2>&1 && xsetroot -solid '#303030' ) || true
exec sh -c 'while :; do sleep 3600; done'
EOF
chmod +x "$HOME/.vnc/xstartup"

# Clean old session
vncserver -kill "$DISPLAY" >/dev/null 2>&1 || true
rm -f "$HOME/.vnc/*${DISPLAY}*" 2>/dev/null || true

# Start in foreground with classic auth; bind for novnc
RFBAUTH="$HOME/.vnc/passwd"

exec vncserver "$DISPLAY" \
  -geometry "$GEOMETRY" \
  -depth "$DEPTH" \
  -localhost yes \
  -SecurityTypes VncAuth \
  -rfbauth "$RFBAUTH" \
  -xstartup "$HOME/.vnc/xstartup" \
  -fg
