#!/usr/bin/env bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

# Run with GUI (needs DISPLAY from VNC)
exec ign gazebo -v 4 "$@"