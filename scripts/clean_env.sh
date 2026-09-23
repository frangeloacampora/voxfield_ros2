#!/usr/bin/env bash
# Runs a command in a clean environment: only /opt/ros/jazzy + the voxfield_ws
# workspace install are sourced (no Hector underlay, see ROS2_PORT_PLAN.md §2.1).
#
# Usage:
#   scripts/clean_env.sh colcon build --symlink-install
#   scripts/clean_env.sh ros2 run voxfield_ros voxfield_server
set -euo pipefail

WS="${VOXFIELD_WS:-$HOME/voxfield_ws}"

if [ "$#" -eq 0 ]; then
  echo "Usage: $0 <command> [args...]" >&2
  exit 1
fi

env -i HOME="$HOME" USER="$USER" TERM="$TERM" DISPLAY="${DISPLAY:-}" \
  bash --noprofile --norc -c '
    # -u is deliberately not set here: /opt/ros/jazzy/setup.bash references
    # unset variables (e.g. AMENT_TRACE_SETUP_FILES) and is not nounset-safe.
    set -e -o pipefail
    source /opt/ros/jazzy/setup.bash
    if [ -f "'"$WS"'/install/setup.bash" ]; then
      source "'"$WS"'/install/setup.bash"
    fi
    cd "'"$WS"'"
    exec "$@"
  ' _ "$@"
