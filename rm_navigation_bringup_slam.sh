#!/usr/bin/env bash
set -eo pipefail
workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace_root/install/setup.bash"
exec ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  slam:=True use_robot_state_pub:=True "$@"
