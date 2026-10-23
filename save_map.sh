#!/usr/bin/env bash
set -eo pipefail
workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace_root/install/setup.bash"
exec ros2 run nav2_map_server map_saver_cli -f "${1:-scans}" "${@:2}"
