#!/usr/bin/env bash
set -eo pipefail
workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace_root/install/setup.bash"
# 固定 cwd 到工作空间根目录：建图结束时 pcd_map_saver 会把 scans.pcd 写到这里，
# 与 save_map.sh 默认生成的 scans.pgm/scans.yaml 相邻。
cd "$workspace_root"
exec ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  slam:=True use_robot_state_pub:=True "$@"
