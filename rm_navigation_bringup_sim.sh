#!/usr/bin/env bash
set -eo pipefail
workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace_root/install/setup.bash"

# 仿真专用：自动读取地图旁的 rmuc_2025.passages.json，并模拟机构完成反馈。
# 参数可在命令末尾覆写，例如：bash rm_navigation_bringup_sim.sh passage_file:=/path/field.passages.json
exec ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
       world:=scans \
       slam:=False \
       passage_file:=auto \
       simulate_deformation:=true \
       "$@"
