#!/usr/bin/env bash
set -eo pipefail
workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace_root/install/setup.bash"

#exec ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
#slam:=True

exec ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
       world:=rmuc_2025 \
       slam:=False \
       passage_file:=/home/spaaaaace/Code/HBUT2025_rm_vision/2027_navigation_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/rmuc2025.json