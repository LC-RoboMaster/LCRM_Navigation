#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/nuc03/HBUT2025_rm_vision/lc_navigation/lc_navigation_2025/navigation_ws/install/setup.bash

ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
slam:=True \
use_robot_state_pub:=True