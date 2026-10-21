#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/nuc03/HBUT2025_rm_vision/lc_navigation/lc_navigation_2025/navigation_ws/install/setup.bash

ros2 run nav2_map_server map_saver_cli -f scans
