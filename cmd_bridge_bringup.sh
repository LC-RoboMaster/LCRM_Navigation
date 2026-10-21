#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20
ros2 run domain_bridge domain_bridge /home/nuc03/HBUT2025_rm_vision/lc_navigation/lc_navigation_2025/navigation_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/cmd_bridge.yaml
