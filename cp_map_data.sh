rm -rf /home/nuc03/HBUT2025_rm_vision/lc_navigation/lc_navigation_2025/navigation_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/scans.pcd
rm -rf /home/nuc03/HBUT2025_rm_vision/lc_navigation/lc_navigation_2025/navigation_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/scans.pgm
rm -rf /home/nuc03/HBUT2025_rm_vision/lc_navigation/lc_navigation_2025/navigation_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/scans.yaml

cp /home/nuc03/HBUT2025_rm_vision/lc_navigation/lc_navigation_2025/navigation_ws/src/pb2025_sentry_nav/point_lio/PCD/scans.pcd /home/nuc03/HBUT2025_rm_vision/lc_navigation/lc_navigation_2025/navigation_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality
cp /home/nuc03/HBUT2025_rm_vision/lc_navigation/lc_navigation_2025/navigation_ws/scans.pgm /home/nuc03/HBUT2025_rm_vision/lc_navigation/lc_navigation_2025/navigation_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality
cp /home/nuc03/HBUT2025_rm_vision/lc_navigation/lc_navigation_2025/navigation_ws/scans.yaml /home/nuc03/HBUT2025_rm_vision/lc_navigation/lc_navigation_2025/navigation_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality
colcon build