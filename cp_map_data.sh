#!/usr/bin/env bash
# 将一次建图的三个产物拷进 pb2025_nav_bringup 并重新构建：
#   scans.pcd  —— 由 pcd_map_saver 在 odom 系累积（不要用 point_lio/PCD/scans.pcd）
#   scans.pgm  —— 由 save_map.sh 保存
#   scans.yaml
# 默认从工作空间根目录读取，也可按 PCD、PGM、YAML 的顺序显式传入路径。
set -eo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
pkg_dir="$workspace_root/src/pb2025_sentry_nav/pb2025_nav_bringup"

src_pcd="${1:-$workspace_root/scans.pcd}"
src_pgm="${2:-$workspace_root/scans.pgm}"
src_yaml="${3:-$workspace_root/scans.yaml}"

for file in "$src_pcd" "$src_pgm" "$src_yaml"; do
  if [[ ! -f "$file" ]]; then
    echo "缺少文件: $file" >&2
    echo "请先用建图模式跑一遍（结束时会写 scans.pcd），再执行 save_map.sh 保存 pgm/yaml。" >&2
    exit 1
  fi
done

mkdir -p "$pkg_dir/pcd/reality" "$pkg_dir/map/reality"
cp -f "$src_pcd" "$pkg_dir/pcd/reality/scans.pcd"
cp -f "$src_pgm" "$pkg_dir/map/reality/scans.pgm"
cp -f "$src_yaml" "$pkg_dir/map/reality/scans.yaml"

source /opt/ros/humble/setup.bash
cd "$workspace_root"
colcon build --packages-select pb2025_nav_bringup
