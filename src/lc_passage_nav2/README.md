# LCRM 通道导航

ROS 2 Humble / Nav2 全局规划插件，普通区域复用 TDT 的代价 A*、安全走廊和 OSQP 后端，特殊区域通过带方向的图边连接，最终路径中的通道段固定为中线直线。Qt 编辑器位于仓库 `tools/passage_map_editor`，没有 ROS 或规划库链接依赖。

## 两种地图模式

- **边建图边导航**：`passage_file` 为空。每次规划复制当前 global costmap，允许地图宽高、分辨率及原点变化；未知区域默认不可通行。保留现有 Point-LIO、SLAM、定位和 Nav2 目标接口。
- **离线地图与通道**：加载原始地图 YAML 和编辑器导出的 `*.passages.json`。定位与地图保存使用原始 `map`；导航静态层使用 `passage_map`；实时障碍来自经过梁高过滤的 `terrain_map_nav`。

实车与仿真参数中的 `GridBased` 已替换为 `lc_passage_nav2/TdtPassagePlanner`。普通区域增加距障碍代价，改善最短路贴墙问题。通道区域按“机械保证中线可通过”的约定检查真实占据，不用普通膨胀宽度否决中线；真正落在中线上的实时障碍仍会阻止规划和执行。

## 运行接口

```bash
# 在线建图导航，不需要编辑器文件
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True

# 已建地图 + 原有 PCD 定位 + 通道
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  slam:=False map:=/absolute/field.yaml prior_pcd_file:=/absolute/field.pcd \
  passage_file:=/absolute/field.passages.json
```

`passage_file:=auto` 是顶层启动默认值：离线地图自动查找与 YAML 同目录、同名的 `field.passages.json`，文件不存在或处于建图模式则使用空配置。可显式传绝对路径；传空字符串可禁用自动加载。

`passage_file` 在启动时统一传给地图过滤、规划器和 PID 控制器。文件错误会使配置/启动失败，不会静默退回无通道模式。编辑后重新启动导航加载配置；运行期间不热改通道，以免执行中的路径与地图语义不一致。

`use_composition:=True/False` 两种启动方式均加载地图过滤组件。仿真可以显式增加 `simulate_deformation:=true` 运行变形反馈模拟器；默认关闭，实车必须接真实反馈。

## 构建与验证

在现有工作空间中按原方式 `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`。新增依赖为 `lc_passage_msgs`、Eigen、OpenCV core/imgproc、nlohmann-json、BehaviorTree.CPP v3；OSQP、OsqpEigen 和 QDLDL 已随源码提供，构建不下载求解器。系统依赖可由 `rosdep install --from-paths src --ignore-src -r -y` 安装。

```bash
colcon test --packages-select lc_passage_nav2 --event-handlers console_direct+
colcon test-result --verbose
```

独立核心验证：`bash src/lc_passage_nav2/scripts/check_sources.sh`。独立 Qt 工具的构建和操作说明见 [编辑器说明](../../tools/passage_map_editor/README.md)。

## 文档

- [架构、执行状态与边界](docs/ARCHITECTURE.md)
- [通道格式、参数与底盘接口](docs/INTERFACES.md)
- [测试覆盖与验证记录](docs/VALIDATION.md)
- [第三方来源与许可证](THIRD_PARTY_NOTICES.md)
