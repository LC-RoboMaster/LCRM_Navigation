# LCRM 通道导航

包拆分后的源码位置见 [过洞代码归属](../../docs/passage_packages.md)。本包只构建全局规划插件及其离线工具；状态机位于 PID 控制器包，地图和模拟反馈各有独立节点包。

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

## 仿真与 RViz 通道检查

`bash rm_navigation_bringup_sim.sh` 使用当前工作空间安装的 `rmuc_2025.yaml`，自动加载同目录的 `rmuc_2025.passages.json`，并开启**仿真专用**的变形完成反馈。可在脚本末尾传入 launch 参数覆写默认值。编辑工程为 `map/simulation/rmuc2025.json`；该工程已按 YAML 的 `origin=[-3.58,-9.44,0]` 转换全部通道、A/B 和测试点，测试机器人半径为 0.2 m、安全边距为 0.02 m。更换地图时请先在 editor 导入实际 YAML；单独打开图片会丢失地图原点。

默认 RViz 新增两个 MarkerArray 显示，话题随机器人 namespace 变化：

- `passage/markers`：旋转矩形、A/B、方向、中线、通道速度和梁高过滤区间。青色表示已启用，灰色表示禁用。控制器采用路径后，额外显示绿色固定线和 PREPARE/FOLD、ENTER、EXIT、RELEASE/RESTORE。准备点取自实际采用路径的弧长，不能简单用入口直线后退代替。
- `passage/planner_markers`：最近一次规划的方向检查。绿色为选中，黄色为物理检查可用但本次未选中，红色为拒绝；文字给出原因。黄色也可能是普通道路连接不可达，不能仅凭黄色判定“代价较高”。

两个主题可靠且 transient_local，RViz 后打开也能收到最近快照。没有通道文件时配置标记为空，普通在线建图导航保持可用。

```bash
# 使用当前仿真默认命名空间；查看每个通道方向的可用性和拒绝原因
ros2 topic echo /red_standard_robot1/passage/planner_diagnostics --field data --once --qos-durability transient_local
# 查看控制器实际采用的通道，空数组说明该路径没有过特殊路段
ros2 topic echo /red_standard_robot1/passage/active_plan --field traversals --once --qos-durability transient_local
```

矩形只是机器人中心的普通路段禁行约束，先膨胀真实障碍，再屏蔽矩形；不会把语义边界额外膨胀到 A/B。中线出现真实占据或未知时仍拒绝，入口与释放点仍须满足普通机器人半径。规划比较完整路线代价，包含普通道路、通道速度和 `action_cost_seconds`，不强制每个目标都使用通道。

对于已确认的洞口，若门梁抗锯齿灰度被 map_server 的 trinary 阈值判成静态未知，可在 editor 勾选“同时清除区域内静态未知栅格”（`clear_unknown=true`）。默认关闭；仿真示例已对标注的四处开启。清理只发生在静态地图矩形内，master costmap 的实时占据和未知仍能否决固定边。机器人高度没有写死为 260 mm，实时梁高范围来自每个通道的配置。

离线启动会检查工程/新导出文件携带的参考原点、分辨率与 yaw 是否匹配地图 YAML，防止把像素原点 `(0,0)` 的标注直接用于另一张世界坐标地图。旧版无参考元数据的语义 JSON 保持兼容；在线 SLAM 不用此检查限制扩图。

`use_composition:=True/False` 两种启动方式均加载地图过滤组件。仿真可以显式增加 `simulate_deformation:=true` 运行变形反馈模拟器；默认关闭，实车必须接真实反馈。

## 构建与验证

在现有工作空间中按原方式 `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`。新增依赖为 `lc_passage_msgs`、Eigen、OpenCV core/imgproc、nlohmann-json、BehaviorTree.CPP v3；OSQP、OsqpEigen 和 QDLDL 已随源码提供，构建不下载求解器。系统依赖可由 `rosdep install --from-paths src --ignore-src -r -y` 安装。

```bash
colcon test --packages-select lc_passage_common lc_passage_map lc_passage_checkers lc_passage_nav2 pb_omni_pid_pursuit_controller --event-handlers console_direct+
colcon test-result --verbose
```

独立核心验证：`bash src/lc_passage_nav2/scripts/check_sources.sh`。独立 Qt 工具的构建和操作说明见 [编辑器说明](../../tools/passage_map_editor/README.md)。

## 文档

- [架构、执行状态与边界](docs/ARCHITECTURE.md)
- [通道格式、参数与底盘接口](docs/INTERFACES.md)
- [测试覆盖与验证记录](docs/VALIDATION.md)
- [第三方来源与许可证](THIRD_PARTY_NOTICES.md)
