# 文件、参数与 ROS 接口

## 通道文件

编辑器工程 `*.json` 包含图片、地图坐标、测试参数；运行文件 `*.passages.json` 包含 `schema_version: 1`、`frame_id`、`passages`，新版还附带 `map_reference: {origin: [x,y], yaw: ..., resolution: ...}` 用于离线启动防错，不依赖图片。运行时也可直接读取旧版工程 JSON 的通道字段，忽略测试障碍与图片路径。地图 YAML 仍由 map_server 加载；参考元数据不匹配时启动报错，不能只改 origin 数字而不转换已有通道坐标。

```json
{
  "schema_version": 1,
  "frame_id": "map",
  "passages": [{
    "id": "door_01",
    "region": {"center": [5.0, 3.0], "length": 0.8, "width": 0.6, "yaw": 0.0},
    "endpoint_a": [3.8, 3.0], "endpoint_b": [6.2, 3.0],
    "enabled": true, "direction": "both", "clear_static": true, "clear_unknown": false,
    "prepare_distance": 0.8, "release_distance": 0.4,
    "speed": 0.3, "mode": "folded", "action_cost_seconds": 1.0,
    "fold_hold_seconds": 0.3, "restore_hold_seconds": 0.3,
    "beam_min_height": 0.26, "beam_max_height": 2.0
  }]
}
```

坐标为 map 米制世界坐标，yaw 为弧度，时间为秒。A/B 必须在矩形长轴中线上、位于矩形外且分居两侧；编号唯一，最多 16 个不相交区域。`direction` 可为 both/a_to_b/b_to_a。准备点位于入口前的最终路径上，释放点沿中线延伸到出口后。反向通行自动交换入口与出口。

`speed` 单位为 m/s，仅限制 `PASSING`（准备点变形确认后至释放点）的平移速度，不要求 PID 达到该速度。洞外接近准备点与两个独立通道之间不套用通道限速，也不叠加专用接近制动包络；普通 PID 和其他已有约束继续控制速度。准备点仍需停车并确认变形完成，释放点仍需停车并确认恢复完成。连续通道合并穿越时使用该组最小 `speed`。

`clear_static` 清理已标注矩形的已知静态占据；`clear_unknown` 默认 false，设为 true 时同时清除该矩形内的静态未知，用于已确认门梁投影的中间灰度被 trinary 地图判为未知的情况。两个选项均不清除合并后的实时 costmap；禁用通道不做清理。`beam_min_height`/`beam_max_height` 使用 terrain_analysis 计算的 intensity 相对地面高度。请按真实地图只标门梁投影，不要把邻近墙体或地面障碍一并圈入矩形。

## 参数

实车与仿真 `nav2_params.yaml` 已给出完整配置。入口参数 `passage_file` 是绝对路径或空字符串，由 launch 同时写入所有消费者。修改 `GridBased` 参数可调整普通区域规划，不要求重新导出通道。

| 参数 | 默认值 / 含义 |
| --- | --- |
| GridBased.robot_radius | 使用 global costmap 配置半径，仅普通区膨胀 |
| safety_margin | 0.02 m，普通区额外距离 |
| obstacle_weight / obstacle_decay | 2.0 / 0.35 m，附加项为 weight / (decay + 障碍距离)，decay 是距离偏置 |
| source_cost_weight | 0.5，继承 Nav2 软代价权重 |
| unknown_is_obstacle | true |
| optimize / allow_geometric_fallback | true / true；优化失败可用已验证几何路线 |
| graph_budget_ms | 1000，构建普通连接边时检查的累计预算，不是整个 createPlan 的硬超时 |
| search_timeout_ms / optimization_timeout_ms | 200 / 200，单次后端预算 |
| sample_spacing / cruise_speed | 0.025 m / 1.0 m/s |
| FollowPath.enable_passage_management | true，使用候选路径采用与执行状态机 |
| FollowPath.passage_stop_tolerance | 0.04 m |
| FollowPath.passage_action_timeout | 8.0 s，变形完成超时 |
| FollowPath.passage_deceleration | 0.8 m/s²，仅 PASSING 末尾接近释放点的制动限速，不作用于洞外接近准备点 |

## 主题和底盘对接

以下为相对主题，支持 Nav2 namespace；domain_bridge 示例针对当前实车无 namespace、导航域 20 / 底盘域 10。

| 主题 | 类型 | 含义 |
| --- | --- | --- |
| passage/plan | lc_passage_msgs/PassagePlan | 候选路径与对应弧长/通道标记，可靠且 transient_local |
| passage/active_plan | lc_passage_msgs/PassagePlan | 控制器实际采用的完整路径 |
| passage/state | lc_passage_msgs/PassageState | 当前阶段、锁定、变形、停车、故障状态 |
| passage/markers | visualization_msgs/MarkerArray | 地图通道几何及最近采用路径的实际事件点，可靠且 transient_local |
| passage/planner_markers | visualization_msgs/MarkerArray | 最近规划的通道方向检查：选中绿色、可用未选中黄色、拒绝红色 |
| passage/planner_diagnostics | std_msgs/String | JSON：规划状态、地图原点/尺寸、实际半径、每个方向的 valid/selected/reason、完整路线代价与纯普通道路代价 |
| passage/deformation_command | lc_passage_msgs/DeformationCommand | 请求模式 folded/normal；相同 request_id 可重发 |
| passage/deformation_feedback | lc_passage_msgs/DeformationFeedback | 真实机构完成或故障反馈 |

底盘接收到新的 request_id 后执行 mode，重复请求应幂等；**机构完成后**回传相同 request_id 与 mode，`completed=true`。发生机械错误回传 `fault=true`。旧 ID、错误 mode 或未完成反馈均不会解除等待。发布器不会把“命令已发送”伪装成“机构已完成”。

现有 `lc_rm_serial` 的速度帧和导航 Twist 桥没有变形字段。本次不虚构下位机二进制布局：底盘/CV 侧在已有通信程序中转换上述两个类型；`cmd_bridge.yaml` 已添加跨域命令与反馈。两个域都需要编译并 source `lc_passage_msgs`。缺少真实反馈时，导航会在准备点停车并在超时后锁存故障。

`passage_deformation_simulator` 提供可执行的仿真接口，收到命令后延迟确认，默认不启动。仿真参数 `simulate_deformation:=true` 不应在实车使用。

`passage_checks` 同时由独立 CLI 的结果 JSON 和 Nav2 诊断输出提供。`centerline_obstacle at (x,y)` 为中线真实占据，`centerline_unknown` 为未知，`centerline_outside_map` 为越界，`entry_inflation_or_forbidden` / `release_inflation_or_forbidden` 为普通端点未通过膨胀或语义禁行检查，`crosses_other_region` 为固定线穿越另一个特殊区域。`available` 只代表该方向固定边有效；`selected` 才代表进入最终路径。节点日志在检查结果变化时输出原因，避免每次重规划刷屏。
