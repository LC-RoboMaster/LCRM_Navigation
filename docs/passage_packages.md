# 过洞代码归属

| 包 | 内容 | ROS 节点 |
| --- | --- | --- |
| `lc_passage_nav2` | TDT 全局规划插件、其私有算法和离线规划/地图导出工具 | 无 |
| `pb_omni_pid_pursuit_controller` | PID 跟踪插件、`Execution` 状态机、`PassageController` ROS 适配 | 无 |
| `lc_passage_map` | 静态地图处理、门梁点云过滤、通道 Marker | `PassageMapNode` |
| `lc_passage_simulator` | 仿真机构完成反馈 | `DeformationSimulator` |
| `lc_passage_checkers` | Nav2 进度检查和目标检查插件 | 无 |
| `lc_passage_bt_plugins` | 路径锁定和恢复保护 BT 插件 | 无 |
| `lc_passage_common` | 多包使用的几何、配置读取与 Marker 基础函数 | 无 |
| `lc_passage_msgs` | 跨节点消息定义 | 无 |

每个节点包只定义一个 ROS 节点；组件库与独立可执行文件是同一个节点的两种启动形式。插件包不启动自己的 ROS 节点。

## 查代码

- 状态机：[execution.hpp](../src/pb2025_sentry_nav/pb_omni_pid_pursuit_controller/include/pb_omni_pid_pursuit_controller/execution.hpp)
- 控制器适配：[passage_controller.cpp](../src/pb2025_sentry_nav/pb_omni_pid_pursuit_controller/src/passage_controller.cpp)
- 地图节点：[passage_map_node.cpp](../src/lc_passage_map/src/passage_map_node.cpp)
- 门梁过滤：[beam_filter.hpp](../src/lc_passage_map/include/lc_passage_map/beam_filter.hpp)
- 全局规划：[tdt_passage_planner.cpp](../src/lc_passage_nav2/src/tdt_passage_planner.cpp)
- 检查插件：[checkers.cpp](../src/lc_passage_checkers/src/checkers.cpp)
- 行为树插件：[bt_guards.cpp](../src/lc_passage_bt_plugins/src/bt_guards.cpp)

`pb_omni_pid_pursuit_controller` 不依赖全局规划包。节点、规划插件和控制器通过 `lc_passage_msgs` 消息通信；只有共同使用的配置/几何辅助头文件进入 `lc_passage_common`。状态机核心仅包含共用几何头文件，不包含 JSON、ROS 或规划器代码。

## 构建与测试

```bash
colcon build --packages-up-to lc_passage_nav2 lc_passage_map lc_passage_simulator lc_passage_checkers lc_passage_bt_plugins pb_omni_pid_pursuit_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

路径、话题、通道文件和动作确认流程保持一致。两个检查插件的配置名改为 `lc_passage_checkers/PassageProgressChecker` 和 `lc_passage_checkers/PassageGoalChecker`，实车/仿真配置与 launch 已同步。

原 ROS 联调用例按所属组件分到地图包、规划包和控制器包；控制器执行测试也随控制器移动。测试地图采用固定夹具，避免编辑器工程的日常调整改变回归基线。地图原点检查位于 bringup 的 `test/`。
