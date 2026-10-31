# HBUT 力创战队 RoboMaster 2027 导航工作空间

湖北工业大学（HBUT）力创战队 RoboMaster 2027 赛季哨兵/步兵导航工作空间，基于 **ROS2 Humble + Nav2** 导航框架。

## 通道导航

已集成 TDT 全局规划插件、PID 控制器内的变形状态管理和独立 C++/Qt 通道编辑器。支持无通道配置的在线建图导航，也支持保存地图后编辑旋转矩形通道，再加载地图与通道文件按中线通行。实时低矮障碍继续参与避障。

- [过洞代码包归属](docs/passage_packages.md)
- [导航使用、运行参数及验证](src/lc_passage_nav2/README.md)
- [Qt 地图编辑器](tools/passage_map_editor/README.md)
- [底盘变形命令与反馈接口](src/lc_passage_nav2/docs/INTERFACES.md)
- [lc_sentry2027 机器人模型与启动方式](src/lc_robot_description/README.md)

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- colcon（ROS2 构建工具）

## 构建

```bash
cd 2027_navigation_ws
source /opt/ros/humble/setup.bash
./build.sh            # 等价于 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 常用脚本

| 脚本 | 作用 |
| --- | --- |
| `build.sh` | colcon 构建（Release 模式，并导出 compile_commands.json） |
| `cmd_launch.sh` | 启动导航决策节点 `lc_robot_cmd` |
| `cmd_bridge_bringup.sh` | 启动 `domain_bridge` 消息桥接（ROS_DOMAIN_ID=20） |
| `rm_navigation_bringup.sh` | 实车导航（定位模式，`slam:=False`） |
| `rm_navigation_bringup_slam.sh` | 实车导航（建图模式，`slam:=True`） |
| `save_map.sh` | 保存当前地图为 `scans.pgm` / `scans.yaml` |
| `cp_map_data.sh` | 将 `scans.pcd`(odom) + `scans.pgm` + `scans.yaml` 拷到 `pb2025_nav_bringup` 并重新构建 |

> 导航和地图保存脚本按当前仓库定位工作空间，并转发命令行参数。

### 建图产物与坐标系

一次建图会产生三份数据，必须在**同一坐标系**下配套使用：

| 文件 | 产生者 | 坐标系 |
| --- | --- | --- |
| `scans.pcd` | `pcd_map_saver`（建图 launch 自动启动，默认写到启动目录） | `odom` |
| `scans.pgm` / `scans.yaml` | `save_map.sh`（nav2 `map_saver`） | `map` = 建图时的 `odom` |

定位时 `small_gicp` 直接把这张 PCD 当作 `map` 系先验，不做任何预旋转。因此**不要**再用
`point_lio/PCD/scans.pcd`（它是 `camera_init` 系，和 pgm 差一个与建图启动云台角有关的
yaw，会导致重定位后机器人相对地图“歪着”）；建图 launch 已把 `pcd_save` 关闭。

建图结束时，先在另一个终端执行 `bash save_map.sh scans`（`map_saver` 还在运行）保存
pgm/yaml，再 Ctrl-C 结束 launch；`pcd_map_saver` 会在退出时把 `scans.pcd` 写到同一目录，
看到 `Saved ... points` 日志后再关闭终端。最后执行 `bash cp_map_data.sh`。

## 子项目说明

`src/` 下包含以下子包（本仓库由多个上游仓库整合而来，已移除各子仓库的 `.git` 元数据，统一纳入本仓库管理）：

| 子项目 | 路径 | 说明 | 上游来源 | 开源协议 |
| --- | --- | --- | --- | --- |
| lc_rm_serial | `src/lc_rm_serial` | 力创战队串口通信协议包（波特率 921600，与 basic_framework 电控框架通信） | [Spaaaace-yyj/lc_rm_serial](https://github.com/Spaaaace-yyj/lc_rm_serial) | — |
| lc_robot_cmd | `src/lc_robot_cmd` | 机器人导航决策节点，基于 FSM 状态机 + Nav2 Action 实现决策/追击/回家等行为 | 本仓库 | — |
| lc_robot_description | `src/lc_robot_description` | 力创机器人 URDF 描述文件及 launch | 本仓库 | — |
| fake_localization | `src/fake_localization` | 仿真定位节点，订阅 `/initialpose` 广播 `map -> odom` 变换 | 本仓库 | — |
| navigation_msg_bridge | `src/navigation_msg_bridge` | 导航消息桥接节点（`/cmd_vel` → `/cmd_vel_to_cv`） | 本仓库 | — |
| pb2025_sentry_nav | `src/pb2025_sentry_nav` | 北极熊战队 2025 赛季哨兵导航仿真/实车包（含 point_lio、livox_ros_driver2、pb_nav2_plugins 等子模块） | [SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav) | Apache-2.0 |
| pb2025_robot_description | `src/pb2025_robot_description` | 北极熊战队机器人关节描述包（含 sdformat_tools、rmoss_gz_resources、joint_state_publisher） | [SMBU-PolarBear-Robotics-Team/pb2025_robot_description](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description) | Apache-2.0 |
| sdformat_tools | `src/sdformat_tools` | SDF/URDF 转换工具（xmacro4sdf、sdf2urdf、urdf_generator） | [gezp/sdformat_tools](https://github.com/gezp/sdformat_tools) | Apache-2.0 |

## 许可证

本项目整体遵循 [Apache License 2.0](LICENSE)。

`src/` 下的第三方子项目保留各自上游的开源协议（以 Apache-2.0 为主），详情见各子项目目录内的 `LICENSE` 文件。

## 致谢

本项目大量参考、复用并二次开发了以下开源项目，特此致谢：

- [pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav)（深圳北理莫斯科大学北极熊战队）
- [pb2025_robot_description](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description)（深圳北理莫斯科大学北极熊战队）
- [sdformat_tools](https://github.com/gezp/sdformat_tools)（gezp）
- [NAV2](https://github.com/ros-navigation/navigation2)

