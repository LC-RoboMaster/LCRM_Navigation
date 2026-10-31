# lc_sentry2027 机器人模型

模型来自 `urdf_temp/sentry`，正式资源安装到 `lc_robot_description`，可与原有 `lc2025_sentry_robot` 按启动参数切换。原始导出包保留，并通过 `urdf_temp/COLCON_IGNORE` 避免将 ROS 1 导出包加入 ROS 2 构建。

```bash
colcon build --packages-select lc_robot_description pb2025_nav_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 单独查看机器人和坐标系，不启动雷达或导航
ros2 launch lc_robot_description lc_sentry2027.launch.py

# 实车建图：现有脚本已开启 robot_state_publisher
bash rm_navigation_bringup_slam.sh robot_name:=lc_sentry2027

# 已建地图导航，地图和 PCD 可继续按原方式传入
bash rm_navigation_bringup.sh robot_name:=lc_sentry2027 \
  map:=/absolute/field.yaml prior_pcd_file:=/absolute/field.pcd
```

直接使用 launch 时：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  robot_name:=lc_sentry2027 use_robot_state_pub:=True slam:=True
```

在 `dev_new_urdf` 分支中，实车 launch 和模型启动入口默认使用 `lc_sentry2027`，实车 launch 默认开启 `use_robot_state_pub`。旧模型可用 `robot_name:=lc2025_sentry_robot` 选择。已有独立机器人启动模块发布 TF 时，显式传入 `use_robot_state_pub:=False`，由该模块加载本包 URDF，避免重复发布。

## 坐标系与参数

| 坐标系 | 用途 |
| --- | --- |
| `base_footprint` | 导航平面参考；当前与原始 `base_link` 重合，没有套用旧机器人高度 |
| `base_link` | 原始导出模型的底盘坐标系 |
| `yaw_link` | 原始云台坐标系，由 `yaw_joint` 驱动 |
| `gimbal_yaw` | 与 `yaw_link` 重合的导航兼容坐标系 |
| `mid360_link` | 原始 MID360 点云坐标系；保留图纸计算的位置和方向 |
| `front_mid360` | 与 `mid360_link` 重合的雷达兼容坐标系，供驱动和原有配置使用 |
| `mid360_imu` | Point-LIO 的 IMU/body 参考；按当前 `mapping.extrinsic_T/R` 加入 |
| `gimbal_yaw_fake` | 由现有 `fake_vel_transform` 从 `gimbal_yaw` 动态发布，URDF 不重复定义 |

选择模型后，启动层在原 `params_file` 上叠加 `pb2025_nav_bringup/config/reality/lc_sentry2027.yaml`，仅覆盖坐标系及雷达配置路径；无需复制整份 Nav2 参数。两种 composition 模式、SLAM 和离线导航共享这一入口。

原始点云 `livox/lidar` 的 frame 为 `front_mid360`，其原点和方向与 `mid360_link` 一致。Point-LIO 用内部雷达/IMU 外参将其变换到 IMU 参考，故 `loam_interface`、`sensor_scan_generation` 和重定位的 `lidar_frame` 参数绑定 `mid360_imu`；这些参数名沿用现有接口。`sensor_scan` 随之处于 IMU 坐标系，地形分析的 `sensor_frame` 同步设置。URDF 的 `mid360_to_imu` 是 `p_IMU = R_LI * p_LiDAR + T_LI` 的逆变换，当前为 `[0.011, 0.02329, -0.04412]` m、单位旋转。更换 IMU 或修改内部外参时须同步更新二者。

实体 TF 链为 `base_footprint → base_link → yaw_link → mid360_link`（中间保留 CAD 支架关节），导航别名接在对应实体坐标系下。`sensor_scan_generation` 在收到 Point-LIO 里程计后发布 `odom → base_footprint`；`fake_vel_transform` 根据 `odometry` 中的云台姿态发布 `gimbal_yaw → gimbal_yaw_fake`，抵消 yaw 旋转。`map → odom` 由 SLAM/重定位提供。这三个运行时变换不写入 URDF；只有模型而没有里程计输入时，模型自身可显示，但不会产生完整的导航 TF 树。

新模型使用 `config/reality/lc_sentry2027_mid360.json`。其中网络地址沿用原雷达配置，驱动层安装外参归零，由 URDF 描述底盘/云台安装关系；旧模型仍使用原 JSON。如需更换雷达 IP/网卡地址，修改新 JSON。

## 关节反馈

模型保留原始轮子、yaw 和 pitch 关节。默认 `joint_state_publisher` 从 `serial/gimbal_joint_state` 接收反馈；消息中的关节名须使用 `yaw_joint`、`pitch1_joint`、`pitch2_joint` 等 URDF 原名。没有反馈时使用零位，适合模型预览；雷达随云台运动时必须提供真实 `yaw_joint` 角度。

可传 `joint_state_source:=your_joint_states_topic` 更改输入话题。已有发布器直接发布 `joint_states` 时，传 `use_joint_state_publisher:=false`。这些参数在实车顶层和独立模型启动中均可用。

本次接入面向实车导航和 RViz。Gazebo 仍使用现有仿真机器人；SolidWorks 导出的惯量和关节限位尚未按动力学仿真标定。URDF 不会自动改变 Nav2 的 `robot_radius`/footprint，现有半径保持原值，需按实际外轮廓另行配置。
