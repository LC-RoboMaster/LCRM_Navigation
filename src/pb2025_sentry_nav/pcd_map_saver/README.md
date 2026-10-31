# pcd_map_saver

建图时把**已经在 `odom` 系里**的点云话题累积成一张 PCD，退出（Ctrl-C）时写盘。

## 为什么需要它

`map_saver` 保存的 `.pgm/.yaml` 在 **建图时的 `odom`(= `map`)** 系，而 `point_lio` 自己
`pcd_save` 出来的 `scans.pcd` 在 **`camera_init`** 系。两者相差
`T_odom←camera_init`，其中包含一个与建图启动瞬间云台 yaw 有关的旋转；直接把这个 PCD
交给 `small_gicp` 重定位，GICP 会把这段 yaw 反向吃进 `map→odom`，表现为“重定位后机器人
相对地图歪着”。

本节点订阅 `loam_interface` 发布的 `registered_scan`（已经变换到 `odom`），累积后直接
写 PCD，因此点云与 pgm 同帧，`small_gicp` 不需要任何预旋转。

## 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `cloud_topic` | `registered_scan` | 要累积的点云话题，必须已在 odom 系 |
| `output_file` | `scans.pcd` | 输出路径；相对路径按进程 cwd 解析，父目录会自动创建 |
| `expected_frame` | `odom` | 只用于校验；首帧 frame 不一致会打印一次 ERROR |

## 使用

`pb2025_nav_bringup/launch/slam_launch.py` 在 `slam:=True` 时会自动启动它，输出默认是
**启动时 cwd 下的 `scans.pcd`**。建图结束时先在另一个终端用 `save_map.sh` 保存 pgm/yaml，
再 Ctrl-C 结束 launch；本节点在退出时写盘（日志会打印 `Saved N points to ...`），
最后用 `cp_map_data.sh` 把三者一起拷进 `pb2025_nav_bringup` 并重新构建。
