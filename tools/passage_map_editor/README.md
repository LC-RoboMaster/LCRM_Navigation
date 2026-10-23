# Passage Map Editor（C++ / Qt）

独立桌面地图与通道编辑器，支持 Qt 5/6 和 yaml-cpp。工具不链接 ROS、Nav2、TDT 或 OSQP；需要试规划时通过 QProcess 调用独立 `passage_cli`。

```bash
cmake -S tools/passage_map_editor -B build-editor -DCMAKE_BUILD_TYPE=Release
cmake --build build-editor --parallel
./build-editor/passage_map_editor --cli "$PWD/install/lc_passage_nav2/lib/lc_passage_nav2/passage_cli"
```

“导入地图”读取 map_server 保存的 YAML 与 PGM/PNG，保留分辨率、origin、yaw、negate 和阈值。三次点击绘制旋转矩形，支持拖动、旋转、缩放、删除、复制、撤销与重做；A/B 默认沿长轴自动放置，可在属性栏调整。A/B 必须仍在长轴中线上，位于区域两侧外部。

“保存/另存为”保存可继续编辑的工程 JSON 和图片；“导出导航通道文件”（Ctrl+Shift+E）输出 `field.passages.json`，加载现有地图后可“导入导航通道文件”（Ctrl+Shift+I）继续编辑。通道导出不包含测试起终点和测试障碍，普通地图文件由原 map_server 加载。

属性包含方向、准备/释放距离、穿洞速度、变形/恢复停留时间和门梁相对高度过滤范围。可设置起终点、调用 CLI 试规划、查看中间地图与回放通道状态。试规划中的模拟障碍只供测试，不作为导航运行配置。

CLI 试规划使用工程内的机器人半径等参数，实车运行参数则来自 nav2_params.yaml，比较两者结果时应保持参数一致。编辑完成后重新启动导航加载通道文件。运行接口详见 [导航说明](../../src/lc_passage_nav2/README.md)。

```bash
ctest --test-dir build-editor --output-on-failure
```
