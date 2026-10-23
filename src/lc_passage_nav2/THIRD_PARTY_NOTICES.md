# 来源与许可证

vendor/tdt_passage 基于前一版独立规划器 2.0，其中 tdt_core 来自用户提供的 tdt-nav-kit(1).zip。TDT 原作者：Copyright (c) 2026 Rongxuan Ye and BoLin Li，MIT；完整许可见 vendor/tdt_passage/tdt_core/LICENSE。

本次增加实时地图输入、通道中线语义、ROS 执行接口、地图/点云过滤、状态机及回归测试；保留独立核心结构和上游求解器源码。ROS 包装、地图适配、工具、测试和文档采用根目录 MIT 许可。未删除上游版权声明。

| 随包源码 | 版本 | 许可位置 |
| --- | --- | --- |
| OSQP | 1.0.0 | vendor/tdt_passage/third_party/osqp/LICENSE、NOTICE，Apache-2.0 |
| OsqpEigen | 0.11.0 | vendor/tdt_passage/third_party/osqp-eigen/LICENSE，BSD-3-Clause |
| QDLDL | 0.1.8 | vendor/tdt_passage/third_party/qdldl/LICENSE，Apache-2.0 |
| AMD | OSQP 携带版本 | vendor/tdt_passage/third_party/osqp/algebra/_common/lin_sys/qdldl/amd/LICENSE，BSD-3-Clause |
| stb_image / stb_image_write | 2.30 / 1.16 | 对应头文件末尾的 MIT / Public Domain 双重许可 |

ROS2、Nav2、Eigen、OpenCV、nlohmann-json 通过用户环境安装，没有将它们的二进制发行包嵌入源码 ZIP；各自遵循其许可证。插件按官方 navigation2 的 humble 分支接口编写，没有将新发行版接口当作 Humble 接口使用。

特殊地形组织思路参考 Polyacetone/HWSentryNav26。插件没有复制其 ROS 节点、MPC 或机械状态机。本仓库不安装演示地图；测试使用生成的栅格。
