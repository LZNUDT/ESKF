# 方案二：IMU、GNSS/RTK、底盘与激光雷达

在方案一的全局约束之外，用激光雷达点到平面残差约束短期几何。推荐的可运行 ESKF 框架是已下载的 `02_codes/08_FAST-LIO-Multi-Sensor-Fusion`。它不是 2026 年的新论文代码，而是 2024 年仍可见更新、并把 GNSS 与轮速放进 FAST-LIO2 迭代误差状态滤波的公开分支。

## 建议采用的代码框架

| 角色 | 路径 | 原因 |
| --- | --- | --- |
| 雷达与 IMU 的 ESIKF 主干 | `02_codes/07_FAST_LIO` | 官方实现，用来确认环境、外参和时间戳 |
| 四传感器同一滤波器 | `02_codes/08_FAST-LIO-Multi-Sensor-Fusion` | GNSS 位置、航向初始化、轮速、轮子外参和刻度因数 |

阅读顺序固定为先官方 FAST-LIO2，再多传感器分支。分支内已经带有 ikd-Tree 和 IKFoM，不需要再链到 `07_FAST_LIO` 才能理解结构。

HKU 官方后续的 FAST-LIVO2 仍是 ESIKF，增加的是相机光度，不是 RTK 和底盘。LIO-Fusion（RA-L 2023）的传感器列表与本方案一致，后端是因子图。需要 ESKF 时不以它为主代码。

GNSS 在该分支中是位置结果松组合。它不会处理载波模糊度。若 RTK 固定解由接收机或独立解算库给出，再把经纬高送进 `GNSS_Processing.hpp` 对应的话题。

## 滤波器里各传感器的分工

IMU 负责扫描间的前向传播和点云去畸变。激光雷达在迭代更新中提供大量点到平面残差，并把点插入 ikd-Tree。GNSS 以较低频率拉住绝对位置，并参与航向初始化。轮速提供车体系速度，同时估计外参和刻度，用来度过雷达几何退化或 GNSS 遮挡。

长隧道里雷达若同时退化，ESIKF 会退化为 IMU 加轮速，航向仍然会漂。这种场景需要方案一里的非完整约束和零速检测，多传感器分支没有完整复现 KF-GINS 的地球模型和 21 维 IMU 误差。

## 开源数据

| 顺序 | 数据 | 作用 |
| --- | --- | --- |
| 1 | FAST-LIO 仓库 README 给出的官方 rosbag | 只验证雷达与 IMU |
| 2 | KAIST Complex Urban 的一条短城市序列 | 作者用来测试 GNSS 与编码器，https://sites.google.com/view/complex-urban-dataset |
| 3 | i2Nav-Robot 室外序列 | 同时具备 MID360、ADIS16465、OEM719 RTK 和 Ranger 底盘，https://github.com/i2Nav-WHU/i2Nav-Robot |

KAIST 是与这份代码最匹配的公开数据。i2Nav-Robot 的传感器更贴近当前轮式机器人，但要自行对齐话题、外参和 ENU 原点。M3DGR 被 2026 年的 FAST-LIVGO 预印本用于雷达、视觉、惯导和 GNSS，本次没有确认其官方 ESKF 代码，因此不作为本方案的第一验证集。

## 软硬件环境

上游编译说明面向 Ubuntu，不面向原生 Windows。

| 项目 | 要求 |
| --- | --- |
| 系统 | Ubuntu 20.04，ROS Noetic；或 Ubuntu 18.04，ROS Melodic。Windows 使用 WSL2 Ubuntu 20.04，不使用 WSL1 |
| 处理器与内存 | 8 核、32 GB。只看状态、关闭稠密地图显示时可降到 16 GB |
| 磁盘 | 系统与 ROS 预留 40 GB；KAIST 一条序列数 GB 到数十 GB；i2Nav-Robot 全套预留 200 GB 以上 |
| 依赖 | PCL 1.8 以上，Eigen 3.3.4 以上，C++14 |
| 显示 | RViz。WSL2 需要可用的图形转发或 WSLg |
| 实车计算 | 官方 FAST-LIO2 可在 NVIDIA TX2、Khadas VIM3、树莓派 4B 8 GB 上跑雷达与 IMU。加上 GNSS、轮速和 RViz 后，车载计算机建议使用 8 核 x86 或同等 Jetson，32 GB 内存更稳 |
| 传感器 | 机械雷达或固态雷达，硬件同步 IMU，RTK 接收机，轮速或编码器。点云必须带逐点时间 |
| 标定 | 雷达与 IMU 外参，轮速参考点杆臂，GNSS 天线杆臂，轮半径。多传感器分支可在线估计轮子外参和刻度，初始值仍要接近真值 |

办公室复现不需要实车，但需要 Ubuntu 或 WSL2、ROS Noetic 和至少一条 KAIST 或 FAST-LIO 小 bag。Matlab 与 Visual Studio 不能替代这一环境。
