# 方案一：IMU、GNSS/RTK 与底盘

目标是车载或轮式机器人上的松组合：IMU 积分名义轨迹，RTK 位置约束绝对坐标，底盘轮速和非完整约束在 GNSS 中断时限制速度漂移。后端使用误差状态卡尔曼滤波。

## 建议采用的代码框架

主框架是已下载的两份 i2Nav 代码。

| 角色 | 路径 | 原因 |
| --- | --- | --- |
| 可编译的 GNSS/INS 引擎 | `02_codes/01_KF-GINS` | 21 维 $\phi$ 角误差状态，双样本机械编排，杆臂，自带 RTK 样例 |
| 底盘量测的阅读和补全入口 | `02_codes/02_KF-GINS-Matlab` | `GetOdoVel.m`、`ODONHCUpdate.m`、`ProcessConfig3.m` 已划分数据与更新 |

截至 2026-09，没有一份仍在活跃维护、并且把原始 RTK 载波、四元数 ESKF 和底盘标度同时做完的官方仓库。KF-GINS 的 C++ 版没有里程计更新；Matlab 版有接口但上游声明 ODO/NHC 未完成。这是方案一的真实起点，而不是缺陷被忽略后的推荐。

Solà 路线的 `04_imu_gps_localization` 与 `06_eskf_gnss_imu_localization` 适合检查四元数重置，不要用来替代 KF-GINS 做车载 RTK。它们没有地球模型和底盘约束。

`05_imu_x_fusion` 的六自由度里程计是位姿松组合，不是轮速。轮速项在该仓库仍未实现。

## 状态与量测

在 KF-GINS 的 21 维上，底盘扩展至少再考虑里程计标度因数 $\delta s_{\mathrm{odo}}$。若安装角也在线估计，再增加小角度 $\delta\boldsymbol{\alpha}$。名义状态仍然只由 IMU 机械编排推进。

RTK 量测是导航系位置，用经纬高与标准差。接收机给出的速度可以并行更新，Matlab 的 `GNSSUpdate.m` 已包含这一支。底盘量测是车体系速度：前向分量来自轮速，侧向和垂向按非完整约束取零，并用 IMU 到车辆参考点的杆臂去掉转动引起的速度。

RTK 质量不能只看有无定位结果。i2Nav-Robot 的室外 RTK 仍有粗差，更新前应按位置标准差或新息检验剔除。室内序列不应打开 GNSS 更新。

## 开源数据

按这个顺序使用，避免先下载整套城市数据：

1. `02_codes/01_KF-GINS/KF-GINS-main/dataset`，确认 21 维松组合。
2. Matlab `dataset3`，阅读 ODO/NHC 配置，并在补全更新后做回归。
3. https://github.com/i2Nav-WHU/awesome-gins-datasets ，开阔天空 RTK，比较不同等级 MEMS。
4. https://github.com/i2Nav-WHU/i2Nav-Robot 的室外序列，同时拥有 OEM719 RTK、ADIS16465 和 Ranger 底盘。

GREAT 数据集含多频原始观测和战术级 IMU，适合以后做载波紧组合。当前 KF-GINS 只吃定位结果，不能直接读 RINEX。

## 软硬件环境

软件以 Windows 10/11 为日常阅读和 Matlab 调试环境，C++ 版同样可以在 Windows 编译。

| 项目 | 要求 |
| --- | --- |
| 处理器与内存 | 4 核、16 GB 足够跑 KF-GINS 样例和 Matlab |
| 磁盘 | 源码与短数据 2 GB 以内；i2Nav-Robot 全套 rosbag 预留 200 GB 以上 |
| C++ | CMake 3.10 以上，Visual Studio 2019 或更新版本；第三方库已随 KF-GINS 提供 |
| Matlab | R2018b 或更新版本，无需工具箱 |
| Python | 3.8 以上，NumPy、Matplotlib，用于 `plot_navresult.py` |
| 实车传感器 | MEMS 或工业级 IMU，RTK 接收机与天线，轮速或四轮转速；IMU、轮速、GNSS 时间对齐到毫秒 |
| 标定 | IMU 到 GNSS 天线杆臂，IMU 到车辆后轴或轮速参考点杆臂，轮半径与标度 |

不需要 ROS，也不需要激光雷达驱动。若只在办公室复现，有 Windows、Matlab 或 Visual Studio 即可，传感器可以用数据集代替。
