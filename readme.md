# PLDX SENTRY VISION

<div align="center">

**面向 RoboMaster 机器人平台的实时视觉自瞄与云台控制工程**

<img src="https://img.shields.io/badge/C%2B%2B-20-00599C.svg?style=for-the-badge&logo=cplusplus">
<img src="https://img.shields.io/badge/CMake-3.16%2B-064F8C.svg?style=for-the-badge">
<img src="https://img.shields.io/badge/LibXR-Topic-2E7D32.svg?style=for-the-badge">
<img src="https://img.shields.io/badge/Ubuntu-24.04-E95420.svg?style=for-the-badge&logo=ubuntu">
<img src="https://img.shields.io/badge/ROS_2-Jazzy-22314E.svg?style=for-the-badge&logo=ros">

</div>

---

## 项目简介

PLDX Sentry Vision 是面向 RoboMaster 步兵、哨兵等机器人的实时视觉工程。项目运行在机器人上位机，负责相机采集、装甲板识别、目标状态估计、轨迹规划、瞄准和开火决策，并通过 LibXR Topic 将控制指令发送给云台控制器。

工程采用模块化 C++ 架构，支持在线运行、离线视频测试、相机标定、手眼标定和通信链路测试。ROS 2 接口仅在检测到对应依赖时构建，用于与导航工程交换追击相关数据。

## 目录

- [亮点](#亮点)
- [功能模块](#功能模块)
- [依赖](#依赖)
- [Quick Start](#quick-start)
- [运行程序](#运行程序)
- [云台通信](#云台通信)
- [配置文件](#配置文件)
- [测试与验证](#测试与验证)
- [数据流与软件架构](#数据流与软件架构)
- [文件结构](#文件结构)

## 亮点

- **完整的视觉自瞄链路**：覆盖相机采集、识别、位姿解算、目标跟踪、轨迹规划、瞄准和开火决策。
- **面向实时性的轨迹规划**：根据云台运动约束生成可执行目标轨迹，并结合开火延迟进行决策。
- **模块化硬件抽象**：相机、云台、IMU、SocketCAN 和 ROS 2 接口均位于 `io/` 层。
- **LibXR 二进制通信**：视觉端使用 `Topic::PackRaw` 发送、`Topic::Server` 接收，协议布局和关键安全行为由测试固定。
- **故障安全**：AHRS 过期、通信恢复、非法指令和非法反馈均采用 fail-closed 策略；自动化测试不会发送真实开火。

## 功能模块

| 模块 | 目录或入口 | 说明 |
| :--- | :--- | :--- |
| 相机采集 | `io/hikrobot`、`io/mindvision`、`io/usbcamera` | 工业相机和 USB 相机驱动 |
| 云台通信 | `io/gimbal` | LibXR UART、AHRS 时间线、目标指令、反馈和安全闸门 |
| 自瞄 | `tasks/auto_aim` | 装甲板检测、目标跟踪、位姿估计、弹道解算和轨迹规划 |
| 打符 | `tasks/auto_buff` | 打符识别、状态估计和控制逻辑 |
| 全向感知 | `tasks/omniperception` | 全向相机或多目标感知相关模块 |
| 标定工具 | `calibration` | 相机内参、手眼和机器人世界坐标标定 |
| ROS 2 接口 | `io/ros2` | 可选的导航数据发布和追击指令订阅 |

## 依赖

### 已验证环境

| 组件 | 版本 |
| :--- | :--- |
| 操作系统 | Ubuntu 24.04 |
| ROS 2 | Jazzy Jalisco |
| 编译器 | GCC 13.3，C++20 |
| CMake | 3.28.3（工程最低要求 3.16.3） |

本文档中的构建和测试命令均以 Ubuntu 24.04 + ROS 2 Jazzy 为基线。
该环境已完成从 CMake 重新配置到全工程构建的验证，ROS 2 相关目标会正常启用。

主要库：

- OpenCV、Eigen3、yaml-cpp
- fmt、spdlog、nlohmann-json
- OpenVINO 2024.6
- Ceres Solver、libudev、libusb-1.0
- LibXR（固定在 `third_party/libxr`）

海康或迈德威视相机还需要安装对应厂商 SDK。ROS 2 目标使用 Jazzy，并依赖 `rclcpp`、`std_msgs`、`sp_msgs` 和 `pldx_vision_interfaces`；缺少这些依赖时，非 ROS 目标仍可构建。

## Quick Start

### 初始化 LibXR 子模块

~~~bash
cd /home/wanqiq/2026Sentry/PLDX_Sentry_Vision
git submodule update --init --recursive
~~~

### 构建

~~~bash
cd /home/wanqiq/2026Sentry/PLDX_Sentry_Vision
source /opt/ros/jazzy/setup.bash
cmake -S . -B build-jazzy -DCMAKE_BUILD_TYPE=Release
cmake --build build-jazzy -j$(nproc)
~~~

## 运行程序

~~~bash
./build-jazzy/standard
./build-jazzy/standard_mpc
./build-jazzy/auto_buff_debug
~~~

运行前确认配置中的模型路径、相机参数和 `com_port` 正确。真实机器人运行前应确认发射机构处于安全状态。

### 被动 CDC 检查

~~~bash
./build-jazzy/libxr_cdc_smoke path/to/gimbal.yaml
~~~

`libxr_cdc_smoke` 只接收并打印 AHRS 四元数，同时发送被动的 `fire=false` 保活，不发送目标控制包，也不会启用开火。云台设备默认使用稳定的 `/dev/gimbal` udev 别名。

## 云台通信

云台控制器通过 USB CDC 与视觉端连接。视觉端不是使用进程间共享内存形式的 `SharedTopic`，而是直接使用 LibXR Topic 二进制帧：

| Topic | 方向 | Payload |
| :--- | :--- | :--- |
| `target_euler` | 视觉 -> 云台 | 目标欧拉角及导数，36 字节 |
| `fire_notify` | 视觉 -> 云台 | `isfire`，1 字节 |
| `ahrs_quaternion` | 云台 -> 视觉 | `x,y,z,w` 四元数，16 字节 |
| `nav_gimbal_feedback_v1` | 云台 -> 视觉 | 弹速、弹量、云台模式和射击模式，12 字节 |

协议常量、字段偏移、payload 尺寸和小端约束位于 `io/gimbal/libxr_protocol.hpp`。运行时只创建一个 `LinuxUART`、一个接收 `Topic::Server` 和对应 Topic 回调。

### 安全行为

- AHRS 过期后拒绝目标和开火指令，并发送安全的 `fire=false`。
- 非法四元数或非法云台反馈会被丢弃。
- `control=false` 时不发送目标角，只发送被动安全火指令。
- 不要在 `/dev/gimbal` 上附加 `LibXR::Terminal`，避免与二进制 Topic 流竞争 CDC 端点。

### 传输诊断

默认关闭诊断。需要排查链路时，在 YAML 中加入：

~~~yaml
transport_diagnostics_enabled: true
~~~

运行时每 5 秒输出原始接收字节数、解析 topic 数、发布请求数、打包失败、非法 payload、读失败和写失败计数。计数只反映运行时可观察到的事件，不代表硬件端 CRC 错误或物理层重连次数。

## 配置文件

配置文件位于 `configs/`：

| 字段 | 必填 | 默认值 | 说明 |
| :--- | :---: | :--- | :--- |
| `com_port` | 是 | - | 推荐 `/dev/gimbal` |
| `baudrate` | 否 | `921600` | CDC 链路形式参数 |
| `default_mode` | 否 | `AUTO_AIM` | 在线反馈优先 |
| `default_bullet_speed` | 否 | `23.0` | 在线反馈优先 |
| `transport_diagnostics_enabled` | 否 | `false` | 是否周期输出通信诊断 |

建议通过 udev 规则将控制器绑定到 `/dev/gimbal`。配置解析会拒绝空设备路径、非法波特率、未知模式和非正弹速。

## 测试与验证

~~~bash
source /opt/ros/jazzy/setup.bash
ctest --test-dir build-jazzy --output-on-failure
~~~

当前 CTest 清单包含 9 项硬件无关测试，覆盖导航指令新鲜度、协议布局、PTY 分片收发、四元数插值、安全闸门、队列覆盖策略和配置解析。自动化测试不会向硬件发送 `fire=true`。

## 数据流与软件架构

~~~text
相机线程 -> 识别器 -> 目标状态估计器 -> 轨迹规划器/开火决策
                                      |
                                      v
                              GimbalRuntime
                         LibXR Topic over USB CDC
                                      |
                                      v
                              云台控制器与执行机构

云台 AHRS / feedback -> Topic::Server -> GimbalRuntime -> 视觉算法
~~~

算法模块通过 `io/` 层获取相机、云台和 IMU 数据；`GimbalRuntime` 负责进程级 LibXR 资源、UART 收发和通信安全闸门。ROS 2 适配层只负责导航接口，不直接操作云台串口。

## 文件结构

~~~text
PLDX_Sentry_Vision/
├── assets/                 # 模型、演示视频和标定素材
├── calibration/            # 相机、手眼和机器人世界坐标标定
├── configs/                # 各机器人配置文件
├── io/                     # 相机、云台、IMU、CAN 和 ROS 2 硬件抽象
│   └── gimbal/             # LibXR 协议、运行时、时间线和安全闸门
├── tasks/                  # auto_aim、auto_buff、omniperception
├── tests/                  # 算法、协议、PTY 和硬件抽象测试
├── tools/                  # 日志、轨迹、滤波和通用工具
├── third_party/libxr/      # 固定版本 LibXR 子模块
├── CMakeLists.txt
└── readme.md
~~~

## 致谢

本项目基于 Tongji SuperPower 往届视觉工程持续演进，感谢开源社区提供的识别模型、TinyMPC、OpenVINO、LibXR 及相关工具链。

项目仅供学习、研究和竞赛使用。涉及真实机器人和发射机构时，请遵守所在场地的安全规范。
