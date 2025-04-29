# Alicia Duo Leader ROS 驱动程序

## 概述

本 ROS 软件包 (`alicia_duo_driver`) 旨在处理与机械臂底层硬件的串口通信，解析接收到的数据，并将机械臂的状态（如关节角度、夹爪状态、按钮信息）发布为标准的 ROS 消息。

该驱动程序包含多个节点，协同工作以实现完整的功能：
*   **串口服务节点 (`serial_server_node`)**: C++ 实现，负责底层的串口读写和原始数据帧的校验与发布/接收。
*   **数据类型处理节点 (`serial_data_type_node.py`)**: Python 实现，订阅原始串口数据，根据指令ID将数据分发到不同的处理话题。
*   **舵机状态节点 (`servo_states_node.py`)**: Python 实现，处理来自 `serial_data_type_node.py` 的舵机和夹爪状态数据，将其转换为标准的 `ArmJointState` 消息（使用弧度单位）并发布。


## 系统依赖

在开始部署之前，请确保您的系统满足以下要求：

*   **操作系统**: Ubuntu 18.04 (Melodic) 或 Ubuntu 20.04 (Noetic) - 推荐使用 Ubuntu 20.04 / ROS Noetic。
*   **ROS**: ROS Melodic 或 ROS Noetic (推荐 Desktop-Full 安装)。
*   **构建工具**: `catkin` (通常随 ROS 安装)。
*   **编译器**: C++11 或更高版本的编译器 (如 g++)。
*   **Python**: Python 3.x (通常随较新版 ROS 安装)。
*   **ROS `serial` 库**: 用于串口通信 (`sudo apt-get install ros-<ros-distro>-serial`)。
*   **Boost 库**: (通常随 ROS Desktop-Full 安装)。

`<ros-distro>` 应替换为您使用的 ROS 发行版名称，例如 `noetic` 或 `melodic`。

## 安装与部署

以下步骤将指导您完成 `alicia_duo_driver` 的安装和部署。假设您已经安装了 ROS 并且熟悉基本的 Linux 命令行操作。

### 1. 环境准备

确保您的 ROS 环境已正确安装并配置。如果您尚未安装 ROS，请遵循官方 ROS 安装指南。

### 2. 创建并进入 Catkin 工作空间

如果您还没有 Catkin 工作空间，请创建一个：
```bash
# 创建工作空间目录
mkdir -p ~/alicia_ws
cd ~/alicia_ws
```

### 3. 获取代码

将本代码库克隆到您的 Catkin 工作空间的 src 目录下：
```bash
git clone https://github.com/Xianova-Robotics/Alicia_duo_leader_ros.git
```

### 4. 安装依赖

进入工作空间根目录，并使用 `rosdep` 安装软件包依赖：
```bash
cd ~/alicia_ws
rosdep install --from-paths src --ignore-src -r -y
```
*如果 `rosdep` 提示找不到某些依赖，您可能需要手动安装 (例如 `sudo apt-get install ros-<ros-distro>-serial`)。*

### 5. 编译工作空间

使用 `catkin_make` 编译工作空间中的所有软件包：
```bash
cd ~/alicia_ws
catkin_make
```
*编译过程可能需要一些时间。如果遇到错误，请检查依赖项是否已正确安装以及代码是否完整。*

### 6. 配置环境

每次打开新的终端时，都需要 source 工作空间的配置文件，以便 ROS 能够找到您的软件包：
```bash
echo "source ~/alicia_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
*如果您使用的是 `zsh`，请将 `.bashrc` 替换为 `.zshrc`。*

### 7. 串口权限 (重要)

Linux 系统通常需要将用户添加到 `dialout` 组才能访问串口设备：
```bash
sudo usermod -a -G dialout $USER
```
**执行此命令后，您需要完全注销并重新登录系统，或者重启计算机，才能使权限生效。**


## 使用说明

### 启动核心服务

要启动机械臂的驱动程序和所有相关节点，请运行 `serial_server.launch` 文件。确保机械臂已通过 USB 连接到计算机。

```bash
roslaunch alicia_duo_driver serial_server.launch
```

该命令将启动以下节点：
1.  `serial_server_node`: 处理底层串口通信。
2.  `serial_data_type_node.py`: 分类转发串口数据。
3.  `servo_states_node.py`: 处理状态数据并发布 `/arm_joint_state`。

您可以通过添加参数来修改启动行为，例如启用调试模式或指定串口：
```bash
# 启用调试模式
roslaunch alicia_duo_driver serial_server.launch debug_mode:=true

# 指定串口为 ttyUSB0
roslaunch alicia_duo_driver serial_server.launch port:=ttyUSB0

# 同时指定端口和启用调试
roslaunch alicia_duo_driver serial_server.launch port:=ttyUSB0 debug_mode:=true
```

### 运行状态读取示例

项目提供了一个简单的 Python 脚本 (`arm_read_demo.py`) 来演示如何订阅和打印机械臂的状态信息。

1.  确保核心服务 (`serial_server.launch`) 正在运行。
2.  打开一个新的终端 (确保已 source `setup.bash`)。
3.  运行示例脚本：

```bash
rosrun alicia_duo_driver arm_read_demo.py
```

您将在终端中看到持续打印的机械臂关节角度（度和弧度）、夹爪角度和按钮状态。

## 核心接口：状态发布

成功部署并启动驱动程序后，最关键的信息来源是 `/arm_joint_state` 话题。该话题发布了机械臂的完整状态，是您开发上层应用（如运动规划、状态监控）时需要订阅的主要接口。

该话题使用自定义的 `alicia_duo_driver/ArmJointState` 消息类型，其结构如下：

*   **`alicia_duo_driver/ArmJointState`**:
    *   `std_msgs/Header header`: 标准消息头，包含时间戳和坐标系 ID。
    *   `float32 joint1` 到 `float32 joint6`: 六个主要关节的角度，单位为 **弧度 (radians)**。
    *   `float32 gripper`: 夹爪的角度，单位为 **弧度 (radians)**。
    *   `int32 but1`, `int32 but2`: 机械臂基座上的两个按钮状态 (通常 0 表示未按下，1 表示按下)。
    *   `float32 time`: (此字段在状态消息中通常不直接使用，主要用于控制指令) 期望的运动时间 (秒)。

理解并正确使用 `/arm_joint_state` 话题及其 `ArmJointState` 消息是与 Alicia Duo Leader 机械臂进行交互的基础。后续的 "ROS API" 部分将提供更详细的话题和节点信息。


## ROS API

### 主要节点

| 节点名称                | 类型   | 包                  | 描述                                                                 |
| :---------------------- | :----- | :------------------ | :------------------------------------------------------------------- |
| `serial_server_node`    | C++    | `alicia_duo_driver` | 负责底层串口通信、数据帧校验和原始数据收发。                         |
| `serial_data_type_node` | Python | `alicia_duo_driver` | 订阅原始串口数据，根据指令 ID 分类转发到不同话题。                   |
| `servo_states_node`     | Python | `alicia_duo_driver` | 处理舵机/夹爪状态，发布标准 `ArmJointState` 消息 (弧度)。            |

### 重要话题 (Topics)

| 话题名称              | 消息类型                          | 发布者                  | 订阅者                                | 描述                                                                   |
| :-------------------- | :-------------------------------- | :---------------------- | :------------------------------------ | :--------------------------------------------------------------------- |
| `/read_serial_data`   | `std_msgs/UInt8MultiArray`        | `serial_server_node`    | `serial_data_type_node`               | 从串口接收到的原始字节数据帧。                                         |
| `/send_serial_data`   | `std_msgs/UInt8MultiArray`        | `servo_control_node`    | `serial_server_node`                  | 要发送到串口的原始字节数据帧。                                         |
| `/gripper_angle`      | `std_msgs/UInt32MultiArray`       | `serial_data_type_node` | `servo_states_node`                   | 处理后的夹爪原始值和按钮状态 `[gripper_raw, but1, but2]`。             |
| `/servo_states`       | `std_msgs/UInt8MultiArray`        | `serial_data_type_node` | `servo_states_node`                   | 处理后的舵机状态原始数据帧 (指令 ID 0x04)。                            |
| `/servo_states_6`     | `std_msgs/UInt8MultiArray`        | `serial_data_type_node` | (无标准订阅者)                        | 处理后的扩展舵机状态原始数据帧 (指令 ID 0x06)，用途待定。             |
| `/error_frame_deal`   | `std_msgs/UInt8MultiArray`        | `serial_data_type_node` | (无标准订阅者)                        | 从串口接收到的错误帧 (指令 ID 0xEE)。                                  |
| `/arm_joint_state`    | `alicia_duo_driver/ArmJointState` | `servo_states_node`     | `arm_read_demo.py`, 其他用户节点      | **主要的机械臂状态话题**，包含所有关节和夹爪的角度 (弧度) 及按钮状态。 |
| `/servo_states_main`  | `std_msgs/Float32MultiArray`      | `servo_states_node`     | (用于向后兼容)                        | 包含 6 个关节角度和夹爪角度 (弧度) 的数组。                            |
| `/startup_complete`   | `std_msgs/String`                 | `serial_server.launch`  | (无标准订阅者)                        | 在所有核心节点启动后发布一次，表示系统准备就绪。                       |

### 消息类型 (Messages)

*   **`alicia_duo_driver/ArmJointState`**:
    *   `std_msgs/Header header`: 标准消息头 (时间戳, frame_id)。
    *   `float32 joint1` 到 `float32 joint6`: 六个主要关节的角度 (弧度)。
    *   `float32 gripper`: 夹爪的角度 (弧度)。
    *   `int32 but1`, `int32 but2`: 按钮状态 (通常为 0 或 1)。
    *   `float32 time`:期望的运动时间 (秒)，0 表示立即执行。

## 配置参数 (`serial_server.launch`)

您可以在启动 `serial_server.launch` 时通过命令行参数或直接修改 launch 文件来调整以下配置：

| 参数名称      | 默认值   | 可选值                  | 描述                                                                 |
| :------------ | :------- | :---------------------- | :------------------------------------------------------------------- |
| `debug_mode`  | `false`  | `true`, `false`         | 是否启用调试模式，启用后会打印详细的串口收发数据日志。               |
| `port`        | `ttyUSB3`| 例如 `ttyUSB0`, `ttyACM0`, "" | 指定要连接的串口设备名称 (位于 `/dev` 下)。如果留空 (`""`)，节点将自动搜索可用的 `ttyUSB` 设备。 |
| `baudrate`    | `921600` | 整数                    | 串口通信波特率。必须与机械臂硬件设置一致。                           |
| `servo_count` | `9`      | 整数                    | 机械臂上的舵机总数 (包括夹爪)。影响状态解析节点的配置。        |
| `output`      | `screen` | `screen`, `log`         | ROS 节点的日志输出位置。`screen` 输出到终端，`log` 输出到日志文件。 |

## 故障排查

*   **无法连接串口 / Permission denied**:
    *   **确认权限**: 确保您已将用户添加到 `dialout` 组，并且已经注销/重新登录或重启。运行 `groups $USER` 查看当前用户所属的组，应包含 `dialout`。
    *   **检查设备**: 确认机械臂已连接，并通过 `ls /dev/ttyUSB*` 或 `ls /dev/ttyACM*` 查看设备是否存在。
    *   **检查端口**: 如果指定了 `port` 参数，请确保指定的端口号正确且存在。尝试不指定 `port` 让节点自动检测。
    *   **设备冲突**: 确保没有其他程序 (如 Arduino IDE 的串口监视器) 正在占用该串口。
*   **节点崩溃或无响应**:
    *   **检查日志**: 查看 `roslaunch` 输出或 ROS 日志文件 (`~/.ros/log`) 获取详细错误信息。
    *   **检查依赖**: 确保所有依赖项已正确安装。
    *   **检查代码**: 确认代码已完整克隆或下载，并且已成功编译 (`catkin_make` 无错误)。
*   **数据接收不正确或不稳定**:
    *   **检查波特率**: 确认 launch 文件中的 `baudrate` 与机械臂硬件设置完全一致。
    *   **检查连接**: 检查 USB 线缆是否连接牢固，尝试更换 USB 端口或线缆。
    *   **启用调试模式**: 运行 `roslaunch alicia_duo_driver serial_server.launch debug_mode:=true` 查看原始串口数据，判断问题出在底层通信还是上层解析。