# RH6 手部控制系统运行指南

## 系统概述

本系统支持多种通信方式控制 RH6 手部，包括 EtherCAT、CAN、串口、TCP、UDP 和共享内存通信。系统采用桥梁架构，支持双手同时控制。

## 系统架构

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   EtherCAT      │    │   ROS2 Bridge    │    │   ROS2 Topics   │
│   Master        │◄──►│   System         │◄──►│   /hand_cmd     │
│   (ryhand6_ncb) │    │   (hand_system_  │    │   /hand_state   │
│                 │    │    bridge)       │    │   /joint_states │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │
         ▼                       ▼
┌─────────────────┐    ┌──────────────────┐
│   EtherCAT      │    │   Shared Memory  │
│   Slaves        │    │   /ethercat_data │
│   (Hand Motors) │    │                  │
└─────────────────┘    └──────────────────┘
```

## 环境要求

### 硬件要求
- x86_64 或 ARM64 开发板
- EtherCAT 网络接口（用于 EtherCAT 通信）
- CAN 接口（用于 CAN 通信，可选）
- 串口设备（用于串口通信，可选）

### 软件要求
- Ubuntu 20.04/22.04 或兼容系统
- ROS2 Humble/Iron
- EtherCAT 主站库 (libethercat)
- 睿研手部库 (libRyhand.so)

## 安装步骤

### 1. 安装系统依赖

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装基础依赖
sudo apt install -y build-essential cmake git wget curl

# 安装 ROS2 Humble
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop python3-argcomplete python3-colcon-common-extensions python3-rosdep python3-vcstool

# 安装 EtherCAT 依赖
sudo apt install -y ethercat-tools
# 注意：需要手动安装 libethercat 库

# 设置环境
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. 编译系统

```bash
# 进入工作空间
cd /path/to/ros2_6/cpp

# 安装依赖
rosdep install --from-paths . --ignore-src -r -y

# 编译所有包
colcon build

# 设置环境
source install/setup.bash
```

## 运行方法

### 方法1：EtherCAT 通信（推荐）

#### 步骤1：启动 EtherCAT 主站
```bash
# 终端1 - 启动 EtherCAT 主站
cd /path/to/hand_ecat/ryhand_ecat

# 编译 EtherCAT 主站（如果未编译）
make clean && make

# 启动 EtherCAT 主站（需要 root 权限）
sudo ./ryhand6_ncb

# 或者启动调试版本（查看详细日志）
sudo ./ryhand6_ncb_debug
```

#### 步骤2：启动 ROS2 桥梁系统
```bash
# 终端2 - 启动 ROS2 系统
cd /path/to/ros2_6/cpp
source install/setup.bash

# 启动 EtherCAT 桥梁系统
ros2 run rh6_ecat hand_system_bridge --comm ethercat
```

#### 步骤3：测试控制
```bash
# 终端3 - 测试控制
cd /path/to/ros2_6/cpp
source install/setup.bash

# 查看可用话题
ros2 topic list

# 查看手部状态
ros2 topic echo /hand_state

# 查看关节状态
ros2 topic echo /joint_states

# 发送位置控制命令
ros2 topic pub /hand_cmd std_msgs/msg/Float64MultiArray "data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]"

# 发送角度控制命令
ros2 topic pub /hand_cmd std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

### 方法2：CAN 通信

#### 步骤1：配置 CAN 接口
```bash
# 配置 CAN 接口
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
```

#### 步骤2：启动 ROS2 系统
```bash
# 终端1 - 启动 ROS2 系统
cd /path/to/ros2_6/cpp
source install/setup.bash

# 启动 CAN 桥梁系统
ros2 run rh6_ecat hand_system_bridge --comm can
```

#### 步骤3：测试控制
```bash
# 终端2 - 测试控制
cd /path/to/ros2_6/cpp
source install/setup.bash

# 发送控制命令
ros2 topic pub /hand_cmd std_msgs/msg/Float64MultiArray "data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]"
```

### 方法3：使用启动脚本

```bash
# 给脚本执行权限
chmod +x rh6_ecat/scripts/start_bridge_system.sh

# 启动 EtherCAT 系统
sudo ./rh6_ecat/scripts/start_bridge_system.sh ethercat

# 启动 CAN 系统
sudo ./rh6_ecat/scripts/start_bridge_system.sh can
```

### 方法4：使用 Launch 文件

```bash
# 启动 EtherCAT 系统
ros2 launch rh6_ecat hand_system_bridge.launch.py comm_type:=ethercat

# 启动 CAN 系统
ros2 launch rh6_ecat hand_system_bridge.launch.py comm_type:=can
```

## 控制命令详解

### 话题接口

#### 输入话题
- **`/hand_cmd`** (std_msgs/Float64MultiArray)
  - 手部控制命令
  - 数据格式：`[pos1, pos2, pos3, pos4, pos5, pos6]` 或 `[angle1, angle2, ..., angle11]`

#### 输出话题
- **`/hand_state`** (std_msgs/Float64MultiArray)
  - 手部状态反馈
  - 包含位置、速度、力矩等信息

- **`/joint_states`** (sensor_msgs/JointState)
  - 关节状态信息
  - 包含关节名称、位置、速度、力矩

### 控制模式

#### 模式0：原始数据控制
```bash
# 发送原始位置和速度命令
ros2 topic pub /hand_cmd std_msgs/msg/Float64MultiArray "data: [2048, 2048, 2048, 2048, 2048, 2048]"
```

#### 模式1：角度控制
```bash
# 发送关节角度命令（11个关节）
ros2 topic pub /hand_cmd std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

#### 模式2：末端位置控制
```bash
# 发送末端位置命令
ros2 topic pub /hand_cmd std_msgs/msg/Float64MultiArray "data: [0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0]"
```

## 故障排除

### 常见问题

#### 1. EtherCAT 主站启动失败
```bash
# 检查 EtherCAT 库是否安装
ldconfig -p | grep ethercat

# 检查网络接口
ip link show

# 检查权限
sudo -v
```

#### 2. ROS2 节点启动失败
```bash
# 检查环境设置
echo $ROS_DISTRO
source /opt/ros/humble/setup.bash

# 检查包是否编译
colcon list --packages-select rh6_ecat

# 重新编译
colcon build --packages-select rh6_ecat
```

#### 3. 共享内存问题
```bash
# 检查共享内存
ls -la /dev/shm/ethercat_data

# 清理共享内存
sudo rm -f /dev/shm/ethercat_data

# 重新启动 EtherCAT 主站
```

#### 4. 通信问题
```bash
# 检查话题
ros2 topic list

# 检查节点
ros2 node list

# 检查服务
ros2 service list
```

### 调试命令

#### 查看系统状态
```bash
# 查看所有话题
ros2 topic list

# 查看话题信息
ros2 topic info /hand_cmd
ros2 topic info /hand_state

# 查看节点信息
ros2 node info /hand_system_bridge

# 查看话题数据
ros2 topic echo /hand_state --once
```

#### 监控系统性能
```bash
# 查看话题频率
ros2 topic hz /hand_state

# 查看话题带宽
ros2 topic bw /hand_cmd

# 查看系统资源使用
htop
```

## 配置参数

### 手部参数配置
编辑 `rh6_ecat/params/hand_params.yaml`：

```yaml
# 手部索引 (0=左手, 1=右手)
hand_index: 0

# 控制周期 (毫秒)
period_ms: 10

# 共享内存名称
shm_name: "/ethercat_data"

# 手部模式
hand_mode: 0  # 0=位置, 1=速度, 2=力矩
```

### 通信参数配置
```bash
# EtherCAT 参数
--comm ethercat --config /path/to/ethercat_config.yaml

# CAN 参数
--comm can --config /path/to/can_config.yaml

# 串口参数
--comm serial --config /path/to/serial_config.yaml
```

## 性能优化

### 实时性优化
```bash
# 设置实时优先级
sudo chrt -f 99 ros2 run rh6_ecat hand_system_bridge --comm ethercat

# 设置 CPU 亲和性
taskset -c 0 ros2 run rh6_ecat hand_system_bridge --comm ethercat
```

### 网络优化
```bash
# 优化网络接口
sudo ethtool -G eth0 rx 1024 tx 1024
sudo ethtool -K eth0 gro off
sudo ethtool -K eth0 gso off
```

## 安全注意事项

1. **权限管理**：EtherCAT 主站需要 root 权限，确保系统安全
2. **网络隔离**：EtherCAT 网络应与办公网络隔离
3. **紧急停止**：确保有紧急停止机制
4. **数据备份**：定期备份配置参数

## 技术支持

如遇到问题，请提供以下信息：
1. 系统版本和硬件信息
2. 错误日志和输出
3. 配置参数
4. 复现步骤

---

**注意**：本系统支持多种通信方式，请根据实际硬件配置选择合适的通信方式。EtherCAT 通信需要专业的 EtherCAT 硬件和网络配置。
