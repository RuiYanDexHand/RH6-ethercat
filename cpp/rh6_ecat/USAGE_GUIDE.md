# RH6 手部控制系统使用指南

## 🎯 **系统架构**

本系统采用桥梁架构，支持多种通信方式，将ROS和底层通信完全分离：

```
ROS Node ←→ ROS Interface ←→ Communication Bridge ←→ 底层通信
                ↓                    ↓
            共享内存              共享内存
```

### 📡 **支持的通信方式**

| 通信方式 | 状态 | 描述 | 适用场景 |
|----------|------|------|----------|
| **EtherCAT** | ✅ 已实现 | 实时以太网通信 | 高精度控制、工业应用 |
| **CAN** | 🚧 待实现 | CAN总线通信 | 汽车、工业自动化 |
| **串口** | 🚧 待实现 | RS232/RS485通信 | 简单设备、调试 |
| **TCP** | 🚧 待实现 | TCP网络通信 | 远程控制、网络设备 |
| **UDP** | 🚧 待实现 | UDP网络通信 | 实时数据传输 |
| **共享内存** | 🚧 待实现 | 本地进程间通信 | 高性能、低延迟 |

## 🚀 **快速开始**

### 1. 启动EtherCAT主站（已完成）

您已经成功启动了EtherCAT主站，检测到1个从站：

```bash
# 检查主站状态
sudo ethercat master

# 应该看到类似输出：
# Master0
#   Phase: Idle
#   Active: no
#   Slaves: 1
#   Ethernet devices:
#     Main: 3c:6d:66:b2:b4:03 (attached)
#       Link: UP
```

### 2. 编译ROS包

```bash
# 进入工作空间
cd ~/ros2_6/cpp

# 编译包
colcon build --packages-select rh6_ecat

# 设置环境
source install/setup.bash
```

### 3. 启动桥梁系统

#### 方法1: 使用启动脚本（推荐）

```bash
# 启动桥梁系统（默认EtherCAT）
sudo ./scripts/start_bridge_system.sh

# 停止桥梁系统
sudo ./scripts/stop_bridge_system.sh
```

#### 方法2: 使用ROS Launch

```bash
# 启动桥梁系统
ros2 launch rh6_ecat hand_system_bridge.launch.py

# 带参数启动
ros2 launch rh6_ecat hand_system_bridge.launch.py hand_index:=0 period_ms:=10
```

#### 方法3: 手动启动（支持通信方式选择）

```bash
# 使用EtherCAT通信（默认）
sudo ./install/rh6_ecat/lib/rh6_ecat/hand_system_bridge --comm ethercat

# 使用CAN通信（待实现）
sudo ./install/rh6_ecat/lib/rh6_ecat/hand_system_bridge --comm can

# 使用串口通信（待实现）
sudo ./install/rh6_ecat/lib/rh6_ecat/hand_system_bridge --comm serial

# 使用TCP通信（待实现）
sudo ./install/rh6_ecat/lib/rh6_ecat/hand_system_bridge --comm tcp

# 使用UDP通信（待实现）
sudo ./install/rh6_ecat/lib/rh6_ecat/hand_system_bridge --comm udp

# 使用共享内存通信（待实现）
sudo ./install/rh6_ecat/lib/rh6_ecat/hand_system_bridge --comm shared_memory

# 带配置文件启动
sudo ./install/rh6_ecat/lib/rh6_ecat/hand_system_bridge --comm ethercat --config /path/to/config.yaml

# 查看帮助
./install/rh6_ecat/lib/rh6_ecat/hand_system_bridge --help
```

#### 方法4: 控制手部

```bash
# 终端2: 控制手部
ros2 topic pub /hand_cmd std_msgs/msg/Float64MultiArray "data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]"
```

## 🎮 **控制手部**

### 位置控制

```bash
# 发送位置命令（0.0-1.0，6个手指）
ros2 topic pub /hand_cmd std_msgs/msg/Float64MultiArray "data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]"

# 张开所有手指
ros2 topic pub /hand_cmd std_msgs/msg/Float64MultiArray "data: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"

# 闭合所有手指
ros2 topic pub /hand_cmd std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

### 查看状态

```bash
# 查看手部状态
ros2 topic echo /hand_state

# 查看关节状态
ros2 topic echo /joint_states

# 查看话题列表
ros2 topic list
```

### 手部模式控制

#### 动态切换手部模式

```bash
# 设置左手为位置控制模式
ros2 service call /set_hand_mode rh6_ecat/srv/SetHandMode "{hand_index: 0, mode: 0}"

# 设置右手为速度控制模式
ros2 service call /set_hand_mode rh6_ecat/srv/SetHandMode "{hand_index: 1, mode: 1}"

# 设置左手为力矩控制模式
ros2 service call /set_hand_mode rh6_ecat/srv/SetHandMode "{hand_index: 0, mode: 2}"
```

#### 手部模式说明

- **模式 0 (位置控制)**: 控制手指位置，适合精确抓取
- **模式 1 (速度控制)**: 控制手指速度，适合快速运动
- **模式 2 (力矩控制)**: 控制手指力矩，适合力控制抓取

#### 手部模式控制示例

```bash
# 启动手部模式控制示例
ros2 run rh6_ecat hand_mode_control
```

## 🔧 **系统监控**

### 检查系统状态

```bash
# 检查桥梁系统进程
ps aux | grep hand_system_bridge

# 检查共享内存
ls -la /dev/shm/ethercat_data

# 检查EtherCAT主站
sudo ethercat master

# 检查从站信息
sudo ethercat slaves
```

### 日志查看

```bash
# 查看ROS日志
ros2 log list

# 查看特定节点日志
ros2 log get_logger_level /ros_hand_interface
```

## 🛠️ **故障排除**

### 常见问题

1. **EtherCAT主站未运行**
   ```bash
   # 启动主站
   sudo /usr/local/etc/init.d/ethercat start
   ```

2. **未检测到从站**
   ```bash
   # 检查网络连接
   sudo ethercat master
   
   # 重启主站
   sudo /usr/local/etc/init.d/ethercat restart
   ```

3. **桥梁系统启动失败**
   ```bash
   # 检查编译
   colcon build --packages-select rh6_ecat
   
   # 检查权限
   sudo ./install/rh6_ecat/lib/rh6_ecat/hand_system_bridge
   ```

4. **共享内存错误**
   ```bash
   # 清理共享内存
   sudo rm -f /dev/shm/ethercat_data
   
   # 重启桥梁系统
   sudo ./scripts/start_bridge_system.sh
   ```

### 调试模式

```bash
# 启用调试输出
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# 启动桥梁系统
sudo ./install/rh6_ecat/lib/rh6_ecat/hand_system_bridge
```

## 📊 **系统参数**

### 可配置参数

- `hand_index`: 手部索引（0=左手，1=右手）
- `period_ms`: 控制周期（毫秒）
- `shm_name`: 共享内存名称

### 修改参数

```bash
# 通过launch文件修改
ros2 launch rh6_ecat hand_system_bridge.launch.py hand_index:=1 period_ms:=5

# 通过参数文件修改
# 编辑 params/hand_params.yaml
```

## 🔄 **系统架构优势**

1. **高移植性**: 可以轻松替换EtherCAT为CAN、串口等
2. **模块化**: 每个组件职责清晰
3. **可测试**: 可以独立测试每个组件
4. **可扩展**: 容易添加新的通信方式
5. **易维护**: 代码结构清晰，易于维护

## 📝 **注意事项**

1. 确保EtherCAT主站已启动并检测到从站
2. 使用sudo权限运行桥梁系统
3. 确保网络接口配置正确
4. 定期检查系统状态和日志
5. 在停止系统前先停止ROS节点

## 🆘 **获取帮助**

如果遇到问题，请检查：

1. EtherCAT主站状态
2. 网络接口配置
3. 系统日志
4. 共享内存状态
5. 进程状态

---

**祝您使用愉快！** 🎉
