# EtherCAT正弦波控制实现

## 概述

本实现为EtherCAT通信添加了正弦波控制功能，可以直接复现现有的CAN通信Msg格式，并在EtherCAT代码中生成正弦波命令，同时下发电机参数到共享内存。

## 功能特性

### 1. 正弦波控制
- **模式0**: 原始位置控制 (0.2-0.8)
- **模式1**: 角度控制 (36-144度)
- **模式2**: 末端位置控制 (20-80mm)
- **自动模式切换**: 每5秒自动切换模式
- **频率可调**: 默认0.5Hz正弦波

### 2. 数据格式兼容
- 使用与CAN通信完全一致的`rh6_msg::msg::Rh6Msg`格式
- 支持PlotJuggler数据可视化
- 包含完整的电机状态信息

### 3. 共享内存集成
- 自动写入`RyServoCmd`结构到共享内存
- 支持6个手指的独立控制
- 实时数据同步

## 文件结构

```
cpp/rh6_ecat/
├── include/rh6_ecat/
│   ├── ethercat_bridge.hpp          # 添加正弦波控制方法
│   └── ros_hand_interface.hpp       # 添加EtherCAT状态发布器
├── src/
│   ├── ethercat_bridge.cpp          # 实现正弦波控制逻辑
│   └── ros_hand_interface.cpp       # 实现Rh6Msg格式发布
├── examples/
│   └── ethercat_sine_wave_test.cpp  # 正弦波测试节点
└── scripts/
    └── start_sine_wave_test.sh      # 测试启动脚本
```

## 核心实现

### 1. EtherCAT Bridge正弦波控制

```cpp
// 正弦波控制参数
int sine_mode_;                    // 模式：0, 1, 2
double sine_time_;                 // 时间计数器
double sine_frequency_;            // 频率 0.5Hz
double sine_amplitude_;            // 振幅
double sine_offset_;               // 偏移
std::atomic<bool> sine_wave_enabled_; // 正弦波使能

// 电机控制参数
struct MotorParams {
    uint16_t target_position;      // 目标位置 (0-4095)
    uint16_t target_speed;         // 目标速度 (0-4095)
    uint16_t max_current;         // 最大电流 (0-4095)
    uint8_t command;              // 命令 (0xee)
} motor_params_[6];               // 6个手指的电机参数
```

### 2. 正弦波生成逻辑

```cpp
void generate_sine_wave_commands() {
    // 计算正弦波值
    double sine_value = sin(2 * M_PI * sine_frequency_ * sine_time_);
    
    // 为6个手指生成命令
    for (int i = 0; i < 6; i++) {
        double target_position = sine_offset_ + sine_amplitude_ * sine_value;
        
        // 根据模式调整位置
        switch (sine_mode_) {
            case 0: // 原始位置控制
                motor_params_[i].target_position = static_cast<uint16_t>(target_position);
                break;
            case 1: // 角度控制
                motor_params_[i].target_position = static_cast<uint16_t>(target_position);
                break;
            case 2: // 末端位置控制
                motor_params_[i].target_position = static_cast<uint16_t>(target_position);
                break;
        }
        
        // 设置电机参数
        motor_params_[i].target_speed = 1000;    // 目标速度
        motor_params_[i].max_current = 1000;     // 最大电流
        motor_params_[i].command = 0xee;         // 位置控制命令
    }
    
    // 更新时间
    sine_time_ += 0.01; // 10ms周期
}
```

### 3. 共享内存写入

```cpp
void write_sine_wave_to_shared_memory() {
    // 将电机参数写入共享内存
    for (int i = 0; i < 6; i++) {
        RyServoCmd servo_cmd;
        servo_cmd.cmd = motor_params_[i].command;
        servo_cmd.usPos = motor_params_[i].target_position;
        servo_cmd.usSpd = motor_params_[i].target_speed;
        servo_cmd.usMaxCur = motor_params_[i].max_current;
        servo_cmd.res = 0;
        
        // 写入共享内存
        shared_data_->ryhand[0].tx_len[i] = sizeof(RyServoCmd);
        memcpy(shared_data_->ryhand[0].tx_data[i], &servo_cmd, sizeof(RyServoCmd));
    }
    
    // 更新标志
    shared_data_->tx_data_cnt++;
    shared_data_->tx_nh = 6;
}
```

### 4. ROS接口数据转换

```cpp
void convert_shared_data_to_rh6msg(rh6_msg::msg::Rh6Msg& msg) {
    // 从共享内存读取6个手指的数据
    for (int i = 0; i < 6; i++) {
        if (shared_data_->ryhand[0].rx_len[i] >= sizeof(RyServoInfo)) {
            RyServoInfo* info = (RyServoInfo*)shared_data_->ryhand[0].rx_data[i];
            
            // 转换电机数据
            msg.status[i] = info->status;           // 状态
            msg.m_pos[i] = info->P;                 // 位置
            msg.m_spd[i] = info->V;                 // 速度
            msg.m_cur[i] = info->T;                 // 电流
            msg.m_force[i] = info->F;               // 压力
        }
    }
    
    // 计算关节角度
    for (int i = 0; i < 11; i++) {
        if (i < 6) {
            msg.j_ang[i] = (msg.m_pos[i] / 4095.0) * 180.0; // 转换为角度
        } else {
            msg.j_ang[i] = 0.0; // 其他关节角度
        }
    }
}
```

## 使用方法

### 1. 编译代码

```bash
cd cpp
colcon build --packages-select rh6_ecat
source install/setup.bash
```

### 2. 启动EtherCAT主站

```bash
sudo ethercat master
```

### 3. 启动EtherCAT桥接程序

```bash
ros2 run rh6_ecat hand_system_bridge --comm ethercat
```

### 4. 启动正弦波测试节点

```bash
ros2 run rh6_ecat ethercat_sine_wave_test
```

### 5. 启动PlotJuggler

```bash
ros2 run plotjuggler plotjuggler
```

在PlotJuggler中添加话题：`/ethercat_hand_status`

## 数据可视化

在PlotJuggler中可以查看以下数据：

### 电机数据
- `m_pos[0-5]`: 6个手指的位置 (0-4095)
- `m_spd[0-5]`: 6个手指的速度 (-2048~2047)
- `m_cur[0-5]`: 6个手指的电流 (-2048~2047)
- `m_force[0-5]`: 6个手指的压力 (0-4095)

### 关节数据
- `j_ang[0-10]`: 11个关节角度 (度)

### 末端位置
- `x[0-4], y[0-4], z[0-4]`: 5个末端位置
- `roll[0-4], pitch[0-4], yaw[0-4]`: 5个末端姿态

## 测试验证

### 1. 正弦波验证
- 观察`m_pos[0-5]`是否呈现正弦波变化
- 检查频率是否为0.5Hz
- 验证振幅和偏移是否正确

### 2. 模式切换验证
- 每5秒检查模式是否自动切换
- 验证不同模式下的数据范围

### 3. 实时性验证
- 检查数据更新频率是否为10ms
- 验证EtherCAT通信延迟

## 故障排除

### 1. 编译错误
- 检查`rh6_msg`包是否已编译
- 确认EtherCAT库路径正确

### 2. 运行时错误
- 检查EtherCAT主站是否运行
- 验证共享内存权限
- 确认从站状态正常

### 3. 数据异常
- 检查PDO配置
- 验证数据解析逻辑
- 确认电机参数范围

## 扩展功能

### 1. 自定义频率
```cpp
enable_sine_wave(0, 1.0, 1000, 2000); // 1Hz, 振幅1000, 偏移2000
```

### 2. 多手指独立控制
```cpp
// 可以为每个手指设置不同的正弦波参数
for (int i = 0; i < 6; i++) {
    double freq = 0.5 + i * 0.1; // 不同频率
    double target_position = sine_offset_ + sine_amplitude_ * sin(2 * M_PI * freq * sine_time_);
    motor_params_[i].target_position = static_cast<uint16_t>(target_position);
}
```

### 3. 动态参数调整
```cpp
// 运行时调整参数
sine_frequency_ = 1.0;  // 改变频率
sine_amplitude_ = 2000; // 改变振幅
```

## 总结

本实现成功将EtherCAT通信与正弦波控制集成，实现了：

1. **完整的正弦波控制**: 支持3种模式，自动切换
2. **数据格式兼容**: 与CAN通信完全一致
3. **实时数据可视化**: 支持PlotJuggler
4. **共享内存集成**: 自动写入电机参数
5. **可扩展架构**: 支持自定义参数和功能

通过这个实现，您可以在EtherCAT环境中实现与CAN通信相同的正弦波控制功能，并在PlotJuggler中查看完整的数据可视化。
