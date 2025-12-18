# RH6 EtherCAT Hand Control Package

这个包提供了基于共享内存的 EtherCAT 手部控制系统，直接与 EtherCAT 主站进程通过共享内存通信。

## 功能特性

- **完整的 EtherCAT 主站支持** - 包含从站配置和主站程序
- **C/C++ 语言兼容** - 完美解决 C 语言 EtherCAT 代码与 C++ ROS 代码的兼容性问题
- **基于共享内存的通信** - 高效的进程间通信
- **支持睿研手部控制协议** - 完整的手部控制功能
- **多种控制模式** - 位置控制、速度控制、力矩控制
- **ROS2 接口** - 标准的话题通信接口
- **自动化启动** - 一键启动整个系统
- **支持双手控制** - 左手/右手独立控制

## 依赖项

- ROS2 (Humble/Iron)
- EtherCAT 主站库 (libethercat)
- 共享内存支持 (POSIX)
- 睿研手部库 (libRyhand.so)
- 线程支持 (pthread)
- 睿研手部控制库 (可选)

## 安装和编译

1. 编译包：
```bash
cd cpp
colcon build --packages-select rh6_ecat
```

## 使用方法

### 前提条件

根据选择的通信方式，确保相应的环境已配置：
- **EtherCAT**: 确保 EtherCAT 主站进程正在运行并创建了共享内存 `/ethercat_data`
- **CAN**: 确保 CAN 接口已配置并可用
- **串口**: 确保串口设备已连接并配置
- **TCP/UDP**: 确保网络连接正常
- **共享内存**: 确保共享内存已创建

### 启动系统

**重要**: 必须明确指定通信方式，系统不会使用默认值。

#### 方法1: 使用 ROS2 命令

```bash
# 编译包
colcon build --packages-select rh6_ecat
source install/setup.bash

# 启动系统（必须指定通信方式）
ros2 run rh6_ecat hand_system_bridge --comm ethercat
ros2 run rh6_ecat hand_system_bridge --comm can
ros2 run rh6_ecat hand_system_bridge --comm serial
ros2 run rh6_ecat hand_system_bridge --comm tcp
ros2 run rh6_ecat hand_system_bridge --comm udp
ros2 run rh6_ecat hand_system_bridge --comm shared_memory
```

#### 方法2: 使用启动脚本

```bash
# 给脚本执行权限
chmod +x rh6_ecat/scripts/start_bridge_system.sh

# 启动系统（必须指定通信方式）
sudo ./rh6_ecat/scripts/start_bridge_system.sh ethercat
sudo ./rh6_ecat/scripts/start_bridge_system.sh can
sudo ./rh6_ecat/scripts/start_bridge_system.sh serial
```

#### 方法3: 使用 Launch 文件

```bash
# 使用 launch 文件启动（必须指定通信方式）
ros2 launch rh6_ecat hand_system_bridge.launch.py comm_type:=ethercat
ros2 launch rh6_ecat hand_system_bridge.launch.py comm_type:=can
ros2 launch rh6_ecat hand_system_bridge.launch.py comm_type:=serial
```

### 手部控制节点参数

```bash
# 控制左手 (默认)
ros2 run rh6_ecat hand_node

# 控制右手
ros2 run rh6_ecat hand_node --ros-args -p hand_index:=1

# 使用自定义共享内存名称
ros2 run rh6_ecat hand_node --ros-args -p shm_name:="/my_ethercat_data"

# 使用参数文件
ros2 run rh6_ecat hand_node --ros-args --params-file install/rh6_ecat/share/rh6_ecat/params/hand_params.yaml
```

### 发布手部命令

```bash
# 发布位置命令 (6个关节，范围 0.0-1.0)
ros2 topic pub /hand_cmd std_msgs/msg/Float64MultiArray "data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]"
```

### 查看手部状态

```bash
# 查看手部状态
ros2 topic echo /hand_state
```

### 手部控制模式

系统支持三种控制模式：

1. **模式 0 - 位置控制**：精确的位置控制，适合抓取任务
2. **模式 1 - 速度控制**：连续运动控制，适合连续操作
3. **模式 2 - 力矩控制**：力控制，适合精细操作

### 运行手部模式控制示例

```bash
# 运行手部模式控制示例（自动演示三种模式）
ros2 run rh6_ecat hand_mode_control
```

### C/C++ 兼容性测试

```bash
# 运行兼容性测试（验证 C 语言和 C++ 代码的兼容性）
./install/rh6_ecat/lib/rh6_ecat/compatibility_test
```

### 停止系统

```bash
# 停止完整系统
sudo ./install/rh6_ecat/lib/rh6_ecat/stop_hand_system.sh
```

## 配置参数

可以通过参数文件或命令行参数配置：

- `period_ms`: 控制周期 (毫秒)，默认 10ms
- `shm_name`: 共享内存名称，默认 "/ethercat_data"
- `hand_index`: 手部索引，0=左手，1=右手，默认 0

## 文件结构

```
rh6_ecat/
├── include/rh6_ecat/
│   ├── shared_data.h              # 共享内存数据结构
│   ├── ryhand_types.hpp           # 睿研类型定义
│   ├── ethercat_slave_config.h    # EtherCAT 从站配置
│   ├── shm_utils.h                # C 语言共享内存工具
│   ├── shm_utils_cpp.hpp          # C++ 兼容层
│   └── RyHandLib.h                # 睿研库头文件
├── src/
│   ├── hand_node.cpp              # ROS2 手部控制节点
│   ├── ethercat_master.cpp        # EtherCAT 主站程序
│   ├── ethercat_slave_config.c    # 从站配置实现
│   └── shm_utils.c                # C 语言共享内存实现
├── lib/
│   ├── libRyhand.so               # 睿研库文件
│   └── libRyhand64.so             # 64位睿研库文件
├── launch/
│   └── hand_ctrl.launch.py        # 启动文件
├── params/
│   └── hand_params.yaml           # 参数文件
├── scripts/
│   ├── start_hand_system.sh       # 系统启动脚本
│   └── stop_hand_system.sh        # 系统停止脚本
├── examples/
│   ├── shm_initialization_example.cpp  # 共享内存初始化示例
│   ├── hand_mode_control.cpp      # 手部模式控制示例
│   └── compatibility_test.cpp     # C/C++ 兼容性测试
└── CMakeLists.txt                 # 构建配置
```

## C/C++ 兼容性说明

### 问题背景
您的 `hand_ecat` 代码是 C 语言，而 ROS 代码是 C++，在共享内存通信中可能出现兼容性问题：

1. **结构体布局差异** - C 和 C++ 编译器可能对结构体进行不同的内存对齐
2. **名称修饰** - C++ 会进行名称修饰，C 不会
3. **头文件包含** - 需要正确的 `extern "C"` 声明
4. **数据类型兼容性** - 某些 C++ 类型在 C 中不存在

### 解决方案
本包提供了完整的 C/C++ 兼容层：

1. **C 语言共享内存工具** (`shm_utils.c/h`) - 与您的 C 语言代码完全兼容
2. **C++ 兼容层** (`shm_utils_cpp.hpp`) - 提供 C++ 风格的接口
3. **统一的数据结构** - 确保 C 和 C++ 使用相同的内存布局
4. **兼容性测试** - 验证两种语言的兼容性

### 使用方式
```cpp
// C++ 代码中使用
#include "rh6_ecat/shm_utils_cpp.hpp"

auto shm_manager = SharedMemoryUtils::create_manager(false, "/ethercat_data");
SharedData_t* data = shm_manager->get();
```

```c
// C 代码中使用
#include "rh6_ecat/shm_utils.h"

SharedData_t* data = create_shared_memory(0);
```

## 注意事项

1. 需要以 root 权限运行 EtherCAT 相关程序
2. 确保 EtherCAT 主站已正确配置
3. 运行兼容性测试确保 C/C++ 代码正常工作
3. 手部设备需要正确连接到 EtherCAT 网络
4. 控制命令的范围是 0.0-1.0，对应手部的 0-100% 位置

## 故障排除

1. 如果编译失败，检查 EtherCAT 库是否正确安装
2. 如果运行时出现权限错误，使用 sudo 运行
3. 如果手部无响应，检查 EtherCAT 网络连接和设备配置
