#!/bin/bash

# RH6 手部控制系统启动脚本 - 桥梁架构版本
# 使用桥梁架构启动完整的 EtherCAT + ROS 系统

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查是否以 root 权限运行
check_root() {
    if [ "$EUID" -ne 0 ]; then
        log_error "请以 root 权限运行此脚本"
        log_info "使用方法: sudo ./start_bridge_system.sh"
        exit 1
    fi
}

# 检查 EtherCAT 环境
check_ethercat() {
    log_info "检查 EtherCAT 环境..."
    
    # 检查 EtherCAT 工具是否安装
    if ! command -v ethercat &> /dev/null; then
        log_error "EtherCAT 工具未安装，请先安装"
        log_info "安装命令: sudo apt-get install ethercat-tools"
        exit 1
    fi
    
    # 检查 EtherCAT 主站是否运行
    if ethercat master &> /dev/null; then
        # 检查是否有从站
        slave_count=$(ethercat master | grep "Slaves:" | awk '{print $2}')
        if [ "$slave_count" -gt 0 ]; then
            log_success "EtherCAT 主站已运行，检测到 $slave_count 个从站"
        else
            log_warning "EtherCAT 主站已运行，但未检测到从站"
            log_info "请检查 EtherCAT 设备连接"
        fi
        return 0
    else
        log_error "EtherCAT 主站未运行，请先启动主站"
        log_info "启动命令: sudo /usr/local/etc/init.d/ethercat start"
        exit 1
    fi
}

# 检查网络接口
check_network() {
    log_info "检查 EtherCAT 网络接口..."
    
    # 检查 EtherCAT 主站使用的网络接口
    if ethercat master &> /dev/null; then
        # 从 ethercat master 输出中提取网络接口信息
        main_device=$(ethercat master | grep "Main:" | awk '{print $2}')
        if [ -n "$main_device" ]; then
            log_success "EtherCAT 主站使用网络接口: $main_device"
        else
            log_warning "无法确定 EtherCAT 主站使用的网络接口"
        fi
    fi
    
    log_success "网络接口检查完成"
}

# 启动桥梁系统
start_bridge_system() {
    log_info "启动 RH6 手部系统桥梁..."
    
    # 检查是否指定了通信方式
    if [ -z "$1" ]; then
        log_error "请指定通信方式！"
        log_info "使用方法: sudo ./start_bridge_system.sh <communication_type>"
        log_info "支持的通信方式:"
        log_info "  - ethercat"
        log_info "  - can"
        log_info "  - serial"
        log_info "  - tcp"
        log_info "  - udp"
        log_info "  - shared_memory"
        log_info ""
        log_info "示例:"
        log_info "  sudo ./start_bridge_system.sh ethercat"
        log_info "  sudo ./start_bridge_system.sh can"
        exit 1
    fi
    
    local comm_type="$1"
    log_info "使用通信方式: $comm_type"
    
    # 切换到项目根目录
    cd "$PROJECT_ROOT"
    
    # 检查桥梁程序是否存在
    if [ ! -f "install/rh6_ecat/lib/rh6_ecat/hand_system_bridge" ]; then
        log_error "桥梁系统程序不存在，请先编译"
        log_info "运行: colcon build --packages-select rh6_ecat"
        exit 1
    fi
    
    # 检查 ROS 环境
    if [ -z "$ROS_DISTRO" ]; then
        log_warning "ROS 环境未设置，尝试设置..."
        source /opt/ros/humble/setup.bash
    fi
    
    # 检查工作空间
    if [ ! -f "install/setup.bash" ]; then
        log_error "ROS 工作空间未构建，请先编译"
        log_info "运行: colcon build --packages-select rh6_ecat"
        exit 1
    fi
    
    # 设置工作空间环境
    source install/setup.bash
    
    # 启动桥梁系统
    log_info "启动桥梁系统程序..."
    ./install/rh6_ecat/lib/rh6_ecat/hand_system_bridge --comm "$comm_type" &
    BRIDGE_PID=$!
    
    # 等待系统启动
    sleep 3
    
    # 检查系统是否正在运行
    if ! kill -0 $BRIDGE_PID 2>/dev/null; then
        log_error "桥梁系统启动失败"
        exit 1
    fi
    
    log_success "桥梁系统启动成功 (PID: $BRIDGE_PID)"
    echo $BRIDGE_PID > /tmp/hand_system_bridge.pid
}

# 显示状态信息
show_status() {
    log_info "系统状态:"
    echo "  - 桥梁系统 PID: $(cat /tmp/hand_system_bridge.pid 2>/dev/null || echo 'N/A')"
    echo "  - 共享内存: $(ls -la /dev/shm/ethercat_data 2>/dev/null || echo 'N/A')"
    echo ""
    log_info "控制命令:"
    echo "  - 位置控制: ros2 topic pub /hand_cmd std_msgs/msg/Float64MultiArray \"data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]\""
    echo "  - 查看状态: ros2 topic echo /hand_state"
    echo "  - 查看关节状态: ros2 topic echo /joint_states"
    echo "  - 停止系统: sudo ./stop_bridge_system.sh"
}

# 清理函数
cleanup() {
    log_info "正在停止桥梁系统..."
    
    # 停止桥梁系统
    if [ -f "/tmp/hand_system_bridge.pid" ]; then
        BRIDGE_PID=$(cat /tmp/hand_system_bridge.pid)
        if kill -0 $BRIDGE_PID 2>/dev/null; then
            log_info "停止桥梁系统..."
            kill $BRIDGE_PID
            rm -f /tmp/hand_system_bridge.pid
        fi
    fi
    
    # 清理共享内存
    if [ -f "/dev/shm/ethercat_data" ]; then
        log_info "清理共享内存..."
        rm -f /dev/shm/ethercat_data
    fi
    
    log_success "桥梁系统已停止"
}

# 主函数
main() {
    log_info "RH6 手部系统桥梁启动脚本"
    log_info "=========================="
    
    # 检查权限
    check_root
    
    # 检查 EtherCAT 环境
    check_ethercat
    
    # 检查网络接口
    check_network
    
    # 启动桥梁系统
    start_bridge_system "$1"
    
    # 显示状态
    show_status
    
    log_success "桥梁系统启动完成！"
    log_info "按 Ctrl+C 停止系统"
    
    # 等待中断信号
    trap cleanup EXIT INT TERM
    wait
}

# 运行主函数
main "$@"
