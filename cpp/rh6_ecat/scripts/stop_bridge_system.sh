#!/bin/bash

# RH6 手部系统桥梁停止脚本

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

# 停止桥梁系统
stop_bridge_system() {
    log_info "停止 RH6 手部系统桥梁..."
    
    # 停止桥梁系统
    if [ -f "/tmp/hand_system_bridge.pid" ]; then
        BRIDGE_PID=$(cat /tmp/hand_system_bridge.pid)
        if kill -0 $BRIDGE_PID 2>/dev/null; then
            log_info "停止桥梁系统 (PID: $BRIDGE_PID)..."
            kill $BRIDGE_PID
            sleep 2
            
            # 强制杀死如果还在运行
            if kill -0 $BRIDGE_PID 2>/dev/null; then
                log_warning "强制停止桥梁系统..."
                kill -9 $BRIDGE_PID
            fi
            
            rm -f /tmp/hand_system_bridge.pid
            log_success "桥梁系统已停止"
        else
            log_warning "桥梁系统未运行"
            rm -f /tmp/hand_system_bridge.pid
        fi
    else
        log_warning "未找到桥梁系统 PID 文件"
    fi
}

# 清理共享内存
cleanup_shared_memory() {
    log_info "清理共享内存..."
    
    if [ -f "/dev/shm/ethercat_data" ]; then
        rm -f /dev/shm/ethercat_data
        log_success "共享内存已清理"
    else
        log_info "共享内存不存在"
    fi
}

# 清理其他相关进程
cleanup_related_processes() {
    log_info "清理相关进程..."
    
    # 清理可能残留的 ROS 节点
    pkill -f "hand_system_bridge" 2>/dev/null || true
    pkill -f "hand_node" 2>/dev/null || true
    pkill -f "ethercat_master" 2>/dev/null || true
    
    log_success "相关进程已清理"
}

# 主函数
main() {
    log_info "RH6 手部系统桥梁停止脚本"
    log_info "========================"
    
    # 停止桥梁系统
    stop_bridge_system
    
    # 清理相关进程
    cleanup_related_processes
    
    # 清理共享内存
    cleanup_shared_memory
    
    log_success "系统停止完成！"
}

# 运行主函数
main "$@"

