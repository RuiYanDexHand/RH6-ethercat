#!/bin/bash
# EtherCAT 延迟监控脚本
# 用法: ./monitor_latency.sh <pid> [interval_ms]

PID=$1
INTERVAL=${2:-1000}  # 默认1000ms

if [ -z "$PID" ]; then
    echo "用法: $0 <进程PID> [采样间隔(ms)]"
    echo "示例: $0 12345 1000"
    exit 1
fi

echo "监控进程 PID: $PID"
echo "采样间隔: ${INTERVAL}ms"
echo "按 Ctrl+C 停止"
echo "----------------------------------------"

while true; do
    # 获取进程状态
    if ! kill -0 $PID 2>/dev/null; then
        echo "进程 $PID 不存在或已退出"
        exit 1
    fi
    
    # 获取进程信息
    PS_INFO=$(ps -p $PID -o pid,ppid,cmd,etime,pcpu,pmem 2>/dev/null)
    
    # 获取线程信息（实时优先级）
    RT_THREADS=$(ps -T -p $PID -o tid,class,rtprio,ni,cmd 2>/dev/null | grep -E "FF|RR" || echo "无实时线程")
    
    # 获取系统负载
    LOAD=$(uptime | awk -F'load average:' '{print $2}')
    
    # 获取CPU频率（如果可用）
    CPU_FREQ=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null || echo "N/A")
    if [ "$CPU_FREQ" != "N/A" ]; then
        CPU_FREQ=$(echo "scale=2; $CPU_FREQ/1000" | bc)
        CPU_FREQ="${CPU_FREQ} MHz"
    fi
    
    # 清屏并显示信息
    clear
    echo "========== EtherCAT 延迟监控 =========="
    echo "时间: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "----------------------------------------"
    echo "$PS_INFO"
    echo ""
    echo "实时线程:"
    echo "$RT_THREADS"
    echo ""
    echo "系统负载: $LOAD"
    echo "CPU频率: $CPU_FREQ"
    echo ""
    
    # 检查 EtherCAT 主站（如果可用）
    if command -v ethercat &> /dev/null; then
        echo "EtherCAT 主站状态:"
        ethercat master 2>/dev/null | head -5 || echo "无法获取EtherCAT状态"
        echo ""
    fi
    
    # 检查网络延迟（如果使用EtherCAT网络）
    if command -v ping &> /dev/null; then
        # 替换为你的EtherCAT从站IP
        # PING_IP="192.168.1.100"
        # echo "网络延迟: $(ping -c 1 -W 1 $PING_IP 2>/dev/null | grep 'time=' | awk '{print $7}')"
    fi
    
    echo "----------------------------------------"
    echo "按 Ctrl+C 停止监控"
    
    sleep $(echo "scale=3; $INTERVAL/1000" | bc)
done

