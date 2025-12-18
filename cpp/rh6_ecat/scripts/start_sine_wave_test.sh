#!/bin/bash

# EtherCAT正弦波测试启动脚本

echo "=== EtherCAT正弦波测试启动脚本 ==="
echo ""

# 检查EtherCAT主站是否运行
echo "1. 检查EtherCAT主站状态..."
if pgrep -f "ethercat" > /dev/null; then
    echo "✅ EtherCAT主站正在运行"
else
    echo "❌ EtherCAT主站未运行，请先启动EtherCAT主站"
    echo "   命令: sudo ethercat master"
    exit 1
fi

# 检查共享内存
echo ""
echo "2. 检查共享内存..."
if [ -f "/dev/shm/ethercat_data" ]; then
    echo "✅ 共享内存文件存在"
else
    echo "⚠️ 共享内存文件不存在，将自动创建"
fi

# 启动EtherCAT桥接程序
echo ""
echo "3. 启动EtherCAT桥接程序..."
echo "   命令: ros2 run rh6_ecat hand_system_bridge --comm ethercat"
echo "   请在新终端中运行上述命令"
echo ""

# 等待用户确认
read -p "按Enter键继续启动正弦波测试节点..."

# 启动正弦波测试节点
echo ""
echo "4. 启动正弦波测试节点..."
echo "   命令: ros2 run rh6_ecat ethercat_sine_wave_test"
echo ""

# 启动PlotJuggler
echo "5. 启动PlotJuggler进行数据可视化..."
echo "   命令: ros2 run plotjuggler plotjuggler"
echo "   在PlotJuggler中添加话题: /ethercat_hand_status"
echo ""

echo "=== 测试说明 ==="
echo "1. 正弦波测试节点会自动切换3种模式："
echo "   - 模式0: 原始位置控制 (0.2-0.8)"
echo "   - 模式1: 角度控制 (36-144度)"
echo "   - 模式2: 末端位置控制 (20-80mm)"
echo ""
echo "2. 每5秒自动切换模式"
echo ""
echo "3. 在PlotJuggler中可以查看："
echo "   - m_pos[0-5]: 6个手指的位置"
echo "   - m_spd[0-5]: 6个手指的速度"
echo "   - m_cur[0-5]: 6个手指的电流"
echo "   - m_force[0-5]: 6个手指的压力"
echo "   - j_ang[0-10]: 11个关节角度"
echo ""
echo "4. 按Ctrl+C停止测试"
echo ""

# 启动正弦波测试节点
ros2 run rh6_ecat ethercat_sine_wave_test
