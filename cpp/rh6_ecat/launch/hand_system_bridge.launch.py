#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # 声明启动参数
    comm_type_arg = DeclareLaunchArgument(
        'comm_type',
        default_value='',
        description='Communication type (REQUIRED: ethercat, can, serial, tcp, udp, shared_memory)'
    )
    
    hand_index_arg = DeclareLaunchArgument(
        'hand_index',
        default_value='0',
        description='Hand index: 0=left hand, 1=right hand'
    )
    
    period_ms_arg = DeclareLaunchArgument(
        'period_ms',
        default_value='10',
        description='Control period in milliseconds'
    )
    
    shm_name_arg = DeclareLaunchArgument(
        'shm_name',
        default_value='/ethercat_data',
        description='Shared memory name'
    )
    
    use_bridge_arg = DeclareLaunchArgument(
        'use_bridge',
        default_value='true',
        description='Use bridge architecture (true) or direct mode (false)'
    )
    
    # 获取启动参数
    comm_type = LaunchConfiguration('comm_type')
    hand_index = LaunchConfiguration('hand_index')
    period_ms = LaunchConfiguration('period_ms')
    shm_name = LaunchConfiguration('shm_name')
    use_bridge = LaunchConfiguration('use_bridge')
    
    # 桥梁模式节点
    bridge_node = Node(
        package='rh6_ecat',
        executable='hand_system_bridge',
        name='hand_system_bridge',
        arguments=['--comm', comm_type],
        parameters=[{
            'hand_index': hand_index,
            'period_ms': period_ms,
            'shm_name': shm_name,
        }],
        output='screen',
        condition=IfCondition(use_bridge)
    )
    
    # 直接模式节点
    direct_node = Node(
        package='rh6_ecat',
        executable='hand_node',
        name='hand_node',
        parameters=[{
            'hand_index': hand_index,
            'period_ms': period_ms,
            'shm_name': shm_name,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_bridge', default='false'))
    )
    
    # 手部模式控制示例节点
    mode_control_node = Node(
        package='rh6_ecat',
        executable='hand_mode_control',
        name='hand_mode_control',
        output='screen'
    )
    
    return LaunchDescription([
        # 启动参数
        comm_type_arg,
        hand_index_arg,
        period_ms_arg,
        shm_name_arg,
        use_bridge_arg,
        
        # 日志信息
        LogInfo(
            msg=['Starting RH6 Hand System with communication type: ', comm_type, ' and bridge mode: ', use_bridge]
        ),
        
        # 节点
        bridge_node,
        direct_node,
        mode_control_node,
    ])

