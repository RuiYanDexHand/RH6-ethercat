from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    period_ms = LaunchConfiguration('period_ms')
    master_index = LaunchConfiguration('master_index')
    period_ns = LaunchConfiguration('period_ns')

    return LaunchDescription([
        DeclareLaunchArgument('period_ms', default_value='1'),
        DeclareLaunchArgument('master_index', default_value='0'),
        DeclareLaunchArgument('period_ns', default_value='1000000'),

        Node(
            package='rh6_ecat',
            executable='hand_node',
            name='hand_ecat_node',
            parameters=[{
                'period_ms': period_ms,
                'master_index': master_index,
                'period_ns': period_ns,
            }]
        )
    ])



