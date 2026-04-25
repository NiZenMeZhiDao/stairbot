from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wit_ros2_imu',
            executable='wit_ros2_imu',
            name='imu_driver_node',
            output='screen',
            parameters=[{
                'port': '/dev/imu_usb',
                'baudrate': 115200,
                'protocol': 'TTL_STD',
                'modbusID': 0x50,
                'console_debug': False,
            }]
        ),

        # 1. 启动主动悬挂控制节点
        Node(
            package='active_suspension_control',
            executable='suspension_node',
            name='suspension_controller',
            output='screen'  # 将节点的 log 输出到终端屏幕
        ),
        
        
        Node(
            package='ares_usb',
            executable='usb_bridge_node',
            name='usb_bridge_node',
            output='screen'
        ),
        Node(
            package='multi_serial_sensor',
            executable='multi_serial_node',
            name='multi_serial_node',
            output='screen'
        ),
    ])
