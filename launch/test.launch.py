from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vl53l0x_driver',
            executable='vl53l0x_driver_node',
            name='vl53l0x_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baud_rate': 57600},
                {'frame_id': 'ir_range'}
            ]
        )
    ])
