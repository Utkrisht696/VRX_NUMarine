import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kvaser_interface',
            executable='kvaser_can_bridge',
            name='kvaser_can_bridge',
            output='screen',
            parameters=[
                {'hardware_id': 110490},
                {'bit_rate': 1000000},
                {'enable_echo': True}
            ]
        ),
        TimerAction(
            period=5.0,  # Delay in seconds
            actions=[
                Node(
                    package='my_can_pkg',
                    executable='can_float_publisher',
                    name='can_float_publisher',
                    output='screen'
                )
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
