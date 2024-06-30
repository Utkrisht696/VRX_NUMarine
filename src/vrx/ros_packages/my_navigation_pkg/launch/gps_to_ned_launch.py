from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_navigation_pkg',
            executable='gps_to_ned_node',
            name='gps_to_ned_node',
            output='screen'
        )
    ])
