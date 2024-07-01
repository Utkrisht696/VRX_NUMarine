from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_navigation_pkg',
            executable='gps_to_ned_node',
            name='gps_to_ned_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='my_navigation_pkg',
            executable='pid_controller_node',
            name='pid_controller_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
