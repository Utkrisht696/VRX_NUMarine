from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch',
            output='screen',
            parameters=['Home/Documents/GitHub/VRX_NUMarine/src/vrx/ros_packages/my_navigation_pkg/config/mppi_params.yaml']
        ),
        Node(
            package='my_navigation_pkg',
            executable='gps_to_ned_node',
            name='gps_to_ned_node',
            output='screen'
        ),
        Node(
            package='my_navigation_pkg',
            executable='thruster_command_node',
            name='thruster_command_node',
            output='screen'
        ),
    ])
