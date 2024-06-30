from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=['src/my_navigation_pkg/config/mppi_params.yaml']
        ),
        Node(
            package='my_navigation_pkg',
            executable='gps_to_ned_node',
            name='gps_to_ned_node',
            output='screen'
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=['src/my_navigation_pkg/config/waypoints.yaml']
        )
    ])
