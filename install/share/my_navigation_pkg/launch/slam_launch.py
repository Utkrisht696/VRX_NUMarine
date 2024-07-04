from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the configuration file
    config_file = PathJoinSubstitution([
        FindPackageShare('my_navigation_pkg'),
        'config',
        'slam_toolbox_params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[config_file]
        )
    ])
