from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import os
import launch.actions

def generate_launch_description():
    pkg_share = get_package_share_directory('gps_odom_pkg')
    rl_params_file = os.path.join(pkg_share, 'config', 'dual_ekf_navsat_params.yaml')

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'output_final_position', default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            'output_location', default_value='~/dual_ekf_navsat_example_debug.txt'
        ),
        LogInfo(msg="Launching ekf_filter_node_odom..."),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[rl_params_file],
            remappings=[('odometry/filtered', 'odometry/local')],
        ),
        LogInfo(msg="Launching ekf_filter_node_map..."),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[rl_params_file],
            remappings=[('odometry/filtered', 'odometry/global')],
        ),
        LogInfo(msg="Launching navsat_transform..."),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[rl_params_file],
            remappings=[
                ('imu/data', '/Imu'),
                ('gps/fix', '/NavSatFix'),
                ('gps/filtered', 'gps/filtered'),
                ('odometry/gps', 'odometry/gps'),
                ('odometry/filtered', 'odometry/global'),
            ],
        ),
    ])
