import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    mapping_param_dir = launch.substitutions.LaunchConfiguration(
        'mapping_param_dir',
        default=os.path.join(
            get_package_share_directory('scanmatcher'),
            'param',
            'mapping_robot.yaml'))

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[mapping_param_dir],
        remappings=[('/input_cloud','/wamv/sensors/lidars/lidar_wamv_sensor/points'),('/imu','/wamv/sensors/imu/imu/data')],
        #remappings=[('/imu','/gpsimu_driver/imu_data')],# for imu debug
        output='screen'
        )

    tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.7','0','1.8','0','0','0','1','wamv/base_link','wamv/base_link/lidar_wamv_sensor']
        )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'mapping_param_dir',
            default_value=mapping_param_dir,
            description='Full path to mapping parameter file to load'),
        mapping,
        tf
            ])