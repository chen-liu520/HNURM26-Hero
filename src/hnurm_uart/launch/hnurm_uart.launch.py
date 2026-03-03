import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    uart_dir = get_package_share_directory('hnurm_uart')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(uart_dir, 'params', 'default.yaml'),
            description='uart params file'
        ),
        Node(
            package='hnurm_uart',
            executable='hnurm_uart_node',
            output='screen',
            parameters=[params_file]
        ),
    ])
