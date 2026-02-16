import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_dir = get_package_share_directory('registration')
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        # 设置环境变量：启用彩色日志输出
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        # 设置环境变量：使用 stdout 输出（确保颜色正常显示）
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        # 设置环境变量：禁用日志缓冲，实时输出
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(camera_dir, 'params', 'default.yaml'),
            description='small gicp params'
        ),
        Node(
            package='registration',
            executable='registration_node',
            output='screen',
            emulate_tty=True,  # 模拟 TTY，确保颜色正常显示
            parameters=[params_file]
        )
    ])
