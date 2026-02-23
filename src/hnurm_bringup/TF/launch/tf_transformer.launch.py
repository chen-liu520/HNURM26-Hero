import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable  # 添加这行


def generate_launch_description():
    camera_dir = get_package_share_directory('tf_transformer')
    params_file = LaunchConfiguration('params_file')
    
    # rviz 配置文件路径
    rviz_config_path = os.path.join(camera_dir, 'rviz', 'tf.rviz')
    
    return LaunchDescription([

        # 启用彩色日志输出
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        
        # 设置日志级别: DEBUG, INFO, WARN, ERROR, FATAL
        SetEnvironmentVariable('RCUTILS_LOGGING_SEVERITY', 'INFO'),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(camera_dir, 'params', 'extrinsic.yaml'),
            description='extrinsic to descripe static tf'
        ),
        Node(
            package='tf_transformer',
            executable='tf_transformer_node',
            output='screen',
            emulate_tty=True,  # 启用TTY模拟，确保颜色正常显示
            parameters=[params_file]
        ),
        启动 RViz2 并加载 tf.rviz 配置
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
