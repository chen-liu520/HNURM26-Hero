#!/usr/bin/python3
# -- coding: utf-8 --**

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    # Find path
    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")

    #这里我们修改加载的雷达参数配置文件：mid360.yaml
    avia_config_cmd = os.path.join(config_file_dir, "low_mid360.yaml")
    #相机内参配置文件保持不变
    camera_config_cmd = os.path.join(config_file_dir, "camera_pinhole.yaml")  # 添加这行

    # 打开 use_rviz
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Whether to launch Rviz2",
    )

    avia_config_arg = DeclareLaunchArgument(
        'avia_params_file',
        default_value=avia_config_cmd,
        description='Full path to the ROS2 parameters file to use for fast_livo2 nodes',
    )
    camera_config_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_config_cmd,
        description='Full path to the ROS2 parameters file to use for vikit_ros nodes',
    )

    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', 
        default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    avia_params_file = LaunchConfiguration('avia_params_file')
    camera_params_file = LaunchConfiguration('camera_params_file')
    use_respawn = LaunchConfiguration('use_respawn')

    return LaunchDescription([
        use_rviz_arg,
        avia_config_arg,
        camera_config_arg,
        use_respawn_arg,

        Node(
            package="image_transport",
            executable="republish",
            name="republish",
            arguments=[ 
                'compressed', 
                'raw',
            ],
            remappings=[
                ("in",  "/left_camera/image"), 
                ("out", "/left_camera/image")
            ],
            output="screen",
            respawn=use_respawn,
        ),

        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[
                avia_params_file,
                camera_params_file, 
            ],
            output="screen"
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen"
        ),
    ])