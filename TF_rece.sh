#!/bin/bash

# TF 测试启动脚本
# 在两个终端分别启动 tf_transformer 和 tf_test_node

WORKSPACE_DIR="/home/robot/hnurm_Hero2"

# 检查并 source ROS2 环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source "/opt/ros/foxy/setup.bash"
elif [ -f "/opt/ros/galactic/setup.bash" ]; then
    source "/opt/ros/galactic/setup.bash"
else
    echo "警告: 未找到 ROS2 环境，请手动 source"
fi

# source 工作空间
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
    echo "已加载工作空间: $WORKSPACE_DIR"
else
    echo "错误: 未找到 install/setup.bash，请先编译工作空间"
    echo "运行: cd $WORKSPACE_DIR && colcon build --symlink-install"
    exit 1
fi

echo "==================================="
echo "  启动 TF 测试节点"
echo "==================================="
echo ""
echo "终端1: 启动 tf_transformer (带 RViz)"
echo "终端2: 启动 tf_test_node (测试数据)"
echo ""

# 检测可用的终端模拟器
if command -v gnome-terminal &> /dev/null; then
    TERMINAL="gnome-terminal"
    TERMINAL_OPTS="-- bash -c"
elif command -v konsole &> /dev/null; then
    TERMINAL="konsole"
    TERMINAL_OPTS="-e bash -c"
elif command -v xterm &> /dev/null; then
    TERMINAL="xterm"
    TERMINAL_OPTS="-e bash -c"
elif command -v terminator &> /dev/null; then
    TERMINAL="terminator"
    TERMINAL_OPTS="-e bash -c"
else
    echo "错误: 未找到支持的终端模拟器 (gnome-terminal/konsole/xterm/terminator)"
    exit 1
fi

echo "使用终端: $TERMINAL"
echo ""

# 终端1: 启动 tf_transformer (launch 文件会同时启动 RViz)
$TERMINAL $TERMINAL_OPTS "
    source /opt/ros/*/setup.bash 2>/dev/null || true
    source $WORKSPACE_DIR/install/setup.bash
    echo '==================================='
    echo '  tf_transformer + RViz'
    echo '==================================='
    ros2 launch tf_transformer tf_transformer.launch.py
    exec bash
" &
