#!/bin/bash

# 完整的机器人启动脚本（包含trigger）
# 每个模块在新终端窗口中运行

echo "=========================================="
echo "启动完整机器人系统（含trigger）"
echo "=========================================="

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

echo "[1/4] 启动雷达驱动 (Livox MID360)..."
gnome-terminal -- bash -c "cd /home/rm/nav2 && source install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_launch.py; exec bash"

sleep 3

echo "[2/4] 启动重定位节点..."
gnome-terminal -- bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 launch registration registration_rviz.launch.py; exec bash"

sleep 1

echo "[3/4] 启动 TF树..."
gnome-terminal -- bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 launch tf_transformer tf_transformer.launch.py; exec bash"

sleep 1

echo "[4/4] 启动 trigger 节点..."
gnome-terminal -- bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 run trigger_registration trigger_hero_node; exec bash"

echo "=========================================="
echo "所有模块已在新窗口启动！"
echo "=========================================="
