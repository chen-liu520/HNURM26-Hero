#!/bin/bash

# 精简版机器人启动脚本（不含trigger）
# 每个模块在新终端窗口中运行

echo "=========================================="
echo "启动精简机器人系统（不含trigger）"
echo "=========================================="

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

echo "[1/4] 启动雷达驱动 (Livox MID360)..."
gnome-terminal -- bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_launch.py; exec bash"

sleep 2

echo "[2/4] 启动 FAST_LIVO2 里程计..."
gnome-terminal -- bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 launch fast_livo mapping_mid360.launch.py; exec bash"

sleep 2

echo "[3/4] 启动重定位节点..."
gnome-terminal -- bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 launch registration registration_rviz.launch.py; exec bash"

sleep 2

echo "[4/4] 启动 TF树..."
gnome-terminal -- bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 launch tf_transformer tf_transformer.launch.py; exec bash"

echo "=========================================="
echo "所有模块已在新窗口启动！"
echo "=========================================="
