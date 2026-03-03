#!/bin/bash

# 完整的机器人启动脚本（包含trigger）
# 每个模块在新终端窗口中运行

echo "=========================================="
echo "启动完整机器人系统（含trigger）"
echo "=========================================="

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

echo "[1/5] 启动串口..."
gnome-terminal -- bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 launch hnurm_uart hnurm_uart.launch.py; exec bash"

sleep 2

echo "[2/5] 启动 FAST_LIVO2 里程计..."
gnome-terminal -- bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 launch fast_livo low_mid360.launch.py; exec bash"

sleep 2

echo "[3/5] 启动重定位节点..."
gnome-terminal -- bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 launch registration registration_rviz.launch.py; exec bash"

sleep 2

echo "[4/5] 启动 TF树..."
gnome-terminal -- bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 launch tf_transformer tf_transformer.launch.py; exec bash"

sleep 2

echo "[5/5] 启动 trigger 节点..."
gnome-terminal -- bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 run trigger_registration trigger_hero_node; exec bash"

echo "=========================================="
echo "所有模块已在新窗口启动！"
echo "=========================================="
