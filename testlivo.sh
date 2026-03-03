#!/bin/bash

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

gnome-terminal -- bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 launch fast_livo mapping_mid360.launch.py; exec bash"
