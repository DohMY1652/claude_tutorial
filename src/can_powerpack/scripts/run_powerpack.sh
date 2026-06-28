#!/bin/bash
source ~/claude_tutorial/install/setup.zsh

SHARE=$(ros2 pkg prefix can_powerpack)/share/can_powerpack
CONFIG=$SHARE/config/powerpack_config.yaml
MONITOR=$(ros2 pkg prefix can_powerpack)/lib/can_powerpack/pp_monitor.py

ros2 run can_powerpack can_bridge_node \
    --ros-args --namespace pack2 --params-file "$CONFIG" \
    >/tmp/can_bridge.log 2>&1 &
BRIDGE_PID=$!

ros2 run can_powerpack pp_controller \
    --ros-args --namespace pack2 --params-file "$CONFIG" \
    >/tmp/pp_ctrl.log 2>&1 &
CTRL_PID=$!

cleanup() {
    echo ""
    kill $BRIDGE_PID $CTRL_PID 2>/dev/null
    wait $BRIDGE_PID $CTRL_PID 2>/dev/null
}
trap cleanup EXIT INT TERM

sleep 0.5
python3 "$MONITOR"
