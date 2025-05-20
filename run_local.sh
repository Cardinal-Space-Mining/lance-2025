#!/bin/bash

FG_BRIDGE_MODE=test
MOTOR_LOGGING=true

# local bridge mode starts the robot state publisher and foxglove bridge nodes on the client rather than robot
LOCAL_BRIDGE=false
if [[ "$1" == "--local-bridge" ]]; then
    LOCAL_BRIDGE=true
fi

ros2 launch lance remote.launch.py \
    foxglove_gui:=true \
    foxglove_bridge:=$LOCAL_BRIDGE \
    foxglove_bridge_mode:=$FG_BRIDGE_MODE \
    enable_state_pub:=$LOCAL_BRIDGE \
    record_motor:=$MOTOR_LOGGING
