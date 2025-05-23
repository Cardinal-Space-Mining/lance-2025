#!/bin/bash

FG_BRIDGE_MODE=live
MOTOR_LOGGING=true
LIDAR_LOGGING=false

# local bridge mode starts the robot state publisher and foxglove bridge nodes on the client rather than robot
LOCAL_BRIDGE=true
if [[ "$1" == "--remote-bridge" ]]; then
    echo "Remote bridge mode set. NOT running state publisher and foxglove bridge locally."
    LOCAL_BRIDGE=false
fi

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

source $SCRIPTPATH/../install/setup.bash

ros2 launch lance client.launch.py \
    foxglove_gui:=true \
    foxglove_bridge:=$LOCAL_BRIDGE \
    foxglove_bridge_mode:=$FG_BRIDGE_MODE \
    enable_state_pub:=$LOCAL_BRIDGE \
    record_motor:=$MOTOR_LOGGING \
    record_lidar:=$LIDAR_LOGGING
