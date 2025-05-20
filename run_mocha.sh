#!/bin/bash

RUN_PERCEPTION=false
FG_ENABLED=true
FG_BRIDGE_MODE=test
LIDAR_LOGGING=false
MOTOR_LOGGING=true

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

$SCRIPTPATH/motor-control/scripts/can_bringup.sh

RUN_FG=$FG_ENABLED
DISABLE_STATE_PUB=false
if [[ "$1" == "--remote-bridge" ]]; then
    echo "Remote bridge mode set. Not running state publisher or foxglove bridge."
    RUN_FG=false
    DISABLE_STATE_PUB=true
fi

# source install/setup.bash
ros2 launch lance robot.launch.py \
    foxglove:=$RUN_FG \
    foxglove_mode:=$FG_BRIDGE_MODE \
    perception:=$RUN_PERCEPTION \
    record_lidar:=$LIDAR_LOGGING \
    record_motor:=$MOTOR_LOGGING \
    disable_state_pub:=$DISABLE_STATE_PUB \
    phoenix_driver:=6 \
    controller:=true

$SCRIPTPATH/motor-control/scripts/can_shutdown.sh
