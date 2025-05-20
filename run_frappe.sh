#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
$SCRIPTPATH/motor-control/scripts/can_bringup.sh

LIDAR_LOGGING=false
MOTOR_LOGGING=true

# source install/setup.bash
ros2 launch lance robot.launch.py \
    foxglove:=false \
    foxglove_mode:=live \
    perception:=false \
    record_lidar:=$LIDAR_LOGGING \
    record_motor:=$MOTOR_LOGGING \
    disable_state_pub:=true \
    phoenix_driver:=5 \
    controller:=false
