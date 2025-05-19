#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
$SCRIPTPATH/motor-control/scripts/can_bringup.sh

# source install/setup.bash
ros2 launch lance robot.launch.py \
    foxglove:=false \
    foxglove_mode:=test \
    perception:=false \
    record_lidar:=false \
    record_motor:=false \
    disable_state_pub:=true \
    phoenix_driver:=6 \
    controller:=true
