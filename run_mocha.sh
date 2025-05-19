#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
$SCRIPTPATH/motor-control/scripts/can_bringup.sh

# source install/setup.bash
ros2 launch lance robot.launch.py \
    foxglove:=true \
    foxglove_mode:=test \
    perception:=true \
    record_lidar:=false \
    record_motor:=true \
    disable_state_pub:=false \
    phoenix_driver:=6 \
    controller:=true
