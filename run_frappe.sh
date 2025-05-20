#!/bin/bash

LIDAR_LOGGING=false
MOTOR_LOGGING=true

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

if [[ "$1" == "--full" ]]; then
    $SCRIPTPATH/motor-control/scripts/can_bringup.sh

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
else
    $SCRIPTPATH/motor-control/scripts/launch_phoenix5_standalone.sh
fi
