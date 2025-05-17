#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
$SCRIPTPATH/motor-control/scripts/can_bringup.sh

source install/setup.bash
ros2 launch lance robot.launch.py foxglove:=false foxglove_mode:=live perception:=false record:=false phoenix_driver:=5 controller:=false
