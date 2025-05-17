#!/bin/bash

$cwd/motor-control/scripts/can_bringup.sh
source install/setup.bash
ros2 launch lance robot.launch.py foxglove:=false foxglove_mode:=live perception:=true record:=false phoenix_driver:=6 controller:=true
