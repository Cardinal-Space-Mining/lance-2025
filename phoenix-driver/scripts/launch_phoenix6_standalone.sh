#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

$SCRIPTPATH/can_bringup.sh
ros2 launch phoenix_ros_driver phx6.launch.py
$SCRIPTPATH/can_shutdown.sh
