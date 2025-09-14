#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

source $SCRIPTPATH/../install/setup.bash

$SCRIPTPATH/phoenix-driver/scripts/can_bringup.sh

BASE_CMD=(
    ros2
    launch
    lance
    lance.launch.py
    preset:=frappe
)
"${BASE_CMD[@]}" "$@"

$SCRIPTPATH/phoenix-driver/scripts/can_shutdown.sh
