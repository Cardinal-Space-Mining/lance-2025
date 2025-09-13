#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

source $SCRIPTPATH/../install/setup.bash

BASE_CMD=(
    ros2
    launch
    lance
    lance.launch.py
    preset:=client
)
"${BASE_CMD[@]}" "$@"
