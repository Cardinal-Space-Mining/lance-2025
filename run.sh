#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

source "$SCRIPTPATH/../install/setup.bash"

# Check if first argument is --canbus
if [[ "$1" == "--canbus" ]]; then
    # Run the can bus bringup script
    "$SCRIPTPATH/phoenix-driver/scripts/can_bringup.sh"
    
    # Remove the --canbus argument so ROS2 launch gets the rest
    shift
    RUN_CANBUS=true
else
    RUN_CANBUS=false
fi

# Base ROS2 launch command
BASE_CMD=(
    ros2
    launch
    lance
    lance.launch.py
)
"${BASE_CMD[@]}" "$@"

# Run the can bus shutdown script if bringup was run
if [[ "$RUN_CANBUS" == true ]]; then
    "$SCRIPTPATH/phoenix-driver/scripts/can_shutdown.sh"
fi
