#!/bin/bash

mem_kb=$(awk '/MemTotal/ {print $2}' /proc/meminfo)

if [ "$mem_kb" -le 8196044 ];   # less than or equal to ~8GB RAM
then
    echo "USING LIMITED THREADS DUE TO LOW RAM AMOUNT"
    export MAKEFLAGS="-j 1"
fi

if [[ "$1" == "--frappe-only" ]]; then
    colcon build \
        --symlink-install \
        --executor parallel \
        --event-handlers console_direct+ \
        --packages-select phoenix_ros_driver launch_utils lance \
        --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON
else
    colcon build \
        --symlink-install \
        --executor parallel \
        --event-handlers console_direct+ \
        --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON
fi
