#!/bin/bash

ros2 launch lance remote.launch.py \
    foxglove_gui:=true \
    foxglove_bridge:=true \
    foxglove_bridge_mode:=test \
    enable_state_pub:=true \
    record_motor:=false
