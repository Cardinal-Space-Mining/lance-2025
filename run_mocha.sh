#!/bin/bash

echo "Detecting Arduino USB device..."

# Known Arduino vendor:product ID
Arduino_ID="2341:8036"

for DEV in /dev/ttyACM*; do
    if [ ! -e "$DEV" ]; then
        continue
    fi

    # Extract the USB ID for the device
    USB_PATH=$(udevadm info -q path -n "$DEV")
    USB_ID=$(udevadm info -q all -p "$USB_PATH" | grep "ID_VENDOR_ID\|ID_MODEL_ID")

    VID=$(echo "$USB_ID" | grep ID_VENDOR_ID | cut -d'=' -f2)
    PID=$(echo "$USB_ID" | grep ID_MODEL_ID | cut -d'=' -f2)

    DEVICE_ID="$VID:$PID"

    if [ "$DEVICE_ID" == "$Arduino_ID" ]; then
        DEVICE="$DEV"
        echo "Arduino device is $DEVICE"
        break
    fi
done

if [ -z "$DEVICE" ]; then
    echo "ERROR: WELP"
    exit 1
fi

#--------------------------------------------------

RUN_PERCEPTION=false
FG_ENABLED=true
FG_BRIDGE_MODE=test
LIDAR_LOGGING=false
MOTOR_LOGGING=true

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

$SCRIPTPATH/motor-control/scripts/can_bringup.sh

RUN_FG=$FG_ENABLED
DISABLE_STATE_PUB=false
if [[ "$1" == "--remote-bridge" ]]; then
    echo "Remote bridge mode set. Not running state publisher or foxglove bridge."
    RUN_FG=false
    DISABLE_STATE_PUB=true
fi

# source install/setup.bash
ros2 launch lance robot.launch.py \
    foxglove:=$RUN_FG \
    foxglove_mode:=$FG_BRIDGE_MODE \
    perception:=$RUN_PERCEPTION \
    record_lidar:=$LIDAR_LOGGING \
    record_motor:=$MOTOR_LOGGING \
    disable_state_pub:=$DISABLE_STATE_PUB \
    phoenix_driver:=6 \
    controller:=true \
    arduino_device:=$DEVICE

$SCRIPTPATH/motor-control/scripts/can_shutdown.sh
