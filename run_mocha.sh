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
        ARDUINO_DEVICE="$DEV"
        echo "Arduino device is $ARDUINO_DEVICE"
        break
    fi
done

if [ -z "$ARDUINO_DEVICE" ]; then
    echo "Failed to detect serial arduino."
    ARDUINO_DEVICE=/dev/ttyACM0
    # exit 1
fi

#--------------------------------------------------

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

$SCRIPTPATH/phoenix-driver/scripts/can_bringup.sh

source $SCRIPTPATH/../install/setup.bash

BASE_CMD=(
    ros2
    launch
    lance
    lance.launch.py
    preset:=mocha
    arduino_device:=$ARDUINO_DEVICE
)
"${BASE_CMD[@]}" "$@"

$SCRIPTPATH/phoenix-driver/scripts/can_shutdown.sh
