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

# Default values
RUN_PERCEPTION=false
FG_ENABLED=true
FG_BRIDGE_MODE=test
LIDAR_LOGGING=false
MOTOR_LOGGING=true

RUN_FG=$FG_ENABLED
DISABLE_STATE_PUB=false

# Parse named arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    perception=*)
      RUN_PERCEPTION="${1#*=}"
      shift
      ;;
    lidar-logging=*)
      LIDAR_LOGGING="${1#*=}"
      shift
      ;;
    --client-bridge)
      RUN_FG=false
      DISABLE_STATE_PUB=true
      shift
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

$SCRIPTPATH/motor-control/scripts/can_bringup.sh

# echo "RUN_FG: $RUN_FG"
# echo "FG_BRIDGE_MODE: $FG_BRIDGE_MODE"
# echo "RUN_PERCEPTION: $RUN_PERCEPTION"
# echo "DISABLE_STATE_PUB: $DISABLE_STATE_PUB"

source $SCRIPTPATH/../install/setup.bash

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
    arduino_device:=$ARDUINO_DEVICE \
    force_lidar_driver:=$LIDAR_LOGGING

$SCRIPTPATH/motor-control/scripts/can_shutdown.sh
