#!/bin/bash

echo "Detecting CANable USB device..."

# Known CANable vendor:product ID
CANABLE_ID="16d0:117e"

# Search all ttyACM devices for the correct ID
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

    if [ "$DEVICE_ID" == "$CANABLE_ID" ]; then
        DEVICE="$DEV"
        break
    fi
done

if [ -z "$DEVICE" ]; then
    echo "ERROR: No CANable device found. Please check the connection."
    exit 1
fi

echo "Found CANable device: $DEVICE"
echo "Bringing up can0..."

# Start slcand and bring up can0
sudo slcand -o -c -s0 "$DEVICE" can0
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to start slcand on $DEVICE"
    echo "Shutting down the can device."

    sudo ip link set can0 down
    sudo pkill slcand
    sudo ip link delete can0
    exit 1
fi
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000

echo "can0 is up and running."