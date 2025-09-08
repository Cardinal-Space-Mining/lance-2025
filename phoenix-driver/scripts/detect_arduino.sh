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
