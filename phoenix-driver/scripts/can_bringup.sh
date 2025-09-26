#!/bin/bash
set -euo pipefail

echo "[INFO] Starting CANable bringup..."

# Known CANable vendor:product ID
CANABLE_VID="16d0"
CANABLE_PID="117e"

# Each record: "serial:interface:bitrate"
CAN_DEVICES=(
    # TODO: replace or identify robot phx5 canable uid
    "206531765230:can_phx5:1000000"     # testing only - this is actually the phx6 canable on the robot
    "207B338D3630:can_phx6:1000000"
)

FOUND=0

for DEV in /dev/ttyACM*; do
    [ -e "$DEV" ] || continue

    # Get USB device info
    USB_PATH=$(udevadm info -q path -n "$DEV")
    USB_INFO=$(udevadm info -q all -p "$USB_PATH")

    VID=$(echo "$USB_INFO" | grep ID_VENDOR_ID | cut -d'=' -f2)
    PID=$(echo "$USB_INFO" | grep ID_MODEL_ID | cut -d'=' -f2)
    SERIAL=$(echo "$USB_INFO" | grep ID_SERIAL_SHORT= | cut -d'=' -f2)

    # Skip non-CANable devices
    if [[ "$VID" != "$CANABLE_VID" || "$PID" != "$CANABLE_PID" ]]; then
        echo "[INFO] Skipping $DEV: not a known CANable device ($VID:$PID)"
        continue
    fi

    FOUND_DEVICE=false
    for REC in "${CAN_DEVICES[@]}"; do
        IFS=":" read -r REC_SERIAL IFACE BITRATE <<< "$REC"
        if [[ "$SERIAL" == "$REC_SERIAL" ]]; then
            FOUND_DEVICE=true
            echo "[INFO] Found $DEV (serial $SERIAL) → $IFACE @ ${BITRATE}bps"

            # Clean up old interface
            sudo pkill -f "slcand.*$IFACE" || true
            sudo ip link delete "$IFACE" 2>/dev/null || true

            # Start slcand
            sudo slcand -o -c -s8 "$DEV" "$IFACE"
            sleep 0.5

            # Set CAN bitrate and bring up interface
            sudo ip link set "$IFACE" type can bitrate "$BITRATE"
            sudo ip link set "$IFACE" up
            sudo ip link set "$IFACE" txqueuelen 1000

            echo "[OK] $IFACE is up and running."
            FOUND=$((FOUND+1))
            break
        fi
    done

    if [ "$FOUND_DEVICE" = false ]; then
        echo "[INFO] Found CANable $DEV with serial $SERIAL, no mapping configured."
    fi
done

if [ "$FOUND" -eq 0 ]; then
    echo "[ERROR] No configured CANable devices found."
    exit 1
fi
