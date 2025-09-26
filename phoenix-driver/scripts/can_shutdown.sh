#!/bin/bash
set -euo pipefail

echo "[INFO] Shutting down all slcand-managed CAN interfaces..."

# Bring down all canX interfaces
for IFACE in $(ip -o link show | awk -F': ' '{print $2}' | grep '^can'); do
    echo " - Bringing down $IFACE"
    sudo ip link set "$IFACE" down || true
    sudo ip link delete "$IFACE" 2>/dev/null || true
done

# Kill any slcand processes
if pgrep slcand > /dev/null; then
    echo " - Killing slcand processes"
    sudo pkill slcand
fi

echo "[OK] All CAN interfaces shut down."
