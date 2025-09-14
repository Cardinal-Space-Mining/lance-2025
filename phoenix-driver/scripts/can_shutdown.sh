#!/bin/bash
echo "Shutting down the can device."

sudo ip link set can0 down
sudo pkill slcand
sudo ip link delete can0
