#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

ARDUINO_DEVICE=/dev/ttyACM0

arduino-cli compile --fqbn arduino:avr:leonardo $SCRIPTPATH/../src/MotorResetControl
arduino-cli upload -p $ARDUINO_DEVICE --fqbn arduino:avr:leonardo $SCRIPTPATH/../src/MotorResetControl
