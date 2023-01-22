#!/usr/bin/env bash

# Run `python3 -m serial.tools.list_ports -v` to list available serial ports
# Specify your devboard console device file name here
CONSOLE_DEV=/dev/tty.usbmodem0004831324171

python3 -m serial.tools.miniterm ${CONSOLE_DEV} 115200
