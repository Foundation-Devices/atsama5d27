#!/usr/bin/env bash

export JLINK_GDB_SERVER_PATH=JLinkGDBServer

set -e

GDB_SPEED_KHZ=100000
$JLINK_GDB_SERVER_PATH -select USB -device ATSAMA5D27C-CU -endian little -if JTAG -speed $GDB_SPEED_KHZ -noir -noLocalhostOnly \
  -nologtofile -port 3334 -SWOPort 2311 -TelnetPort 2333
