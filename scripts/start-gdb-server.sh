#!/usr/bin/env bash

export JLINK_GDB_SERVER_PATH=/Applications/SEGGER/JLink_V782c/JLinkGDBServer

set -e

$JLINK_GDB_SERVER_PATH -select USB -device ATSAMA5D27C-CU -endian little -if JTAG -speed auto -noir -noLocalhostOnly \
  -nologtofile -port 3333 -SWOPort 2311 -TelnetPort 2333
