#!/usr/bin/env bash

# Configure these if needed
export JLINK_RTT_LOGGER_PATH=/Applications/SEGGER/JLink_V782c/JLinkRTTLoggerExe

export DEVICE=ATSAMA5D27C-CU
export BINARY=../target/armv7a-none-eabi/debug/atsama5d27
export RTT_CHANNEL=0

set -e

log_name="$(pwd)/logs/$(date).log"

# Build binary if needed
if [ ! -f $BINARY ]; then
  cargo build
fi

# Extract the RTT control block address from the symbol table as auto search doesn't work for some reason
rtt_address=$(arm-none-eabi-objdump -t $BINARY | grep "_SEGGER_RTT" | awk '{print "0x" $1;}')
echo "RTT control block address: $rtt_address"
echo "Log name: logs/$log_name"

$JLINK_RTT_LOGGER_PATH -Device $DEVICE -RTTAddress $rtt_address -RTTChannel $RTT_CHANNEL -If JTAG -Speed 4000 "${log_name}"