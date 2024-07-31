#!/usr/bin/env bash

set -euo pipefail

arm-none-eabi-gdb -q <<SCRIPT
target remote :3334
monitor halt
set  *((unsigned*)0xF8048054) = 0x66830000
monitor reset
SCRIPT
