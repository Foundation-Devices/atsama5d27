#!/usr/bin/env bash

set -euo pipefail

arm-none-eabi-gdb -q <<SCRIPT
target remote :3334
monitor reset
monitor halt
set  *((unsigned*)0xF8045400) = 0xfff
set  *((unsigned*)0xF8048054) = 0x66830004
monitor reset
SCRIPT
