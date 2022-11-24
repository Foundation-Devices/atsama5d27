#!/usr/bin/env bash

set -e

# Configure to your sdcard path
export VOLUME="/Volumes/NO NAME"

cargo build
arm-none-eabi-objcopy -I elf32-littlearm -O binary ../target/armv7a-none-eabi/debug/atsama5d27 "$VOLUME/app.bin"
