#!/usr/bin/env bash

set -e

cargo build --release --bin usb
arm-none-eabi-gdb -q ../target/armv7a-none-eabi/release/usb -x init.gdb
