#!/usr/bin/env bash

set -e

cargo build --release
arm-none-eabi-gdb -q ../target/armv7a-none-eabi/release/atsama5d27 -x init.gdb
