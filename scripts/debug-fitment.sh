#!/usr/bin/env bash

set -e

cargo build --release --bin fitment
arm-none-eabi-gdb -q ../target/armv7a-none-eabi/release/fitment -x init.gdb
