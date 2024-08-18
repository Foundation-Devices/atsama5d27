#!/usr/bin/env bash

set -e

cargo build --release --bin aesb
arm-none-eabi-gdb -q ../target/armv7a-none-eabi/release/aesb -x init.gdb
