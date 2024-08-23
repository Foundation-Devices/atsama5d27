#!/usr/bin/env bash

set -e

cargo build --release --bin aes --features aes
arm-none-eabi-gdb -q ../target/armv7a-none-eabi/release/aes -x init.gdb
