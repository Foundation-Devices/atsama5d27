#!/usr/bin/env bash

set -e

cargo build --release --bin se
arm-none-eabi-gdb -q ../target/armv7a-none-eabi/release/se -x init.gdb
