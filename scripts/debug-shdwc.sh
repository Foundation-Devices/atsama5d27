#!/usr/bin/env bash

set -e

cargo build --release --bin shdwc
arm-none-eabi-gdb -q ../target/armv7a-none-eabi/release/shdwc -x init.gdb
