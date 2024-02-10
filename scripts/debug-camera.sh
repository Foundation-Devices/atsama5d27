#!/usr/bin/env bash

set -e

cargo build --release --bin camera --features camera,camera-prod
arm-none-eabi-gdb -q ../target/armv7a-none-eabi/release/camera -x init.gdb
