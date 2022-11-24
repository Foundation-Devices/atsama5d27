#!/usr/bin/env bash

set -e

cargo build
arm-none-eabi-gdb -q ../target/armv7a-none-eabi/debug/atsama5d27
