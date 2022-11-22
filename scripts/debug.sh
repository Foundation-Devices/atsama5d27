#!/usr/bin/env bash

set -e

cargo build
arm-none-eabi-gdb ../target/armv7a-none-eabi/debug/atsama5d27
