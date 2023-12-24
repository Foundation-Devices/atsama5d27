#!/usr/bin/env bash

set -e

cargo build --release --bin nfc --features nfc
arm-none-eabi-gdb -q ../target/armv7a-none-eabi/release/nfc -x init.gdb
