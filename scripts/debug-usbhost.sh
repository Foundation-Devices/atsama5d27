#!/usr/bin/env bash

# SPDX-FileCopyrightText: 2023 Foundation Devices, Inc. <hello@foundationdevices.com>
# SPDX-License-Identifier: MIT OR Apache-2.0

set -e

cargo build --release --bin usbhost --features usb-host
arm-none-eabi-gdb -q ../target/armv7a-none-eabi/release/usbhost -x init.gdb
