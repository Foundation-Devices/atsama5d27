#!/usr/bin/env bash

# SPDX-FileCopyrightText: 2023 Foundation Devices, Inc. <hello@foundationdevices.com>
# SPDX-License-Identifier: MIT OR Apache-2.0

set -e

cargo build --release --bin rtc
arm-none-eabi-gdb -q ../target/armv7a-none-eabi/release/rtc -x init.gdb
