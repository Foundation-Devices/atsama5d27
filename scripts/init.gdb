# SPDX-FileCopyrightText: 2023 Foundation Devices, Inc. <hello@foundationdevices.com>
# SPDX-License-Identifier: GPL-3.0-or-later

set pagination off

# Add and enable fancy text-based UI
#tui new-layout mylayout {-horizontal src 1 asm 1} 2 status 0 cmd 1
#layout mylayout

# JLinkGDB server is expected to be running at this port
target remote :3334

load
# b hal_uart_control
c
