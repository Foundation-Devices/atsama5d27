This is a set of helpful scripts that I use to have a reproducible setup.

### Requirements

- SEGGER [JLink software](https://www.segger.com/downloads/jlink/) to be installed
- [`arm-none-eabi`](https://developer.arm.com/downloads/-/gnu-rm) toolchain

### The flow

To set up the environment for iteration, you may want to open three terminal tabs and run
these scripts there in the following order:

1. [`./start-gdb-server.sh`](./start-gdb-server.sh) to connect to the board via JTAG and start the GDB server
2. [`./rtt-log.sh`](./rtt-log.sh) to connect to the board via RTT and collect console logs. Logs are saved in [`logs/`](logs).
3. [`./debug.sh`](./debug.sh) to build a debug binary and load it on the device via GDB session

Edit each script to ensure it's configured for your environment.
