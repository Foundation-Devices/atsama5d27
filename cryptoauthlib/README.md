<!--
SPDX-FileCopyrightText: 2023 Foundation Devices, Inc. <hello@foundationdevices.com>
SPDX-License-Identifier: GPL-3.0-or-later
-->

Rust bindings for the [Microchip CryptoAuthentication Library](https://github.com/MicrochipTech/cryptoauthlib) version 3.7.0.

The underlying C library is in the `cryptoauthlib` subdir. To update, remove it and clone the new version of the
library from [GitHub](https://github.com/MicrochipTech/cryptoauthlib), then remove the `.git` dir to keep it
vendored instead of being a submodule.

The bindings can be generated automatically using the following [bindgen](https://lib.rs/crates/bindgen-cli) command
in the `cryptoauthlib` crate directory:
```
bindgen cryptoauthlib/lib/cryptoauthlib.h -o src/bindings.rs -- -I ./cryptoauthlib/lib/ -D ATCA_NO_HEAP -D ATCA_HAL_SWI -D ATCA_ECC_SUPPORT
```
