// SPDX-FileCopyrightText: 2023 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: GPL-3.0-or-later

fn main() {
    let dst = cmake::Config::new("cryptoauthlib")
        .no_build_target(true) // Prevent installing in host system directories
        .define("ATCA_BUILD_SHARED_LIBS", "0") // Build CAL as a static library
        .define("ATCA_HAL_SWI_UART", "ON") // Include swi support
        .define("CMAKE_C_COMPILER_WORKS", "1") // Don't check compiler validity
        // Don't support unneeded devices
        .define("ATCA_ATSHA204A_SUPPORT", "OFF")
        .define("ATCA_ATSHA206A_SUPPORT", "OFF")
        .define("ATCA_ATECC108A_SUPPORT", "OFF")
        .define("ATCA_ATECC508A_SUPPORT", "OFF")
        .define("ATCA_ATECC608_SUPPORT", "ON")
        .define("ATCA_ECC204_SUPPORT", "OFF")
        .define("ATCA_TA010_SUPPORT", "OFF")
        .define("ATCA_SHA104_SUPPORT", "OFF")
        .define("ATCA_SHA105_SUPPORT", "OFF")
        //.define("ATCA_PRINTF", "ON")
        .define("ATCA_NO_HEAP", "ON")
        .build();
    // Below caller LD_FLAGS are defined. First -L then -l
    println!("cargo:rustc-link-search=native={}/build/lib", dst.display());
    println!("cargo:rustc-link-lib=static=cryptoauth");
}
