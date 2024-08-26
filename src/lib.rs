// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]

pub mod adc;
pub mod aes;
pub mod aesb;
pub mod aic;
pub mod cache;
#[cfg(feature = "lcd-console")]
pub mod console;
#[cfg(feature = "lcd-console")]
pub mod display;
pub mod dma;
pub mod flexcom;
#[cfg(feature = "heap")]
pub mod heap;
pub mod isc;
pub mod l1cache;
pub mod l2cc;
pub mod lcdc;
pub mod lcdspi;
#[cfg(feature = "logging")]
pub mod logging;
pub mod pio;
pub mod pit;
pub mod pmc;
pub mod rstc;
pub mod rtc;
pub mod sckc;
pub mod sdmmc;
pub mod sfr;
pub mod sha;
pub mod shdwc;
pub mod spi;
pub mod tc;
pub mod trng;
pub mod twi;
pub mod uart;
