#![no_std]

pub mod aes;
pub mod aic;
pub mod cache;
#[cfg(feature = "lcd-console")]
pub mod console;
#[cfg(feature = "lcd-console")]
pub mod display;
pub mod dma;
pub mod flexcom;
pub mod isc;
pub mod l1cache;
pub mod l2cc;
pub mod lcdc;
pub mod lcdspi;
pub mod pio;
pub mod pit;
pub mod pmc;
pub mod sckc;
pub mod sfr;
pub mod spi;
pub mod tc;
pub mod trng;
pub mod twi;
pub mod uart;
