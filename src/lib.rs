#![no_std]

pub mod aes;
pub mod aic;
pub mod cache;
pub mod l1cache;
pub mod l2cc;
pub mod lcdc;
pub mod pio;
pub mod pit;
pub mod pmc;
pub mod sfr;
pub mod tc;
pub mod trng;
pub mod uart;
pub mod dma;

#[cfg(feature = "lcd-console")]
pub mod console;
#[cfg(feature = "lcd-console")]
pub mod display;
