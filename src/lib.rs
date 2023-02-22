#![no_std]

pub mod aic;
pub mod lcdc;
pub mod pio;
pub mod pit;
pub mod pmc;
pub mod tc;
pub mod trng;
pub mod uart;

#[cfg(feature = "lcd-console")]
pub mod console;
#[cfg(feature = "lcd-console")]
pub mod display;
