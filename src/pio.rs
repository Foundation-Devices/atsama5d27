//! Parallel I/O controller (GPIO).

use core::marker::PhantomData;
use utralib::utra::pio::{HW_PIO_BASE, PIO_CODR0, PIO_ODSR0, PIO_SODR0};
use utralib::*;

pub struct PioA {}
pub struct PioB {}
pub struct PioC {}
pub struct PioD {}

/// Retrieves PIO peripheral base address for the specific PIO Port ID (A, B, C, D).
const fn get_pio_base_address(id: u32) -> u32 {
    HW_PIO_BASE as u32 + id * 0x40
}

pub trait PioPeriph {
    const ID: u32;
}

impl PioPeriph for PioA {
    const ID: u32 = 0;
}
impl PioPeriph for PioB {
    const ID: u32 = 1;
}
impl PioPeriph for PioC {
    const ID: u32 = 2;
}
impl PioPeriph for PioD {
    const ID: u32 = 3;
}

pub struct PioPin<P: PioPeriph> {
    pin_num: u32,
    periph: PhantomData<P>,
}

impl<P: PioPeriph> PioPin<P> {
    pub fn new(pin_num: u32) -> Self {
        PioPin {
            pin_num,
            periph: PhantomData,
        }
    }
}

impl<P: PioPeriph> PioPin<P> {
    /// Sets the pin into HIGH or LOW logic level.
    pub fn set(&mut self, hi: bool) {
        let mut pio_csr = CSR::new(get_pio_base_address(P::ID) as *mut u32);
        let pin_bit = 1 << self.pin_num;

        if hi {
            pio_csr.wo(PIO_SODR0, pin_bit);
        } else {
            pio_csr.wo(PIO_CODR0, pin_bit);
        }
    }

    /// Returns `true` if pin is in HIGH logic level.
    pub fn get(&self) -> bool {
        let pio_csr = CSR::new(get_pio_base_address(P::ID) as *mut u32);
        let pin_bit = 1 << self.pin_num;

        pio_csr.r(PIO_ODSR0) & pin_bit != 0
    }
}
