//! Slow Clock (SCKC) module.

use utralib::{utra::sckc::SCKC_CR_OSCSEL, *};

pub struct Sckc {
    base_addr: u32,
}

const SLOW_CLOCK_INTERNAL_HZ: u32 = 64_000;
const SLOW_CLOCK_EXTERNAL_HZ: u32 = 32_768;

impl Sckc {
    pub fn new() -> Self {
        Sckc {
            base_addr: HW_SCKC_BASE as u32,
        }
    }

    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Sckc { base_addr }
    }

    pub fn get_clock_freq_hz(&self) -> u32 {
        if self.is_internal() {
            SLOW_CLOCK_INTERNAL_HZ
        } else {
            SLOW_CLOCK_EXTERNAL_HZ
        }
    }

    pub fn is_internal(&self) -> bool {
        let csr = CSR::new(self.base_addr as *mut u32);
        csr.rf(SCKC_CR_OSCSEL) == 0
    }
}
