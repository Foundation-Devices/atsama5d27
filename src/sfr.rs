//! Special function registers (SFR).

use utralib::utra::sfr::*;
use utralib::{HW_SFR_BASE, *};

pub struct Sfr {
    base_addr: u32,
}

impl Default for Sfr {
    fn default() -> Self {
        Sfr::new()
    }
}

impl Sfr {
    pub fn new() -> Self {
        Self {
            base_addr: HW_SFR_BASE as u32,
        }
    }

    /// Creates L2CC instance with a different base address. Used with virtual memory
    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    pub fn set_l2_cache_sram_enabled(&mut self, enabled: bool) {
        let mut sfr_csr = CSR::new(self.base_addr as *mut u32);
        sfr_csr.rmwf(SFR_L2CC_HRAMC_SRAM_SEL, enabled as u32);
    }

    pub fn l2_cache_sram_enabled(&self) -> bool {
        let mut sfr_csr = CSR::new(self.base_addr as *mut u32);
        sfr_csr.rf(SFR_L2CC_HRAMC_SRAM_SEL) != 0
    }
}
