//! Reset Controller (RSTC)

use utralib::{utra::rstc::*, HW_RSTC_BASE, *};

const RSTC_MR_KEY_PASSWD: u32 = 0xA5 << 24;

pub struct Rstc {
    base_addr: u32,
}

impl Default for Rstc {
    fn default() -> Self {
        Rstc::new()
    }
}

impl Rstc {
    pub fn new() -> Self {
        Self {
            base_addr: HW_RSTC_BASE as u32,
        }
    }

    /// Creates RSTC instance with a different base address. Used with virtual memory
    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    /// Resets the CPU and peripherals but not the backup area.
    pub fn do_reset(&self) {
        let mut rstc_csr = CSR::new(self.base_addr as *mut u32);
        const RSTC_CR_PROCRST_MSK: u32 = 1;
        rstc_csr.wo(CR, RSTC_MR_KEY_PASSWD | RSTC_CR_PROCRST_MSK);
    }
}
