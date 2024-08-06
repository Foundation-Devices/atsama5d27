//! Shutdown Controller (SHDWC)

use utralib::{utra::shdwc::*, HW_SHDWC_BASE, *};

const SHDWC_MR_KEY_PASSWD: u32 = 0xA5 << 24;

pub struct Shdwc {
    base_addr: u32,
}

impl Default for Shdwc {
    fn default() -> Self {
        Shdwc::new()
    }
}

impl Shdwc {
    pub fn new() -> Self {
        Self {
            base_addr: HW_SHDWC_BASE as u32,
        }
    }

    /// Creates `SHDWC` instance with a different base address. Used with virtual memory
    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    /// Shuts the CPU down and asserts the `SHDN` PMIC signal to power off the rest of the
    /// system.
    pub fn do_shutdown(&self) {
        let mut shdwc_csr = CSR::new(self.base_addr as *mut u32);

        let reg = SHDWC_MR_KEY_PASSWD | shdwc_csr.ms(CR_SHDW, 1);
        shdwc_csr.wo(CR, reg);
    }
}
