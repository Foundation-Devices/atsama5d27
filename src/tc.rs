//! Timer-Counter (TC) implementation.

use utralib::utra::tc0::{CCR0, CMR0, CV0, IDR0, IER0, RC0, SR0};
use utralib::*;

const TC_CMR_TCCLKS_TIMER_CLOCK4: u32 = 0x3; // Clock selected: internal System bus clock divided by 128 clock signal (from PMC)
const TC_CMR_WAVEFORM_WAVSEL_UP_RC: u32 = 0x03; // UP mode with automatic trigger on RC Compare Position
const TC_CMR_WAVEFORM_WAVSEL_POS: u32 = 13;
const TC_CMR_WAVEFORM_MODE_MSK: u32 = 1 << 15;

const TC_CCR_CLKEN_MSK: u32 = 1; // Counter Clock Enable Command Mask
const TC_CCR_CLKDIS_MSK: u32 = 1 << 1; // Counter Clock Disable Command Mask
const TC_CCR_SWTRG_MSK: u32 = 1 << 2; // Software Trigger Command Mask

const TC_IER_CPCS_MSK: u32 = 1 << 4; // RC Compare Mask
const TC_IDR_CPCS_MSK: u32 = 1 << 4; // RC Compare Mask

const RC_DEFAULT: u32 = 32768;

pub struct Tc {
    base_addr: u32,
}

impl Default for Tc {
    fn default() -> Tc {
        Self::new()
    }
}

impl Tc {
    pub fn new() -> Self {
        Self {
            base_addr: HW_TC0_BASE as u32,
        }
    }

    /// Creates TC instance with a different base address. Used with virtual memory
    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    pub fn init(&mut self) {
        let mut tc_csr = CSR::new(self.base_addr as *mut u32);

        let cmr0 = TC_CMR_TCCLKS_TIMER_CLOCK4
            | (TC_CMR_WAVEFORM_WAVSEL_UP_RC << TC_CMR_WAVEFORM_WAVSEL_POS)
            | TC_CMR_WAVEFORM_MODE_MSK;
        tc_csr.wo(CMR0, cmr0);

        tc_csr.wo(RC0, RC_DEFAULT);
    }

    pub fn start(&mut self) {
        let mut tc_csr = CSR::new(self.base_addr as *mut u32);

        let ccr0 = TC_CCR_CLKEN_MSK | TC_CCR_SWTRG_MSK;
        tc_csr.wo(CCR0, ccr0);
    }

    pub fn stop(&mut self) {
        let mut tc_csr = CSR::new(self.base_addr as *mut u32);
        let ccr0 = TC_CCR_CLKDIS_MSK;
        tc_csr.wo(CCR0, ccr0);
    }

    pub fn set_period(&mut self, period: u32) {
        let mut tc_csr = CSR::new(self.base_addr as *mut u32);
        tc_csr.wo(RC0, period);
    }

    pub fn set_interrupt(&mut self, enable: bool) {
        let mut tc_csr = CSR::new(self.base_addr as *mut u32);
        if enable {
            tc_csr.wo(IER0, TC_IER_CPCS_MSK);
        } else {
            tc_csr.wo(IDR0, TC_IDR_CPCS_MSK);
        }
    }

    pub fn status(&self) -> u32 {
        let tc_csr = CSR::new(self.base_addr as *mut u32);
        tc_csr.r(SR0)
    }

    pub fn counter(&self) -> u32 {
        let tc_csr = CSR::new(self.base_addr as *mut u32);
        tc_csr.r(CV0)
    }
}
