//! Level-2 cache controller (L2CC).

use utralib::utra::l2cc::*;
use utralib::{HW_L2CC_BASE, *};

#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub enum Counter {
    Counter0 = 0,
    Counter1,
}

#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub enum EventCounterKind {
    Co = 1,
    DrHit,
    DrReq,
    DwHit,
    DwReq,
    DwTreq,
    IrHit,
    IrReq,
    Wa,
    IpfAlloc,
    EpfHit,
    EpfAlloc,
    Srrcvd,
    Srconf,
    Epfrcvd,
}

pub struct L2cc {
    base_addr: u32,
}

impl Default for L2cc {
    fn default() -> Self {
        L2cc::new()
    }
}

impl L2cc {
    pub fn new() -> Self {
        Self {
            base_addr: HW_L2CC_BASE as u32,
        }
    }

    /// Creates L2CC instance with a different base address. Used with virtual memory
    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    /// Enables L2CC module.
    ///
    /// *Note*: the L2 cache SRAM must be enabled in `SFR` before enabling `L2CC`.
    pub fn set_enable(&mut self, enable: bool) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);
        l2cc_csr.rmwf(CR_L2CEN, enable as u32);
    }

    pub fn set_inst_prefetch_enable(&mut self, enable: bool) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);
        l2cc_csr.rmwf(ACR_IPEN, enable as u32);
        l2cc_csr.rmwf(PCR_INSPEN, enable as u32);
    }

    pub fn set_data_prefetch_enable(&mut self, enable: bool) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);
        l2cc_csr.rmwf(ACR_DPEN, enable as u32);
        l2cc_csr.rmwf(PCR_DATPEN, enable as u32);
    }

    pub fn set_double_line_fill_enable(&mut self, enable: bool) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);
        l2cc_csr.rmwf(PCR_IDLEN, enable as u32);
        l2cc_csr.rmwf(PCR_DLEN, enable as u32);
    }

    pub fn set_non_secure_lockdown_enable(&mut self, enable: bool) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);
        l2cc_csr.rmwf(ACR_NSLEN, enable as u32);
    }

    pub fn set_non_secure_interrupt_enable(&mut self, enable: bool) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);
        l2cc_csr.rmwf(ACR_NSLEN, enable as u32);
    }

    pub fn set_cache_replacement_enable(&mut self, enable: bool) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);
        l2cc_csr.rmwf(ACR_CRPOL, enable as u32);
    }

    pub fn set_force_write_alloc(&mut self, fwa: u8) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);
        l2cc_csr.rmwf(ACR_FWA, (fwa & 0x3) as u32);
    }

    pub fn set_prefetch_offset(&mut self, offset: u8) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);
        l2cc_csr.rmwf(PCR_OFFSET, (offset & 0b11111) as u32);
    }

    pub fn set_prefetch_drop_enable(&mut self, enable: bool) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);
        l2cc_csr.rmwf(PCR_PDEN, enable as u32);
    }

    pub fn set_standby_mode_enable(&mut self, enable: bool) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);
        l2cc_csr.rmwf(POWCR_STBYEN, enable as u32);
    }

    pub fn set_dyn_clock_gating_enable(&mut self, enable: bool) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);
        l2cc_csr.rmwf(POWCR_DCKGATEN, enable as u32);
    }

    pub fn enable_event_counter(&mut self, counter: Counter, kind: EventCounterKind) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);

        match counter {
            Counter::Counter0 => {
                l2cc_csr.rmwf(ECFGR0_ESRC, kind as u32);
                self.reset_event_count(counter);
            },
            Counter::Counter1 => {
                l2cc_csr.rmwf(ECFGR1_ESRC, kind as u32);
                self.reset_event_count(counter);
            },
        }

        l2cc_csr.rmwf(ECR_EVCEN, 1);
    }

    pub fn disable_event_counter(&mut self, counter: Counter) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);

        match counter {
            Counter::Counter0 => l2cc_csr.rmwf(ECFGR0_ESRC, 0),
            Counter::Counter1 => l2cc_csr.rmwf(ECFGR1_ESRC, 0),
        }

        let all_counters_disabled = l2cc_csr.rf(ECFGR0_ESRC) == 0 && l2cc_csr.rf(ECFGR1_ESRC) == 0;
        if all_counters_disabled {
            l2cc_csr.rmwf(ECR_EVCEN, 0);
        }
    }

    pub fn get_event_count(&self, counter: Counter) -> u32 {
        let l2cc_csr = CSR::new(self.base_addr as *mut u32);
        match counter {
            Counter::Counter0 => l2cc_csr.rf(EVR0_VALUE),
            Counter::Counter1 => l2cc_csr.rf(EVR1_VALUE),
        }
    }

    pub fn reset_event_count(&mut self, counter: Counter) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);
        match counter {
            Counter::Counter0 => l2cc_csr.rmwf(ECR_EVC0RST, 1),
            Counter::Counter1 => l2cc_csr.rmwf(ECR_EVC1RST, 1),
        }
    }

    pub fn cache_sync(&mut self) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);

        while l2cc_csr.rf(CSR_C) != 0 {}
        l2cc_csr.wo(CSR, 1);
        while l2cc_csr.rf(CSR_C) != 0 {}
    }

    pub fn invalidate_all(&mut self) {
        let mut l2cc_csr = CSR::new(self.base_addr as *mut u32);
        l2cc_csr.wo(CIWR, 0xffff);
        while l2cc_csr.r(CIWR) & 0xffff != 0 {}

        self.cache_sync();
    }
}
