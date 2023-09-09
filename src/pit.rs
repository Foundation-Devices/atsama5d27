use utralib::utra::pit::{MR_PITEN, MR_PITIEN, MR_PIV, PIIR, PIVR};
use utralib::*;

pub struct Pit {
    base_addr: u32,
}

pub const PIV_MAX: u32 = 0xfffff; // PIV is 20 bit wide

impl Default for Pit {
    fn default() -> Pit {
        Self::new()
    }
}

impl Pit {
    pub fn new() -> Self {
        Self {
            base_addr: HW_PIT_BASE as u32,
        }
    }

    /// Creates PIT instance with a different base address. Used with virtual memory
    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    pub fn set_interval(&mut self, interval: u32) {
        let mut pit_csr = CSR::new(self.base_addr as *mut u32);
        pit_csr.rmwf(MR_PIV, interval & PIV_MAX);
    }

    pub fn set_interrupt(&mut self, enabled: bool) {
        let mut pit_csr = CSR::new(self.base_addr as *mut u32);
        pit_csr.rmwf(MR_PITIEN, enabled.into());
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        let mut pit_csr = CSR::new(self.base_addr as *mut u32);
        pit_csr.rmwf(MR_PITEN, enabled.into());
    }

    /// Reads the current timer values and resets the timer.
    pub fn reset(&mut self) -> u32 {
        let pit_csr = CSR::new(self.base_addr as *mut u32);
        pit_csr.r(PIVR)
    }

    /// Reads the current timer values but does not reset it.
    pub fn read(&self) -> u32 {
        let pit_csr = CSR::new(self.base_addr as *mut u32);
        pit_csr.r(PIIR)
    }

    pub fn busy_wait_ms(&mut self, curr_clock_speed: u32, ms: u32) {
        self.reset();
        let base = self.read();
        let delay = ((curr_clock_speed / 1000) * ms) / 16;
        let mut current;

        loop {
            current = self.read();
            current = current.saturating_sub(base);

            if current >= delay {
                break;
            }
        }
    }
}
