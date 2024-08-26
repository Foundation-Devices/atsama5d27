//! Reset Controller (RTC)

use utralib::{utra::rtc::*, HW_RTC_BASE, *};

pub struct Rtc {
    base_addr: u32,
}

impl Default for Rtc {
    fn default() -> Self {
        Rtc::new()
    }
}

impl Rtc {
    pub fn new() -> Self {
        Self {
            base_addr: HW_RTC_BASE as u32,
        }
    }

    /// Creates RTC instance with a different base address. Used with virtual memory
    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    /// Get seconds elapsed since the Unix epoch (disregarding the existence of leap
    /// seconds) Returns None if the clock is not yet configured.
    pub fn time(&self) -> Option<u32> {
        let rtc_csr = CSR::new(self.base_addr as *mut u32);
        if rtc_csr.rf(MR_UTC) == 0 {
            // Only UTC time is supported, because we don't use date alarms, will only use
            // this mode in all code and use rust or chrono Date for all actual operations,
            // so Gregorian mode would be a lot of relatively complicated dead code.
            return None;
        }
        let result1 = rtc_csr.r(TIMR);
        let result2 = rtc_csr.r(TIMR);
        if result1 == result2 {
            Some(result1)
        } else {
            // We ran into an async update, the third access should yield a stable result.
            Some(rtc_csr.r(TIMR))
        }
    }

    /// Enables the per-second and ACKUPD interrupt. Interrupt handler
    /// is called at 1Hz, or more if time is being set.
    pub fn enable_interrupts(&self) {
        let mut rtc_csr = CSR::new(self.base_addr as *mut u32);
        rtc_csr.wfo(IER_ACKEN, 1);
        rtc_csr.wfo(IER_SECEN, 1);
    }

    /// Set seconds elapsed since the Unix epoch.
    /// Should only be called when the RTC is stopped (ACKUPD is true)
    fn set_time(&self, timestamp: u32) {
        let mut rtc_csr = CSR::new(self.base_addr as *mut u32);
        if rtc_csr.rf(MR_UTC) == 0 {
            rtc_csr.rmwf(MR_UTC, 1);
        }
        rtc_csr.wo(TIMR, timestamp);
    }

    /// The RTC timer has stopped, time and date can be modified.
    fn stopped(&self) -> bool {
        let rtc_csr = CSR::new(self.base_addr as *mut u32);
        rtc_csr.rf(SR_ACKUPD) != 0
    }

    /// Request the stop of the RTC timer. Does not stop right away,
    /// stopped has to be polled, or the ACKUPD interrupt has to be enabled.
    fn request_stop(&self) {
        let mut rtc_csr = CSR::new(self.base_addr as *mut u32);
        let mut cr = rtc_csr.r(CR);
        cr |= rtc_csr.ms(CR_UPDTIM, 1);
        // This shouldn't be necessary according to the docs, but if this bit is not set the upper
        // bits of the TIMR value cannot be set either.
        cr |= rtc_csr.ms(CR_UPDCAL, 1);
        rtc_csr.wo(CR, cr)
    }

    /// Restart the RTC timer after stopping.
    pub fn start(&self) {
        let mut rtc_csr = CSR::new(self.base_addr as *mut u32);
        let mut cr = rtc_csr.r(CR);
        cr = rtc_csr.zf(CR_UPDTIM, cr);
        cr = rtc_csr.zf(CR_UPDCAL, cr);
        rtc_csr.wo(CR, cr)
    }

    /// Clear all event flags set currently
    fn clear_events(&self) {
        let mut rtc_csr = CSR::new(self.base_addr as *mut u32);
        let events = rtc_csr.r(SR);
        rtc_csr.wo(SCCR, events);
    }

    /// Helper function to call from an interrupt handler that also does the set time
    /// logic as described in the SAMA5D2 docs. Returns if time was successfully set.
    pub fn handle_interrupt(&self, set_time: Option<u32>) -> bool {
        let mut set_time_happened = false;
        let stopped = self.stopped();
        self.clear_events();
        if let Some(timestamp) = set_time {
            if !stopped {
                self.request_stop();
                // We should get retriggered soon with the ACKUPD interrupt
            } else {
                self.set_time(timestamp);
                self.start();
                set_time_happened = true;
            }
        };
        set_time_happened
    }
}
