// SPDX-FileCopyrightText: 2024-2025 Foundation Devices, Inc.
// <hello@foundationdevices.com> SPDX-License-Identifier: GPL-3.0-or-later

/// Secure Fuse Controller (`SFC`) driver routines.
use bitflags::bitflags;
use utralib::{utra::sfc::KR, CSR, HW_SFC_BASE};

const NUM_FUSE_REGS: usize = 16;
const SFC_KR_KEY: u32 = 0xFB;

bitflags! {
    #[derive(Debug, Copy, Clone)]
    pub struct SfcStatus: u32 {
        /// Manufacturer Area Check Error (cleared on read)
        /// At least one check error in the reserved area since the last read.
        const ACE = 1 << 17;

        /// Live Integrity Checking Error (cleared on read)
        /// At least one live integrity check error since the last read.
        const LCHECK = 1 << 4;

        /// Programming Sequence Failed (cleared on read)
        /// A programming failure occurred since the last read.
        const PGMF = 1 << 1;

        /// PGMC Programming Sequence Completed (cleared on read)
        /// At least one programming sequence completion since the last read.
        const PGMC = 1 << 0;
    }
}

pub struct Sfc {
    base_addr: u32,
}

impl Sfc {
    pub fn new() -> Self {
        Self {
            base_addr: HW_SFC_BASE as u32,
        }
    }

    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    pub fn status(&self) -> SfcStatus {
        let csr = CSR::new(self.base_addr as *mut u32);
        SfcStatus::from_bits_truncate(csr.r(utralib::utra::sfc::SR))
    }

    /// Reads a fuse register with index `fuse_reg` (0..=16) and returns its value.
    /// Returns `None` if `fuse_reg` is out of bounds.
    pub fn read(&self, fuse_reg: usize) -> Option<u32> {
        if fuse_reg > NUM_FUSE_REGS {
            return None;
        }

        unsafe {
            let reg_ptr = self.data_reg(fuse_reg);
            Some(reg_ptr.read_volatile())
        }
    }

    /// Burns `value` into the fuse register with index `fuse_reg`.
    pub fn write(&self, fuse_reg: usize, value: u32) -> Result<(), SfcStatus> {
        if fuse_reg > NUM_FUSE_REGS {
            return Err(SfcStatus::empty());
        }

        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wo(KR, SFC_KR_KEY);

        unsafe {
            self.data_reg(fuse_reg).write_volatile(value);
        }

        loop {
            let status = self.status();
            if status.contains(SfcStatus::PGMF) {
                return Err(status);
            }
            if status.contains(SfcStatus::PGMC) {
                break;
            }
        }

        Ok(())
    }

    unsafe fn data_reg(&self, fuse_reg: usize) -> *mut u32 {
        assert!(fuse_reg <= NUM_FUSE_REGS);

        const DR_OFFSET: usize = 0x20;
        ((self.base_addr as usize + DR_OFFSET) as *mut u32).add(fuse_reg)
    }
}

impl Default for Sfc {
    fn default() -> Self {
        Sfc::new()
    }
}
