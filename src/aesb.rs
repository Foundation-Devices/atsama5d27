// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! AES Bridge (AESB)

use utralib::{utra::aesb::*, HW_AESB_BASE, *};

const CKEY: u32 = 0xE;

pub enum AesMode {
    Ecb {
        key: [u32; 4],
    },

    Cbc {
        key: [u32; 4],
        iv: [u32; 4],
    },

    Counter {
        nonce: [u32; 4],
    }
}

pub struct Aesb {
    base_addr: u32,
}

impl Default for Aesb {
    fn default() -> Self {
        Aesb::new()
    }
}

impl Aesb {
    pub fn new() -> Self {
        Self {
            base_addr: HW_AESB_BASE as u32,
        }
    }

    /// Creates `AESB` instance with a different base address. Used with virtual memory
    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    pub fn init(&self, mode: AesMode, procdly: u32) {
        self.reset();

        let mut csr = CSR::new(self.base_addr as *mut u32);
        let (opmod, aahb) = match mode {
            AesMode::Ecb { key } => {
                self.set_key(&key);
                (csr.ms(MR_OPMOD, 0), csr.ms(MR_AAHB, 0))
            }
            AesMode::Cbc { key, iv } => {
                self.set_key(&key);
                self.set_iv(&iv);
                (csr.ms(MR_OPMOD, 1), csr.ms(MR_AAHB, 0))
            }
            AesMode::Counter { nonce } => {
                self.set_iv(&nonce);
                (csr.ms(MR_OPMOD, 0x4), csr.ms(MR_AAHB, 1))
            }
        };

        let smod = csr.ms(MR_SMOD, 1); // auto-start
        let ckey = csr.ms(MR_CKEY, CKEY);
        let dualbuff = csr.ms(MR_DUALBUFF, 1);
        let procdly = csr.ms(MR_PROCDLY, procdly);
        csr.wo(MR, ckey | opmod | smod | procdly | dualbuff | aahb);
    }

    fn set_key(&self, key: &[u32; 4]) {
        const AESB_KEYWR_OFFSET: usize = 0x20; // TODO: change to utralib when `AESB` registers are fixed in the SVD
        let ivr_base = self.base_addr as usize + AESB_KEYWR_OFFSET;

        for (i, key) in key.iter().enumerate() {
            unsafe {
                let ptr = (ivr_base + i * 4) as *mut u32;
                ptr.write_volatile(*key);
            }
        }
    }

    fn set_iv(&self, iv: &[u32; 4]) {
        const AESB_IVR_OFFSET: usize = 0x60; // TODO: change to utralib when `AESB` registers are fixed in the SVD
        let ivr_base = self.base_addr as usize + AESB_IVR_OFFSET;

        for (i, iv) in iv.iter().enumerate() {
            unsafe {
                let ptr = (ivr_base + i * 4) as *mut u32;
                ptr.write_volatile(*iv);
            }
        }
    }

    /// Performs the software reset of the `AESB`.
    fn reset(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_SWRST, 1);
    }
}
