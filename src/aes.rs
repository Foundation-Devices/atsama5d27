// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

use {
    crate::dma::{DmaChunkSize, DmaDataWidth, DmaPeripheralId, DmaTransferDirection, XdmacChannel},
    utralib::{utra::aes::*, CSR},
};

/// The AES peripheral.
pub struct Aes {
    base_addr: u32,
}

impl Default for Aes {
    fn default() -> Self {
        Self {
            base_addr: utralib::HW_AES_BASE as u32,
        }
    }
}

const BLOCK_SIZE: usize = 16;

const IDATAR_OFFSET: u32 = 0x40;
const ODATAR_OFFSET: u32 = 0x50;

#[derive(Debug)]
enum OpModeValue {
    /// ECB: Electronic Codebook mode
    Ecb = 0,
    /// CBC: Cipher Block Chaining mode
    Cbc = 1,
    /// OFB: Output Feedback mode
    #[allow(dead_code)]
    Ofb = 2,
    /// CFB: Cipher Feedback mode
    #[allow(dead_code)]
    Cfb = 3,
    /// CTR: Counter mode (16-bit internal counter)
    #[allow(dead_code)]
    Ctr = 4,
    /// GCM: Galois/Counter mode
    #[allow(dead_code)]
    Gcm = 5,
    /// XTS: XEX-based tweaked-codebook mode
    Xts = 6,
}

impl Aes {
    /// Create AES with a different base address. Useful with virtual memory.
    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    /// Initialize the AES peripheral for encryption.
    pub fn init_encrypt(&mut self, mode: AesMode) {
        self.init_inner(mode, true)
    }

    /// Initialize the AES peripheral for decryption.
    pub fn init_decrypt(&mut self, mode: AesMode) {
        self.init_inner(mode, false)
    }

    fn init_inner(&self, mode: AesMode, encrypt: bool) {
        self.reset();

        let mut csr = CSR::new(self.base_addr as *mut u32);
        let (opmod, key_size) = match &mode {
            AesMode::Ecb { key } => (OpModeValue::Ecb, key.key_size()),
            AesMode::Cbc { key, .. } => (OpModeValue::Cbc, key.key_size()),
            AesMode::Counter { .. } => {
                unimplemented!()
            }
            AesMode::Xts { key1, .. } => (OpModeValue::Xts, key1.key_size()),
        };

        csr.rmwf(MR_OPMOD, opmod as u32);
        csr.rmwf(MR_CIPHER, if encrypt { 1 } else { 0 });
        csr.rmwf(MR_KEYSIZE, key_size as u32);

        match mode {
            AesMode::Ecb { key } => {
                self.set_key(&key);
            }
            AesMode::Cbc { key, iv } => {
                self.set_key(&key);
                self.set_iv(&iv);
            }
            AesMode::Counter { nonce: _ } => {
                unimplemented!()
            }
            AesMode::Xts { key1, key2, tweak } => {
                // Temporarily switch to ECB to encrypt the tweak value with key2
                let mut sub_aes = Self {
                    base_addr: self.base_addr,
                };
                sub_aes.init_encrypt(AesMode::Ecb { key: key2 });
                let mut encrypted_tweak = [0u8; 16];
                sub_aes.process(&tweak, &mut encrypted_tweak);

                // Switch back to the XTS mode and select encryption or decryption
                csr.rmwf(MR_OPMOD, OpModeValue::Xts as u32);
                csr.rmwf(MR_CIPHER, if encrypt { 1 } else { 0 });
                csr.rmwf(MR_KEYSIZE, key_size as u32);

                // AES_TWRx must be written with the encrypted Tweak Value
                // with bytes swapped as described in AES Register Endianness.
                encrypted_tweak.reverse();
                self.set_tweak(&encrypted_tweak);

                // Set the alpha primitive corresponding to the first block of the sector
                self.set_alpha(&[1, 0, 0, 0]);

                // Set key1 as the main key
                self.set_key(&key1);
            }
        }
    }

    /// Process the data blocks using the AES peripheral.
    /// Panics if the output buffer is smaller than the input buffer.
    pub fn process_dma(
        &self,
        input_phys: &[u8],
        output_phys: &mut [u8],
        ch0: &XdmacChannel,
        ch1: &XdmacChannel,
    ) {
        if output_phys.len() < input_phys.len() {
            panic!("Output buffer too small");
        }
        self.set_mr_for_dma();
        self.execute(input_phys, output_phys, ch0, ch1);
    }

    pub fn process(&self, input: &[u8], output: &mut [u8]) {
        self.set_auto_start();

        for (in_block, out_block) in input
            .chunks_exact(BLOCK_SIZE)
            .zip(output.chunks_exact_mut(BLOCK_SIZE))
        {
            self.set_input_data(in_block);

            while !self.is_data_ready() {
                // Wait for data ready
            }

            self.read_output_data(out_block);
        }
    }

    fn set_input_data(&self, block: &[u8]) {
        let idatar_base = self.base_addr as usize + IDATAR_OFFSET as usize;

        for (i, word) in block.chunks_exact(4).enumerate() {
            unsafe {
                let ptr = (idatar_base + i * 4) as *mut u32;
                let word = u32::from_le_bytes(word.try_into().unwrap());
                ptr.write_volatile(word);
            }
        }
    }

    fn read_output_data(&self, output: &mut [u8]) {
        let odatar_base = self.base_addr as usize + ODATAR_OFFSET as usize;
        for (i, word) in output.chunks_exact_mut(4).enumerate() {
            unsafe {
                let ptr = (odatar_base + i * 4) as *const u32;
                let word_u32 = ptr.read_volatile();
                word.copy_from_slice(&word_u32.to_le_bytes());
            }
        }
    }

    fn execute(
        &self,
        input_phys: &[u8],
        output_phys: &mut [u8],
        ch0: &XdmacChannel,
        ch1: &XdmacChannel,
    ) {
        ch0.configure_peripheral_transfer(
            DmaPeripheralId::AesTx,
            DmaTransferDirection::MemoryToPeripheral,
            DmaDataWidth::D32,
            DmaChunkSize::C4,
        );
        ch1.configure_peripheral_transfer(
            DmaPeripheralId::AesRx,
            DmaTransferDirection::PeripheralToMemory,
            DmaDataWidth::D32,
            DmaChunkSize::C4,
        );

        ch0.execute_transfer(
            input_phys.as_ptr() as _,
            self.base_addr + IDATAR_OFFSET,
            input_phys.len() / 4,
        )
        .expect("dma");
        ch1.execute_transfer(
            self.base_addr + ODATAR_OFFSET,
            output_phys.as_mut_ptr() as _,
            input_phys.len() / 4,
        )
        .expect("dma");
    }

    fn set_key(&self, key: &Key) {
        const AES_KEYWR_OFFSET: usize = 0x20;
        let keywr_base = self.base_addr as usize + AES_KEYWR_OFFSET;

        for (i, key) in key.iter().enumerate() {
            unsafe {
                let ptr = (keywr_base + i * 4) as *mut u32;
                ptr.write_volatile(*key);
            }
        }
    }

    fn set_iv(&self, iv: &Iv) {
        const AES_IVR_OFFSET: usize = 0x60;
        let ivr_base = self.base_addr as usize + AES_IVR_OFFSET;

        for (i, iv) in iv.0.iter().enumerate() {
            unsafe {
                let ptr = (ivr_base + i * 4) as *mut u32;
                ptr.write_volatile(*iv);
            }
        }
    }

    fn set_alpha(&self, alpha: &[u32; 4]) {
        const AES_ALPHAR_OFFSET: usize = 0xD0;
        let alphar_offset = self.base_addr as usize + AES_ALPHAR_OFFSET;

        for (i, alpha) in alpha.iter().enumerate() {
            unsafe {
                let ptr = (alphar_offset + i * 4) as *mut u32;
                ptr.write_volatile(*alpha);
            }
        }
    }

    fn set_tweak(&self, tweak: &[u8; 16]) {
        const TWR_OFFSET: usize = 0xC0;
        let twr_base = self.base_addr as usize + TWR_OFFSET;

        for (i, word) in tweak.chunks_exact(4).enumerate() {
            unsafe {
                let ptr = (twr_base + i * 4) as *mut u32;
                let word = u32::from_le_bytes(word.try_into().unwrap());
                ptr.write_volatile(word);
            }
        }
    }

    fn set_mr_for_dma(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(MR_SMOD, 2); // DMA auto-start
        csr.rmwf(MR_DUALBUFF, 1); // Dual-buffering to increase performance
    }

    fn set_auto_start(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(MR_SMOD, 1);
    }

    fn is_data_ready(&self) -> bool {
        let csr = CSR::new(self.base_addr as *mut u32);
        csr.rf(ISR_DATRDY) != 0
    }

    fn reset(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_SWRST, 1);
        csr.wfo(MR_CKEY, CKEY);
    }
}

const CKEY: u32 = 0xE;

#[derive(Debug, Clone)]
pub enum AesMode {
    Ecb {
        key: Key,
    },

    Cbc {
        key: Key,
        iv: Iv,
    },

    Counter {
        nonce: [u32; 4],
    },

    Xts {
        key1: Key,
        key2: Key,
        tweak: [u8; 16],
    },
}

#[derive(Debug, Clone)]
pub enum Key {
    Key128([u32; 4]),
    Key192([u32; 6]),
    Key256([u32; 8]),
}

#[derive(Debug, Copy, Clone)]
pub enum KeySize {
    Aes128 = 0,
    Aes192 = 1,
    Aes256 = 2,
}

impl Key {
    pub fn try_from_slice(bytes: &[u8]) -> Option<Self> {
        match bytes.len() {
            16 => Some(Self::from_bytes_128(bytes.try_into().ok()?)),
            24 => Some(Self::from_bytes_192(bytes.try_into().ok()?)),
            32 => Some(Self::from_bytes_256(bytes.try_into().ok()?)),
            _ => None,
        }
    }

    pub fn from_bytes_128(bytes: &[u8; 16]) -> Self {
        let mut key = [0; 4];
        for (i, chunk) in bytes.chunks(4).enumerate() {
            key[i] = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        }
        Key::Key128(key)
    }

    pub fn from_bytes_192(bytes: &[u8; 24]) -> Self {
        let mut key = [0; 6];
        for (i, chunk) in bytes.chunks(4).enumerate() {
            key[i] = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        }
        Key::Key192(key)
    }

    pub fn from_bytes_256(bytes: &[u8; 32]) -> Self {
        let mut key = [0; 8];
        for (i, chunk) in bytes.chunks(4).enumerate() {
            key[i] = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        }
        Key::Key256(key)
    }

    pub fn iter(&self) -> impl Iterator<Item = &u32> {
        match self {
            Key::Key128(key) => key.iter(),
            Key::Key192(key) => key.iter(),
            Key::Key256(key) => key.iter(),
        }
    }

    pub fn key_size(&self) -> KeySize {
        match self {
            Key::Key128(_) => KeySize::Aes128,
            Key::Key192(_) => KeySize::Aes192,
            Key::Key256(_) => KeySize::Aes256,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Iv([u32; 4]);

impl Iv {
    pub fn try_from_slice(bytes: &[u8]) -> Option<Self> {
        if bytes.len() != BLOCK_SIZE {
            return None;
        }

        let mut iv = [0; 4];
        for (i, chunk) in bytes.chunks_exact(4).enumerate() {
            iv[i] = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        }
        Some(Self(iv))
    }
}
