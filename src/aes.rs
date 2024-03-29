use {
    crate::dma,
    utralib::{utra::aes, CSR},
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

impl Aes {
    /// Create AES with a different base address. Useful with virtual memory.
    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    /// Encrypt a buffer using AES-256 in CBC mode without DMA.
    pub fn encrypt_no_dma(
        &self,
        key: [u8; 32],
        iv: [u8; 16],
        data: &mut [u8],
    ) -> Result<(), &'static str> {
        if data.len() % 16 != 0 {
            return Err("Plaintext buffer must be aligned to 16 bytes (AES block size).");
        }

        let mut aes = CSR::new(self.base_addr as *mut u32);

        for (i, chunk) in data.chunks_mut(16).enumerate() {
            // Clear the DATRDY bit by reading ODATAR.
            aes.r(aes::ODATAR0);

            // Write the CKEY. This is necessary to allow the MR register to be programmed.
            aes.wfo(aes::MR_CKEY, 0xE);
            // Use manual mode.
            aes.wfo(aes::MR_SMOD, 0);
            // Use CBC.
            aes.wfo(aes::MR_OPMOD, 1);
            // Use 256-bit key size. TODO Benchmark the difference between 128-bit and
            // 256-bit.
            aes.wfo(aes::MR_KEYSIZE, 2);
            // Encrypt data.
            aes.wfo(aes::MR_CIPHER, 1);

            // Set the key.
            aes.wo(
                aes::KEYWR0,
                u32::from_ne_bytes(key[0..4].try_into().unwrap()),
            );
            aes.wo(
                aes::KEYWR1,
                u32::from_ne_bytes(key[4..8].try_into().unwrap()),
            );
            aes.wo(
                aes::KEYWR2,
                u32::from_ne_bytes(key[8..12].try_into().unwrap()),
            );
            aes.wo(
                aes::KEYWR3,
                u32::from_ne_bytes(key[12..16].try_into().unwrap()),
            );
            aes.wo(
                aes::KEYWR4,
                u32::from_ne_bytes(key[16..20].try_into().unwrap()),
            );
            aes.wo(
                aes::KEYWR5,
                u32::from_ne_bytes(key[20..24].try_into().unwrap()),
            );
            aes.wo(
                aes::KEYWR6,
                u32::from_ne_bytes(key[24..28].try_into().unwrap()),
            );
            aes.wo(
                aes::KEYWR7,
                u32::from_ne_bytes(key[28..32].try_into().unwrap()),
            );

            // Set the IV.
            aes.wo(aes::IVR0, u32::from_ne_bytes(iv[0..4].try_into().unwrap()));
            aes.wo(aes::IVR1, u32::from_ne_bytes(iv[4..8].try_into().unwrap()));
            aes.wo(aes::IVR2, u32::from_ne_bytes(iv[8..12].try_into().unwrap()));
            aes.wo(
                aes::IVR3,
                u32::from_ne_bytes(iv[12..16].try_into().unwrap()),
            );

            // Write the input data.
            aes.wo(
                aes::IDATAR0,
                u32::from_ne_bytes(chunk[0..4].try_into().unwrap()),
            );
            aes.wo(
                aes::IDATAR1,
                u32::from_ne_bytes(chunk[4..8].try_into().unwrap()),
            );
            aes.wo(
                aes::IDATAR2,
                u32::from_ne_bytes(chunk[8..12].try_into().unwrap()),
            );
            aes.wo(
                aes::IDATAR3,
                u32::from_ne_bytes(chunk[12..16].try_into().unwrap()),
            );

            // Start the encryption process.
            aes.wfo(aes::CR_START, 1);

            // Wait until the encryption is done.
            while aes.rf(aes::ISR_DATRDY) == 0 {}

            // Copy the output data.
            let start = i * 16;
            chunk[start + 0..start + 4].copy_from_slice(&aes.r(aes::ODATAR0).to_ne_bytes());
            chunk[start + 4..start + 8].copy_from_slice(&aes.r(aes::ODATAR1).to_ne_bytes());
            chunk[start + 8..start + 12].copy_from_slice(&aes.r(aes::ODATAR2).to_ne_bytes());
            chunk[start + 12..start + 16].copy_from_slice(&aes.r(aes::ODATAR3).to_ne_bytes());
        }

        Ok(())
    }

    /// Decrypt a buffer using AES-256 in CBC mode without DMA.
    pub fn decrypt_no_dma(
        &self,
        key: [u8; 32],
        iv: [u8; 16],
        data: &mut [u8],
    ) -> Result<(), &'static str> {
        if data.len() % 16 != 0 {
            return Err("Ciphertext buffer must be aligned to 16 bytes (AES block size).");
        }

        let mut aes = CSR::new(self.base_addr as *mut u32);

        for (i, chunk) in data.chunks_mut(16).enumerate() {
            // Clear the DATRDY bit by reading ODATAR.
            aes.r(aes::ODATAR0);

            // Write the CKEY. This is necessary to allow the MR register to be programmed.
            aes.wfo(aes::MR_CKEY, 0xE);
            // Use manual mode.
            aes.wfo(aes::MR_SMOD, 0);
            // Use CBC.
            aes.wfo(aes::MR_OPMOD, 1);
            // Use 256-bit key size. TODO Benchmark the difference between 128-bit and
            // 256-bit.
            aes.wfo(aes::MR_KEYSIZE, 2);
            // Decrypt data.
            aes.wfo(aes::MR_CIPHER, 0);

            // Set the key.
            aes.wo(
                aes::KEYWR0,
                u32::from_ne_bytes(key[0..4].try_into().unwrap()),
            );
            aes.wo(
                aes::KEYWR1,
                u32::from_ne_bytes(key[4..8].try_into().unwrap()),
            );
            aes.wo(
                aes::KEYWR2,
                u32::from_ne_bytes(key[8..12].try_into().unwrap()),
            );
            aes.wo(
                aes::KEYWR3,
                u32::from_ne_bytes(key[12..16].try_into().unwrap()),
            );
            aes.wo(
                aes::KEYWR4,
                u32::from_ne_bytes(key[16..20].try_into().unwrap()),
            );
            aes.wo(
                aes::KEYWR5,
                u32::from_ne_bytes(key[20..24].try_into().unwrap()),
            );
            aes.wo(
                aes::KEYWR6,
                u32::from_ne_bytes(key[24..28].try_into().unwrap()),
            );
            aes.wo(
                aes::KEYWR7,
                u32::from_ne_bytes(key[28..32].try_into().unwrap()),
            );

            // Set the IV.
            aes.wo(aes::IVR0, u32::from_ne_bytes(iv[0..4].try_into().unwrap()));
            aes.wo(aes::IVR1, u32::from_ne_bytes(iv[4..8].try_into().unwrap()));
            aes.wo(aes::IVR2, u32::from_ne_bytes(iv[8..12].try_into().unwrap()));
            aes.wo(
                aes::IVR3,
                u32::from_ne_bytes(iv[12..16].try_into().unwrap()),
            );

            // Write the input data.
            aes.wo(
                aes::IDATAR0,
                u32::from_ne_bytes(chunk[0..4].try_into().unwrap()),
            );
            aes.wo(
                aes::IDATAR1,
                u32::from_ne_bytes(chunk[4..8].try_into().unwrap()),
            );
            aes.wo(
                aes::IDATAR2,
                u32::from_ne_bytes(chunk[8..12].try_into().unwrap()),
            );
            aes.wo(
                aes::IDATAR3,
                u32::from_ne_bytes(chunk[12..16].try_into().unwrap()),
            );

            // Start the decryption process.
            aes.wfo(aes::CR_START, 1);

            // Wait until the decryption is done.
            while aes.rf(aes::ISR_DATRDY) == 0 {}

            // Copy the output data.
            let start = i * 16;
            chunk[start + 0..start + 4].copy_from_slice(&aes.r(aes::ODATAR0).to_ne_bytes());
            chunk[start + 4..start + 8].copy_from_slice(&aes.r(aes::ODATAR1).to_ne_bytes());
            chunk[start + 8..start + 12].copy_from_slice(&aes.r(aes::ODATAR2).to_ne_bytes());
            chunk[start + 12..start + 16].copy_from_slice(&aes.r(aes::ODATAR3).to_ne_bytes());
        }

        Ok(())
    }

    // TODO I wasn't able to get this to work (due to the DMA module not working). I
    // will tackle it again later.
    /// Encrypt a buffer using AES-256 in CBC mode with DMA.
    pub fn encrypt<const NP: usize, const NC: usize>(
        &self,
        key: [u8; 32],
        iv: [u8; 16],
        plaintext: &dma::Buffer<NP>,
        ciphertext: &mut dma::Buffer<NC>,
    ) -> Result<(), &'static str> {
        todo!();

        /*
        let mut aes = CSR::new(self.base_addr as *mut u32);

        // Write the CKEY. This is necessary to allow the MR register to be programmed.
        aes.wfo(aes::MR_CKEY, 0xE);
        // Use DMA.
        aes.wfo(aes::MR_SMOD, 2);
        // Use CBC.
        aes.wfo(aes::MR_OPMOD, 1);
        // Use 256-bit key size. TODO Benchmark the difference between 128-bit and
        // 256-bit.
        aes.wfo(aes::MR_KEYSIZE, 2);
        // Encrypt data.
        aes.wfo(aes::MR_CIPHER, 1);
        // Use a double buffer.
        aes.wfo(aes::MR_DUALBUFF, 1);

        // Set the key.
        aes.wo(
            aes::KEYWR0,
            u32::from_ne_bytes(key[0..4].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR1,
            u32::from_ne_bytes(key[4..8].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR2,
            u32::from_ne_bytes(key[8..12].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR3,
            u32::from_ne_bytes(key[12..16].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR4,
            u32::from_ne_bytes(key[16..20].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR5,
            u32::from_ne_bytes(key[20..24].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR6,
            u32::from_ne_bytes(key[24..28].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR7,
            u32::from_ne_bytes(key[28..32].try_into().unwrap()),
        );

        // Set the IV.
        aes.wo(aes::IVR0, u32::from_ne_bytes(iv[0..4].try_into().unwrap()));
        aes.wo(aes::IVR1, u32::from_ne_bytes(iv[4..8].try_into().unwrap()));
        aes.wo(aes::IVR2, u32::from_ne_bytes(iv[8..12].try_into().unwrap()));
        aes.wo(
            aes::IVR3,
            u32::from_ne_bytes(iv[12..16].try_into().unwrap()),
        );

        dma::memory_to_peripheral(
            plaintext,
            dma::Peripheral {
                id: 26,
                chunk_size: 2,
                address: (aes::HW_AES_BASE + aes::IDATAR0.offset() * 4) as u32,
            },
        )?;
        dma::peripheral_to_memory(
            dma::Peripheral {
                id: 27,
                chunk_size: 2,
                address: (aes::HW_AES_BASE + aes::ODATAR0.offset() * 4) as u32,
            },
            ciphertext,
        )?;
        dma::execute_peripheral_transfer();

        Ok(())*/
    }
}
