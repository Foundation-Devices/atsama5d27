use {
    crate::{console::DisplayAndUartConsole, dma},
    core::fmt::Write,
    utralib::{
        utra::{aes, xdmac0},
        CSR,
    },
};

#[derive(Debug, Copy, Clone)]
pub struct Key(pub [u8; 32]);

#[derive(Debug, Copy, Clone)]
pub struct Iv(pub [u8; 16]);

/// Encrypt a buffer using AES-256 in CBC mode without DMA.
pub fn encrypt_no_dma(
    key: Key,
    iv: Iv,
    plaintext: &[u8],
    ciphertext: &mut [u8],
) -> Result<(), &'static str> {
    if plaintext.len() > ciphertext.len() {
        return Err("Not enough space in ciphertext buffer.");
    }

    if plaintext.len() % 16 != 0 {
        return Err("Plaintext buffer must be aligned to 16 bytes (AES block size).");
    }

    if ciphertext.len() % 16 != 0 {
        return Err("Ciphertext buffer must be aligned to 16 bytes (AES block size).");
    }

    let mut aes = CSR::new(utralib::HW_AES_BASE as *mut u32);

    for (i, chunk) in plaintext.chunks(16).enumerate() {
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
            u32::from_ne_bytes(key.0[0..4].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR1,
            u32::from_ne_bytes(key.0[4..8].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR2,
            u32::from_ne_bytes(key.0[8..12].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR3,
            u32::from_ne_bytes(key.0[12..16].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR4,
            u32::from_ne_bytes(key.0[16..20].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR5,
            u32::from_ne_bytes(key.0[20..24].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR6,
            u32::from_ne_bytes(key.0[24..28].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR7,
            u32::from_ne_bytes(key.0[28..32].try_into().unwrap()),
        );

        // Set the IV.
        aes.wo(
            aes::IVR0,
            u32::from_ne_bytes(iv.0[0..4].try_into().unwrap()),
        );
        aes.wo(
            aes::IVR1,
            u32::from_ne_bytes(iv.0[4..8].try_into().unwrap()),
        );
        aes.wo(
            aes::IVR2,
            u32::from_ne_bytes(iv.0[8..12].try_into().unwrap()),
        );
        aes.wo(
            aes::IVR3,
            u32::from_ne_bytes(iv.0[12..16].try_into().unwrap()),
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
        ciphertext[start + 0..start + 4].copy_from_slice(&aes.r(aes::ODATAR0).to_ne_bytes());
        ciphertext[start + 4..start + 8].copy_from_slice(&aes.r(aes::ODATAR1).to_ne_bytes());
        ciphertext[start + 8..start + 12].copy_from_slice(&aes.r(aes::ODATAR2).to_ne_bytes());
        ciphertext[start + 12..start + 16].copy_from_slice(&aes.r(aes::ODATAR3).to_ne_bytes());
    }

    Ok(())
}

// TODO I wasn't able to get this to work. I will tackle it again later.
/// Encrypt a buffer using AES-256 in CBC mode with DMA.
pub fn decrypt_no_dma(
    key: Key,
    iv: Iv,
    ciphertext: &[u8],
    plaintext: &mut [u8],
) -> Result<(), &'static str> {
    if ciphertext.len() > plaintext.len() {
        return Err("Not enough space in plaintext buffer.");
    }

    if ciphertext.len() % 16 != 0 {
        return Err("Ciphertext buffer must be aligned to 16 bytes (AES block size).");
    }

    if plaintext.len() % 16 != 0 {
        return Err("Plaintext buffer must be aligned to 16 bytes (AES block size).");
    }

    let mut aes = CSR::new(utralib::HW_AES_BASE as *mut u32);

    for (i, chunk) in ciphertext.chunks(16).enumerate() {
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
            u32::from_ne_bytes(key.0[0..4].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR1,
            u32::from_ne_bytes(key.0[4..8].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR2,
            u32::from_ne_bytes(key.0[8..12].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR3,
            u32::from_ne_bytes(key.0[12..16].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR4,
            u32::from_ne_bytes(key.0[16..20].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR5,
            u32::from_ne_bytes(key.0[20..24].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR6,
            u32::from_ne_bytes(key.0[24..28].try_into().unwrap()),
        );
        aes.wo(
            aes::KEYWR7,
            u32::from_ne_bytes(key.0[28..32].try_into().unwrap()),
        );

        // Set the IV.
        aes.wo(
            aes::IVR0,
            u32::from_ne_bytes(iv.0[0..4].try_into().unwrap()),
        );
        aes.wo(
            aes::IVR1,
            u32::from_ne_bytes(iv.0[4..8].try_into().unwrap()),
        );
        aes.wo(
            aes::IVR2,
            u32::from_ne_bytes(iv.0[8..12].try_into().unwrap()),
        );
        aes.wo(
            aes::IVR3,
            u32::from_ne_bytes(iv.0[12..16].try_into().unwrap()),
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
        plaintext[start + 0..start + 4].copy_from_slice(&aes.r(aes::ODATAR0).to_ne_bytes());
        plaintext[start + 4..start + 8].copy_from_slice(&aes.r(aes::ODATAR1).to_ne_bytes());
        plaintext[start + 8..start + 12].copy_from_slice(&aes.r(aes::ODATAR2).to_ne_bytes());
        plaintext[start + 12..start + 16].copy_from_slice(&aes.r(aes::ODATAR3).to_ne_bytes());
    }

    Ok(())
}

pub fn encrypt<const NP: usize, const NC: usize>(
    key: Key,
    iv: Iv,
    plaintext: &dma::Buffer<NP>,
    ciphertext: &mut dma::Buffer<NC>,
) -> Result<(), &'static str> {
    let mut aes = CSR::new(utralib::HW_AES_BASE as *mut u32);
    let mut dma = CSR::new(utralib::HW_XDMAC0_BASE as *mut u32);

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
        u32::from_ne_bytes(key.0[0..4].try_into().unwrap()),
    );
    aes.wo(
        aes::KEYWR1,
        u32::from_ne_bytes(key.0[4..8].try_into().unwrap()),
    );
    aes.wo(
        aes::KEYWR2,
        u32::from_ne_bytes(key.0[8..12].try_into().unwrap()),
    );
    aes.wo(
        aes::KEYWR3,
        u32::from_ne_bytes(key.0[12..16].try_into().unwrap()),
    );
    aes.wo(
        aes::KEYWR4,
        u32::from_ne_bytes(key.0[16..20].try_into().unwrap()),
    );
    aes.wo(
        aes::KEYWR5,
        u32::from_ne_bytes(key.0[20..24].try_into().unwrap()),
    );
    aes.wo(
        aes::KEYWR6,
        u32::from_ne_bytes(key.0[24..28].try_into().unwrap()),
    );
    aes.wo(
        aes::KEYWR7,
        u32::from_ne_bytes(key.0[28..32].try_into().unwrap()),
    );

    // Set the IV.
    aes.wo(
        aes::IVR0,
        u32::from_ne_bytes(iv.0[0..4].try_into().unwrap()),
    );
    aes.wo(
        aes::IVR1,
        u32::from_ne_bytes(iv.0[4..8].try_into().unwrap()),
    );
    aes.wo(
        aes::IVR2,
        u32::from_ne_bytes(iv.0[8..12].try_into().unwrap()),
    );
    aes.wo(
        aes::IVR3,
        u32::from_ne_bytes(iv.0[12..16].try_into().unwrap()),
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

    Ok(())
}
