use {
    crate::dma::{
        self,
        DmaChannel,
        DmaChunkSize,
        DmaDataWidth,
        DmaPeripheralId,
        DmaTransferDirection,
        XdmacChannel,
    },
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

const IDATAR_OFFSET: u32 = 0x40;
const ODATAR_OFFSET: u32 = 0x50;

impl Aes {
    /// Create AES with a different base address. Useful with virtual memory.
    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    /// Initialize the AES peripheral in CBC mode.
    pub fn init(&self, key: [u32; 4], iv: [u32; 4]) {
        self.reset();
        let mut csr = CSR::new(self.base_addr as *mut u32);
        self.set_key(&key);
        self.set_iv(&iv);
        let opmod = csr.ms(MR_OPMOD, 1); // CBC mode
        let smod = csr.ms(MR_SMOD, 2); // DMA auto-start
        let ckey = csr.ms(MR_CKEY, CKEY);
        // TODO Try disabling this?
        let dualbuff = csr.ms(MR_DUALBUFF, 1);
        csr.wo(MR, ckey | opmod | smod | dualbuff);
    }

    /// Encrypt some data using the AES peripheral. Panics if the output buffer is smaller
    /// than the input buffer.
    pub fn encrypt(
        &self,
        input_phys: &[u8],
        output_phys: &mut [u8],
        ch0: &XdmacChannel,
        ch1: &XdmacChannel,
    ) {
        if output_phys.len() < input_phys.len() {
            panic!("Output buffer too small");
        }
        self.set_cipher(1); // Encrypt data
        self.execute(input_phys, output_phys, ch0, ch1);
    }

    /// Decrypt some data using the AES peripheral. Panics if the output buffer is smaller
    /// than the input buffer.
    pub fn decrypt(
        &self,
        input_phys: &[u8],
        output_phys: &mut [u8],
        ch0: &XdmacChannel,
        ch1: &XdmacChannel,
    ) {
        if output_phys.len() < input_phys.len() {
            panic!("Output buffer too small");
        }
        self.set_cipher(0); // Decrypt data
        self.execute(input_phys, output_phys, ch0, ch1);
    }

    fn execute(
        &self,
        input_phys: &[u8],
        output_phys: &mut [u8],
        ch0: &XdmacChannel,
        ch1: &XdmacChannel,
    ) {
        self.set_len(input_phys.len() as _);

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
            input_phys.as_ptr() as *const u8 as _,
            self.base_addr + IDATAR_OFFSET,
            input_phys.len(),
        )
        .expect("dma");
        ch1.execute_transfer(
            self.base_addr + ODATAR_OFFSET,
            output_phys.as_mut_ptr() as *mut u8 as _,
            input_phys.len(),
        )
        .expect("dma");
    }

    fn set_key(&self, key: &[u32; 4]) {
        const AES_KEYWR_OFFSET: usize = 0x20;
        let ivr_base = self.base_addr as usize + AES_KEYWR_OFFSET;

        for (i, key) in key.iter().enumerate() {
            unsafe {
                let ptr = (ivr_base + i * 4) as *mut u32;
                ptr.write_volatile(*key);
            }
        }
    }

    fn set_iv(&self, iv: &[u32; 4]) {
        const AES_IVR_OFFSET: usize = 0x60;
        let ivr_base = self.base_addr as usize + AES_IVR_OFFSET;

        for (i, iv) in iv.iter().enumerate() {
            unsafe {
                let ptr = (ivr_base + i * 4) as *mut u32;
                ptr.write_volatile(*iv);
            }
        }
    }

    // TODO This should be unnecessary
    fn set_len(&self, len: u32) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CLENR_CLEN, len);
        csr.wfo(BCNT_BCNT, len);
    }

    fn set_cipher(&self, cipher: u32) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(MR_CIPHER, cipher);
    }

    fn reset(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_SWRST, 1);
    }
}

const CKEY: u32 = 0xE;

#[derive(Debug, Clone)]
pub enum AesMode {
    Ecb {
        key: [u32; 4],
        cipher: u32,
    },

    Cbc {
        key: [u32; 4],
        iv: [u32; 4],
        cipher: u32,
        length: u32,
    },

    Counter {
        nonce: [u32; 4],
        cipher: u32,
    },
}
