//! SHA hardware accelerator driver.

use {
    crate::dma::{DmaChunkSize, DmaDataWidth, DmaPeripheralId, DmaTransferDirection, XdmacChannel},
    bitflags::bitflags,
    utralib::{utra::sha::*, HW_SHA_BASE, *},
};

const SHA256_EMPTY_HASH: Sha256Hash = Sha256Hash([
    0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14, 0x9a, 0xfb, 0xf4, 0xc8, 0x99, 0x6f, 0xb9, 0x24,
    0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c, 0xa4, 0x95, 0x99, 0x1b, 0x78, 0x52, 0xb8, 0x55,
]);

const IDATAR_OFFSET: u32 = 0x40;
const IODATAR_OFFSET: u32 = 0x80;

pub struct Sha256Hash(pub [u8; 32]);

bitflags! {
    #[derive(Debug, Copy, Clone)]
    pub struct SHAStatus: u32 {
        /// Data Ready (cleared by writing a 1 to bit `SWRST` or `START` in `CR`, or by reading `IODATARx`)
        const DATARDY = 1 << 0;

        /// Input Data Register Write Ready (1 means `IDATAR0` can be written)
        const WRDY    = 1 << 4;

        /// Unspecified Register Access Detection Status (cleared by writing a 1 to `CR.SWRST`)
        const URAD    = 1 << 8;

        /// Check Done Status (cleared by writing `CR.START` or `CR.SWRST` or by reading `IODATARx`)
        const CHECKF  = 1 << 16;
    }
}

#[derive(Debug, Copy, Clone)]
pub enum Algorithm {
    Sha1 = 0,
    Sha256 = 1,
    Sha384 = 2,
    Sha512 = 3,
    Sha224 = 4,
    HmacSha1 = 8,
    HmacSha256 = 9,
    HmacSha384 = 10,
    HmacSha512 = 11,
    HmacSha224 = 12,
}

#[derive(Debug, Copy, Clone)]
pub enum Check {
    /// No check is performed.
    NoCheck = 0,
    /// Check is performed with expected hash stored in internal expected hash value
    /// registers.
    CheckExpectedHashValue = 1,
    /// Check is performed with expected hash provided after the message.
    CheckMessage = 2,
}

#[derive(Debug, Copy, Clone)]
pub enum StartMode {
    /// Manual mode
    Manual = 0,
    /// Auto mode
    Auto = 1,
    /// `IDATAR0` access only mode (mandatory when DMA is used)
    Idatar0 = 2,
}

#[derive(Debug, Copy, Clone)]
pub enum Buffering {
    /// Single buffer: IDATAR cannot be written to while processing
    Single = 0,
    /// Double buffer: IDATAR can be written to while processing
    Double = 1,
}

pub struct Sha {
    base_addr: u32,
}

impl Default for Sha {
    fn default() -> Self {
        Sha::new()
    }
}

impl Sha {
    pub fn new() -> Self {
        Self {
            base_addr: HW_SHA_BASE as u32,
        }
    }

    /// Creates SHA instance with a different base address. Used with virtual memory
    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    pub fn reset(&self) {
        let mut sha_csr = CSR::new(self.base_addr as *mut u32);
        sha_csr.wfo(CR_SWRST, 1);
    }

    pub fn status(&self) -> SHAStatus {
        let sha_csr = CSR::new(self.base_addr as *mut u32);
        SHAStatus::from_bits_truncate(sha_csr.r(ISR))
    }

    pub fn set_mode(&self, algorithm: Algorithm, mode: StartMode, buffering: Buffering) {
        let mut sha_csr = CSR::new(self.base_addr as *mut u32);
        let mr = sha_csr.ms(MR_SMOD, mode as u32)
            | sha_csr.ms(MR_ALGO, algorithm as u32)
            | sha_csr.ms(MR_DUALBUFF, buffering as u32);
        sha_csr.wo(MR, mr);
    }

    pub fn set_message_size(&self, size: u32) {
        let mut sha_csr = CSR::new(self.base_addr as *mut u32);
        sha_csr.wo(MSR, size);
    }

    /// When the hash processing starts from the beginning of a message (without
    /// preprocessed hash part), `BYTCNT` must be written with the same value as
    /// `MSGSIZE`. If a part of the message has been already hashed and the hash does
    /// not start from the beginning, `BYTCNT` must be configured with the number of bytes
    /// remaining to process before the padding section.
    ///
    /// When read, provides the size in bytes of the message remaining to be written
    /// before the automatic padding starts. `BYTCNT` is automatically updated each
    /// time a write occurs in `IDATARx` and `IODATARx`. When `BYTCNT` reaches 0, the
    /// `MSGSIZE` is converted into a bit count and appended at the end of the message
    /// after the padding, as described in the FIPS 180 specification.
    /// To disable automatic padding, the `MSGSIZE` and `BYTCNT` fields must be written to
    /// 0.
    pub fn set_byte_count(&self, count: u32) {
        let mut sha_csr = CSR::new(self.base_addr as *mut u32);
        sha_csr.wo(BCR, count);
    }

    pub fn byte_count(&self) -> u32 {
        let sha_csr = CSR::new(self.base_addr as *mut u32);
        sha_csr.r(BCR)
    }

    pub fn first(&self) {
        let mut sha_csr = CSR::new(self.base_addr as *mut u32);
        sha_csr.wfo(CR_FIRST, 1);
    }

    pub fn start(&self) {
        let mut sha_csr = CSR::new(self.base_addr as *mut u32);
        sha_csr.wfo(CR_START, 1);
    }

    fn write_sha256_block(&self, block: &[u8]) {
        let idatar_base = (self.base_addr + IDATAR_OFFSET) as *mut u32;

        for (i, word) in block.chunks(4).enumerate() {
            let mut word_u32 = [0u8; 4];
            word_u32[..word.len()].copy_from_slice(word);
            let word_u32 = u32::from_le_bytes(word_u32);

            unsafe {
                idatar_base.add(i).write_volatile(word_u32);
            }
        }
    }

    fn read_sha256_result(&self) -> Sha256Hash {
        let iodatar_base = (self.base_addr + IODATAR_OFFSET) as *mut u32;

        let mut hash = [0u8; 32];
        for (i, word) in hash.chunks_exact_mut(4).enumerate() {
            let hash_word = unsafe { iodatar_base.add(i).read_volatile() };

            word.copy_from_slice(&hash_word.to_le_bytes());
        }

        Sha256Hash(hash)
    }

    fn wait_data_ready(&self) {
        while !self.status().contains(SHAStatus::DATARDY) {}
    }

    pub fn sha256_dma(&self, data_phys: &[u32], dma_channel: XdmacChannel) -> Sha256Hash {
        if data_phys.is_empty() {
            return SHA256_EMPTY_HASH;
        }

        self.reset();
        self.set_mode(Algorithm::Sha256, StartMode::Idatar0, Buffering::Double);
        self.set_message_size(data_phys.len() as u32 * 4);
        self.set_byte_count(data_phys.len() as u32 * 4);
        self.first();
        dma_channel.configure_peripheral_transfer(
            DmaPeripheralId::Sha,
            DmaTransferDirection::MemoryToPeripheral,
            DmaDataWidth::D32,
            DmaChunkSize::C16,
        );
        dma_channel
            .execute_transfer(
                data_phys.as_ptr() as *const u8 as u32,
                self.base_addr + IDATAR_OFFSET,
                data_phys.len(),
            )
            .ok();
        self.wait_data_ready();
        self.read_sha256_result()
    }

    pub fn sha256(&self, data: &[u8]) -> Sha256Hash {
        const SHA256_BLOCK_SIZE_BYTES: usize = 64;
        if data.is_empty() {
            return SHA256_EMPTY_HASH;
        }

        self.reset();
        self.set_mode(Algorithm::Sha256, StartMode::Manual, Buffering::Single);
        self.set_message_size(data.len() as u32);
        self.set_byte_count(data.len() as u32);
        self.first();
        for block in data.chunks(SHA256_BLOCK_SIZE_BYTES) {
            self.write_sha256_block(block);
            self.start();
            self.wait_data_ready();
        }
        self.read_sha256_result()
    }
}
