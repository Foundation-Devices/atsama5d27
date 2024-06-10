//! ATSAMA5D2 TWIHS (I2C) driver.

#[cfg(feature = "hal")]
use embedded_hal::blocking::i2c::SevenBitAddress;
use {
    bitflags::bitflags,
    utralib::{utra::twihs0::*, HW_TWIHS0_BASE, HW_TWIHS1_BASE, *},
};

bitflags! {
    #[derive(Debug, Copy, Clone)]
    pub struct TWIStatus: u32 {
        /// Transmission Completed (cleared by writing `TWIHS_THR`)
        const TXCOMP      = 1 << 0;
        /// Receive Holding Register Ready (cleared by reading `TWIHS_RHR`)
        const RXRDY       = 1 << 1;
        /// Transmit Holding Register Ready (cleared by writing `TWIHS_THR`)
        const TXRDY       = 1 << 2;
        /// Client read
        const SVREAD      = 1 << 3;
        /// Client Access
        const SVACC       = 1 << 4;
        /// General Call Access (cleared on read)
        const GACC        = 1 << 5;
        /// Overrun Error (cleared on read). This bit is used only if clock stretching is disabled.
        const OVRE        = 1 << 6;
        /// Underrun Error (cleared on read). This bit is used only if clock stretching is disabled.
        const UNRE        = 1 << 7;
        /// Not Acknowledged (cleared on read)
        const NACK        = 1 << 8;
        /// Arbitration Lost (cleared on read)
        const ARBLST      = 1 << 9;
        /// Clock Wait State (0 - not stretched, 1 - clock is stretched)
        const SCLWS       = 1 << 10;
        /// End Of Client Access (cleared on read)
        const EOSACC      = 1 << 11;
        /// Host Code Acknowledge (cleared on read)
        const MCACK       = 1 << 16;
        /// Timeout Error (cleared on read)
        const TOUT        = 1 << 18;
        /// PEC Error (cleared on read)
        const PECERR      = 1 << 19;
        /// SMBus Default Address Match (cleared on read)
        const SMBDAM      = 1 << 20;
        /// SMBus Host Header Address Match (cleared on read)
        const SMBHHM      = 1 << 21;
        /// Transmit FIFO Lock
        const TXFLOCK     = 1 << 23;
        /// SCL Line Value
        const SCL         = 1 << 24;
        /// SDA Line Value
        const SDA         = 1 << 25;
    }
}

bitflags! {
    #[derive(Debug, Copy, Clone)]
    pub struct FIFOStatus: u32 {
        /// Transmit FIFO Empty Flag (cleared on read)
        const TXFEF      = 1 << 0;
        /// Transmit FIFO Full Flag (cleared on read)
        const TXFFF      = 1 << 1;
        /// Transmit FIFO Threshold Flag (cleared on read)
        const TXFTHF     = 1 << 2;
        /// Receive FIFO Empty Flag
        const RXFEF      = 1 << 3;
        /// Receive FIFO Full Flag
        const RXFFF      = 1 << 4;
        /// Receive FIFO Threshold Flag
        const RXFTHF     = 1 << 5;
        /// Transmit FIFO Pointer Error Flag (see 45.6.6.10 FIFO Pointer Error)
        const TXFPTEF    = 1 << 6;
        /// Receive FIFO Pointer Error Flag (see 45.6.6.10 FIFO Pointer Error)
        const RXFPTEF    = 1 << 7;
    }
}

const TOP_TIMEOUT_CYCLES: usize = 100_000;

#[derive(Debug)]
pub enum I2cError {
    Nack,
    Timeout,
    TxFifoLocked,
}

const TWI_CLK_OFFSET: usize = 3;
const FIFO_SIZE: usize = 16;

pub struct Twi {
    base_addr: u32,
}

impl Twi {
    pub fn with_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    pub fn twi0() -> Self {
        Self::with_base_addr(HW_TWIHS0_BASE as u32)
    }

    pub fn twi1() -> Self {
        Self::with_base_addr(HW_TWIHS1_BASE as u32)
    }

    pub fn init_master(&self, periph_clock_freq: usize, twi_clock_freq: usize) {
        let mut csr = CSR::new(self.base_addr as *mut u32);

        self.sw_reset();
        csr.wo(MMR, 0);
        self.init_clocks(periph_clock_freq, twi_clock_freq);
        self.set_master_mode();
    }

    pub fn status(&self) -> TWIStatus {
        let csr = CSR::new(self.base_addr as *mut u32);
        let bits = csr.r(SR);
        TWIStatus::from_bits_retain(bits)
    }

    pub fn status_masked(&self) -> TWIStatus {
        let csr = CSR::new(self.base_addr as *mut u32);
        let bits = csr.r(SR);
        let mask = csr.r(IMR);
        TWIStatus::from_bits_retain(bits & mask)
    }

    fn sw_reset(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_SWRST, 1);
    }

    fn set_master_mode(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_SVDIS, 1);
        csr.wfo(CR_MSEN, 1);
    }

    fn init_clocks(&self, periph_clock_freq: usize, twi_clock_freq: usize) {
        let mut clh_div = 0_u32;
        let mut ck_div_outer = 0_u32;
        for ck_div in 0..7 {
            clh_div = ((periph_clock_freq / twi_clock_freq) - 2 * TWI_CLK_OFFSET) as u32 >> ck_div;
            if clh_div <= 511 {
                ck_div_outer = ck_div;
                break;
            }
        }

        let hold = ((0.3_f32 * periph_clock_freq as f32) as u32).div_ceil(1_000_000_u32) - 3;

        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(CWGR_CKDIV, ck_div_outer);
        csr.rmwf(CWGR_CHDIV, clh_div >> 1);
        csr.rmwf(CWGR_CLDIV, clh_div >> 1);
        csr.rmwf(CWGR_HOLD, hold);
    }

    fn send_start(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_START, 1);
    }

    fn init_write(&self, addr: u8, internal_addr: u32, internal_addr_size: u8) {
        self.init_op(addr, internal_addr, internal_addr_size, false);
    }

    fn init_read(&self, addr: u8, internal_addr: u32, internal_addr_size: u8) {
        self.init_op(addr, internal_addr, internal_addr_size, true);
    }

    fn init_op(&self, addr: u8, internal_addr: u32, internal_addr_size: u8, is_read: bool) {
        assert_eq!(addr & 0x80, 0);
        assert_eq!(internal_addr & 0xFF000000, 0);
        assert!(internal_addr_size < 4);

        let mut csr = CSR::new(self.base_addr as *mut u32);

        csr.rmwf(MMR_DADR, addr as u32);
        csr.rmwf(MMR_MREAD, is_read as u32);
        csr.rmwf(MMR_IADRSZ, internal_addr_size as u32);
        csr.wfo(IADR_IADR, internal_addr);
    }

    fn read_byte(&self) -> u8 {
        let reg_addr = self.base_addr + 0x30;
        let mut byte;
        unsafe {
            // In order to read a byte, TWI controller requires an explicit byte access operation.
            // Can't use utralib here because it only does `usize` access.
            core::arch::asm!(
                "ldrbt {}, [{}]",
                out(reg) byte,
                in(reg) reg_addr,
            );
        }

        byte
    }

    fn write_byte(&self, byte: u8) {
        let reg_addr = self.base_addr + 0x34;
        unsafe {
            // In order to write a byte, TWI controller requires an explicit byte access operation.
            // Can't use utralib here because it only does `usize` access.
            core::arch::asm!(
                "strbt {}, [{}]",
                in(reg) byte,
                in(reg) reg_addr,
            );
        }
    }

    fn write_word(&self, word: u32) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wo(THR, word)
    }

    fn read_word(&self) -> u32 {
        let csr = CSR::new(self.base_addr as *mut u32);
        csr.r(RHR)
    }

    pub fn write_bytes(&self, address: u8, bytes: &[u8]) -> Result<(), I2cError> {
        self.enable_fifo();
        self.clear_rx_fifo();
        self.clear_tx_fifo();
        self.enable_acm();

        self.init_write(address, 0, 0);

        self.acm_set_datal(bytes.len() as u32);
        self.acm_set_ndatal(0);
        self.acm_set_direction(false);

        self.send_bytes(bytes)?;
        self.wait_for_status(TWIStatus::TXCOMP)?;

        self.disable_fifo();
        self.disable_acm();

        Ok(())
    }

    pub fn read_bytes(&self, address: u8, buffer: &mut [u8]) -> Result<(), I2cError> {
        self.enable_fifo();
        self.clear_rx_fifo();
        self.clear_tx_fifo();
        self.enable_acm();

        self.init_read(address, 0, 0);

        self.acm_set_datal(buffer.len() as u32);
        self.acm_set_ndatal(0);
        self.acm_set_direction(true);

        self.send_start();
        self.receive_bytes(buffer)?;

        self.disable_fifo();
        self.disable_acm();

        Ok(())
    }

    pub fn write_read_bytes(
        &self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), I2cError> {
        self.enable_fifo();
        self.clear_rx_fifo();
        self.clear_tx_fifo();
        self.enable_acm();

        self.init_write(address, 0, 0);

        self.acm_set_datal(bytes.len() as u32);
        self.acm_set_ndatal(buffer.len() as u32);
        self.acm_set_direction(false);
        self.acm_set_next_direction(true);

        self.send_bytes(bytes)?;
        self.send_start();
        self.wait_for_status(TWIStatus::TXCOMP)?;
        self.receive_bytes(buffer)?;

        self.disable_fifo();
        self.disable_acm();

        Ok(())
    }

    fn send_bytes(&self, bytes: &[u8]) -> Result<(), I2cError> {
        let tx_fifo_available = self.tx_fifo_available();
        let fifo_bytes = tx_fifo_available.min(bytes.len());
        let remaining_bytes = bytes.len().saturating_sub(fifo_bytes);

        // Fill the FIFO with bytes depending on how much free space is in FIFO.
        // TWI controller allows us to use faster 32-bit access to push 4 bytes into FIFO in a
        // single access So we split the byte array into 4 byte chunks and use 32-bit
        // (word) access to send these Then use bytewise access to send the rest (if any)
        let mut words = bytes[..fifo_bytes].chunks_exact(4);
        self.set_tx_data_length(4);
        for word in words.by_ref() {
            let bytes = [word[0], word[1], word[2], word[3]];
            self.write_word(u32::from_le_bytes(bytes));
        }
        self.set_tx_data_length(1);
        for byte in words.remainder() {
            self.write_byte(*byte);
        }

        // Send the bytes which didn't fit into FIFO while waiting for the TXRDY flag
        if remaining_bytes > 0 {
            for byte in &bytes[fifo_bytes - 1..] {
                self.wait_for_status(TWIStatus::TXRDY)?;
                self.write_byte(*byte);
            }
        }

        Ok(())
    }

    fn receive_bytes(&self, bytes: &mut [u8]) -> Result<(), I2cError> {
        let mut num_bytes_received = 0;
        while num_bytes_received < bytes.len() {
            // RX level is clipped by the buffer size to avoid overflowing if FIFO suddenly reports
            // receiving more bytes than expected
            let rx_level = usize::min(self.rx_fifo_level(), bytes.len() - num_bytes_received);

            // TWI controller allows us to use faster 32-bit access to pop 4 bytes from FIFO in a
            // single access So we split the byte array into 4 byte chunks and use
            // 32-bit (word) access to read these Then use bytewise access to read the
            // rest (if any)
            let mut words =
                bytes[num_bytes_received..num_bytes_received + rx_level].chunks_exact_mut(4);
            self.set_rx_data_length(4);
            for word in words.by_ref() {
                word.copy_from_slice(&self.read_word().to_le_bytes());
            }
            self.set_rx_data_length(1);
            for byte in words.into_remainder() {
                *byte = self.read_byte();
            }
            num_bytes_received += rx_level;
        }

        Ok(())
    }

    fn wait_for_status(&self, status: TWIStatus) -> Result<(), I2cError> {
        for _ in 0..TOP_TIMEOUT_CYCLES {
            let curr_status = self.status();

            if curr_status.contains(TWIStatus::NACK) {
                return Err(I2cError::Nack);
            }
            if curr_status.contains(TWIStatus::TXFLOCK) {
                self.unlock_tx_fifo();
                return Err(I2cError::TxFifoLocked);
            }
            if curr_status.contains(status) {
                return Ok(());
            }
        }

        Err(I2cError::Timeout)
    }

    fn enable_fifo(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_FIFOEN, 1)
    }

    fn disable_fifo(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_FIFODIS, 1)
    }

    #[allow(dead_code)]
    fn unlock_tx_fifo(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_TXFLCLR, 1)
    }

    #[allow(dead_code)]
    fn clear_tx_fifo(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_TXFCLR, 1)
    }

    #[allow(dead_code)]
    fn clear_rx_fifo(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_RXFCLR, 1)
    }

    fn tx_fifo_level(&self) -> usize {
        let csr = CSR::new(self.base_addr as *mut u32);
        csr.rf(FLR_TXFL) as usize
    }

    fn rx_fifo_level(&self) -> usize {
        let csr = CSR::new(self.base_addr as *mut u32);
        csr.rf(FLR_RXFL) as usize
    }

    fn tx_fifo_available(&self) -> usize {
        FIFO_SIZE - self.tx_fifo_level()
    }

    fn set_tx_data_length(&self, len: u32) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(FMR_TXRDYM, len)
    }

    fn set_rx_data_length(&self, len: u32) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(FMR_RXRDYM, len)
    }

    fn acm_set_datal(&self, datal: u32) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(ACR_DATAL, datal)
    }

    fn acm_set_ndatal(&self, ndatal: u32) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(ACR_NDATAL, ndatal)
    }

    fn acm_set_direction(&self, is_read: bool) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(ACR_DIR, is_read as u32)
    }

    fn acm_set_next_direction(&self, is_read: bool) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(ACR_NDIR, is_read as u32)
    }

    fn enable_acm(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_ACMEN, 1)
    }

    fn disable_acm(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_ACMDIS, 1)
    }

    /// Clones the TWI instance for the use with `embedded-hal` drivers that want to own
    /// I2C bus.
    ///
    /// # Safety
    /// Ensure no two drivers are using the bus at the same time, or else everything will
    /// break.
    pub unsafe fn clone(&self) -> Self {
        Self {
            base_addr: self.base_addr,
        }
    }

    // TODO: DMA
}

#[cfg(feature = "hal")]
impl embedded_hal::blocking::i2c::Write for Twi {
    type Error = I2cError;

    fn write(&mut self, address: SevenBitAddress, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_bytes(address, bytes)
    }
}

#[cfg(feature = "hal")]
impl embedded_hal::blocking::i2c::Read for Twi {
    type Error = I2cError;

    fn read(&mut self, address: SevenBitAddress, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.read_bytes(address, buffer)
    }
}

#[cfg(feature = "hal")]
impl embedded_hal::blocking::i2c::WriteRead for Twi {
    type Error = I2cError;

    fn write_read(
        &mut self,
        address: SevenBitAddress,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.write_read_bytes(address, bytes, buffer)
    }
}
