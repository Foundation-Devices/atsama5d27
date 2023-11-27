//! ATSAMA5D2 TWIHS (I2C) driver.

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

#[derive(Debug)]
pub enum I2cError {
    Nack,
}

const TWI_CLK_OFFSET: usize = 3;

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

    fn send_stop(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_STOP, 1);
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
        let csr = CSR::new(self.base_addr as *mut u32);
        (csr.r(RHR) & 0xff) as u8
    }

    fn write_byte(&self, byte: u8) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wo(THR, byte as u32)
    }

    pub fn write_reg(&self, address: u8, reg: u8, bytes: &[u8]) -> Result<(), I2cError> {
        self.init_write(address, reg as u32, 1);
        self.send_start();

        if bytes.len() == 1 {
            self.write_byte(bytes[0]);
            self.send_stop();
            self.wait_for_status(TWIStatus::TXRDY)?;
        } else {
            self.send_bytes(bytes)?;
            self.send_stop();
        }

        self.wait_for_status(TWIStatus::TXCOMP)?;

        Ok(())
    }

    pub fn read_reg(&self, address: u8, reg: u8, bytes: &mut [u8]) -> Result<(), I2cError> {
        self.init_read(address, reg as u32, 1);
        self.send_start();

        // Initiate STOP at the same time as START if we're only reading a single byte
        if bytes.len() == 1 {
            self.send_stop();
        }
        self.receive_bytes(bytes)?;
        if bytes.len() > 1 {
            self.send_stop();
        }

        Ok(())
    }

    fn send_bytes(&self, bytes: &[u8]) -> Result<(), I2cError> {
        for byte in bytes {
            self.write_byte(*byte);
            self.wait_for_status(TWIStatus::TXRDY)?;
        }

        Ok(())
    }

    fn receive_bytes(&self, bytes: &mut [u8]) -> Result<(), I2cError> {
        for byte in bytes {
            self.wait_for_status(TWIStatus::RXRDY)?;
            *byte = self.read_byte();
            self.wait_for_status(TWIStatus::TXCOMP)?;
        }

        Ok(())
    }

    fn wait_for_status(&self, status: TWIStatus) -> Result<(), I2cError> {
        loop {
            let curr_status = self.status();
            if curr_status.contains(TWIStatus::NACK) {
                return Err(I2cError::Nack);
            }

            if curr_status.contains(status) {
                break;
            }

            armv7::asm::nop();
        }

        Ok(())
    }

    fn enable_fifo(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_FIFOEN, 1)
    }

    fn unlock_tx_fifo(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_TXFLCLR, 1)
    }

    fn clear_tx_fifo(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_TXFCLR, 1)
    }

    fn clear_rx_fifo(&self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CR_RXFCLR, 1)
    }

    // TODO: DMA
}
