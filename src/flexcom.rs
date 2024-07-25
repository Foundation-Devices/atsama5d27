use {
    crate::uart::{Uart, Uart1},
    bitflags::bitflags,
    core::fmt::Write,
    utralib::{utra::flexcom0::*, HW_FLEXCOM2_BASE, *},
};

bitflags! {
    #[derive(Debug, Copy, Clone)]
    pub struct FlexcomStatus: u32 {
        /// Receiver Ready (cleared by reading `FLEX_US_RHR`)
        const RXRDY       = 1 << 0;
        /// Transmitter Ready (cleared by writing `FLEX_US_THR`)
        const TXRDY       = 1 << 1;
        /// Break Received/End of Break
        const RXBRK       = 1 << 2;
        /// Overrun Error
        const OVRE        = 1 << 5;
        /// Framing Error
        const FRAME       = 1 << 6;
        /// Parity Error
        const PARE        = 1 << 7;
        /// Receiver Timeout
        const TIMEOUT     = 1 << 8;
        /// Transmitter Empty
        const TXEMPTY     = 1 << 9;
        /// Max Number of Repetitions Reached
        const ITER        = 1 << 10;
        /// Non Acknowledge Interrupt
        const NACK        = 1 << 13;
        /// Clear to Send Input Change Flag
        const CTSIC       = 1 << 19;
        /// Comparison Status
        const CMP         = 1 << 22;
        /// CTS Input
        const CTS         = 1 << 23;
        /// Manchester Error
        const MANE        = 1 << 24;
    }
}

const TOP_TIMEOUT_CYCLES: usize = 500_000;

#[derive(Debug)]
pub enum UsartMode {
    Normal = 0x00,
    Rs485 = 0x01,
    HwHandshaking = 0x02,
    Iso7816T0 = 0x04,
    Iso7816T1 = 0x06,
    IrDA = 0x08,
    LinMaster = 0x0A,
    LinSlave = 0x0B,
    SpiMaster = 0x0E,
    SpiSlave = 0x0F,
}

#[derive(Debug)]
pub enum OpMode {
    Disabled = 0,
    Usart = 1,
    Spi = 2,
    Twi = 3,
}

#[derive(Debug)]
pub enum ChMode {
    Normal = 0,
    AutoEcho = 1,
    LocalLoopback = 2,
    RemoteLoopback = 3,
}

#[derive(Debug)]
pub enum Parity {
    Even = 0,
    Odd = 1,
    Space = 2,
    Mark = 3,
    No = 4,
    Multidrop = 6,
}

#[derive(Debug)]
pub enum CharLength {
    FiveBit = 0,
    SixBit = 1,
    SevenBit = 2,
    EightBit = 3,
}

#[derive(Debug)]
pub enum ClockSource {
    /// Peripheral clock is selected
    Mck = 0,
    /// Peripheral clock divided (DIV = 8) is selected
    Div = 1,
    /// PMC generic clock is selected. If the SCK pin is driven (CLKO = 1), the CD field
    /// must be greater than 1.
    GClk = 2,
    /// External pin SCK is selected
    Sck = 3,
}

#[derive(Debug)]
pub enum FlexcomError {
    Timeout = 0,
}

const WPKEY: u32 = 0x55_53_41; // "USA" 🦅🇺🇸

pub struct Flexcom {
    base_addr: u32,
}

impl Flexcom {
    pub fn with_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    pub fn flexcom2() -> Self {
        Self::with_base_addr(HW_FLEXCOM2_BASE as u32)
    }

    pub fn init_baud(
        &mut self,
        clock_speed: u32,
        baud_rate: u32,
        mode: UsartMode,
        clock_source: ClockSource,
    ) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(US_MR_USART_MODE, mode as u32);
        csr.rmwf(US_MR_USCLKS, clock_source as u32);

        self.set_baud(clock_speed, baud_rate);
    }

    pub fn set_baud(&mut self, clock_speed: u32, baud_rate: u32) {
        let mut csr = CSR::new(self.base_addr as *mut u32);

        csr.wfo(US_CR_RSTTX, 1);
        csr.wfo(US_CR_RSTRX, 1);
        self.set_tx(false);
        self.set_rx(false);

        csr.rmwf(US_MR_SYNC, 0);
        csr.rmwf(US_MR_OVER, 0);
        csr.rmwf(US_BRGR_CD, clock_speed / (16 * baud_rate));
    }

    pub fn set_tx(&mut self, enable: bool) {
        let mut csr = CSR::new(self.base_addr as *mut u32);

        if enable {
            csr.wfo(US_CR_TXEN, 1);
        } else {
            csr.wfo(US_CR_TXDIS, 1);
        }
    }

    pub fn set_rx(&mut self, enable: bool) {
        let mut csr = CSR::new(self.base_addr as *mut u32);

        if enable {
            csr.wfo(US_CR_RXEN, 1);
        } else {
            csr.wfo(US_CR_RXDIS, 1);
        }
    }

    pub fn set_op_mode(&mut self, mode: OpMode) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(MR_OPMODE, mode as u32);
    }

    pub fn set_parity(&mut self, parity: Parity) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(US_MR_PAR, parity as u32);
    }

    pub fn set_char_length(&mut self, len: CharLength) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(US_MR_CHRL, len as u32);
    }

    pub fn set_ch_mode(&mut self, mode: ChMode) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(US_MR_CHMODE, mode as u32);
    }

    pub fn read_byte(&self) -> Result<u8, FlexcomError> {
        self.wait_for_status(FlexcomStatus::RXRDY)?;

        let reg_addr = self.base_addr + 0x010;
        let mut byte;
        unsafe {
            // In order to read a byte, FLEXCOM requires an explicit byte access operation.
            // Can't use utralib here because it only does `usize` access.
            core::arch::asm!(
                "ldrbt {}, [{}]",
                out(reg) byte,
                in(reg) reg_addr,
            );
        }

        Ok(byte)
    }

    pub fn write_byte(&self, byte: u8) -> Result<(), FlexcomError> {
        self.wait_for_status(FlexcomStatus::TXRDY)?;

        let reg_addr = self.base_addr + 0x020;
        unsafe {
            // In order to write a byte, FLEXCOM requires an explicit byte access operation.
            // Can't use utralib here because it only does `usize` access.
            core::arch::asm!(
                "strbt {}, [{}]",
                in(reg) byte,
                in(reg) reg_addr,
            );
        }

        self.wait_for_status(FlexcomStatus::TXEMPTY)?;

        Ok(())
    }

    pub fn unlock(&mut self) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wo(US_WPMR, WPKEY | 1);
    }

    pub fn is_wp_violated(&self) -> bool {
        let csr = CSR::new(self.base_addr as *mut u32);
        csr.rf(US_WPSR_WPVS) != 0
    }

    fn status(&self) -> FlexcomStatus {
        let csr = CSR::new(self.base_addr as *mut u32);
        let bits = csr.r(US_CSR);
        FlexcomStatus::from_bits_retain(bits)
    }

    fn wait_for_status(&self, status: FlexcomStatus) -> Result<(), FlexcomError> {
        let mut counter = TOP_TIMEOUT_CYCLES;
        while counter > 0 {
            let curr_status = self.status();
            if counter - 1 == 0 {
                writeln!(
                    Uart::<Uart1>::new(),
                    "About to timeout, status: {curr_status:?}"
                )
                .ok();
            }

            if curr_status.contains(FlexcomStatus::TIMEOUT) {
                return Err(FlexcomError::Timeout);
            }

            if curr_status.contains(status) {
                return Ok(());
            }

            counter -= 1;
        }

        Err(FlexcomError::Timeout)
    }
}
