// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

use {
    bitflags::*,
    utralib::{utra::sdmmc0::*, CSR, HW_SDMMC0_BASE, HW_SDMMC1_BASE},
};

const WAIT_TIMEOUT: usize = 1_000_000;

#[derive(Debug)]
pub struct DmaParams<'a, const DMA_DESC_TABLE_SIZE: usize> {
    pub dma_desc_table_phys_addr: u32,
    pub dma_desc_table: &'a mut [ADMADesc; DMA_DESC_TABLE_SIZE],
}

pub enum DmaSel {
    SDma = 0x0,
    ADma32 = 0x2,
}

#[derive(Debug)]
pub enum RespType {
    /// No Response
    NoResp = 0,
    /// Response Length 136
    Rl136 = 1,
    /// Response Length 48
    Rl48 = 2,
    /// Response Length 48 with Busy
    Rl48Busy = 3,
}

#[derive(Debug)]
pub enum SdMmcError {
    InhibitWaitTimeout,
    TooManyBlocksRequested,
    NormalStatusWaitTimeout,
    Error(ErrorStatus),
}

#[derive(Debug, Copy, Clone)]
pub enum SDRespType {
    NoResp = 0x00,
    R1 = 0x10,
    R1B = 0x11,
    R2 = 0x20,
    R3 = 0x30,
    R4 = 0x40,
    R5 = 0x50,
    R6 = 0x60,
    R7 = 0x70,
}

bitflags! {
    #[derive(Debug, Copy, Clone)]
    pub struct DMADescAttr: u16 {
        const VALID = 1 << 0;
        const END   = 1 << 1;
        const INT   = 1 << 2;
        const ACT1  = 1 << 4;
        const ACT2  = 1 << 5;
        const TRAN  = Self::ACT2.bits();
        const LINK  = Self::ACT1.bits() | Self::ACT1.bits();
    }
}

#[derive(Debug, Copy, Clone)]
#[repr(C, align(4))]
pub struct ADMADesc {
    attr: DMADescAttr,
    len: u16,
    addr: u32,
}

impl ADMADesc {
    pub const fn new() -> Self {
        Self {
            attr: DMADescAttr::empty(),
            len: 0,
            addr: 0,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum DataDirection {
    Read = 0x11,
    Write = 0x22,
}

#[derive(Debug, Copy, Clone)]
pub struct SDData {
    pub buf_phys_addr: u32,
    pub direction: DataDirection,
    pub blocks: usize,
    pub block_size: u16,
}

#[derive(Debug, Copy, Clone)]
pub enum SDCmdInner {
    GoIdleState = 0,
    AllSendCid = 2,
    SendRelativeAddr = 3,
    SwitchFun = 6,
    SelectCard = 7,
    SendIfCond = 8,
    SendCsd = 9,
    SendCid = 10,
    StopTransmission = 12,
    SendStatus = 13,
    SetBlockLen = 16,
    ReadSingleBlock = 17,
    ReadMultipleBlocks = 18,
    SetBlockCount = 23,
    WriteSingleBlock = 24,
    WriteMultipleBlocks = 25,

    AppCmd = 55,
}

#[derive(Debug, Copy, Clone)]
pub enum SdAppCommand {
    AppSetBusWidth = 6,
    AppSdStatus = 13,
    AppSdSendOpCond = 41,
    AppSendScr = 51,
}

#[derive(Debug, Copy, Clone)]
pub enum MmcCommand {
    SendOpCond = 1,
    SendExtCsd = 8,
    BusTestR = 14,
    BusTestW = 19,
}

#[derive(Debug, Copy, Clone)]
pub enum SDCmd {
    Sd(SDCmdInner),
    SdApp(SdAppCommand),
    Mmc(MmcCommand),
}

impl From<SDCmd> for u32 {
    fn from(value: SDCmd) -> Self {
        match value {
            SDCmd::Sd(cmd) => cmd as u32,
            SDCmd::SdApp(cmd) => cmd as u32,
            SDCmd::Mmc(cmd) => cmd as u32,
        }
    }
}

#[derive(Debug, Copy, Clone)]
enum AutoCmd {
    Cmd12 = 1,
    #[allow(dead_code)]
    Cmd23 = 2,
}

pub type SdResponse = [u32; 4];

#[derive(Debug, Copy, Clone)]
pub struct SDCommand {
    cmd: SDCmd,
    resp_type: SDRespType,
    argu: u32,
}

impl SDCommand {
    pub fn new(cmd: SDCmd, resp_type: SDRespType, argu: u32) -> Self {
        Self {
            cmd,
            resp_type,
            argu,
        }
    }
}

bitflags! {
    #[derive(Debug, Copy, Clone)]
    pub struct SdmmcStatus: u32 {
        /// CMD Line level.
        /// This status is used to check the CMD line level to recover from errors, and for debugging.
        const CMDLL = 1 << 24;
        /// DAT3 pin level.
        const DAT3  = 1 << 23;
        /// DAT2 pin level.
        const DAT2  = 1 << 22;
        /// DAT1 pin level.
        const DAT1  = 1 << 21;
        /// DAT0 pin level.
        const DAT0  = 1 << 20;
        /// Write Protect pin level.
        const WRPPL = 1 << 19;
        /// Card Detect pin level (inverse).
        const CARDDPL = 1 << 18;
        /// Card State Stable.
        const CARDSS = 1 << 17;
        /// Card Inserted.
        const CARDINS = 1 << 16;
        /// Buffer Read Enable.
        const BUFRDEN = 1 << 11;
        /// Buffer Write Enable.
        const BUFWREN = 1 << 10;
        /// Read Transfer Active.
        const RTACT = 1 << 9;
        /// Write Transfer Active.
        const WTACT = 1 << 8;
        /// DAT Line Active.
        const DLACT = 1 << 2;
        /// Command Inhibit (DAT).
        const CMDINHD = 1 << 1;
        /// Command Inhibit (CMD).
        const CMDINHC = 1 << 0;
    }
}

bitflags! {
    #[derive(Debug, Copy, Clone)]
    pub struct NormalStatus: u16 {
        const CMDC   = 0x1 << 0;  /* Command Complete */
        const TRFC   = 0x1 << 1;  /* Transfer Complete */
        const BLKGE  = 0x1 << 2;  /* Block Gap Event */
        const DMAINT = 0x1 << 3;  /* DMA Interrupt */
        const BWRRDY = 0x1 << 4;  /* Buffer Write Ready */
        const BRDRDY = 0x1 << 5;  /* Buffer Read Ready */
        const CINS   = 0x1 << 6;  /* Card Insertion */
        const CREM   = 0x1 << 7;  /* Card Removal */
        const CINT   = 0x1 << 8;  /* Card Interrupt */
        const BOOTAR = 0x1 << 14;  /* Boot Acknowledge Received */
        const ERRINT = 0x1 << 15;  /* Error Interrupt */
    }
}

bitflags! {
    #[derive(Debug, Copy, Clone)]
    pub struct ErrorStatus: u16 {
        const CMDTEO = 0x1 << 0;  /* Command Timeout Error */
        const CMDCRC = 0x1 << 1;  /* Command CRC Error */
        const CMDEND = 0x1 << 2;  /* Command End Bit Error */
        const CMDIDX = 0x1 << 3;  /* Command Index Error */
        const DATTEO = 0x1 << 4;  /* Data Timeout Error */
        const DATCRC = 0x1 << 5;  /* Data CRC Error */
        const DATEND = 0x1 << 6;  /* Data End Bit Error */
        const CURLIM = 0x1 << 7;  /* Current Limit Error */
        const ACMD   = 0x1 << 8;  /* Auto CMD Error */
        const ADMA   = 0x1 << 9;  /* ADMA Error */
        const BOOTAE = 0x1 << 12; /* Boot Acknowledge Error */
    }
}

pub struct Sdmmc {
    base_addr: u32,
}

impl Sdmmc {
    pub fn sdmmc0() -> Self {
        Self {
            base_addr: HW_SDMMC0_BASE as u32,
        }
    }

    pub fn sdmmc1() -> Self {
        Self {
            base_addr: HW_SDMMC1_BASE as u32,
        }
    }

    pub fn status(&self) -> SdmmcStatus {
        let csr = CSR::new(self.base_addr as *mut u32);
        SdmmcStatus::from_bits_truncate(csr.r(PSR))
    }

    pub fn normal_status(&self) -> NormalStatus {
        NormalStatus::from_bits_truncate(self.read_u16_reg(OFFSET_NISTR))
    }

    pub fn error_status(&self) -> ErrorStatus {
        ErrorStatus::from_bits_truncate(self.read_u16_reg(OFFSET_EISTR))
    }

    /// Sends the command to the card then waits for the response.
    /// In case of block(s) read/write, uses ADMA so the application code must wait for
    /// the `TRFC` (transfer complete) interrupt in order to get the valid data or
    /// proceed to the next command.
    pub fn send_async_command<const N: usize>(
        &self,
        dma_params: &mut DmaParams<N>,
        cmd: SDCommand,
        data: Option<SDData>,
        cache_maintenance: impl Fn(),
    ) -> Result<SdResponse, SdMmcError> {
        self.inhibit_wait()?;

        let needs_auto_cmd12 = matches!(
            cmd.cmd,
            SDCmd::Sd(SDCmdInner::ReadMultipleBlocks | SDCmdInner::WriteMultipleBlocks)
        );

        let use_dma = matches!(
            cmd.cmd,
            SDCmd::Sd(
                SDCmdInner::ReadSingleBlock
                    | SDCmdInner::ReadMultipleBlocks
                    | SDCmdInner::WriteSingleBlock
                    | SDCmdInner::WriteMultipleBlocks
            )
        );

        let mut csr = CSR::new(self.base_addr as *mut u32);

        let mut cmd_reg = (u32::from(cmd.cmd) << 8) as u16;
        let mut normal_status_mask = NormalStatus::CMDC;

        match cmd.resp_type {
            SDRespType::R1 | SDRespType::R5 | SDRespType::R6 | SDRespType::R7 => {
                cmd_reg |= RespType::Rl48 as u16;
                cmd_reg |= 1 << 3; // CR_CMDCCEN
                cmd_reg |= 1 << 4; // CR_CMDICEN
            }

            SDRespType::R1B => {
                cmd_reg |= RespType::Rl48Busy as u16;
                cmd_reg |= 1 << 3; // CR_CMDCCEN
                cmd_reg |= 1 << 4; // CR_CMDICEN

                normal_status_mask |= NormalStatus::TRFC;
            }

            SDRespType::R2 => {
                cmd_reg |= RespType::Rl136 as u16;
                cmd_reg |= 1 << 4; // CR_CMDICEN
            }

            SDRespType::R3 | SDRespType::R4 => cmd_reg |= RespType::Rl48 as u16,

            _ => cmd_reg |= RespType::NoResp as u16,
        }

        if let Some(data) = data {
            // Can use ADMA for these commands
            if use_dma {
                self.write_byte_reg(
                    OFFSET_HC1R,
                    self.read_byte_reg(OFFSET_HC1R) | (DmaSel::ADma32 as u8) << OFFSET_H1CR_DMASEL,
                );
            }

            cmd_reg |= 1 << 5; // CR_DPSEL

            let mut tmr = 1 << 1; // TMR_BCEN
            if data.blocks > 1 {
                tmr |= 1 << 5; // TMR_MSBSEL
            }
            if let DataDirection::Read = data.direction {
                tmr |= 1 << 4; // TMR_DTDSEL_READ
            }
            // Enable DMA for these commands
            if use_dma {
                tmr |= 1; // TMR_DMAEN
            }
            if needs_auto_cmd12 {
                tmr |= (AutoCmd::Cmd12 as u16) << 2; // Auto CMD12
            }

            self.write_byte_reg(OFFSET_TCR, 0xe);
            self.write_u16_reg(OFFSET_BSR, data.block_size);
            if data.blocks > 1 {
                self.write_u16_reg(OFFSET_BCR, data.blocks as u16);
            }

            self.write_u16_reg(OFFSET_TMR, tmr);

            // Configure DMA descriptors
            if use_dma {
                if data.blocks > N {
                    return Err(SdMmcError::TooManyBlocksRequested);
                }

                for (i, dma_desc) in (0..data.blocks).zip(dma_params.dma_desc_table.iter_mut()) {
                    // Last descriptor must have the end bit
                    if i == data.blocks - 1 {
                        dma_desc.attr = DMADescAttr::TRAN | DMADescAttr::VALID | DMADescAttr::END;
                    // 0x23
                    } else {
                        dma_desc.attr = DMADescAttr::TRAN | DMADescAttr::VALID; // 0x21
                    }

                    dma_desc.len = data.block_size;
                    dma_desc.addr = data.buf_phys_addr + data.block_size as u32 * i as u32;
                }

                cache_maintenance(); // Clean caches so the new table is visible to the system
                csr.wo(ASAR0, dma_params.dma_desc_table_phys_addr);
            }
        }

        csr.wo(ARG1R, cmd.argu);
        self.write_u16_reg(OFFSET_CR, cmd_reg); // Send the command

        if !use_dma {
            // Wait for command to finish
            self.wait_normal_status(NormalStatus::CMDC)?;
        }

        let last_status = self.normal_status();
        if last_status.contains(NormalStatus::ERRINT) {
            return Err(SdMmcError::Error(self.error_status()));
        }

        // Clear the status, except for read and write ready.
        // Those will be cleared by the read/write data routine, which
        // bases itself on the fact that the hardware is ready to receive data
        // or has data ready to be read
        self.write_u16_reg(
            OFFSET_NISTR,
            last_status.bits() & !(NormalStatus::BWRRDY | NormalStatus::BRDRDY).bits(),
        );

        // Read the command response
        let mut resp = [0; 4];
        if let SDRespType::R2 = cmd.resp_type {
            for (i, resp) in resp.iter_mut().enumerate() {
                *resp = self.read_u32_reg(OFFSET_RR0 + 4 * i as u32);
            }
        } else {
            resp[0] = self.read_u32_reg(OFFSET_RR0);
        }

        if !use_dma {
            // If we have data but not using block transfer, we use PIO mode
            todo!()
        }

        Ok(resp)
    }

    pub fn enable_interrupt_signal(&mut self, interrupts: NormalStatus) {
        self.write_u16_reg(OFFSET_NISIER, interrupts.bits());
    }

    pub fn enable_error_interrupt_signal(&mut self, interrupts: ErrorStatus) {
        self.write_u16_reg(OFFSET_EISIER, interrupts.bits());
    }

    pub fn acknowledge_interrupt(&mut self, interrupts: NormalStatus) {
        self.write_u16_reg(OFFSET_NISTR, interrupts.bits());
    }

    pub fn acknowledge_error_interrupt(&mut self, interrupts: ErrorStatus) {
        self.write_u16_reg(OFFSET_EISTR, interrupts.bits());
    }

    fn inhibit_wait(&self) -> Result<SdmmcStatus, SdMmcError> {
        let mut timeout = WAIT_TIMEOUT;

        while timeout > 0 {
            let status = self.status();
            if !status.contains(SdmmcStatus::CMDINHC) && !status.contains(SdmmcStatus::CMDINHD) {
                return Ok(status);
            }

            timeout -= 1;
        }

        Err(SdMmcError::InhibitWaitTimeout)
    }

    fn wait_normal_status(&self, ns: NormalStatus) -> Result<NormalStatus, SdMmcError> {
        let mut timeout = WAIT_TIMEOUT;

        while timeout > 0 {
            let status = self.normal_status();
            if status.contains(ns) {
                return Ok(ns);
            }

            if status.contains(NormalStatus::ERRINT) {
                return Err(SdMmcError::Error(self.error_status()));
            }

            if timeout - 1 == 0 {
                use core::fmt::Write;
                core::writeln!(crate::uart::Uart::<crate::uart::Uart1>::new(), "about to timeout, status: {:?} | expected {:?}", status, ns).ok();
                core::writeln!(crate::uart::Uart::<crate::uart::Uart1>::new(), "state: {:?}", self.status()).ok();
            }

            timeout -= 1;
        }

        Err(SdMmcError::NormalStatusWaitTimeout)
    }

    fn write_u16_reg(&self, offset: u32, data: u16) {
        let reg_addr = self.base_addr + offset;
        unsafe {
            core::arch::asm!(
                "strh {}, [{}]",
                in(reg) data,
                in(reg) reg_addr,
            );
        }
    }

    fn write_byte_reg(&self, offset: u32, byte: u8) {
        let reg_addr = self.base_addr + offset;
        unsafe {
            core::arch::asm!(
                "strb {}, [{}]",
                in(reg) byte,
                in(reg) reg_addr,
            );
        }
    }

    fn read_byte_reg(&self, offset: u32) -> u8 {
        let reg_addr = self.base_addr + offset;
        let mut byte;
        unsafe {
            core::arch::asm!(
                "ldrb {}, [{}]",
                out(reg) byte,
                in(reg) reg_addr,
            );
        }

        byte
    }

    fn read_u16_reg(&self, offset: u32) -> u16 {
        let reg_addr = self.base_addr + offset;
        let mut data;
        unsafe {
            core::arch::asm!(
                "ldrh {}, [{}]",
                out(reg) data,
                in(reg) reg_addr,
            );
        }

        data
    }

    fn read_u32_reg(&self, offset: u32) -> u32 {
        let reg_addr = self.base_addr + offset;
        let mut data;
        unsafe {
            core::arch::asm!(
                "ldr {}, [{}]",
                out(reg) data,
                in(reg) reg_addr,
            );
        }

        data
    }
}

const OFFSET_HC1R: u32 = 0x28;
const OFFSET_H1CR_DMASEL: u8 = 3;
const OFFSET_CR: u32 = 0x0E;
const OFFSET_TMR: u32 = 0x0C;
const OFFSET_BSR: u32 = 0x04;
const OFFSET_BCR: u32 = 0x06;
const OFFSET_TCR: u32 = 0x2e;
const OFFSET_NISTR: u32 = 0x30;
const OFFSET_NISIER: u32 = 0x38;
const OFFSET_EISTR: u32 = 0x32;
const OFFSET_EISIER: u32 = 0x3A;
const OFFSET_RR0: u32 = 0x10;
