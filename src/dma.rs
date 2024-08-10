// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

use utralib::{utra::xdmac0::*, CSR, HW_XDMAC0_BASE, HW_XDMAC1_BASE};

pub struct Xdmac {
    base_addr: u32,
}

impl Xdmac {
    pub fn xdmac0() -> Xdmac {
        Xdmac {
            base_addr: HW_XDMAC0_BASE as u32,
        }
    }

    pub fn xdmac1() -> Xdmac {
        Xdmac {
            base_addr: HW_XDMAC1_BASE as u32,
        }
    }

    pub fn with_alt_base_addr(addr: usize) -> Xdmac {
        Xdmac {
            base_addr: addr as u32,
        }
    }

    pub fn channel(&self, ch: DmaChannel) -> XdmacChannel {
        XdmacChannel {
            xdmac_base_addr: self.base_addr,
            channel: ch,
        }
    }

    pub fn gis(&self) -> u32 {
        let dma = CSR::new(self.base_addr as *mut u32);
        dma.r(XDMAC_GIS)
    }

    pub fn gim(&self) -> u32 {
        let dma = CSR::new(self.base_addr as *mut u32);
        dma.r(XDMAC_GIM)
    }
}

pub struct XdmacChannel {
    xdmac_base_addr: u32,
    channel: DmaChannel,
}

impl XdmacChannel {
    /// Sets up a peripheral-to-memory DMA transfer.
    pub fn configure_peripheral_transfer(
        &self,
        id: DmaPeripheralId,
        direction: DmaTransferDirection,
        data_width: DmaDataWidth,
        chunk_size: DmaChunkSize,
    ) {
        let mut dma = CSR::new(self.xdmac_base_addr as *mut u32);

        let cc_reg = match self.channel {
            DmaChannel::Channel0 => XDMAC_CC0,
            DmaChannel::Channel1 => XDMAC_CC1,
        };

        let direction_flags = match direction {
            DmaTransferDirection::PeripheralToMemory => {
                dma.ms(XDMAC_CC0_SAM, 0) // Source address constant
                | dma.ms(XDMAC_CC0_DAM, 1) // Destination address auto-increments
                | dma.ms(XDMAC_CC0_DSYNC, 0) // PER2MEM
            }
            DmaTransferDirection::MemoryToPeripheral => {
                dma.ms(XDMAC_CC0_SAM, 1) // Source address auto-increments
                | dma.ms(XDMAC_CC0_DAM, 0) // Destination address constant
                | dma.ms(XDMAC_CC0_DSYNC, 1) // MEM2PER
            }
        };

        let cc: u32 = dma.ms(XDMAC_CC0_TYPE, 1) // Synchronized mode
            | dma.ms(XDMAC_CC0_PERID, id as u32)
            | dma.ms(XDMAC_CC0_PROT, 0) // Secured channel
            | dma.ms(XDMAC_CC0_SWREQ, 0) // Hardware request line
            | dma.ms(XDMAC_CC0_SIF, 1)
            | dma.ms(XDMAC_CC0_DIF, 0)
            | dma.ms(XDMAC_CC0_DWIDTH, data_width as u32)
            | dma.ms(XDMAC_CC0_CSIZE, chunk_size as u32)
            | dma.ms(XDMAC_CC0_MBSIZE, 3) // Memory burst size: 16
            | direction_flags;

        dma.wo(cc_reg, cc);
    }

    /// Starts the DMA transfer.
    ///
    /// The DMA transfer must be configured beforehand by calling one of the following
    /// methods:
    /// - [`XdmacChannel::configure_peripheral_to_memory`]
    /// - Memory-memory: TODO
    pub fn execute_transfer(
        &self,
        src: u32,
        dst: u32,
        data_size: usize,
    ) -> Result<(), &'static str> {
        let mut dma = CSR::new(self.xdmac_base_addr as *mut u32);
        let (cis_reg, sa, da, ublen, en) = match self.channel {
            DmaChannel::Channel0 => (
                XDMAC_CIS0,
                XDMAC_CSA0_SA,
                XDMAC_CDA0_DA,
                XDMAC_CUBC0_UBLEN,
                XDMAC_GE_EN0,
            ),
            DmaChannel::Channel1 => (
                XDMAC_CIS1,
                XDMAC_CSA1_SA,
                XDMAC_CDA1_DA,
                XDMAC_CUBC1_UBLEN,
                XDMAC_GE_EN1,
            ),
        };

        // Clear the channel status
        dma.r(cis_reg);

        // Configure the transfer parameters
        dma.wfo(sa, src);
        dma.wfo(da, dst);
        dma.wfo(ublen, data_size as u32);

        // Make sure all memory transfers are completed before enabling the DMA
        armv7::asm::dmb();

        // Start the transfer
        dma.wfo(en, 1);

        Ok(())
    }

    pub fn interrupt_status(&self) -> u32 {
        let dma = CSR::new(self.xdmac_base_addr as *mut u32);

        match self.channel {
            DmaChannel::Channel0 => dma.r(XDMAC_CIS0),
            DmaChannel::Channel1 => dma.r(XDMAC_CIS1),
        }
    }

    pub fn is_transfer_complete(&self) -> bool {
        let cis = self.interrupt_status();

        // Check if BIS=1
        cis & 1 != 0
    }

    pub fn suspend(&self) {
        let mut dma = CSR::new(self.xdmac_base_addr as *mut u32);
        let ch_bit = self.channel as u32;

        dma.wo(XDMAC_GRWS, 1 << ch_bit);
    }

    pub fn enable(&self) {
        let mut dma = CSR::new(self.xdmac_base_addr as *mut u32);
        let ch_bit = self.channel as u32;

        dma.wo(XDMAC_GE, 1 << ch_bit);
    }

    pub fn disable(&self) {
        let mut dma = CSR::new(self.xdmac_base_addr as *mut u32);
        let ch_bit = self.channel as u32;

        dma.wo(XDMAC_GD, 1 << ch_bit);
    }

    pub fn software_flush(&self) {
        let mut dma = CSR::new(self.xdmac_base_addr as *mut u32);
        let ch_bit = self.channel as u32;

        dma.wo(XDMAC_GSWF, 1 << ch_bit);
    }

    pub fn software_request(&self) {
        let mut dma = CSR::new(self.xdmac_base_addr as *mut u32);
        let ch_bit = self.channel as u32;

        dma.wo(XDMAC_GSWR, 1 << ch_bit);
    }

    pub fn set_interrupt(&self, enable: bool) {
        let mut dma = CSR::new(self.xdmac_base_addr as *mut u32);
        let ch_bit = self.channel as u32;

        if enable {
            dma.wo(XDMAC_GIE, 1 << ch_bit);
        } else {
            dma.wo(XDMAC_GID, 1 << ch_bit);
        }
    }

    pub fn set_bi_interrupt(&self, enable: bool) {
        let mut dma = CSR::new(self.xdmac_base_addr as *mut u32);

        let (cie_reg, cid_reg) = match self.channel {
            DmaChannel::Channel0 => (XDMAC_CIE0, XDMAC_CID0),
            DmaChannel::Channel1 => (XDMAC_CIE1, XDMAC_CID1),
        };

        if enable {
            dma.wo(cie_reg, 1 << 0); // BI
        } else {
            dma.wo(cid_reg, 1 << 0); // BI
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum DmaChannel {
    Channel0 = 0,
    Channel1 = 1,
}

#[derive(Debug, Copy, Clone)]
pub enum DmaDataWidth {
    D8 = 0,
    D16 = 1,
    D32 = 2,
    D64 = 3,
}

#[derive(Debug, Copy, Clone)]
pub enum DmaChunkSize {
    C1 = 0,
    C2 = 1,
    C4 = 2,
    C8 = 3,
    C16 = 4,
}

#[derive(Debug, Copy, Clone)]
pub enum DmaTransferDirection {
    PeripheralToMemory,
    MemoryToPeripheral,
}

#[derive(Debug, Copy, Clone)]
pub enum DmaPeripheralId {
    AesTx = 26,
    AesRx = 27,
    Sha = 30,
    Uart0Tx = 35,
    Uart0Rx = 36,
    Uart1Tx = 37,
    Uart1Rx = 38,
    Uart2Tx = 39,
    Uart2Rx = 40,
    Uart3Tx = 41,
    Uart3Rx = 42,
    Uart4Tx = 43,
    Uart4Rx = 44,
}
