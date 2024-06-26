// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

use utralib::{
    utra::{xdmac0::*},
    CSR,
    HW_XDMAC0_BASE,
    HW_XDMAC1_BASE,
};

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
    pub fn configure_peripheral_to_memory(&self, peripheral: Peripheral) {
        let mut dma = CSR::new(self.xdmac_base_addr as *mut u32);

        let cc_reg = match self.channel {
            DmaChannel::Channel0 => XDMAC_CC0,
            DmaChannel::Channel1 => XDMAC_CC1,
        };

        let cc = dma.ms(XDMAC_CC0_TYPE, 1)
            | dma.ms(XDMAC_CC0_PERID, peripheral.id)
            | dma.ms(XDMAC_CC0_DSYNC, 0) // PER2MEM
            | dma.ms(XDMAC_CC0_PROT, 0)
            | dma.ms(XDMAC_CC0_SWREQ, 0)
            | dma.ms(XDMAC_CC0_DAM, 1)
            | dma.ms(XDMAC_CC0_SAM, 0)
            | dma.ms(XDMAC_CC0_SIF, 1)
            | dma.ms(XDMAC_CC0_DIF, 0)
            | dma.ms(XDMAC_CC0_DWIDTH, 0)
            | dma.ms(XDMAC_CC0_CSIZE, 0)
            | dma.ms(XDMAC_CC0_MBSIZE, 0);

        dma.wo(cc_reg, cc);
    }

    /// Starts the DMA transfer.
    ///
    /// The DMA transfer must be configured beforehand by calling one of the following
    /// methods:
    /// - [`configure_peripheral_to_memory`](configure_peripheral_to_memory)
    /// - TODO
    pub fn execute_peripheral_transfer<const N: usize>(
        &self,
        peripheral: Peripheral,
        dst: &mut Buffer<N>,
        block_size: u32,
    ) -> Result<(), &'static str> {
        if N % 4 != 0 {
            return Err("Destination buffer length is not a multiple of 4 bytes.");
        }

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
        dma.wfo(sa, peripheral.address);
        dma.wfo(da, dst.0.as_ptr() as u32);
        dma.wfo(ublen, block_size);

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

/// Every memory buffer used for DMA must be aligned to 4 bytes.
#[derive(Debug, Clone, Copy)]
#[repr(C)]
#[repr(align(4))]
pub struct Buffer<const N: usize>(pub [u8; N]);

#[derive(Debug, Clone, Copy)]
pub struct Peripheral {
    /// Peripheral ID, referred to as PERID in the datasheet.
    pub id: u32,
    /// The chunk size used by this peripheral.
    pub chunk_size: u32,
    /// The address of the input register for this peripheral.
    pub address: u32,
}
