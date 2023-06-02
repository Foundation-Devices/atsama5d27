use utralib::{utra::xdmac0, CSR};

// TODO I was not able to get the DMA functions to work. I'm going to leave them
// as-is for now and tackle them again later.

/// Every memory buffer used for DMA must be aligned to 4 bytes.
#[derive(Debug, Clone, Copy)]
#[repr(C)]
#[repr(align(4))]
pub struct Buffer<const N: usize>(pub [u8; N]);

/// Execute a memory-to-memory DMA transfer using channel 0.
pub fn memory_to_memory<const NSRC: usize, const NDST: usize>(
    src: &Buffer<NSRC>,
    dst: &mut Buffer<NDST>,
) -> Result<(), &'static str> {
    let mut dma = CSR::new(utralib::HW_XDMAC0_BASE as *mut u32);

    if NDST < NSRC {
        return Err("Destination buffer is too small.");
    }

    if NSRC % 4 != 0 {
        return Err("Source buffer length is not a multiple of 4 bytes.");
    }

    // Clear the pending Interrupt Status bits by reading the selected XDMAC
    // Channel x Interrupt Status Register.
    dma.r(xdmac0::XDMAC_CIS0);

    // Write the XDMAC Source Address Register.
    dma.wo(xdmac0::XDMAC_CSA0, src.0.as_ptr() as u32);

    // Write the XDMAC Destination Address Register.
    dma.wo(xdmac0::XDMAC_CDA0, dst.0.as_ptr() as u32);

    // Set the UBLEN to the size of data.
    dma.wfo(xdmac0::XDMAC_CUBC0_UBLEN, NSRC as u32);

    // Clear the TYPE bit for a memory-to-memory transfer.
    dma.wfo(xdmac0::XDMAC_CC0_TYPE, 0);

    // Set the memory burst size to one byte.
    dma.wfo(xdmac0::XDMAC_CC0_MBSIZE, 0);

    // Set the source memory addressing mode to increment.
    dma.wfo(xdmac0::XDMAC_CC0_SAM, 1);

    // Set the destination memory addressing to increment.
    dma.wfo(xdmac0::XDMAC_CC0_DAM, 1);

    // Use a secured channel.
    dma.wfo(xdmac0::XDMAC_CC0_PROT, 0);

    // Data width is 1 byte.
    dma.wfo(xdmac0::XDMAC_CC0_DWIDTH, 0);

    // Set the peripheral ID for a memory-to-memory transfer.
    dma.wfo(xdmac0::XDMAC_CC0_PERID, 127);

    // Enable the DMA channel.
    dma.wfo(xdmac0::XDMAC_GE_EN0, 1);

    // Wait until the transfer is done.
    while dma.rf(xdmac0::XDMAC_GS_ST0) == 1 {}

    Ok(())
}

#[derive(Debug, Clone, Copy)]
pub struct Peripheral {
    /// Peripheral ID, referred to as PERID in the datasheet.
    pub id: u32,
    /// The chunk size used by this peripheral.
    pub chunk_size: u32,
    /// The address of the input register for this peripheral.
    pub address: u32,
}

/// Execute a memory-to-peripheral DMA transfer using channel 0.
pub fn memory_to_peripheral<const N: usize>(
    src: &Buffer<N>,
    peripheral: Peripheral,
) -> Result<(), &'static str> {
    let mut dma = CSR::new(utralib::HW_XDMAC0_BASE as *mut u32);

    if N % 4 != 0 {
        return Err("Source buffer length is not a multiple of 4 bytes.");
    }

    // Clear the pending Interrupt Status bits by reading the selected XDMAC
    // Channel x Interrupt Status Register.
    dma.r(xdmac0::XDMAC_CIS0);

    // Write the XDMAC Source Address Register.
    dma.wo(xdmac0::XDMAC_CSA0, src.0.as_ptr() as u32);

    // Write the XDMAC Destination Address Register.
    dma.wo(xdmac0::XDMAC_CDA0, peripheral.address);

    // Set the TYPE bit for a peripheral transfer.
    dma.wfo(xdmac0::XDMAC_CC0_TYPE, 1);

    // Data width is one word (32 bits).
    dma.wfo(xdmac0::XDMAC_CC0_DWIDTH, 2);

    // Set the chunk size for the peripheral.
    dma.wfo(xdmac0::XDMAC_CC0_CSIZE, peripheral.chunk_size);

    // MBSIZE does not matter for a memory-to-peripheral transfer.
    dma.wfo(xdmac0::XDMAC_CC0_MBSIZE, 0);

    // Set the UBLEN to the size of data. Divided by four because the DWIDTH is 4
    // bytes.
    dma.wfo(xdmac0::XDMAC_CUBC0_UBLEN, N as u32 / 4);

    // Set the source memory addressing mode to increment.
    dma.wfo(xdmac0::XDMAC_CC0_SAM, 1);

    // Set the destination memory addressing mode to not change the address.
    dma.wfo(xdmac0::XDMAC_CC0_DAM, 0);

    // Memory-to-peripheral transfer.
    dma.wfo(xdmac0::XDMAC_CC0_DSYNC, 1);

    // Set the peripheral ID.
    dma.wfo(xdmac0::XDMAC_CC0_PERID, peripheral.id);

    Ok(())
}

/// Set up a peripheral-to-memory DMA transfer using channel 1.
pub fn peripheral_to_memory<const N: usize>(
    peripheral: Peripheral,
    dst: &mut Buffer<N>,
) -> Result<(), &'static str> {
    let mut dma = CSR::new(utralib::HW_XDMAC0_BASE as *mut u32);

    if N % 4 != 0 {
        return Err("Destination buffer length is not a multiple of 4 bytes.");
    }

    // Clear the pending Interrupt Status bits by reading the selected XDMAC
    // Channel x Interrupt Status Register.
    dma.r(xdmac0::XDMAC_CIS1);

    // Write the XDMAC Source Address Register.
    dma.wo(xdmac0::XDMAC_CSA1, peripheral.address);

    // Write the XDMAC Destination Address Register.
    dma.wo(xdmac0::XDMAC_CDA1, dst.0.as_ptr() as u32);

    // Data width is one word (32 bits).
    dma.wfo(xdmac0::XDMAC_CC1_DWIDTH, 2);

    // Set the chunk size for the peripheral.
    dma.wfo(xdmac0::XDMAC_CC1_CSIZE, peripheral.chunk_size);

    // Use one chunk per memory burst.
    dma.wfo(xdmac0::XDMAC_CC1_MBSIZE, 0);

    dma.wfo(xdmac0::XDMAC_CUBC1_UBLEN, N as u32 / 4);

    // Set the TYPE bit for a peripheral transfer.
    dma.wfo(xdmac0::XDMAC_CC1_TYPE, 1);

    // Set the source memory addressing mode to not change the address.
    dma.wfo(xdmac0::XDMAC_CC1_SAM, 0);

    // Set the destination memory addressing mode to increment.
    dma.wfo(xdmac0::XDMAC_CC1_DAM, 1);

    // Peripheral-to-memory transfer.
    dma.wfo(xdmac0::XDMAC_CC1_DSYNC, 0);

    // Set the peripheral ID.
    dma.wfo(xdmac0::XDMAC_CC1_PERID, peripheral.id);

    Ok(())
}

/// Execute the peripheral transfer. This should be called after
/// [`memory_to_peripheral`] and [`peripheral_to_memory`] have been called.
pub fn execute_peripheral_transfer() {
    let mut dma = CSR::new(utralib::HW_XDMAC0_BASE as *mut u32);

    // Enable input and output channels to start the transfer.
    dma.wfo(xdmac0::XDMAC_GE_EN0, 1);
    dma.wfo(xdmac0::XDMAC_GE_EN1, 1);

    // Wait until the transfer is done.
    while dma.rf(xdmac0::XDMAC_GS_ST0) == 1 || dma.rf(xdmac0::XDMAC_GS_ST1) == 1 {}
}
