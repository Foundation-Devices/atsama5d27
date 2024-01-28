use {
    bitflags::bitflags,
    utralib::{utra::isc::*, CSR, HW_ISC_BASE},
};

bitflags! {
    #[derive(Debug, Copy, Clone)]
    pub struct ISCStatus: u32 {
        /// Vertical Synchronization Detected Interrupt (cleared on read)
        const VD       = 1 << 0;
        /// Horizontal Synchronization Detected Interrupt (cleared on read)
        const HD       = 1 << 1;
        /// Software Reset Completed Interrupt (cleared on read)
        const SWRST    = 1 << 4;
        /// Disable Completed Interrupt (cleared on read)
        const DIS      = 1 << 5;
        /// DMA Done Interrupt (cleared on read)
        const DDONE    = 1 << 8;
        /// DMA List Done Interrupt (cleared on read)
        const LDONE    = 1 << 9;
        /// Histogram Completed Interrupt (cleared on read)
        const HISDONE  = 1 << 12;
        /// Histogram Clear Interrupt (cleared on read)
        const HISCLR   = 1 << 13;
        /// Write Channel Error Interrupt (cleared on read)
        const WERR     = 1 << 16;
        /// Write Channel Error Identifier 0 (cleared on read)
        const WERRID1  = 1 << 18;
        /// Write Channel Error Identifier 1 (cleared on read)
        const WERRID0  = 1 << 17;
        /// Read Channel Error Interrupt (cleared on read)
        const RERR     = 1 << 20;
        /// Vertical Front Porch Overflow Interrupt (cleared on read)
        const VFPOV    = 1 << 24;
        /// Data Overflow Interrupt (cleared on read)
        const DAOV     = 1 << 25;
        /// Vertical Synchronization Timeout Interrupt (cleared on read)
        const VDTO     = 1 << 26;
        /// Horizontal Synchronization Timeout Interrupt (cleared on read)
        const HDTO     = 1 << 27;
        /// CCIR Decoder Error Interrupt (cleared on read)
        const CCIRERR  = 1 << 28;
    }
}

/// DMA Memory Burst Size
#[derive(Debug)]
enum DMABurstSize {
    /// DMA single access
    Single = 0,
    /// 4-beat burst access
    Beats4,
    /// 8-beat burst access
    Beats8,
    /// 16-beat burst access
    Beats16,
}

/// DMA Input Mode Selection
#[derive(Debug)]
enum DmaInputMode {
    /// 8 bits, single channel packed
    Packed8 = 0,
    /// 16 bits, single channel packed
    Packed16,
    /// 32 bits, single channel packed
    Packed32,
    /// 32 bits, dual channel
    YC422Sp,
    /// 32 bits, triple channel
    Yc422P,
    /// 32 bits, dual channel
    YC420Sp,
    /// 32 bits, triple channel
    YC420P,
}

#[derive(Debug)]
enum RlpMode {
    /// 8-bit data.
    Dat8 = 0,
    /// 9-bit data
    Dat9 = 1,
    /// 10-bit data
    Dat10 = 2,
    /// 11-bit data
    Dat11 = 3,
    /// 12-bit data
    Dat12 = 4,
    /// 8-bit luminance only
    DatY8 = 5,
    /// 10-bit luminance only
    DatY10 = 6,
    /// 12-bit RGB+4-bit Alpha (MSB)
    Argb444 = 7,
    /// 15-bit RGB+1-bit Alpha (MSB)
    Argb555 = 8,
    /// 16-bit RGB
    Rgb565 = 9,
    /// 24-bits RGB mode+8-bit Alpha
    Argb32 = 10,
    /// YCbCr mode (full range, 0–255)
    YyCc = 11,
    /// YCbCr mode (limited range)
    YyCcLimitedYCbCr = 12,
}

#[derive(Debug, Copy, Clone)]
enum Polarity {
    /// Signal is active high, i.e. valid pixels are sampled when the signal is asserted.
    High = 0,

    /// Signal is active low, i.e. valid pixels are sampled when the signal is deasserted.
    Low,
}

#[derive(Debug)]
enum PfeBps {
    Twelve = 0,
    Eleven = 1,
    Ten = 2,
    Nine = 3,
    Eight = 4,
}

#[derive(Debug)]
enum PfeVideoMode {
    /// Video source is progressive.
    Progressive = 0,
    /// Video source is interlaced, two fields are captured starting with top field.
    DfTop = 1,
    /// Video source is interlaced, two fields are captured starting with bottom field.
    DfBottom = 2,
    /// Video source is interlaced, two fields are captured immediately.
    DfImmediate = 3,
    /// Video source is interlaced, one field is captured starting with the top field.
    SfTop = 4,
    /// Video source is interlaced, one field is captured starting with the bottom field.
    SfBottom = 5,
    /// Video source is interlaced, one field is captured starting immediately.
    SfImmediate = 6,
}

#[derive(Debug, Default, Copy, Clone)]
pub enum DmaDescriptorView {
    #[default]
    Packed = 0,
    SemiPlanar = 1,
    Planar = 2,
}

/// DMA descriptor control configuration.
#[derive(Debug, Default)]
pub struct DmaControlConfig {
    /// Enable DMA descriptor
    pub descriptor_enable: bool,

    /// Descriptor View : Packed, semi-planar, planar
    pub descriptor_view: DmaDescriptorView,

    /// DMA Done interrupt enable
    pub interrupt_enable: bool,

    /// Write back operation is enabled
    pub writeback_enable: bool,

    /// Value of Captured Frame Field Signal
    pub field: bool,

    /// Descriptor Processing Status
    pub done: bool,
}

/// DMA view descriptor. Must be aligned in memory at 32-bit boundary.
#[derive(Debug, Copy, Clone)]
#[repr(C, align(32))]
pub struct DmaView {
    /// ISC DMA Control
    ctrl: u32,
    /// Next ISC DMA Descriptor Address number
    next_desc: u32,
    /// Transfer Address
    addr: u32,
    // Stride
    stride: u32,
}

impl DmaView {
    pub const fn new() -> DmaView {
        DmaView {
            ctrl: 0,
            next_desc: 0,
            addr: 0,
            stride: 0,
        }
    }
}

#[derive(Debug, Copy, Clone)]
#[repr(u8)]
pub enum MckSel {
    Hclock = 0,
    Isclk,
    Gck,
}

pub struct Isc {
    base_addr: u32,
}

impl Isc {
    pub fn new() -> Self {
        Isc {
            base_addr: HW_ISC_BASE as u32,
        }
    }

    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Isc { base_addr }
    }

    pub fn init(&mut self, mck_div: u8, mck_sel: MckSel, isp_div: u8) {
        self.reset();

        let mut csr = CSR::new(self.base_addr as *mut u32);

        // Configure master clock
        csr.rmwf(CLKCFG_MCSEL, mck_sel as u32);
        csr.rmwf(CLKCFG_MCDIV, mck_div as u32);

        self.wait_for_sync_clk();
        csr.wfo(CLKEN_MCEN, 1); // Enable master clock

        // Configure ISP clock
        csr.rmwf(CLKCFG_ICDIV, isp_div as u32);
        csr.rmwf(CLKCFG_ICSEL, 0);

        self.wait_for_sync_clk();
        csr.wfo(CLKEN_ICEN, 1); // Enable ISP clock to provide the cam chip with the clock

        assert_ne!(csr.rf(CLKSR_MCSR), 0);
        assert_ne!(csr.rf(CLKSR_ICSR), 0);
    }

    fn wait_for_sync(&self) {
        let csr = CSR::new(self.base_addr as *mut u32);
        while csr.rf(CTRLSR_SIP) != 0 {
            armv7::asm::nop();
        }
    }

    fn wait_for_sync_clk(&self) {
        let csr = CSR::new(self.base_addr as *mut u32);
        while csr.rf(CLKSR_SIP) != 0 {
            armv7::asm::nop();
        }
    }

    fn set_dma_desc_phys_addr(&mut self, dma_desc_phys_addr: u32) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wo(DNDA, dma_desc_phys_addr);
    }

    pub fn configure(
        &mut self,
        dma_desc_addr_1: u32,
        dma_desc_phys_addr_1: u32,
        dma_desc_addr_2: u32,
        dma_desc_phys_addr_2: u32,
        fb_phys_addr: u32,
        dma_control_config: &DmaControlConfig,
    ) {
        self.pfe_set_continuous_mode(true);

        // Configure the Parallel Front End module performs data
        // re-sampling across clock domain boundary. The PFE module
        // outputs a 12-bit data on the vp_data[11:0] bus
        self.pfe_set_video_mode(PfeVideoMode::Progressive);

        // Set 8 bit per sample as per schematic.
        self.pfe_set_bps(PfeBps::Eight);

        // Set HSYNC and VSYNC signal polarity
        let vpol = Polarity::Low;
        let hpol = Polarity::High;
        self.pfe_set_sync_polarity(vpol, hpol);

        // Set pixel clock polarity
        self.pfe_set_pclk_polarity(Polarity::High);

        // Set color output mode and alpha value
        self.rlp_configure(RlpMode::Rgb565, 0xff); // TODO: try 0xff alpha too

        // Configure DMA
        self.configure_dma(
            dma_desc_addr_1,
            dma_desc_phys_addr_1,
            dma_desc_addr_2,
            dma_desc_phys_addr_2,
            fb_phys_addr,
            dma_control_config,
        );

        // Update profile
        self.update_profile();
    }

    pub fn start_capture(&mut self) {
        self.wait_for_sync();
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CTRLEN_CAPTURE, 1);
    }

    pub fn interrupt_status(&mut self) -> ISCStatus {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        ISCStatus::from_bits_truncate(csr.r(INTSR))
    }

    fn configure_dma(
        &mut self,
        dma_desc_addr_1: u32,
        dma_desc_phys_addr_1: u32,
        dma_desc_addr_2: u32,
        dma_desc_phys_addr_2: u32,
        fb_phys_addr: u32,
        dma_control_config: &DmaControlConfig,
    ) {
        let dma_desc_1 = dma_desc_addr_1 as *mut DmaView;
        unsafe {
            (*dma_desc_1).ctrl = 0b01; // Mode = packed (0), descriptor enable = 1
            (*dma_desc_1).next_desc = dma_desc_phys_addr_2; // Loop the descriptor linked list
            (*dma_desc_1).addr = fb_phys_addr;
            (*dma_desc_1).stride = 0;
        }

        let dma_desc_2 = dma_desc_addr_2 as *mut DmaView;
        unsafe {
            (*dma_desc_2).ctrl = 0b01; // Mode = packed (0), descriptor enable = 1
            (*dma_desc_2).next_desc = dma_desc_phys_addr_1; // Loop the descriptor linked list
            (*dma_desc_2).addr = fb_phys_addr;
            (*dma_desc_2).stride = 0;
        }

        self.set_dma_desc_phys_addr(dma_desc_phys_addr_1);
        self.dma_configure_register(
            DmaInputMode::Packed16,
            DMABurstSize::Single,
            DMABurstSize::Single,
        );
        self.dma_enable(dma_control_config);
    }

    fn update_profile(&mut self) {
        self.wait_for_sync();
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CTRLEN_UPPRO, 1);

        // Wait for the profile update to take place
        while csr.rf(CTRLSR_UPPRO) != 0 {
            armv7::asm::nop();
        }
    }

    fn dma_enable(&mut self, dma_control_config: &DmaControlConfig) {
        let mut csr = CSR::new(self.base_addr as *mut u32);

        csr.rmwf(DCTRL_DE, dma_control_config.descriptor_enable as u32);
        csr.rmwf(DCTRL_DVIEW, dma_control_config.descriptor_view as u32);
        csr.rmwf(DCTRL_IE, dma_control_config.interrupt_enable as u32);
        csr.rmwf(DCTRL_WB, dma_control_config.writeback_enable as u32);
        csr.rmwf(DCTRL_FIELD, dma_control_config.field as u32);
        csr.rmwf(DCTRL_DONE, dma_control_config.done as u32);
    }

    fn dma_configure_register(
        &mut self,
        imode: DmaInputMode,
        ymbsize: DMABurstSize,
        cmbsize: DMABurstSize,
    ) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(DCFG_IMODE, imode as u32);
        csr.rmwf(DCFG_YMBSIZE, ymbsize as u32);
        csr.rmwf(DCFG_CMBSIZE, cmbsize as u32);
    }

    /// Configures Rounding, Limiting and Packing Mode.
    fn rlp_configure(&mut self, mode: RlpMode, alpha: u8) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(RLP_CFG_MODE, mode as u32);
        csr.rmwf(RLP_CFG_ALPHA, alpha as u32);
    }

    fn pfe_set_ccir656(&mut self, set: bool) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(PFE_CFG0_CCIR656, set as u32);
    }

    fn pfe_set_ccir656_10bit(&mut self, set: bool) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(PFE_CFG0_CCIR10_8N, set as u32);
    }

    fn pfe_set_pclk_polarity(&mut self, pol: Polarity) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(PFE_CFG0_PPOL, pol as u32);
    }

    fn pfe_set_sync_polarity(&mut self, vpol: Polarity, hpol: Polarity) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(PFE_CFG0_VPOL, vpol as u32);
        csr.rmwf(PFE_CFG0_HPOL, hpol as u32);
    }

    fn pfe_set_bps(&mut self, bps: PfeBps) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(PFE_CFG0_BPS, bps as u32);
    }

    fn pfe_set_video_mode(&mut self, video_mode: PfeVideoMode) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(PFE_CFG0_MODE, video_mode as u32);
    }

    /// Set PFE (Parallel Front End) in continuous mode.
    fn pfe_set_continuous_mode(&mut self, enable: bool) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(PFE_CFG0_CONT, enable as u32);
    }

    fn reset(&mut self) {
        self.wait_for_sync();
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.wfo(CTRLDIS_SWRST, 1);
    }
}
