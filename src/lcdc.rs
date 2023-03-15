//! LCD controller (LCDC) implementation.

use utralib::utra::lcdc::*;
use utralib::{HW_LCDC_BASE, *};

#[repr(align(8))]
#[derive(Debug, Default)]
pub struct LcdDmaDesc {
    pub addr: u32,
    pub ctrl: u32,
    pub next: u32,
}

#[derive(Debug, Copy, Clone)]
#[allow(dead_code)]
#[repr(u8)]
pub enum LcdcLayerId {
    /// Base layer.
    Base = 0,

    /// Overlay layer 1.
    Ovr1 = 1,

    /// Overlay layer 2.
    Ovr2 = 2,

    /// High-end overlay.
    Heo = 3,
}

#[derive(Debug)]
#[repr(u8)]
#[allow(dead_code)]
enum LcdcPwmClockSource {
    Slow = 0,
    System,
}

#[derive(Debug)]
#[repr(u8)]
#[allow(dead_code)]
enum LcdcClockSource {
    System = 0,
    System2x,
}

#[derive(Debug)]
#[allow(dead_code)]
enum OutputColorMode {
    Mode12bpp = 0,
    Mode16Bpp = 1,
    Mode18Bpp = 2,
    Mode24Bpp = 3,
}

#[derive(Debug)]
#[allow(dead_code)]
enum VsyncSyncEdge {
    First = 0,
    Second,
}

#[derive(Debug)]
#[allow(dead_code)]
enum SignalPolarity {
    Positive = 0,
    Negative,
}

#[derive(Debug)]
#[allow(dead_code)]
enum ColorMode {
    Rgb444 = 0,
    Argb444,
    Rgba444,
    Rgb565,
    Trgb1555,
    Rgb666,
    Rgb666Packed,
    Trgb1666,
    TrgbPacked,
    Rgb888,
    Rgb888Packed,
    Trgb1888,
    Argb8888,
    Rgba8888,
    Lut8,
}

#[derive(Debug)]
#[allow(dead_code)]
enum BurstLength {
    Single = 0,
    Incr4,
    Incr8,
    Incr16,
}

const PWM_CLOCK_SOURCE: LcdcPwmClockSource = LcdcPwmClockSource::System;
const PWM_PRESCALER: u8 = 5;
const HSYNC_LENGTH: u16 = 48;
const VSYNC_LENGTH: u16 = 3;
const PIXEL_CLOCK_DIV: u8 = 6;
const DISPLAY_GUARD_NUM_FRAMES: u16 = 30;
const OUTPUT_COLOR_MODE: OutputColorMode = OutputColorMode::Mode24Bpp;
const SYNC_EDGE: VsyncSyncEdge = VsyncSyncEdge::First;
const VSYNC_POLARITY: SignalPolarity = SignalPolarity::Negative;
const HSYNC_POLARITY: SignalPolarity = SignalPolarity::Negative;
const DEFAULT_BRIGHTNESS_PCT: u32 = 100;
const PWM_SIGNAL_POLARITY: SignalPolarity = SignalPolarity::Positive;
const DEFAULT_GFX_COLOR_MODE: ColorMode = ColorMode::Argb8888;
const LOWER_MARGIN: u16 = 3;
const UPPER_MARGIN: u16 = 29;
const RIGHT_MARGIN: u16 = 40;
const LEFT_MARGIN: u16 = 40;

pub struct LayerConfig {
    id: LcdcLayerId,
    fb_phys_addr: usize,
    dma_desc_addr: usize,
    dma_desc_phys_addr: usize,
}

impl LayerConfig {
    pub fn new(
        id: LcdcLayerId,
        fb_phys_addr: usize,
        dma_desc_addr: usize,
        dma_desc_phys_addr: usize,
    ) -> LayerConfig {
        Self {
            id,
            fb_phys_addr,
            dma_desc_addr,
            dma_desc_phys_addr,
        }
    }
}

pub struct Lcdc {
    base_addr: u32,
    w: u16,
    h: u16,
}

impl Lcdc {
    pub fn new(w: u16, h: u16) -> Lcdc {
        Lcdc {
            base_addr: HW_LCDC_BASE as u32,
            w,
            h,
        }
    }

    /// Creates a new LCDC instance with the specified base address and
    /// DMA descriptor's virtual and physical addresses.
    pub fn new_vma(base_addr: u32, w: u16, h: u16) -> Lcdc {
        Lcdc { base_addr, w, h }
    }

    pub fn init(&mut self, layers: &[LayerConfig]) {
        // Configure the LCD timing parameters
        self.wait_for_sync_in_progress();
        self.select_pwm_clock_source(PWM_CLOCK_SOURCE);
        self.set_clock_divider(PIXEL_CLOCK_DIV);

        // Disable all layers for now
        self.set_layer_clock_gating_disable(LcdcLayerId::Base, false);
        self.set_layer_clock_gating_disable(LcdcLayerId::Ovr1, false);
        self.set_layer_clock_gating_disable(LcdcLayerId::Ovr2, false);
        self.set_layer_clock_gating_disable(LcdcLayerId::Heo, false);

        self.wait_for_sync_in_progress();
        self.set_hsync_pulse_width(HSYNC_LENGTH);
        self.set_vsync_pulse_width(VSYNC_LENGTH);

        self.wait_for_sync_in_progress();
        self.set_vertical_front_porch_width(LOWER_MARGIN); //Set the vertical porches
        self.set_vertical_back_porch_width(UPPER_MARGIN);

        self.wait_for_sync_in_progress();
        self.set_horizontal_front_porch_width(RIGHT_MARGIN); //Set the horizontal porches
        self.set_horizontal_back_porch_width(LEFT_MARGIN);

        self.wait_for_sync_in_progress();
        self.set_num_active_rows(self.h);
        self.set_num_pixels_per_line(self.w);

        self.wait_for_sync_in_progress();
        self.set_display_guard_time(DISPLAY_GUARD_NUM_FRAMES);
        self.set_output_mode(OUTPUT_COLOR_MODE);
        self.set_display_signal_synchronization(true);
        self.set_vsync_pulse_start(SYNC_EDGE);
        self.set_vsync_polarity(VSYNC_POLARITY);
        self.set_hsync_polarity(HSYNC_POLARITY);

        self.wait_for_sync_in_progress();
        self.set_pwm_compare_value(DEFAULT_BRIGHTNESS_PCT * 0xFF / 100);
        self.set_pwm_signal_polarity(PWM_SIGNAL_POLARITY);
        self.set_pwm_prescaler(PWM_PRESCALER);

        // 2. Enable pixel clock
        self.wait_for_sync_in_progress();
        self.set_pixel_clock_enable(true);

        // 3. Check that the clock is running
        self.wait_for_clock_running();

        // 4. Enable Horizontal and Vertical Synchronization
        self.wait_for_sync_in_progress();
        self.set_sync_enable(true);

        // 5. Check that synchronization is up
        self.wait_for_sync();

        // 6. Enable the display power signal
        self.wait_for_sync_in_progress();
        self.set_disp_signal_enable(true);

        // 7. Wait for power signal to be activated
        self.wait_for_disp_signal();

        // 8. Enable the backlight
        self.wait_for_sync_in_progress();
        self.set_pwm_enable(true);

        for layer in layers {
            self.update_layer(layer);

            self.set_layer_clock_gating_disable(layer.id, false);
            self.set_use_dma_path_enable(layer.id, true);
            self.set_rgb_mode_input(layer.id, DEFAULT_GFX_COLOR_MODE);

            self.set_transfer_descriptor_fetch_enable(layer.id, true);
            self.update_overlay_attributes_enable(layer.id);
            self.update_attribute(layer.id);

            self.set_system_bus_dma_burst_length(layer.id, BurstLength::Incr16);
            self.set_system_bus_dma_burst_enable(layer.id, true);

            self.set_channel_enable(layer.id, true);
        }
    }

    pub fn update_layer(&self, layer: &LayerConfig) {
        let dma_desc = layer.dma_desc_addr as *mut LcdDmaDesc;
        unsafe {
            (*dma_desc).addr = layer.fb_phys_addr as u32;
            (*dma_desc).ctrl = 0x01;
            (*dma_desc).next = layer.dma_desc_phys_addr as u32;
        }

        self.set_dma_address_register(layer.id, layer.fb_phys_addr as u32);
        self.set_dma_descriptor_next_address(layer.id, layer.dma_desc_phys_addr as u32);
        self.set_dma_head_pointer(layer.id, layer.dma_desc_phys_addr as u32);
    }

    pub fn update_buffer(&self, id: LcdcLayerId, next_dma_addr_phys: u32) {
        // self.set_channel_enable(id, false);
        self.set_dma_head_pointer(id, next_dma_addr_phys);
        // self.add_dma_desc_to_queue(id);
        // self.reset_channel(id);
        // self.set_channel_enable(id, true);
    }

    fn wait_for_sync_in_progress(&self) {
        let lcdc_csr = CSR::new(self.base_addr as *mut u32);

        while lcdc_csr.rf(LCDSR_SIPSTS) != 0 {
            armv7::asm::nop();
        }
    }

    fn select_pwm_clock_source(&mut self, source: LcdcPwmClockSource) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG0_CLKPWMSEL, source as u32);
    }

    fn set_clock_divider(&self, value: u8) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG0_CLKDIV, value.saturating_sub(2) as u32);
    }

    fn set_hsync_pulse_width(&self, value: u16) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG1_HSPW, value as u32);
    }

    fn set_vsync_pulse_width(&self, value: u16) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG1_VSPW, value as u32);
    }

    fn set_layer_clock_gating_disable(&mut self, layer: LcdcLayerId, disable: bool) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => lcdc_csr.rmwf(LCDCFG0_CGDISBASE, !disable as u32),
            LcdcLayerId::Ovr1 => lcdc_csr.rmwf(LCDCFG0_CGDISOVR1, !disable as u32),
            LcdcLayerId::Ovr2 => lcdc_csr.rmwf(LCDCFG0_CGDISOVR2, !disable as u32),
            LcdcLayerId::Heo => lcdc_csr.rmwf(LCDCFG0_CGDISHEO, !disable as u32),
        }
    }

    fn set_num_active_rows(&mut self, value: u16) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG4_RPF, value.saturating_sub(1) as u32);
    }

    fn set_num_pixels_per_line(&mut self, value: u16) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG4_PPL, value.saturating_sub(1) as u32);
    }

    fn set_display_guard_time(&mut self, frames: u16) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG5_GUARDTIME, frames as u32);
    }

    fn set_output_mode(&mut self, mode: OutputColorMode) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG5_MODE, mode as u32);
    }

    fn set_display_signal_synchronization(&mut self, synchronous: bool) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG5_DISPDLY, !synchronous as u32);
    }

    fn set_vsync_pulse_start(&mut self, edge: VsyncSyncEdge) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG5_VSPDLYS, edge as u32);
    }

    fn set_vsync_polarity(&mut self, polarity: SignalPolarity) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG5_VSPOL, polarity as u32);
    }

    fn set_hsync_polarity(&mut self, polarity: SignalPolarity) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG5_HSPOL, polarity as u32);
    }

    fn set_pwm_compare_value(&mut self, value: u32) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG6_PWMCVAL, value);
    }

    fn set_pwm_signal_polarity(&mut self, polarity: SignalPolarity) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG6_PWMCVAL, polarity as u32);
    }

    fn set_pwm_prescaler(&mut self, div: u8) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG6_PWMPS, div as u32);
    }

    fn set_pixel_clock_enable(&mut self, enable: bool) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        if enable {
            lcdc_csr.rmwf(LCDEN_CLKEN, 1);
        } else {
            lcdc_csr.rmwf(LCDDIS_CLKDIS, 1);
        }
    }

    fn wait_for_clock_running(&self) {
        let lcdc_csr = CSR::new(self.base_addr as *mut u32);

        while lcdc_csr.rf(LCDSR_CLKSTS) == 0 {
            armv7::asm::nop();
        }
    }

    fn set_sync_enable(&mut self, enable: bool) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        if enable {
            lcdc_csr.rmwf(LCDEN_SYNCEN, 1);
        } else {
            lcdc_csr.rmwf(LCDDIS_SYNCDIS, 1);
        }
    }

    fn wait_for_sync(&self) {
        let lcdc_csr = CSR::new(self.base_addr as *mut u32);

        while lcdc_csr.rf(LCDSR_LCDSTS) == 0 {
            armv7::asm::nop();
        }
    }

    fn set_disp_signal_enable(&mut self, enable: bool) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        if enable {
            lcdc_csr.rmwf(LCDEN_DISPEN, 1);
        } else {
            lcdc_csr.rmwf(LCDDIS_DISPDIS, 1);
        }
    }

    fn wait_for_disp_signal(&self) {
        let lcdc_csr = CSR::new(self.base_addr as *mut u32);

        while lcdc_csr.rf(LCDSR_DISPSTS) == 0 {
            armv7::asm::nop();
        }
    }

    fn set_pwm_enable(&self, enable: bool) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        if enable {
            lcdc_csr.rmwf(LCDEN_PWMEN, 1);
        } else {
            lcdc_csr.rmwf(LCDDIS_PWMDIS, 1);
        }
    }

    #[allow(dead_code)]
    fn set_window_size(&mut self, layer: LcdcLayerId, _x: u16, _y: u16) {
        let mut _lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Ovr1 => todo!(),
            LcdcLayerId::Ovr2 => todo!(),
            LcdcLayerId::Heo => todo!(),
            LcdcLayerId::Base => (), // Unsupported
        }
    }

    fn set_use_dma_path_enable(&self, layer: LcdcLayerId, enable: bool) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => lcdc_csr.rmwf(BASECFG4_DMA, enable as u32),
            LcdcLayerId::Ovr1 => lcdc_csr.rmwf(OVR1CFG9_DMA, enable as u32),
            LcdcLayerId::Ovr2 => lcdc_csr.rmwf(OVR2CFG9_DMA, enable as u32),
            LcdcLayerId::Heo => lcdc_csr.rmwf(HEOCFG12_DMA, enable as u32),
        }
    }

    fn set_rgb_mode_input(&self, layer: LcdcLayerId, mode: ColorMode) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => lcdc_csr.rmwf(BASECFG1_RGBMODE, mode as u32),
            LcdcLayerId::Ovr1 => lcdc_csr.rmwf(OVR1CFG1_RGBMODE, mode as u32),
            LcdcLayerId::Ovr2 => lcdc_csr.rmwf(OVR2CFG1_RGBMODE, mode as u32),
            LcdcLayerId::Heo => lcdc_csr.rmwf(HEOCFG1_RGBMODE, mode as u32),
        }
    }

    fn set_dma_address_register(&self, layer: LcdcLayerId, addr: u32) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => lcdc_csr.rmwf(BASEADDR_ADDR, addr),
            LcdcLayerId::Ovr1 => lcdc_csr.rmwf(OVR1ADDR_ADDR, addr),
            LcdcLayerId::Ovr2 => lcdc_csr.rmwf(OVR2ADDR_ADDR, addr),
            LcdcLayerId::Heo => lcdc_csr.rmwf(HEOADDR_ADDR, addr),
        }
    }

    fn set_dma_head_pointer(&self, layer: LcdcLayerId, addr: u32) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => lcdc_csr.rmwf(BASEHEAD_HEAD, addr),
            LcdcLayerId::Ovr1 => lcdc_csr.rmwf(OVR1HEAD_HEAD, addr),
            LcdcLayerId::Ovr2 => lcdc_csr.rmwf(OVR2HEAD_HEAD, addr),
            LcdcLayerId::Heo => lcdc_csr.rmwf(HEOHEAD_HEAD, addr),
        }
    }

    fn get_dma_head_pointer(&self, layer: LcdcLayerId) -> u32 {
        let lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => lcdc_csr.rf(BASEHEAD_HEAD),
            LcdcLayerId::Ovr1 => lcdc_csr.rf(OVR1HEAD_HEAD),
            LcdcLayerId::Ovr2 => lcdc_csr.rf(OVR2HEAD_HEAD),
            LcdcLayerId::Heo => lcdc_csr.rf(HEOHEAD_HEAD),
        }
    }

    fn set_dma_descriptor_next_address(&self, layer: LcdcLayerId, addr: u32) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => lcdc_csr.rmwf(BASENEXT_NEXT, addr),
            LcdcLayerId::Ovr1 => lcdc_csr.rmwf(OVR1NEXT_NEXT, addr),
            LcdcLayerId::Ovr2 => lcdc_csr.rmwf(OVR2NEXT_NEXT, addr),
            LcdcLayerId::Heo => lcdc_csr.rmwf(HEONEXT_NEXT, addr),
        }
    }

    fn get_dma_descriptor_next_address(&self, layer: LcdcLayerId) -> u32 {
        let lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => lcdc_csr.rf(BASENEXT_NEXT),
            LcdcLayerId::Ovr1 => lcdc_csr.rf(OVR1NEXT_NEXT),
            LcdcLayerId::Ovr2 => lcdc_csr.rf(OVR2NEXT_NEXT),
            LcdcLayerId::Heo => lcdc_csr.rf(HEONEXT_NEXT),
        }
    }

    fn set_transfer_descriptor_fetch_enable(&self, layer: LcdcLayerId, enable: bool) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => lcdc_csr.rmwf(BASECTRL_DFETCH, enable as u32),
            LcdcLayerId::Ovr1 => lcdc_csr.rmwf(OVR1CTRL_DFETCH, enable as u32),
            LcdcLayerId::Ovr2 => lcdc_csr.rmwf(OVR2CTRL_DFETCH, enable as u32),
            LcdcLayerId::Heo => lcdc_csr.rmwf(HEOCTRL_DFETCH, enable as u32),
        }
    }

    fn set_system_bus_dma_burst_enable(&self, layer: LcdcLayerId, enable: bool) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => lcdc_csr.rmwf(BASECFG0_DLBO, enable as u32),
            LcdcLayerId::Ovr1 => lcdc_csr.rmwf(OVR1CFG0_DLBO, enable as u32),
            LcdcLayerId::Ovr2 => lcdc_csr.rmwf(OVR2CFG0_DLBO, enable as u32),
            LcdcLayerId::Heo => lcdc_csr.rmwf(HEOCFG0_DLBO, enable as u32),
        }
    }

    fn set_system_bus_dma_burst_length(&self, layer: LcdcLayerId, len: BurstLength) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => lcdc_csr.rmwf(BASECFG0_BLEN, len as u32),
            LcdcLayerId::Ovr1 => lcdc_csr.rmwf(OVR1CFG0_BLEN, len as u32),
            LcdcLayerId::Ovr2 => lcdc_csr.rmwf(OVR2CFG0_BLEN, len as u32),
            LcdcLayerId::Heo => lcdc_csr.rmwf(HEOCFG0_BLEN, len as u32),
        }
    }

    fn set_channel_enable(&self, layer: LcdcLayerId, enable: bool) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => {
                if enable {
                    lcdc_csr.rmwf(BASECHER_CHEN, 1);
                } else {
                    lcdc_csr.rmwf(BASECHDR_CHDIS, 1);
                }
            }
            LcdcLayerId::Ovr1 => {
                if enable {
                    lcdc_csr.rmwf(OVR1CHER_CHEN, 1);
                } else {
                    lcdc_csr.rmwf(OVR1CHDR_CHDIS, 1);
                }
            }
            LcdcLayerId::Ovr2 => {
                if enable {
                    lcdc_csr.rmwf(OVR2CHER_CHEN, 1);
                } else {
                    lcdc_csr.rmwf(OVR2CHDR_CHDIS, 1);
                }
            }
            LcdcLayerId::Heo => {
                if enable {
                    lcdc_csr.rmwf(HEOCHER_CHEN, 1);
                } else {
                    lcdc_csr.rmwf(HEOCHDR_CHDIS, 1);
                }
            }
        }
    }

    fn set_vertical_front_porch_width(&mut self, margin: u16) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG2_VFPW, margin as u32);
    }

    fn set_vertical_back_porch_width(&mut self, margin: u16) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG2_VBPW, margin as u32);
    }

    fn set_horizontal_front_porch_width(&mut self, margin: u16) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG3_HFPW, margin as u32);
    }

    fn set_horizontal_back_porch_width(&mut self, margin: u16) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);
        lcdc_csr.rmwf(LCDCFG3_HBPW, margin as u32);
    }

    fn update_overlay_attributes_enable(&self, layer: LcdcLayerId) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => lcdc_csr.rmwf(BASECHER_UPDATEEN, 1),
            LcdcLayerId::Ovr1 => lcdc_csr.rmwf(OVR1CHER_UPDATEEN, 1),
            LcdcLayerId::Ovr2 => lcdc_csr.rmwf(OVR2CHER_UPDATEEN, 1),
            LcdcLayerId::Heo => lcdc_csr.rmwf(HEOCHER_UPDATEEN, 1),
        }
    }

    fn update_attribute(&self, layer: LcdcLayerId) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => lcdc_csr.rmwf(ATTR_BASE, 1),
            LcdcLayerId::Ovr1 => lcdc_csr.rmwf(ATTR_OVR1, 1),
            LcdcLayerId::Ovr2 => lcdc_csr.rmwf(ATTR_OVR2, 1),
            LcdcLayerId::Heo => lcdc_csr.rmwf(ATTR_HEO, 1),
        }
    }

    fn reset_channel(&self, layer: LcdcLayerId) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => lcdc_csr.rmwf(BASECHDR_CHRST, 1),
            LcdcLayerId::Ovr1 => lcdc_csr.rmwf(OVR1CHDR_CHRST, 1),
            LcdcLayerId::Ovr2 => lcdc_csr.rmwf(OVR2CHDR_CHRST, 1),
            LcdcLayerId::Heo => lcdc_csr.rmwf(HEOCHDR_CHRST, 1),
        }
    }

    fn add_dma_desc_to_queue(&self, layer: LcdcLayerId) {
        let mut lcdc_csr = CSR::new(self.base_addr as *mut u32);

        match layer {
            LcdcLayerId::Base => lcdc_csr.rmwf(BASECHER_A2QEN, 1),
            LcdcLayerId::Ovr1 => lcdc_csr.rmwf(OVR1CHER_A2QEN, 1),
            LcdcLayerId::Ovr2 => lcdc_csr.rmwf(OVR2CHER_A2QEN, 1),
            LcdcLayerId::Heo => lcdc_csr.rmwf(HEOCHER_A2QEN, 1),
        }
    }
}
