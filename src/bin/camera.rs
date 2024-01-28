#![no_std]
#![no_main]

use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use ovm7690_rs::Ovm7690;
use {
    atsama5d27::{
        aic::{Aic, InterruptEntry, SourceKind},
        display::FramebufDisplay,
        l2cc::{Counter, EventCounterKind, L2cc},
        lcdc::{LayerConfig, LcdDmaDesc, Lcdc, LcdcLayerId},
        lcdspi::LcdSpi,
        pio::{Direction, Event, Func, Pio, PioB, PioC, PioPort},
        pit::{Pit, PIV_MAX},
        pmc::{PeripheralId, Pmc},
        sfr::Sfr,
        spi::{ChipSelect, Spi},
        tc::Tc,
        twi::Twi,
        uart::{Uart, Uart1},
    },
    core::{
        arch::global_asm,
        fmt::Write,
        panic::PanicInfo,
        sync::atomic::{
            compiler_fence,
            AtomicBool,
            Ordering::{Relaxed, SeqCst},
        },
    },
    embedded_graphics::{
        mono_font::{ascii::FONT_9X18, MonoTextStyle},
        pixelcolor::Rgb888,
        prelude::*,
        primitives::{Circle, Line, PrimitiveStyleBuilder, Rectangle, StyledDrawable},
        text::Text,
    },
};
use atsama5d27::isc::{DmaControlConfig, DmaDescriptorView, DmaView, Isc, MckSel};
use atsama5d27::lcdc::ColorMode;

const WIDTH: usize = 480;
const HEIGHT: usize = 800;

#[repr(align(4))]
struct Aligned4<const SIZE: usize>([u32; SIZE]);

static mut FRAMEBUFFER_ONE: Aligned4<{WIDTH * HEIGHT}> = Aligned4([0; WIDTH * HEIGHT]);
static mut DMA_DESC_ONE: LcdDmaDesc = LcdDmaDesc {
    addr: 0,
    ctrl: 0,
    next: 0,
};

const CAM_WIDTH: usize = 480;
const CAM_HEIGHT: usize = 640;

static mut FRAMEBUFFER_CAM_ONE: Aligned4<{CAM_WIDTH * CAM_HEIGHT}> = Aligned4([0; CAM_WIDTH * CAM_HEIGHT]);
static mut DMA_DESC_CAM_ONE: LcdDmaDesc = LcdDmaDesc {
    addr: 0,
    ctrl: 0,
    next: 0,
};

static mut FRAMEBUFFER_CAM_TWO: Aligned4<{CAM_WIDTH * CAM_HEIGHT}> = Aligned4([0; CAM_WIDTH * CAM_HEIGHT]);
static mut DMA_DESC_CAM_TWO: LcdDmaDesc = LcdDmaDesc {
    addr: 0,
    ctrl: 0,
    next: 0,
};

const CAM_LAYER: LcdcLayerId = LcdcLayerId::Ovr1;

const ISC_MASTER_CLK_DIV: u8 = 7;
const ISC_MASTER_CLK_SEL: MckSel = MckSel::Isclk;
const ISC_ISP_CLK_DIV: u8 = 2;
static mut ISC_DMA_VIEW_ONE: DmaView = DmaView::new();
static mut ISC_DMA_VIEW_TWO: DmaView = DmaView::new();

global_asm!(include_str!("../start.S"));

type UartType = Uart<Uart1>;
const UART_PERIPH_ID: PeripheralId = PeripheralId::Uart1;
const TC_PERIPH_ID: PeripheralId = PeripheralId::Tc0;

static HAD_TC0_IRQ: AtomicBool = AtomicBool::new(false);

// MCK: 164MHz
// Clock frequency is divided by 2 because of the default `h32mxdiv` PMC setting
const MASTER_CLOCK_SPEED: u32 = 164000000 / 2;

static mut TWI0: Option<Twi> = None;
static mut ISC: Option<Isc> = None;

static mut DISPLAY: Option<FramebufDisplay> = None;

#[no_mangle]
fn _entry() -> ! {
    extern "C" {
        // These symbols come from `link.ld`
        static mut _sbss: u32;
        static mut _ebss: u32;
    }

    // Initialize RAM
    unsafe {
        r0::zero_bss(&mut _sbss, &mut _ebss);
    }

    atsama5d27::l1cache::disable_dcache();

    let mut pmc = Pmc::new();
    pmc.enable_peripheral_clock(PeripheralId::Pit);
    pmc.enable_peripheral_clock(PeripheralId::Tc0);
    pmc.enable_peripheral_clock(PeripheralId::Aic);
    pmc.enable_peripheral_clock(PeripheralId::Pioa);
    pmc.enable_peripheral_clock(PeripheralId::Piob);
    pmc.enable_peripheral_clock(PeripheralId::Pioc);
    pmc.enable_peripheral_clock(PeripheralId::Piod);
    pmc.enable_peripheral_clock(PeripheralId::Twi0);
    pmc.enable_peripheral_clock(PeripheralId::Spi0);
    pmc.enable_peripheral_clock(PeripheralId::Isi); // Isi = Isc
    pmc.enable_system_clock_isc(); // Allows to use MCKx2 clock freq

    let mut tc0 = Tc::new();
    tc0.init();

    let mut aic = Aic::new();
    aic.init();
    aic.set_spurious_handler_fn_ptr(aic_spurious_handler as unsafe extern "C" fn() as usize);

    let uart_irq_ptr = uart_irq_handler as unsafe extern "C" fn() as usize;
    aic.set_interrupt_handler(InterruptEntry {
        peripheral_id: UART_PERIPH_ID,
        vector_fn_ptr: uart_irq_ptr,
        kind: SourceKind::LevelSensitive,
        priority: 0,
    });

    let tc0_irq_ptr = tc0_irq_handler as unsafe extern "C" fn() as usize;
    aic.set_interrupt_handler(InterruptEntry {
        peripheral_id: TC_PERIPH_ID,
        vector_fn_ptr: tc0_irq_ptr,
        kind: SourceKind::LevelSensitive,
        priority: 0,
    });

    let pio_irq_ptr = pioa_irq_handler as unsafe extern "C" fn() as usize;
    aic.set_interrupt_handler(InterruptEntry {
        peripheral_id: PeripheralId::Pioa,
        vector_fn_ptr: pio_irq_ptr,
        kind: SourceKind::LevelSensitive,
        priority: 0,
    });

    // Enable interrupts
    unsafe {
        core::arch::asm!("cpsie if");
    }

    // Timer for delays
    let mut pit = Pit::new();
    pit.set_interval(PIV_MAX);
    pit.set_enabled(true);
    pit.set_clock_speed(MASTER_CLOCK_SPEED);

    let mut uart = UartType::new();
    uart.set_rx_interrupt(true);
    uart.set_rx(true);

    let dma_desc_addr_one = (unsafe { &mut DMA_DESC_ONE } as *const _) as usize;
    let fb1 = unsafe { FRAMEBUFFER_ONE.0.as_ptr() as usize };
    unsafe {
        FRAMEBUFFER_ONE.0.fill(0xc0c0c0);
        FRAMEBUFFER_CAM_ONE.0.fill(0x00aa00);
        FRAMEBUFFER_CAM_TWO.0.fill(0x0000ff);
    }

    let dma_desc_cam_one = (unsafe { &mut DMA_DESC_CAM_ONE } as *const _) as usize;
    let fb_cam_one = unsafe { FRAMEBUFFER_CAM_ONE.0.as_ptr() as usize };

    configure_isc_pins();
    configure_lcdc_pins();
    pmc.enable_peripheral_clock(PeripheralId::Lcdc);
    let mut lcdc = Lcdc::new(WIDTH as u16, HEIGHT as u16);
    lcdc.init(&[LayerConfig::new(
        LcdcLayerId::Base,
        fb1,
        dma_desc_addr_one,
        dma_desc_addr_one,
    )], || ());

    lcdc.set_window_size(CAM_LAYER, CAM_WIDTH as u16, CAM_HEIGHT as u16);
    lcdc.set_window_pos(CAM_LAYER, 0, 0);
    lcdc.update_layer(&LayerConfig::new(
        CAM_LAYER,
        fb_cam_one,
        dma_desc_cam_one,
        dma_desc_cam_one,
    ), ||());
    lcdc.enable_layer(CAM_LAYER);
    lcdc.set_rgb_mode_input(CAM_LAYER, ColorMode::Rgb565);

    lcdc.wait_for_sync_in_progress();
    lcdc.set_pwm_compare_value(0xff / 2);

    let mut console = uart;
    let display = FramebufDisplay::new(unsafe { &mut FRAMEBUFFER_ONE.0 }, WIDTH, HEIGHT);
    unsafe {
        DISPLAY = Some(display);
    }

    tc0.set_interrupt(true);

    // Do 8 clock cycles of SCL to reset all the possibly stuck slaves
    let mut scl = Pio::pc28();
    scl.set_func(Func::Gpio);
    scl.set_direction(Direction::Output);
    for _ in 0..1 {
        scl.set(false);
        pit.busy_wait_ms(MASTER_CLOCK_SPEED, 1);
        scl.set(true);
        pit.busy_wait_ms(MASTER_CLOCK_SPEED, 1);
    }

    let scl = Pio::pc28();
    scl.set_func(Func::E); // TWI
    let sda = Pio::pc27();
    sda.set_func(Func::E); // TWI
    let twi0 = Twi::twi0();

    writeln!(console, "TWI0: initializing master").ok();
    twi0.init_master(MASTER_CLOCK_SPEED as usize, 100_000);
    writeln!(console, "TWI0 status: {:?}", twi0.status()).ok();

    let mut isc = Isc::new();
    isc.init(ISC_MASTER_CLK_DIV, ISC_MASTER_CLK_SEL, ISC_ISP_CLK_DIV);

    let mut cam_pwdn = Pio::pa30();
    cam_pwdn.set_func(Func::Gpio);
    cam_pwdn.set_direction(Direction::Output);
    cam_pwdn.set(true);
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 100);
    cam_pwdn.set(false);
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 100);

    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 10); // Wait OVM7690 T2 power-up sequence timing

    let mut camera = Ovm7690::new(unsafe { twi0.clone() });
    assert!(camera.verify_chip_id().expect("verify chip ID"), "failed to verify OVM7690 chip ID");
    camera.init().expect("init OVM7690");
    camera.enable().expect("enable OVM7690");

    let isc_dma_view_one = (unsafe { &mut ISC_DMA_VIEW_ONE } as *const _) as u32;
    let isc_dma_view_two = (unsafe { &mut ISC_DMA_VIEW_TWO } as *const _) as u32;
    let dma_control_config = DmaControlConfig {
        descriptor_enable: true,
        ..Default::default()
    };
    writeln!(console, "Configuring DMA desc addr #1: {:08x}", isc_dma_view_one).ok();
    writeln!(console, "Configuring DMA desc addr #2: {:08x}", isc_dma_view_two).ok();
    writeln!(console, "Configuring DMA fb: {:08x}", fb_cam_one).ok();

    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 1000);
    writeln!(console, "INTSR: {:?}", isc.interrupt_status()).ok();

    isc.configure(isc_dma_view_one, isc_dma_view_one, isc_dma_view_two, isc_dma_view_two, fb_cam_one as u32, &dma_control_config);
    isc.start_capture();

    unsafe { TWI0 = Some(twi0.clone()) };

    loop {
        writeln!(console, "INTSR: {:?}", isc.interrupt_status()).ok();
        armv7::asm::wfi();
    }
}

#[no_mangle]
unsafe extern "C" fn pioa_irq_handler() {

}

#[no_mangle]
unsafe extern "C" fn aic_spurious_handler() {
    core::arch::asm!("bkpt");
}

#[no_mangle]
unsafe extern "C" fn uart_irq_handler() {
    let mut uart = UartType::new();
    let char = uart.getc() as char;
    writeln!(uart, "Received character: {}", char).ok();
}

#[no_mangle]
unsafe extern "C" fn tc0_irq_handler() {
    let tc0 = Tc::new();

    let status = tc0.status();
    if status != 0 {
        HAD_TC0_IRQ.store(true, Relaxed);
    }
}

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    let mut console = Uart::<Uart1>::new();

    compiler_fence(SeqCst);
    writeln!(console, "{}", _info).ok();

    loop {
        unsafe {
            core::arch::asm!("bkpt");
        }
    }
}

fn configure_isc_pins() {
    // Assign from PC13 to PC24 to func C which is ISC
    // PioC::configure_pins_by_mask(None, 0x1ffe000, Func::C, None);
}

fn configure_lcdc_pins() {
    let mut pit = Pit::new();
    pit.set_interval(PIV_MAX);
    pit.set_enabled(true);

    // PB1: reset LCD panel
    let mut pio = Pio::pb1();
    pio.set_func(Func::Gpio);
    pio.set_direction(Direction::Output);
    pio.set(false);
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 100);
    pio.set(true);
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 100);

    let mosi = Pio::pa15();
    mosi.set_func(Func::A); // SPI0_MOSI
    let sck = Pio::pa14();
    sck.set_func(Func::A); // SPI0_SPCK
    let cs = Pio::pa19();
    cs.set_func(Func::A); // SPI0_NPCS0

    let mut lcdspi = LcdSpi::new(Spi::spi0(), ChipSelect::Cs2, MASTER_CLOCK_SPEED, pit);
    lcdspi.run_init_sequence();

    // PB11 - PB31
    PioB::configure_pins_by_mask(None, 0xFFFFF800, Func::A, None);
    PioB::clear_all(None);

    // PC0 - PC8
    PioC::configure_pins_by_mask(None, 0x1ff, Func::A, None);
}
