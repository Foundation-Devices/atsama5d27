#![no_std]
#![no_main]

use {
    atsama5d27::{
        aic::{Aic, InterruptEntry, SourceKind},
        display::FramebufDisplay,
        isc::{ClkSel, DmaBuffer, DmaControlConfig, DmaView, ISCStatus, Isc},
        lcdc::{ColorMode, LayerConfig, LcdDmaDesc, Lcdc, LcdcLayerId},
        lcdspi::LcdSpi,
        pio::{Direction, Func, Pio, PioB, PioC, PioPort},
        pit::{Pit, PIV_MAX},
        pmc::{PeripheralId, Pmc},
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
            AtomicU32,
            AtomicU8,
            Ordering::{Relaxed, SeqCst},
        },
    },
    ovm7690_rs::Ovm7690,
};

const WIDTH: usize = 480;
const HEIGHT: usize = 800;

#[repr(align(16))]
struct Aligned4<const SIZE: usize>([u32; SIZE]);

static mut FRAMEBUFFER_ONE: Aligned4<{ WIDTH * HEIGHT }> = Aligned4([0; WIDTH * HEIGHT]);
static mut DMA_DESC_ONE: LcdDmaDesc = LcdDmaDesc {
    addr: 0,
    ctrl: 0,
    next: 0,
};

const CAM_WIDTH: usize = 480;
const CAM_HEIGHT: usize = 640;

static mut FRAMEBUFFER_CAM_ONE: Aligned4<{ CAM_WIDTH * CAM_HEIGHT }> =
    Aligned4([0; CAM_WIDTH * CAM_HEIGHT]);
static mut DMA_DESC_CAM_ONE: LcdDmaDesc = LcdDmaDesc {
    addr: 0,
    ctrl: 0,
    next: 0,
};

static mut FRAMEBUFFER_CAM_TWO: Aligned4<{ CAM_WIDTH * CAM_HEIGHT }> =
    Aligned4([0; CAM_WIDTH * CAM_HEIGHT]);
static mut FRAMEBUFFER_CAM_THREE: Aligned4<{ CAM_WIDTH * CAM_HEIGHT }> =
    Aligned4([0; CAM_WIDTH * CAM_HEIGHT]);

const CAM_LAYER: LcdcLayerId = LcdcLayerId::Ovr1;

const ISC_MASTER_CLK_DIV: u8 = 18; // This gives around 30 fps
const ISC_MASTER_CLK_SEL: ClkSel = ClkSel::Hclock;
const ISC_ISP_CLK_DIV: u8 = 2;
const ISC_ISP_CLK_SEL: ClkSel = ClkSel::Hclock;

static mut ISC_DMA_VIEW_ONE: DmaView = DmaView::new();
static mut ISC_DMA_VIEW_TWO: DmaView = DmaView::new();

global_asm!(include_str!("../start.S"));

type UartType = Uart<Uart1>;
const UART_PERIPH_ID: PeripheralId = PeripheralId::Uart1;
const TC_PERIPH_ID: PeripheralId = PeripheralId::Tc0;

// MCK: 164MHz
// Clock frequency is divided by 2 because of the default `h32mxdiv` PMC setting
const MASTER_CLOCK_SPEED: u32 = 164000000 / 2;

static mut TWI0: Option<Twi> = None;

static mut DISPLAY: Option<FramebufDisplay> = None;

static NUM_FRAMES: AtomicU32 = AtomicU32::new(0);

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

    let isc_irq_handler = isc_irq_handler as unsafe extern "C" fn() as usize;
    aic.set_interrupt_handler(InterruptEntry {
        peripheral_id: PeripheralId::Isi,
        vector_fn_ptr: isc_irq_handler,
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
        FRAMEBUFFER_CAM_ONE.0.fill(0x00ff00);
        FRAMEBUFFER_CAM_TWO.0.fill(0x0000ff);
        FRAMEBUFFER_CAM_THREE.0.fill(0xff0000);
    }

    let dma_desc_cam_one = (unsafe { &mut DMA_DESC_CAM_ONE } as *const _) as usize;
    let fb_cam_one = unsafe { FRAMEBUFFER_CAM_ONE.0.as_ptr() as usize };
    let fb_cam_two = unsafe { FRAMEBUFFER_CAM_TWO.0.as_ptr() as usize };
    let fb_cam_three = unsafe { FRAMEBUFFER_CAM_THREE.0.as_ptr() as usize };

    unsafe {
        let slice = core::slice::from_raw_parts_mut(fb_cam_one as *mut u32, CAM_WIDTH * CAM_HEIGHT);
        slice.fill(0);
        let slice = core::slice::from_raw_parts_mut(fb_cam_two as *mut u32, CAM_WIDTH * CAM_HEIGHT);
        slice.fill(0);
    }

    configure_isc_pins();
    configure_lcdc_pins();
    // pmc.enable_peripheral_clock(PeripheralId::Lcdc);
    let mut lcdc = Lcdc::new(WIDTH as u16, HEIGHT as u16);
    lcdc.init(
        &[LayerConfig::new(
            LcdcLayerId::Base,
            fb1,
            dma_desc_addr_one,
            dma_desc_addr_one,
        )],
        || (),
    );

    lcdc.set_window_size(CAM_LAYER, CAM_WIDTH as u16, CAM_HEIGHT as u16);
    lcdc.set_window_pos(CAM_LAYER, 0, 0);
    lcdc.update_layer(
        &LayerConfig::new(CAM_LAYER, fb_cam_one, dma_desc_cam_one, dma_desc_cam_one),
        || (),
    );
    lcdc.enable_layer(CAM_LAYER);
    lcdc.set_channel_enable(CAM_LAYER, false);
    lcdc.set_rgb_mode_input(CAM_LAYER, ColorMode::Rgb565);
    lcdc.set_sif(CAM_LAYER, false);

    const CAMERA_BYTES_PER_PX: usize = 2;
    let img_h = CAM_WIDTH as i32;
    let img_w = CAM_HEIGHT as i32;
    let bytes_per_row = img_w * CAMERA_BYTES_PER_PX as i32;
    let bytes_per_pixel = CAMERA_BYTES_PER_PX as i32;

    // Rotate the image 90 degrees
    let _padding = 0;
    let _xstride = -(bytes_per_row * (img_h - 1));
    let _pstride = bytes_per_row - bytes_per_pixel;
    let _offset = 0;

    // Rotate  90: Down,Left -> Top,Right (with w,h swap)
    // let _pstride = 0 - (bytes_per_pixel + bytes_per_row + _padding);
    // let _xstride = (bytes_per_row + padding) * (img_h - 1);
    // let _offset = (bytes_per_row + padding) * (img_h - 1);

    // Rotate 270
    // let _pstride = bytes_per_row + padding - bytes_per_pixel;
    // let _xstride = 0 - 2 * bytes_per_pixel - (bytes_per_row + padding) * (img_h - 1);
    // let _offset = bytes_per_pixel * (img_w - 1);

    lcdc.set_pixel_stride(CAM_LAYER, _pstride);
    lcdc.set_horiz_stride(CAM_LAYER, _xstride);
    lcdc.update_overlay_attributes_enable(CAM_LAYER);
    lcdc.update_attribute(CAM_LAYER);

    lcdc.update_layer(
        &LayerConfig::new(CAM_LAYER, fb_cam_one, dma_desc_cam_one, dma_desc_cam_one),
        || (),
    );
    lcdc.set_channel_enable(CAM_LAYER, true);

    lcdc.wait_for_sync_in_progress();
    lcdc.set_pwm_compare_value(0xff / 2);

    let mut console = uart;
    let display = FramebufDisplay::new(unsafe { &mut FRAMEBUFFER_ONE.0 }, WIDTH, HEIGHT);
    unsafe {
        DISPLAY = Some(display);
    }

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
    isc.init(
        ISC_MASTER_CLK_DIV,
        ISC_MASTER_CLK_SEL,
        ISC_ISP_CLK_DIV,
        ISC_ISP_CLK_SEL,
    );

    #[cfg(feature = "camera-prod")]
    let mut cam_pwdn = Pio::pb0();
    #[cfg(feature = "camera-dev")]
    let mut cam_pwdn = Pio::pa30();

    cam_pwdn.set_func(Func::Gpio);
    cam_pwdn.set_direction(Direction::Output);
    cam_pwdn.set(true);
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 100);
    cam_pwdn.set(false);
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 100);

    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 10); // Wait OVM7690 T2 power-up sequence timing

    let mut camera = Ovm7690::new(unsafe { twi0.clone() });
    while let Err(e) = camera.verify_chip_id() {
        writeln!(console, "failed to verify OVM7690 chip ID: {:?}", e).ok();
        writeln!(console, "TWI0 status: {:?}", twi0.status()).ok();
        pit.busy_wait_ms(MASTER_CLOCK_SPEED, 1000);
    }

    camera.sw_reset().expect("software reset OVM7690");
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 500);
    camera.init().expect("init OVM7690");

    let isc_dma_view_one = (unsafe { &mut ISC_DMA_VIEW_ONE } as *const _) as u32;
    let isc_dma_view_two = (unsafe { &mut ISC_DMA_VIEW_TWO } as *const _) as u32;
    let dma_control_config = DmaControlConfig {
        descriptor_enable: true,
        ..Default::default()
    };
    writeln!(
        console,
        "Configuring DMA desc addr #1: {:08x}",
        isc_dma_view_one
    )
    .ok();
    writeln!(
        console,
        "Configuring DMA desc addr #2: {:08x}",
        isc_dma_view_two
    )
    .ok();
    writeln!(console, "Configuring DMA fb: {:08x}", fb_cam_one).ok();

    let buffers = &[
        DmaBuffer::new(isc_dma_view_one, isc_dma_view_one, fb_cam_one as u32),
        // DmaBuffer::new(isc_dma_view_two, isc_dma_view_two, fb_cam_two as u32),
    ];

    isc.enable_interrupt(ISCStatus::DDONE);
    isc.configure(false, buffers, &dma_control_config, || ());
    writeln!(console, "Status: {:?}", isc.interrupt_status()).ok();
    isc.start_capture();

    NUM_FRAMES.store(0, Relaxed);

    let mut tc0 = Tc::new();
    tc0.init();
    tc0.set_interrupt(true);
    let delay_ms = 1000;
    let ticks_per_ms = MASTER_CLOCK_SPEED / 2 / 128 / 1000;
    tc0.set_period(delay_ms * ticks_per_ms);
    tc0.start();

    VAL.store(0, Relaxed);
    REG.store(0x6f, Relaxed);

    unsafe { TWI0 = Some(twi0.clone()) };

    loop {
        // writeln!(console, "Status: {:?}", isc.interrupt_status()).ok();
        armv7::asm::wfi();
    }
}

#[no_mangle]
unsafe extern "C" fn aic_spurious_handler() {
    core::arch::asm!("bkpt");
}

static VAL: AtomicU8 = AtomicU8::new(0);
static REG: AtomicU8 = AtomicU8::new(0);

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
        writeln!(UartType::new(), "num frames: {}", NUM_FRAMES.load(Relaxed)).ok();
        NUM_FRAMES.store(0, Relaxed);
    }
}

#[no_mangle]
unsafe extern "C" fn isc_irq_handler() {
    let mut isc = Isc::new();
    let mut lcdc = Lcdc::new(WIDTH as u16, HEIGHT as u16);

    let status = isc.interrupt_status();
    if status.contains(ISCStatus::DDONE) {
        let frame_num = NUM_FRAMES.load(Relaxed) + 1;
        NUM_FRAMES.store(frame_num, Relaxed);

        let fb_one = unsafe { FRAMEBUFFER_CAM_ONE.0.as_ptr() as u32 };
        let fb_two = unsafe { FRAMEBUFFER_CAM_TWO.0.as_ptr() as u32 };
        let (fb_back, fb_front) = if frame_num % 2 == 0 {
            (fb_one, fb_two)
        } else {
            (fb_two, fb_one)
        };

        let dma_desc_cam_one = (unsafe { &mut DMA_DESC_CAM_ONE } as *const _) as usize;
        lcdc.update_layer(
            &LayerConfig::new(
                CAM_LAYER,
                fb_front as usize,
                dma_desc_cam_one,
                dma_desc_cam_one,
            ),
            || (),
        );
        let isc_dma_view_one = (unsafe { &mut ISC_DMA_VIEW_ONE } as *const _) as u32;
        isc.configure(
            false,
            &[DmaBuffer::new(isc_dma_view_one, isc_dma_view_one, fb_back)],
            &DmaControlConfig {
                descriptor_enable: true,
                ..Default::default()
            },
            || (),
        );
        while lcdc.is_add_to_queue_pending(CAM_LAYER) {}
        isc.start_capture();
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
    PioC::configure_pins_by_mask(None, 0x1ffe000, Func::C, None);
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
