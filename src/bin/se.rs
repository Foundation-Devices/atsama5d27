#![no_std]
#![no_main]

use {
    atsama5d27::{
        aic::{Aic, InterruptEntry, SourceKind},
        display::FramebufDisplay,
        l2cc::{Counter, EventCounterKind, L2cc},
        lcdc::{LayerConfig, LcdDmaDesc, Lcdc, LcdcLayerId},
        lcdspi::LcdSpi,
        pio::{Direction, Func, Pio, PioB, PioC, PioPort},
        pit::{Pit, PIV_MAX},
        pmc::{PeripheralId, Pmc},
        sfr::Sfr,
        spi::{ChipSelect, Spi},
        tc::Tc,
        uart::{Parity, Uart, Uart1, Uart4},
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
    embedded_graphics::{pixelcolor::Rgb888, prelude::*, primitives::Rectangle},
};

const WIDTH: usize = 480;
const HEIGHT: usize = 800;

#[repr(align(4))]
struct Aligned4([u32; WIDTH * HEIGHT]);
static mut FRAMEBUFFER_ONE: Aligned4 = Aligned4([0; WIDTH * HEIGHT]);
static mut DMA_DESC_ONE: LcdDmaDesc = LcdDmaDesc {
    addr: 0,
    ctrl: 0,
    next: 0,
};

global_asm!(include_str!("../start.S"));

type UartType = Uart<Uart1>;
const UART_PERIPH_ID: PeripheralId = PeripheralId::Uart1;
const TC_PERIPH_ID: PeripheralId = PeripheralId::Tc0;

static HAD_TC0_IRQ: AtomicBool = AtomicBool::new(false);

// MCK: 164MHz
// Clock frequency is divided by 2 because of the default `h32mxdiv` PMC setting
const MASTER_CLOCK_SPEED: u32 = 164000000 / 2;

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

    let mut sfr = Sfr::new();
    sfr.set_l2_cache_sram_enabled(true);

    let mut l2cc = L2cc::new();
    l2cc.set_data_prefetch_enable(true);
    l2cc.set_inst_prefetch_enable(true);
    l2cc.set_double_line_fill_enable(true);
    l2cc.set_force_write_alloc(0);
    l2cc.set_prefetch_offset(1);
    l2cc.set_prefetch_drop_enable(true);
    l2cc.set_standby_mode_enable(true);
    l2cc.set_dyn_clock_gating_enable(true);
    l2cc.enable_event_counter(Counter::Counter0, EventCounterKind::IrHit);
    l2cc.set_enable(true);
    l2cc.invalidate_all();
    l2cc.cache_sync();

    let mut pmc = Pmc::new();
    pmc.enable_peripheral_clock(PeripheralId::Pit);
    pmc.enable_peripheral_clock(PeripheralId::Tc0);
    pmc.enable_peripheral_clock(PeripheralId::Aic);
    pmc.enable_peripheral_clock(PeripheralId::Pioa);
    pmc.enable_peripheral_clock(PeripheralId::Piob);
    pmc.enable_peripheral_clock(PeripheralId::Pioc);
    pmc.enable_peripheral_clock(PeripheralId::Piod);
    // pmc.enable_peripheral_clock(PeripheralId::Flexcom2);
    pmc.enable_peripheral_clock(PeripheralId::Spi0);
    pmc.enable_peripheral_clock(PeripheralId::Uart4);

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

    // Enable interrupts
    unsafe {
        core::arch::asm!("cpsie if");
    }

    let mut uart = UartType::new();
    uart.set_rx_interrupt(true);
    uart.set_rx(true);

    let dma_desc_addr_one = (unsafe { &mut DMA_DESC_ONE } as *const _) as usize;
    let fb1 = unsafe { FRAMEBUFFER_ONE.0.as_ptr() as usize };
    configure_lcdc_pins();
    pmc.enable_peripheral_clock(PeripheralId::Lcdc);
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
    lcdc.wait_for_sync_in_progress();
    lcdc.set_pwm_compare_value(0xff / 2);

    let mut console = uart;
    let display = FramebufDisplay::new(unsafe { &mut FRAMEBUFFER_ONE.0 }, WIDTH, HEIGHT);
    unsafe {
        DISPLAY = Some(display);
    }

    tc0.set_interrupt(true);

    // Timer for delays
    let mut pit = Pit::new();
    pit.set_interval(PIV_MAX);
    pit.set_enabled(true);
    pit.set_clock_speed(MASTER_CLOCK_SPEED);

    // Stabilize PD26 pin to avoid it glitching the Uart4 because I shorted them on my board
    let swi = Pio::pd26();
    swi.set_func(Func::Gpio);
    swi.set_direction(Direction::Input); // FLEXCOM2_IO0

    let uart4_tx = Pio::pb4();
    uart4_tx.set_func(Func::A);
    let uart4_rx = Pio::pb3();
    uart4_rx.set_func(Func::A);

    const SE_BAUD: u32 = 230400;

    let mut swi_uart = Uart::<Uart4>::new();
    swi_uart.set_baud(MASTER_CLOCK_SPEED, SE_BAUD / 2);
    swi_uart.set_parity(Parity::No);

    swi_uart.set_tx(true);
    swi_uart.set_rx(false);
    swi_uart.write_byte(0x00); // Wake-up call
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 3); // Closest to 2.5 ms

    // Restore original baud rate and send calibration command
    swi_uart.set_baud(MASTER_CLOCK_SPEED, SE_BAUD);
    swi_send(&mut swi_uart, &[0x88]);

    let mut response = [0u8; 4];
    swi_receive(&mut swi_uart, &mut response);
    writeln!(console, "Received the response from SE: {response:02x?}").ok();

    match response {
        [0x04, 0x11, 0x33, 0x43] => writeln!(console, "[+] Communication successful!").ok(),
        [0x04, 0x07, 0xC4, 0x40] => writeln!(console, "[!] Self-test failed!").ok(),
        _ => writeln!(console, "Unexpected response").ok(),
    };

    fill_display_background();
    loop {
        armv7::asm::wfi();
    }
}

fn swi_receive(uart: &mut Uart<Uart4>, buf: &mut [u8]) {
    uart.set_rx(true);
    uart.set_tx(false);

    for byte in buf {
        for i in 0..8 {
            let bit_mask = 1 << i;
            if swi_receive_bit(uart) {
                *byte |= bit_mask;
            }
        }
    }
}

fn swi_receive_bit(uart: &mut Uart<Uart4>) -> bool {
    // Convert to 7-bit message since we're running on 8-bit UART
    let byte = (uart.getc() >> 1) & 0x7F;
    (byte ^ 0x7F) < 2
}

fn swi_send(uart: &mut Uart<Uart4>, data: &[u8]) {
    uart.set_rx(false);
    uart.set_tx(true);

    for byte in data {
        for i in 0..8 {
            let bit_mask = 1 << i;
            let bit = byte & bit_mask != 0;
            swi_send_bit(uart, bit);
        }
    }
}

fn swi_send_bit(uart: &mut Uart<Uart4>, bit: bool) {
    let byte = if bit { 0x7F } else { 0x7D };
    uart.write_byte(byte | 1_u8 << 7);
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

// FIXME: this doesn't seem to work well with RTT
#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // TODO: disable interrupts

    #[cfg(feature = "rtt")]
    {
        if let Some(mut channel) = unsafe { UpChannel::conjure(0) } {
            channel.set_mode(ChannelMode::BlockIfFull);

            writeln!(channel, "{}", _info).ok();
        }
    }

    let mut console = Uart::<Uart1>::new();

    loop {
        compiler_fence(SeqCst);
        writeln!(console, "{}", _info).ok();
        unsafe {
            core::arch::asm!("bkpt");
        }
    }
}

#[cfg_attr(not(feature = "lcd-console"), allow(dead_code))]
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

fn fill_display_background() {
    if let Some(display) = unsafe { &mut DISPLAY } {
        display
            .fill_solid(
                &Rectangle::new(Point::new(0, 0), Size::new(WIDTH as u32, HEIGHT as u32)),
                Rgb888::CSS_GRAY,
            )
            .expect("fill");
    }
}
