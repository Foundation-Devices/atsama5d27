#![no_std]
#![no_main]

// use atsama5d27::pit::{Pit, PIV_MAX};
#[cfg(feature = "lcd-console")]
use atsama5d27::lcdc::{LayerConfig, LcdDmaDesc, Lcdc, LcdcLayerId};
use {
    atsama5d27::{
        aic::{Aic, InterruptEntry, SourceKind},
        pio::{Direction, Func, Pio, PioB, PioC, PioPort},
        pmc::{PeripheralId, Pmc},
        tc::Tc,
        trng::Trng,
        uart::{Uart, Uart1},
    },
    core::{
        arch::global_asm,
        fmt::Write,
        panic::PanicInfo,
        sync::atomic::{
            compiler_fence,
            AtomicBool,
            Ordering::{Acquire, Relaxed, Release, SeqCst},
        },
    },
};

#[cfg(feature = "lcd-console")]
const WIDTH: usize = 480;
#[cfg(feature = "lcd-console")]
const HEIGHT: usize = 800;

#[cfg(feature = "lcd-console")]
#[repr(align(4))]
struct Aligned4([u32; WIDTH * HEIGHT]);
#[cfg(feature = "lcd-console")]
static mut FRAMEBUFFER_ONE: Aligned4 = Aligned4([0; WIDTH * HEIGHT]);
static mut DMA_DESC_ONE: LcdDmaDesc = LcdDmaDesc {
    addr: 0,
    ctrl: 0,
    next: 0,
};

#[cfg(feature = "lcd-console")]
use atsama5d27::{console::DisplayAndUartConsole, display::FramebufDisplay};
use atsama5d27::{
    l2cc::{Counter, EventCounterKind, L2cc},
    lcdspi::LcdSpi,
    pit::{Pit, PIV_MAX},
    sfr::Sfr,
};
#[cfg(feature = "rtt")]
use rtt_target::{rprintln, rtt_init_print, ChannelMode, UpChannel};

global_asm!(include_str!("start.S"));

type UartType = Uart<Uart1>;
const UART_PERIPH_ID: PeripheralId = PeripheralId::Uart1;
const TC_PERIPH_ID: PeripheralId = PeripheralId::Tc0;

static HAD_TC0_IRQ: AtomicBool = AtomicBool::new(false);

// MCK: 164MHz
// Clock frequency is divided by 2 because of the default `h32mxdiv` PMC setting
const MASTER_CLOCK_SPEED: u32 = 164000000 / 2;

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

    #[cfg(feature = "rtt")]
    {
        rtt_init_print!(BlockIfFull);
        rprintln!("");
        rprintln!("");
        rprintln!("Hello from ATSAMA5D27 & Rust");
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
    pmc.enable_peripheral_clock(PeripheralId::Trng);
    pmc.enable_peripheral_clock(PeripheralId::Pit);
    pmc.enable_peripheral_clock(PeripheralId::Tc0);
    pmc.enable_peripheral_clock(PeripheralId::Aic);
    pmc.enable_peripheral_clock(PeripheralId::Pioc);
    pmc.enable_peripheral_clock(PeripheralId::Piod);
    pmc.enable_peripheral_clock(PeripheralId::Aes);
    pmc.enable_peripheral_clock(PeripheralId::Xdmac0);

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

    let mut uart = UartType::new();
    uart.set_rx_interrupt(true);
    uart.set_rx(true);

    let dma_desc_addr_one = (unsafe { &mut DMA_DESC_ONE } as *const _) as usize;
    let fb1 = unsafe { FRAMEBUFFER_ONE.0.as_ptr() as usize };
    configure_lcdc_pins();
    pmc.enable_peripheral_clock(PeripheralId::Lcdc);
    let mut lcdc = Lcdc::new(WIDTH as u16, HEIGHT as u16);
    lcdc.init(&[LayerConfig::new(
        LcdcLayerId::Base,
        fb1,
        dma_desc_addr_one,
        dma_desc_addr_one,
    )], ||());
    lcdc.wait_for_sync_in_progress();
    lcdc.set_pwm_compare_value(0xff / 2);

    #[cfg(not(feature = "lcd-console"))]
    let mut console = uart;
    #[cfg(feature = "lcd-console")]
    let display = FramebufDisplay::new(unsafe { &mut FRAMEBUFFER_ONE.0 }, WIDTH, HEIGHT);
    #[cfg(feature = "lcd-console")]
    let mut console = DisplayAndUartConsole::new(display, uart);

    print_banner(&mut console);

    // Enable the TRNG
    let trng = Trng::new().enable();
    // Warm-up the TRNG (must wait least 5ms as per datasheet)
    // pit.busy_wait_ms(MASTER_CLOCK_SPEED, 10);

    #[cfg(feature = "rtt")]
    rprintln!("Running rng..");

    writeln!(console, "Running rng..").ok();

    let mut red_led = Pio::pa10();
    red_led.set_func(Func::Gpio);
    red_led.set_direction(Direction::Output);
    red_led.set(false); // Disable red led

    let mut blue_led_pin = Pio::pa31(); // PA31 is blue led
    let mut hi = false;

    tc0.set_interrupt(true);

    loop {
        let rnd_val = trng.read_u32();

        #[cfg(feature = "rtt")]
        rprintln!("Random bits: {:032b}", rnd_val);

        writeln!(console, "Random bits: {:032b}", rnd_val).ok();

        blue_led_pin.set(hi);

        // Random delay from 250ms to 1250ms
        let delay_ms = 250 + rnd_val % 250;
        let ticks_per_ms = MASTER_CLOCK_SPEED / 128 / 1000;

        tc0.set_period(delay_ms * ticks_per_ms);
        tc0.start();

        // Uncomment for cache benchmark
        // writeln!(console, "Starting").ok();
        // tc0.start();
        // let mut x = 0.0f32;
        // for i in 0..1_000_000 {
        //     x += i as f32 * core::f32::consts::PI;
        // }
        // tc0.stop();
        // let elapsed = tc0.counter() as f32 / ticks_per_ms as f32;
        // writeln!(console, "Elapsed {} ms: {}", elapsed, x).ok();
        // writeln!(console, "Instruction cache hits {}",
        // l2cc.get_event_count(Counter::Counter0)).ok();

        loop {
            armv7::asm::wfi();

            let had_irq = HAD_TC0_IRQ.load(Acquire);
            if had_irq {
                HAD_TC0_IRQ.store(false, Release);
                break;
            }
        }
        tc0.stop();

        hi = !hi;

        let mut data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
        let aes = atsama5d27::aes::Aes::default();
        aes.encrypt_no_dma([2; 32], [12; 16], &mut data).unwrap();
        writeln!(console, "ciphertext: {data:?}").ok();
        aes.decrypt_no_dma([2; 32], [12; 16], &mut data).unwrap();
        writeln!(console, "plaintext: {data:?}").ok();
    }
}

#[rustfmt::skip]
fn print_banner<U: Write>(console: &mut U) {
    writeln!(console, r"        ______  ____   _    _  _   _  _____         _______  _____  ____   _   _ ").ok();
    writeln!(console, r"       |  ____|/ __ \ | |  | || \ | ||  __ \    /\ |__   __||_   _|/ __ \ | \ | |").ok();
    writeln!(console, r"       | |__  | |  | || |  | ||  \| || |  | |  /  \   | |     | | | |  | ||  \| |").ok();
    writeln!(console, r"       |  __| | |  | || |  | || . ` || |  | | / /\ \  | |     | | | |  | || . ` |").ok();
    writeln!(console, r"       | |    | |__| || |__| || |\  || |__| |/ /  \ \ | |    _| |_| |__| || |\  |").ok();
    writeln!(console, r"       |_|     \____/  \____/ |_| \_||_____//_/    \_\|_|   |_____|\____/ |_| \_|").ok();
    writeln!(console).ok();
    writeln!(console, "Hello from ATSAMA5D27 & Rust").ok();
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
    mosi.set_func(Func::Gpio);
    mosi.set_direction(Direction::Output);
    let sck = Pio::pa14();
    sck.set_func(Func::Gpio);
    sck.set_direction(Direction::Output);
    let cs = Pio::pa19();
    cs.set_func(Func::Gpio);
    cs.set_direction(Direction::Output);
    let mut lcdspi = LcdSpi::new(mosi, sck, cs, MASTER_CLOCK_SPEED, pit);
    lcdspi.run_init_sequence();

    // PB11 - PB31
    PioB::configure_pins_by_mask(None, 0xFFFFF800, Func::A, None);
    PioB::clear_all(None);

    // PC0 - PC8
    PioC::configure_pins_by_mask(None, 0x1ff, Func::A, None);
}
