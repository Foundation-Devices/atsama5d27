#![no_std]
#![no_main]

use {
    atsama5d27::{
        aic::{Aic, InterruptEntry, SourceKind},
        flexcom::{ChMode, CharLength, Flexcom, OpMode, Parity},
        l2cc::{Counter, EventCounterKind, L2cc},
        pio::{Func, Pio},
        pit::{Pit, PIV_MAX},
        pmc::{PeripheralId, Pmc},
        sfr::Sfr,
        tc::Tc,
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
};


global_asm!(include_str!("../start.S"));

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
    pmc.enable_peripheral_clock(PeripheralId::Flexcom2);
    pmc.enable_peripheral_clock(PeripheralId::Spi0);

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

    let mut console = uart;
    tc0.set_interrupt(true);

    // Timer for delays
    let mut pit = Pit::new();
    pit.set_interval(PIV_MAX);
    pit.set_enabled(true);
    pit.set_clock_speed(MASTER_CLOCK_SPEED);

    let flexcom_tx = Pio::pd26();
    flexcom_tx.set_func(Func::C);
    let flexcom_rx = Pio::pd27();
    flexcom_rx.set_func(Func::C);

    const SE_BAUD: u32 = 230400;

    let mut swi = Flexcom::flexcom2();
    swi.set_parity(Parity::No);
    swi.set_op_mode(OpMode::Usart);
    swi.set_ch_mode(ChMode::Normal);
    swi.set_char_length(CharLength::SevenBit);
    swi.set_baud(MASTER_CLOCK_SPEED, SE_BAUD / 2);

    swi.set_tx(true);
    swi.set_rx(false);
    swi.write_byte(0x00).unwrap(); // Wake-up call
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 3); // Closest to 2.5 ms

    // Restore original baud rate and send calibration command
    swi.set_baud(MASTER_CLOCK_SPEED, SE_BAUD);

    // Every time the baud rate is changed we need to reconfigure the UART too
    swi.set_parity(Parity::No);
    swi.set_op_mode(OpMode::Usart);
    swi.set_ch_mode(ChMode::Normal);
    swi.set_char_length(CharLength::SevenBit);
    swi_send(&mut swi, &[0x88]);

    let mut response = [0u8; 4];
    swi_receive(&mut swi, &mut response);
    writeln!(console, "Received the response from SE: {response:02x?}").ok();

    match response {
        [0x04, 0x11, 0x33, 0x43] => writeln!(console, "[+] Communication successful!").ok(),
        [0x04, 0x07, 0xC4, 0x40] => writeln!(console, "[!] Self-test failed!").ok(),
        _ => writeln!(console, "Unexpected response").ok(),
    };

    loop {
        armv7::asm::wfi();
    }
}

fn swi_receive(uart: &mut Flexcom, buf: &mut [u8]) {
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

fn swi_receive_bit(uart: &mut Flexcom) -> bool {
    let byte = uart.read_byte().expect("read_byte") & 0x7F;
    (byte ^ 0x7F) < 2
}

fn swi_send(swi: &mut Flexcom, data: &[u8]) {
    swi.set_rx(false);
    swi.set_tx(true);

    for byte in data {
        for i in 0..8 {
            let bit_mask = 1 << i;
            let bit = byte & bit_mask != 0;
            swi_send_bit(swi, bit);
        }
    }
}

fn swi_send_bit(swi: &mut Flexcom, bit: bool) {
    let byte = if bit { 0x7F } else { 0x7D };
    swi.write_byte(byte).expect("write_byte");
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
