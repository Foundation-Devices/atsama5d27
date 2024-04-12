// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

use {
    atsama5d27::{
        adc::{Adc, AdcChannel, StartupTime},
        aic::{Aic, InterruptEntry, SourceKind},
        pio::{Direction, Func, Pio},
        pit::{Pit, PIV_MAX},
        pmc::{PeripheralId, Pmc},
        tc::Tc,
        uart::{Uart, Uart1},
    },
    core::{
        arch::global_asm,
        fmt::Write,
        panic::PanicInfo,
        sync::atomic::{compiler_fence, Ordering::SeqCst},
    },
};

global_asm!(include_str!("../start.S"));

type UartType = Uart<Uart1>;
const UART_PERIPH_ID: PeripheralId = PeripheralId::Uart1;

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

    atsama5d27::l1cache::disable_dcache();

    let mut pmc = Pmc::new();
    pmc.enable_peripheral_clock(PeripheralId::Pit);
    pmc.enable_peripheral_clock(PeripheralId::Aic);
    pmc.enable_peripheral_clock(PeripheralId::Pioa);
    pmc.enable_peripheral_clock(PeripheralId::Piob);
    pmc.enable_peripheral_clock(PeripheralId::Pioc);
    pmc.enable_peripheral_clock(PeripheralId::Piod);
    pmc.enable_peripheral_clock(PeripheralId::Adc);

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

    // Enable interrupts
    unsafe {
        core::arch::asm!("cpsie if");
    }

    // Timer for delays
    let mut pit = Pit::new();
    pit.set_interval(PIV_MAX);
    pit.set_enabled(true);
    pit.set_clock_speed(MASTER_CLOCK_SPEED);
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 500);

    let mut uart = UartType::new();
    uart.set_rx_interrupt(true);
    uart.set_rx(true);

    const NOISE_CHANNEL: AdcChannel = AdcChannel::Channel5;
    const BITS_PER_NOISE_SAMPLE: usize = 1; // original 4 bps fails more statistical tests

    let adc = Adc::new();
    adc.reset();
    adc.set_prescaler(255);
    adc.set_startup_time(StartupTime::StartupTime960);
    adc.enable_channel(NOISE_CHANNEL);

    let mut noise_enable = Pio::pa31();
    noise_enable.set_func(Func::Gpio);
    noise_enable.set_direction(Direction::Output);
    noise_enable.set(true);

    // Wait for the startup lag of the noise source
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 100);

    // The loop below generates a stream of random bits from the ADC noise source.
    // Copy generated stream of bits into https://mzsoltmolnar.github.io/random-bitstream-tester/
    // To run a set of statistical randomness tests to verify the soundness of the noise
    // source. Make sure to collect at least 1,000,000 bits or some of the tests may show
    // an error.

    loop {
        let noise_value = noise_get_u16(&adc, NOISE_CHANNEL, BITS_PER_NOISE_SAMPLE);
        write!(uart, "{:016b}", noise_value).ok();
    }
}

fn noise_get_u16(adc: &Adc, ch: AdcChannel, bits_per_noise_sample: usize) -> u16 {
    assert_ne!(
        bits_per_noise_sample, 0,
        "bits_per_noise_sample can't be zero"
    );
    let mut res = 0u16;

    for _ in 0..(16 / bits_per_noise_sample) {
        res <<= bits_per_noise_sample;
        adc.start();
        let noise = adc.read(ch) & ((1 << bits_per_noise_sample) - 1);
        res ^= noise;
    }

    res
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
