#![no_std]
#![no_main]

use core::arch::global_asm;
use core::fmt::Write;
use core::panic::PanicInfo;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};

use atsama5d27::pio::Pio;
use atsama5d27::pmc::{PeripheralId, Pmc};
use atsama5d27::trng::Trng;
use atsama5d27::uart::{Uart, Uart1};

use atsama5d27::aic::{Aic, InterruptEntry, SourceKind};
use atsama5d27::pit::{Pit, PIV_MAX};
#[cfg(feature = "rtt")]
use rtt_target::{rprintln, rtt_init_print, ChannelMode, UpChannel};

global_asm!(include_str!("start.S"));

type UartType = Uart<Uart1>;
const UART_PERIPH_ID: PeripheralId = PeripheralId::Uart1;

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

        rprintln!(r" ______  ____   _    _  _   _  _____         _______  _____  ____   _   _ ");
        rprintln!(r"|  ____|/ __ \ | |  | || \ | ||  __ \    /\ |__   __||_   _|/ __ \ | \ | |");
        rprintln!(r"| |__  | |  | || |  | ||  \| || |  | |  /  \   | |     | | | |  | ||  \| |");
        rprintln!(r"|  __| | |  | || |  | || . ` || |  | | / /\ \  | |     | | | |  | || . ` |");
        rprintln!(r"| |    | |__| || |__| || |\  || |__| |/ /  \ \ | |    _| |_| |__| || |\  |");
        rprintln!(r"|_|     \____/  \____/ |_| \_||_____//_/    \_\|_|   |_____|\____/ |_| \_|");

        rprintln!("");
        rprintln!("");
        rprintln!("Hello from ATSAMA5D27 & Rust");
    }

    let mut pmc = Pmc::new();
    pmc.enable_peripheral_clock(PeripheralId::Trng);
    pmc.enable_peripheral_clock(PeripheralId::Pit);
    pmc.enable_peripheral_clock(PeripheralId::Aic);

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

    let mut console = UartType::new();
    console.set_rx_interrupt(true);
    console.set_rx(true);

    writeln!(console, "Hello from ATSAMA5D27 & Rust").ok();

    let mut pit = Pit::new();
    pit.set_interval(PIV_MAX);
    pit.set_enabled(true);

    // MCK: 164MHz
    // Clock frequency is divided by 2 because of the default `h32mxdiv` PMC setting
    const MASTER_CLOCK_SPEED: u32 = 164000000 / 2;

    // Enable the TRNG
    let trng = Trng::new().enable();
    // Warm-up the TRNG (must wait least 5ms as per datasheet)
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 10);

    #[cfg(feature = "rtt")]
    rprintln!("Running rng..");

    let mut green_led_pin = Pio::pb1(); // PB1 is green/red
    let mut hi = false;

    loop {
        let rnd_val = trng.read_u32();

        #[cfg(feature = "rtt")]
        rprintln!("Random bits: {:032b}", rnd_val);

        writeln!(console, "Random bits: {:032b}", rnd_val).ok();

        green_led_pin.set(hi);

        // Random delay from 250ms to 1250ms
        let delay_ms = 250 + rnd_val % 1000;
        pit.busy_wait_ms(MASTER_CLOCK_SPEED, delay_ms);

        hi = !hi;
    }
}

#[no_mangle]
#[export_name = "aic_spurious_handler"]
unsafe extern "C" fn aic_spurious_handler() {
    core::arch::asm!("bkpt");
}

#[no_mangle]
#[export_name = "pit_irq_handler"]
unsafe extern "C" fn uart_irq_handler() {
    let mut uart = UartType::new();
    let char = uart.getc() as char;
    writeln!(uart, "Received character: {}", char).ok();
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
    writeln!(console, "{}", _info).ok();

    loop {
        compiler_fence(SeqCst);
        armv7::asm::nop();
    }
}
