#![no_std]
#![no_main]

use core::arch::{asm, global_asm};
use core::panic::PanicInfo;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};

use atsama5d27::pmc::{PeripheralId, Pmc};
use atsama5d27::trng::Trng;

#[cfg(feature = "rtt")]
use rtt_target::{rprintln, rtt_init_print, ChannelMode, UpChannel};

global_asm!(include_str!("start.S"));

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

        rprintln!(r" ______  ____   _    _  _   _  _____         _______  _____  ____   _   _");
        rprintln!(r"|  ____|/ __ \ | |  | || \ | ||  __ \    /\ |__   __||_   _|/ __ \ | \ | |");
        rprintln!(r"| |__  | |  | || |  | ||  \| || |  | |  /  \   | |     | | | |  | ||  \| |");
        rprintln!(r"|  __| | |  | || |  | || . ` || |  | | / /\ \  | |     | | | |  | || . ` |");
        rprintln!(r"| |    | |__| || |__| || |\  || |__| |/ /  \ \ | |    _| |_| |__| || |\  |");
        rprintln!(r"|_|     \____/  \____/ |_| \_||_____//_/    \_\|_|   |_____|\____/ |_| \_|");

        rprintln!("");
        rprintln!("");
        rprintln!("Hello from ATSAMA5D27 & Rust");
    }

    Pmc::enable_peripheral_clock(PeripheralId::Trng);
    let trng = Trng::new().enable();

    // Warm-up TRNG (must wait least 5ms per datasheet)
    for _ in 0..100_000 {
        unsafe {
            asm!("nop");
        }
    }

    #[cfg(feature = "rtt")]
    rprintln!("Running rng..");

    loop {
        let res = trng.read_u32();

        #[cfg(feature = "rtt")]
        rprintln!("Random bits: {:032b}", res);
    }
}

// FIXME: this doesn't seem to work well
#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // TODO: disable interrupts

    #[cfg(feature = "rtt")]
    {
        use core::fmt::Write;

        if let Some(mut channel) = unsafe { UpChannel::conjure(0) } {
            channel.set_mode(ChannelMode::BlockIfFull);

            writeln!(channel, "{}", _info).ok();
        }
    }

    loop {
        compiler_fence(SeqCst);
    }
}
