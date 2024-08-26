// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

use {
    atsama5d27::{
        aic::{Aic, InterruptEntry, SourceKind},
        pmc::{PeripheralId, Pmc},
        rtc::Rtc,
        uart::{Uart, Uart1},
    },
    core::{
        arch::global_asm,
        fmt::Write,
        panic::PanicInfo,
        ptr::addr_of_mut,
        sync::atomic::{compiler_fence, Ordering::SeqCst},
    },
};

global_asm!(include_str!("../start.S"));

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
        r0::zero_bss(addr_of_mut!(_sbss), addr_of_mut!(_ebss));
    }

    atsama5d27::l1cache::disable_dcache();

    let mut pmc = Pmc::new();
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

    aic.set_interrupt_handler(InterruptEntry {
        peripheral_id: PeripheralId::Sys,

        vector_fn_ptr: rtc_irq_handler as unsafe extern "C" fn() as usize,
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

    let rtc = Rtc::new();
    rtc.start();
    writeln!(uart, "Current timestamp: {:?}", rtc.time()).ok();
    rtc.enable_interrupts();

    loop {
        armv7::asm::wfi();
    }
}

unsafe extern "C" fn aic_spurious_handler() {
    core::arch::asm!("bkpt");
}

unsafe extern "C" fn uart_irq_handler() {
    let mut uart = UartType::new();
    let char = uart.getc() as char;
    writeln!(uart, "Received character: {}", char).ok();
}

unsafe extern "C" fn rtc_irq_handler() {
    static mut COUNTDOWN: usize = 5;
    static mut SET_TIME: Option<u32> = None;
    let mut uart = UartType::new();
    let rtc = Rtc::new();
    unsafe {
        if COUNTDOWN > 0 {
            COUNTDOWN -= 1;
        } else {
            SET_TIME = Some(0x10000000 - 5);
            COUNTDOWN = 10;
        }
    }

    let set_time_happened = rtc.handle_interrupt(unsafe { SET_TIME });
    if set_time_happened {
        unsafe { SET_TIME = None }
    }

    writeln!(
        uart,
        "RTC interrupt. Time: {:x?}, Countdown: {}, SetTime: {:?}",
        rtc.time(),
        unsafe { COUNTDOWN },
        unsafe { SET_TIME }
    )
    .ok();
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
