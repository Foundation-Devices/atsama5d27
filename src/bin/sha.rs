// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

use {
    atsama5d27::{
        aic::{Aic, InterruptEntry, SourceKind},
        pit::{Pit, PIV_MAX},
        pmc::{PeripheralId, Pmc},
        sha::Sha,
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
    pmc.enable_peripheral_clock(PeripheralId::Sha);

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

    let sha = Sha::new();
    sha.reset();

    sha256_tests(&sha);
    sha256_test_file(&sha);

    loop {
        armv7::asm::wfi();
    }
}

fn sha256_tests(sha: &Sha) {
    let mut uart = UartType::new();

    #[rustfmt::skip]
    const SHA256_TESTS: [(&str, &str); 6] = [
        // Empty string
        ("", "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"),
        // abc
        ("616263", "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"),
        // The quick brown fox jumps over the lazy dog
        ("54686520717569636b2062726f776e20666f78206a756d7073206f76657220746865206c617a7920646f67",
            "d7a8fbb307d7809469ca9abcb0082e4f8d5651e46d3cdb762d02d0bf37c9e592"),
        // The quick brown fox jumps over the lazy dog.
        ("54686520717569636b2062726f776e20666f78206a756d7073206f76657220746865206c617a7920646f672e",
            "ef537f25c895bfa782526529a9b63d97aa631564d5d789c2b765448c8635fb6c"),
        ("cafebabedeadbeef", "ffe50723976ec90e97a3bcdc648ee639384dc8a2515a8b50405422eec7ff0e3e"),
        ("ff00ff00ff00ff", "311268d1c31027d1670ae3eba7b2b90e2c8b900d099346ad140e805fa9288cbf"),
    ];

    let mut temp_buf: [u8; 1024] = [0; 1024];

    for (i, (data, expected_hash)) in SHA256_TESTS.iter().enumerate() {
        writeln!(uart, "Running test #{}", i + 1).ok();

        assert_eq!(data.len() % 2, 0, "uneven data length");
        assert_eq!(expected_hash.len(), 64, "hash length is not 256 bit");

        temp_buf.fill(0);
        let len = data.len() / 2;
        if len != 0 {
            hex::decode_to_slice(data, &mut temp_buf[..len]).unwrap();
        }

        let hash = sha.sha256(&temp_buf[..len]);
        hex::encode_to_slice(hash.0, &mut temp_buf[..64]).unwrap();
        let hash_str = core::str::from_utf8(&temp_buf[..64]).unwrap();

        if hash_str != *expected_hash {
            writeln!(uart, "SHA256 test #{} failed", i + 1).ok();
            writeln!(uart, "Expected: {}", expected_hash).ok();
            writeln!(uart, "Result:   {}", hash_str).ok();
            armv7::asm::bkpt();
        } else {
            writeln!(uart, "SHA256 test #{} passed", i + 1).ok();
        }
    }
}

fn sha256_test_file(sha: &Sha) {
    let mut uart = UartType::new();
    let mut temp_buf: [u8; 64] = [0; 64];
    temp_buf.fill(0);

    let mit_bytes = include_bytes!("../../misc/mit.txt");
    const MIT_HASH: &str = "21785883212d06d2c262b7e9f73f92ae7a03a7f2fe86809d7c7ad9bd6961d265";

    writeln!(uart, "-").ok();
    writeln!(uart, "Hashing {} bytes", mit_bytes.len()).ok();
    let cb = |i, total| {
        if total == 0 {
            return;
        }
        let mut uart = UartType::new();
        writeln!(uart, "Progress: {}%", i * 100 / total).ok();
    };
    let hash = sha.sha256_cb(mit_bytes.as_slice(), 4, cb);

    hex::encode_to_slice(hash.0, &mut temp_buf[..64]).unwrap();
    let hash_str = core::str::from_utf8(&temp_buf[..64]).unwrap();

    if hash_str != MIT_HASH {
        writeln!(uart, "SHA256 file test failed").ok();
        writeln!(uart, "Expected: {}", MIT_HASH).ok();
        writeln!(uart, "Result:   {}", hash_str).ok();
    } else {
        writeln!(uart, "SHA256 file test passed").ok();
    }
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
