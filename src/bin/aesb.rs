// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

use {
    atsama5d27::{
        aesb::{AesMode, Aesb},
        aic::{Aic, InterruptEntry, SourceKind},
        l2cc::Counter,
        pit::{Pit, PIV_MAX},
        pmc::{PeripheralId, Pmc},
        tc::Tc,
        trng::Trng,
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
    pmc.enable_peripheral_clock(PeripheralId::Aesb);
    pmc.enable_peripheral_clock(PeripheralId::Trng);

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
    writeln!(uart, "Running").ok();

    let trng = Trng::new().enable();

    // The IV (Initialization Vector) field of the AESB Initialization Vector register x
    // (AESB_IVRx) can be used to add a nonce in the encryption process in order to bring
    // even more security (ignored if not filled). In this case, any value encrypted with
    // a given nonce can only be decrypted with this nonce. If another nonce is set for
    // the AESB_IVRx.IV, any value encrypted with the previous nonce can no longer be
    // decrypted (see AESB Initialization Vector Register x)
    let mut nonce = [0u32; 4];
    nonce.fill_with(|| trng.read_u32());

    writeln!(uart, "AESB CTR nonce: {:08x?}", &nonce).ok();
    let aesb = Aesb::new();

    writeln!(uart, "Initializing AESB").ok();
    aesb.init(AesMode::Counter { nonce }, 0);

    const TEST_MEM_BASE: usize = 0x10000;

    /// This physical address is the `AESB` DRAM chip select.
    /// This means that accesses to this memory addresses will go through the AES to be
    /// transparently encrypted (writes) or decrypted (reads).
    const AES_CS_BASE: usize = 0x4000_0000 + TEST_MEM_BASE;

    /// This address allows the direct access to the DRAM.
    /// Accessing this memory will return the data encrypted by the `AESB` as ciphertext,
    /// and it won't be automatically decrypted which makes it unreadable.
    const DRAM_CS_BASE: usize = 0x2000_0000 + TEST_MEM_BASE;

    let aes_dram_ptr = AES_CS_BASE as *mut u32;
    let dram_ptr = DRAM_CS_BASE as *mut u32;

    // Test writing a single word
    writeln!(uart, "AESB: single word test").ok();
    unsafe {
        const TEST_VAL: u32 = 0x55555555;
        aes_dram_ptr.write_volatile(TEST_VAL);
        assert_ne!(
            dram_ptr.read_volatile(),
            aes_dram_ptr.read_volatile(),
            "AES encryption failed"
        );
        assert_eq!(
            aes_dram_ptr.read_volatile(),
            TEST_VAL,
            "AES decryption failed"
        );
    }

    // Test with an array
    writeln!(uart, "AESB: array test").ok();
    unsafe {
        const EMPTY_ARRAY: [u8; 256] = [0u8; 256];
        let aes_dram_slice =
            core::slice::from_raw_parts_mut(aes_dram_ptr as *mut u8, EMPTY_ARRAY.len());
        let dram_slice = core::slice::from_raw_parts_mut(dram_ptr as *mut u8, EMPTY_ARRAY.len());
        aes_dram_slice.copy_from_slice(&EMPTY_ARRAY);

        writeln!(uart, "AESB: {:02x?}", aes_dram_slice).ok();
        writeln!(uart, "DRAM: {:02x?}", dram_slice).ok();

        assert_ne!(aes_dram_slice, dram_slice, "AES encryption failed");
        assert_eq!(aes_dram_slice, EMPTY_ARRAY, "AES decryption failed");
    }

    writeln!(uart, "AESB: tests passed").ok();

    loop {
        armv7::asm::wfi();
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
