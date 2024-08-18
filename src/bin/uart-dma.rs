// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! UART + DMA RX demo

#![no_std]
#![no_main]

use {
    atsama5d27::{
        aic::{Aic, InterruptEntry, SourceKind},
        dma::{
            DmaChannel,
            DmaChunkSize,
            DmaDataWidth,
            DmaPeripheralId,
            DmaTransferDirection,
            Xdmac,
        },
        pit::{Pit, PIV_MAX},
        pmc::{PeripheralId, Pmc},
        tc::Tc,
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

// MCK: 164MHz
// Clock frequency is divided by 2 because of the default `h32mxdiv` PMC setting
const MASTER_CLOCK_SPEED: u32 = 164000000 / 2;

const DMA_BUF_LEN: usize = 32;
static mut BUF: [u8; DMA_BUF_LEN] = [0; DMA_BUF_LEN];

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

    let xdmac_irq_ptr = xdmac_irq_handler as unsafe extern "C" fn() as usize;
    aic.set_interrupt_handler(InterruptEntry {
        peripheral_id: PeripheralId::Xdmac0,
        vector_fn_ptr: xdmac_irq_ptr,
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
    uart.set_rx(true);

    writeln!(uart, "Initializing DMA, buf addr: 0x{:08x}", unsafe {
        &BUF as *const _ as usize
    })
    .ok();

    let xdmac = Xdmac::xdmac0();
    let ch1 = xdmac.channel(DmaChannel::Channel1);

    loop {
        ch1.configure_peripheral_transfer(
            DmaPeripheralId::Uart1Rx,
            DmaTransferDirection::PeripheralToMemory,
            DmaDataWidth::D8,
            DmaChunkSize::C1,
        );
        ch1.set_bi_interrupt(true);
        ch1.set_interrupt(true);
        ch1.execute_transfer(
            atsama5d27::uart::UART_BASE_ADDRESS[1] + 0x18,
            unsafe { addr_of_mut!(BUF) as u32 },
            DMA_BUF_LEN,
        )
        .expect("dma");

        writeln!(uart, "gim: {:032b}", xdmac.gim()).ok();
        writeln!(uart, "gis: {:032b}", xdmac.gis()).ok();

        for _ in 0..10 {
            pit.busy_wait_ms(MASTER_CLOCK_SPEED, 1000);
            if ch1.is_transfer_complete() {
                break;
            }
        }

        ch1.software_flush();
        ch1.disable();

        writeln!(uart, "Done DMA").ok();
        writeln!(uart, "gim: {:032b}", xdmac.gim()).ok();
        writeln!(uart, "gis: {:032b}", xdmac.gis()).ok();
        let buf = unsafe { &mut BUF };
        if !buf.iter().all(|x| *x == 0) {
            writeln!(uart, "buf: {:?}", buf).ok();
            let str = if let Ok(str) = core::ffi::CStr::from_bytes_until_nul(buf) {
                str.to_str().unwrap()
            } else {
                core::str::from_utf8(buf).unwrap()
            };
            writeln!(uart, "str: {}", str).ok();
        }
        buf.fill(0);
        pit.busy_wait_ms(MASTER_CLOCK_SPEED, 1000);
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

#[no_mangle]
unsafe extern "C" fn xdmac_irq_handler() {
    let xdmac = Xdmac::xdmac0();
    let ch = xdmac.channel(DmaChannel::Channel1);
    let status = ch.interrupt_status();

    if status & 1 != 0 {
        ch.software_flush();
        ch.disable();
    }

    let mut uart = UartType::new();
    writeln!(uart, "XDMAC IRQ, status: {:032b}", status).ok();
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
