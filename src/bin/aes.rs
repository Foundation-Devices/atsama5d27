// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! UART + DMA RX demo

#![no_std]
#![no_main]

use {
    atsama5d27::{
        aes::{Aes, AesMode},
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

static mut INPUT: [u8; 512] = [2; 512];
static mut OUTPUT: [u8; 512] = [0; 512];

static mut OUTPUT_HARDCODE: [u8; 512] = [
    126, 180, 135, 216, 51, 61, 130, 38, 134, 104, 173, 15, 3, 138, 123, 216, 160, 213, 112, 159,
    96, 195, 168, 145, 233, 120, 186, 215, 148, 155, 246, 206, 43, 162, 46, 27, 50, 146, 186, 186,
    233, 0, 243, 166, 195, 90, 225, 98, 166, 26, 197, 119, 223, 135, 119, 82, 92, 23, 1, 88, 1, 20,
    23, 138, 66, 10, 222, 27, 107, 135, 142, 33, 68, 250, 133, 165, 70, 146, 65, 212, 219, 251,
    173, 52, 247, 8, 175, 246, 238, 176, 76, 64, 189, 163, 16, 188, 18, 149, 84, 50, 107, 92, 0,
    93, 120, 194, 16, 69, 40, 220, 142, 124, 232, 69, 88, 231, 97, 217, 253, 41, 83, 246, 76, 224,
    202, 214, 79, 90, 252, 219, 204, 128, 234, 143, 98, 154, 86, 175, 225, 246, 196, 28, 235, 167,
    183, 177, 220, 152, 18, 45, 221, 180, 44, 88, 188, 24, 60, 116, 65, 114, 130, 151, 88, 138,
    193, 105, 233, 189, 189, 197, 212, 112, 73, 42, 196, 224, 61, 244, 46, 126, 102, 136, 103, 241,
    169, 84, 242, 236, 39, 130, 39, 254, 255, 132, 141, 113, 124, 94, 50, 214, 123, 36, 35, 147,
    38, 69, 82, 177, 120, 102, 138, 71, 194, 55, 198, 247, 227, 50, 166, 215, 221, 174, 235, 225,
    167, 21, 47, 45, 247, 101, 194, 6, 218, 243, 208, 251, 232, 204, 81, 101, 251, 93, 160, 59,
    219, 136, 241, 155, 200, 246, 0, 146, 182, 251, 72, 134, 145, 137, 164, 70, 108, 236, 151, 97,
    94, 8, 135, 242, 123, 54, 230, 213, 207, 59, 250, 139, 15, 232, 143, 182, 216, 128, 96, 65,
    155, 55, 79, 93, 222, 187, 15, 79, 188, 31, 177, 212, 159, 183, 176, 214, 43, 215, 98, 237, 42,
    20, 74, 55, 79, 186, 177, 88, 183, 45, 72, 72, 201, 139, 119, 170, 9, 250, 50, 105, 150, 125,
    159, 251, 184, 66, 230, 175, 69, 199, 146, 29, 190, 168, 252, 43, 183, 190, 31, 100, 106, 24,
    42, 173, 40, 85, 135, 109, 164, 215, 189, 108, 222, 227, 187, 251, 9, 139, 134, 79, 46, 220,
    216, 60, 105, 87, 251, 251, 137, 168, 200, 204, 34, 198, 116, 19, 168, 191, 243, 168, 135, 113,
    172, 245, 247, 205, 101, 137, 223, 147, 194, 204, 86, 176, 17, 238, 87, 195, 52, 238, 252, 195,
    205, 29, 105, 36, 114, 4, 5, 75, 19, 115, 197, 24, 77, 29, 231, 5, 72, 193, 149, 92, 124, 21,
    49, 69, 120, 105, 140, 250, 107, 203, 27, 119, 89, 190, 147, 158, 130, 72, 54, 186, 29, 140,
    202, 68, 174, 22, 126, 43, 17, 161, 58, 239, 5, 219, 78, 80, 178, 7, 85, 240, 123, 49, 230,
    210, 46, 209, 181, 88, 183, 88, 228, 119, 159, 142, 192, 112, 0, 83, 108, 12, 218, 250, 109,
    246, 12, 118, 230, 132, 178, 131, 108, 106, 240, 215, 137, 240, 23, 6, 77, 246, 183, 59, 113,
    216, 11, 106,
];

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
    pmc.enable_peripheral_clock(PeripheralId::Aes);

    let mut tc0 = Tc::new();
    tc0.init();

    let mut aic = Aic::new();
    aic.init();

    // Timer for delays
    let mut pit = Pit::new();
    pit.set_interval(PIV_MAX);
    pit.set_enabled(true);
    pit.set_clock_speed(MASTER_CLOCK_SPEED);
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 500);

    let mut uart = UartType::new();
    uart.set_rx(true);

    writeln!(uart, "Initializing DMA, output addr: 0x{:08x}", unsafe {
        &OUTPUT as *const _ as usize
    })
    .ok();

    let xdmac = Xdmac::xdmac0();
    let ch0 = xdmac.channel(DmaChannel::Channel0);
    let ch1 = xdmac.channel(DmaChannel::Channel1);

    let aes = Aes::default();
    aes.init([2; 4], [3; 4]);

    let input = unsafe { &mut INPUT };
    let output = unsafe { &mut OUTPUT };
    aes.encrypt(input, output, &ch0, &ch1);

    // Wait for DMA to finish.
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 5000);

    // TODO Disabling here might be problematic?
    ch0.software_flush();
    ch0.disable();
    ch1.software_flush();
    ch1.disable();

    writeln!(uart, "output: {:?}", output).ok();

    input.fill(0);
    writeln!(uart, "input: {:?}", input).ok();

    // Wait for the flush to finish (?).
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 5000);

    aes.decrypt(output, input, &ch0, &ch1);

    // Wait for DMA to finish.
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 5000);

    ch0.software_flush();
    ch0.disable();
    ch1.software_flush();
    ch1.disable();

    writeln!(uart, "input: {:?}", input).ok();

    loop {}
}

#[no_mangle]
unsafe extern "C" fn aic_spurious_handler() {
    core::arch::asm!("bkpt");
}

#[no_mangle]
unsafe extern "C" fn aes_irq_handler() {
    let mut uart = UartType::new();
    writeln!(uart, "aes interrupt").ok();
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
