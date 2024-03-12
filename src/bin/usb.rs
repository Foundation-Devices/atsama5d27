#![no_std]
#![no_main]

use {
    atsama5d27::{
        aic::{Aic, InterruptEntry, SourceKind},
        display::FramebufDisplay,
        l2cc::{Counter, EventCounterKind, L2cc},
        lcdc::{LayerConfig, LcdDmaDesc, Lcdc, LcdcLayerId},
        lcdspi::LcdSpi,
        pio::{Direction, Func, Pio, PioB, PioC, PioPort},
        pit::{Pit, PIV_MAX},
        pmc::{PeripheralId, Pmc},
        sfr::Sfr,
        spi::{ChipSelect, Spi},
        tc::Tc,
        uart::{Parity, Uart, Uart1, Uart4},
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
    embedded_graphics::{pixelcolor::Rgb888, prelude::*, primitives::Rectangle},
    usb_device::prelude::*,
    usbd_storage::{
        subclass::{
            scsi::{Scsi, ScsiCommand},
            Command,
        },
        transport::{
            bbb::{BulkOnly, BulkOnlyError},
            TransportError,
        },
    },
    utralib::CSR,
};

static mut USB_EP_MEMORY: [u32; 1024] = [0u32; 1024];
/// Not necessarily `'static`. May reside in some special memory location
static mut USB_TRANSPORT_BUF: [u8; 512] = [0u8; 512];
static mut STORAGE: [u8; (BLOCKS * BLOCK_SIZE) as usize] = [0u8; (BLOCK_SIZE * BLOCKS) as usize];

const BLOCK_SIZE: u32 = 512;
const BLOCKS: u32 = 200;
const USB_PACKET_SIZE: u16 = 64; // 8,16,32,64
const MAX_LUN: u8 = 0; // max 0x0F

const WIDTH: usize = 480;
const HEIGHT: usize = 800;

#[repr(align(4))]
struct Aligned4([u32; WIDTH * HEIGHT]);
static mut FRAMEBUFFER_ONE: Aligned4 = Aligned4([0; WIDTH * HEIGHT]);
static mut DMA_DESC_ONE: LcdDmaDesc = LcdDmaDesc {
    addr: 0,
    ctrl: 0,
    next: 0,
};

global_asm!(include_str!("../start.S"));

type UartType = Uart<Uart1>;
const UART_PERIPH_ID: PeripheralId = PeripheralId::Uart1;
const TC_PERIPH_ID: PeripheralId = PeripheralId::Tc0;

static HAD_TC0_IRQ: AtomicBool = AtomicBool::new(false);

// MCK: 164MHz
// Clock frequency is divided by 2 because of the default `h32mxdiv` PMC setting
const MASTER_CLOCK_SPEED: u32 = 164000000 / 2;

static mut DISPLAY: Option<FramebufDisplay> = None;

static mut STATE: State = State {
    storage_offset: 0,
    sense_key: None,
    sense_key_code: None,
    sense_qualifier: None,
};

#[derive(Default)]
struct State {
    storage_offset: usize,
    sense_key: Option<u8>,
    sense_key_code: Option<u8>,
    sense_qualifier: Option<u8>,
}

impl State {
    fn reset(&mut self) {
        self.storage_offset = 0;
        self.sense_key = None;
        self.sense_key_code = None;
        self.sense_qualifier = None;
    }
}

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
    // pmc.enable_peripheral_clock(PeripheralId::Flexcom2);
    pmc.enable_peripheral_clock(PeripheralId::Spi0);
    pmc.enable_peripheral_clock(PeripheralId::Uart4);

    // TODO Don't forget to do this
    pmc.enable_peripheral_clock(PeripheralId::Udphs);
    let mut pmc_csr = CSR::new(utralib::HW_PMC_BASE as *mut u32);
    pmc_csr.wfo(utralib::utra::pmc::CKGR_UCKR_UPLLEN, 1);
    pmc_csr.wfo(utralib::utra::pmc::PMC_USB_USBS, 1);

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

    // Stabilize PD26 pin to avoid it glitching the Uart4 because I shorted them on my board
    let swi = Pio::pd26();
    swi.set_func(Func::Gpio);
    swi.set_direction(Direction::Input); // FLEXCOM2_IO0

    let uart4_tx = Pio::pb4();
    uart4_tx.set_func(Func::A);
    let uart4_rx = Pio::pb3();
    uart4_rx.set_func(Func::A);

    let usb_bus = atsama5d27::usb_bus::Bus::default();
    let allocator = usb_device::bus::UsbBusAllocator::new(usb_bus);
    let mut scsi =
        usbd_storage::subclass::scsi::Scsi::new(&allocator, USB_PACKET_SIZE, MAX_LUN, unsafe {
            USB_TRANSPORT_BUF.as_mut_slice()
        })
        .unwrap();

    let mut usb_device = UsbDeviceBuilder::new(&allocator, UsbVidPid(0xabcd, 0xabcd))
        .self_powered(true)
        .build();

    loop {
        if !usb_device.poll(&mut [&mut scsi]) {
            continue;
        }

        // clear state if just configured or reset
        if matches!(usb_device.state(), UsbDeviceState::Default) {
            unsafe {
                STATE.reset();
            };
        }

        let _ = scsi.poll(|command| {
            if let Err(err) = process_command(command) {
                writeln!(console, "Error: {:?}", err).ok();
            }
        });
    }
}

fn process_command(
    mut command: Command<ScsiCommand, Scsi<BulkOnly<atsama5d27::usb_bus::Bus, &mut [u8]>>>,
) -> Result<(), TransportError<BulkOnlyError>> {
    writeln!(UartType::new(), "Handling: {:?}", command.kind).ok();

    match command.kind {
        ScsiCommand::TestUnitReady { .. } => {
            command.pass();
        }
        ScsiCommand::Inquiry { .. } => {
            command.try_write_data_all(&[
                0x00, // periph qualifier, periph device type
                0x80, // Removable
                0x04, // SPC-2 compliance
                0x02, // NormACA, HiSu, Response data format
                0x20, // 36 bytes in total
                0x00, // additional fields, none set
                0x00, // additional fields, none set
                0x00, // additional fields, none set
                b'U', b'N', b'K', b'N', b'O', b'W', b'N', b' ', // 8-byte T-10 vendor id
                b'S', b'T', b'M', b'3', b'2', b' ', b'U', b'S', b'B', b' ', b'F', b'l', b'a', b's',
                b'h', b' ', // 16-byte product identification
                b'1', b'.', b'2', b'3', // 4-byte product revision
            ])?;
            command.pass();
        }
        ScsiCommand::RequestSense { .. } => unsafe {
            command.try_write_data_all(&[
                0x70, /* RESPONSE CODE. Set to 70h for information on
                       * current errors */
                0x00, // obsolete
                STATE.sense_key.unwrap_or(0), /* Bits 3..0: SENSE KEY. Contains information
                       * describing the error. */
                0x00,
                0x00,
                0x00,
                0x00, // INFORMATION. Device-specific or command-specific information.
                0x00, // ADDITIONAL SENSE LENGTH.
                0x00,
                0x00,
                0x00,
                0x00,                               // COMMAND-SPECIFIC INFORMATION
                STATE.sense_key_code.unwrap_or(0),  // ASC
                STATE.sense_qualifier.unwrap_or(0), // ASCQ
                0x00,
                0x00,
                0x00,
                0x00,
            ])?;
            STATE.reset();
            command.pass();
        },
        ScsiCommand::ReadCapacity10 { .. } => {
            let mut data = [0u8; 8];
            let _ = &mut data[0..4].copy_from_slice(&u32::to_be_bytes(BLOCKS - 1));
            let _ = &mut data[4..8].copy_from_slice(&u32::to_be_bytes(BLOCK_SIZE));
            command.try_write_data_all(&data)?;
            command.pass();
        }
        ScsiCommand::ReadCapacity16 { .. } => {
            let mut data = [0u8; 16];
            let _ = &mut data[0..8].copy_from_slice(&u32::to_be_bytes(BLOCKS - 1));
            let _ = &mut data[8..12].copy_from_slice(&u32::to_be_bytes(BLOCK_SIZE));
            command.try_write_data_all(&data)?;
            command.pass();
        }
        ScsiCommand::ReadFormatCapacities { .. } => {
            let mut data = [0u8; 12];
            let _ = &mut data[0..4].copy_from_slice(&[
                0x00, 0x00, 0x00, 0x08, // capacity list length
            ]);
            let _ = &mut data[4..8].copy_from_slice(&u32::to_be_bytes(BLOCKS as u32)); // number of blocks
            data[8] = 0x01; //unformatted media
            let block_length_be = u32::to_be_bytes(BLOCK_SIZE);
            data[9] = block_length_be[1];
            data[10] = block_length_be[2];
            data[11] = block_length_be[3];

            command.try_write_data_all(&data)?;
            command.pass();
        }
        ScsiCommand::Read { lba, len } => unsafe {
            let lba = lba as u32;
            let len = len as u32;
            if STATE.storage_offset != (len * BLOCK_SIZE) as usize {
                let start = (BLOCK_SIZE * lba) as usize + STATE.storage_offset;
                let end = (BLOCK_SIZE * lba) as usize + (BLOCK_SIZE * len) as usize;

                // Uncomment this in order to push data in chunks smaller than a USB packet.
                // let end = min(start + USB_PACKET_SIZE as usize - 1, end);

                writeln!(
                    UartType::new(),
                    "Data transfer >>>>>>>> [{}..{}]",
                    start,
                    end
                )
                .ok();
                let count = command.write_data(&mut STORAGE[start..end])?;
                STATE.storage_offset += count;
            } else {
                command.pass();
                STATE.storage_offset = 0;
            }
        },
        ScsiCommand::Write { lba, len } => unsafe {
            let lba = lba as u32;
            let len = len as u32;
            if STATE.storage_offset != (len * BLOCK_SIZE) as usize {
                let start = (BLOCK_SIZE * lba) as usize + STATE.storage_offset;
                let end = (BLOCK_SIZE * lba) as usize + (BLOCK_SIZE * len) as usize;
                writeln!(
                    UartType::new(),
                    "Data transfer <<<<<<<< [{}..{}]",
                    start,
                    end
                )
                .ok();
                let count = command.read_data(&mut STORAGE[start..end])?;
                STATE.storage_offset += count;

                if STATE.storage_offset == (len * BLOCK_SIZE) as usize {
                    command.pass();
                    STATE.storage_offset = 0;
                }
            } else {
                command.pass();
                STATE.storage_offset = 0;
            }
        },
        ScsiCommand::ModeSense6 { .. } => {
            command.try_write_data_all(&[
                0x03, // number of bytes that follow
                0x00, // the media type is SBC
                0x00, // not write-protected, no cache-control bytes support
                0x00, // no mode-parameter block descriptors
            ])?;
            command.pass();
        }
        ScsiCommand::ModeSense10 { .. } => {
            command.try_write_data_all(&[0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])?;
            command.pass();
        }
        ref unknown_scsi_kind => {
            writeln!(
                UartType::new(),
                "Unknown SCSI command: {:?}",
                unknown_scsi_kind
            )
            .ok();
            unsafe {
                STATE.sense_key.replace(0x05); // illegal request Sense Key
                STATE.sense_key_code.replace(0x20); // Invalid command operation ASC
                STATE.sense_qualifier.replace(0x00); // Invalid command operation ASCQ
            }
            command.fail();
        }
    }

    Ok(())
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
    mosi.set_func(Func::A); // SPI0_MOSI
    let sck = Pio::pa14();
    sck.set_func(Func::A); // SPI0_SPCK
    let cs = Pio::pa19();
    cs.set_func(Func::A); // SPI0_NPCS0

    let mut lcdspi = LcdSpi::new(Spi::spi0(), ChipSelect::Cs2, MASTER_CLOCK_SPEED, pit);
    lcdspi.run_init_sequence();

    // PB11 - PB31
    PioB::configure_pins_by_mask(None, 0xFFFFF800, Func::A, None);
    PioB::clear_all(None);

    // PC0 - PC8
    PioC::configure_pins_by_mask(None, 0x1ff, Func::A, None);
}

fn fill_display_background() {
    if let Some(display) = unsafe { &mut DISPLAY } {
        display
            .fill_solid(
                &Rectangle::new(Point::new(0, 0), Size::new(WIDTH as u32, HEIGHT as u32)),
                Rgb888::CSS_GRAY,
            )
            .expect("fill");
    }
}
