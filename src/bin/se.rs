#![no_std]
#![no_main]

use atsama5d27::lcdc::{LcdcLayerId, Lcdc, LayerConfig};

use {
    atsama5d27::{
        display::FramebufDisplay,
        l2cc::{Counter, EventCounterKind, L2cc},
        lcdc::LcdDmaDesc,
        lcdspi::LcdSpi,
        pio::{Direction, Func, Pio, PioB, PioC, PioPort},
        pit::{Pit, PIV_MAX},
        pmc::{PeripheralId, Pmc},
        se_port,
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
    cryptoauthlib::{
        g_atcab_device_ptr,
        ATCAIface,
        ATCAIfaceCfg,
        ATCAIfaceCfg__bindgen_ty_1,
        ATCAIfaceCfg__bindgen_ty_1__bindgen_ty_2,
        ATCAIfaceType_ATCA_SWI_IFACE,
        ATCA_HAL_CHANGE_BAUD,
        ATCA_HAL_CONTROL_DESELECT,
        ATCA_HAL_CONTROL_SELECT,
        ATCA_HAL_FLUSH_BUFFER,
        ATCA_RX_TIMEOUT,
        ATCA_STATUS,
        ATCA_SUCCESS,
        ATCA_UNIMPLEMENTED,
        ATECC608B,
    },
    embedded_graphics::{pixelcolor::Rgb888, prelude::*, primitives::Rectangle},
};

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

static mut DMA_RECV: [u8; 16] = [0; 16];

const UART_RX_TIMEOUT: u32 = 100_000;

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
    pmc.enable_peripheral_clock(PeripheralId::Xdmac0);

    /*
    // Enable interrupts
    unsafe {
        core::arch::asm!("cpsie if");
    }
    */

    let mut uart = UartType::new();
    uart.set_rx_interrupt(true);
    uart.set_rx(true);

    let dma_desc_addr_one = (unsafe { &mut DMA_DESC_ONE } as *const _) as usize;
    let fb1 = unsafe { FRAMEBUFFER_ONE.0.as_ptr() as usize };
    configure_lcdc_pins();
    pmc.enable_peripheral_clock(PeripheralId::Lcdc);
    let mut lcdc = Lcdc::new(WIDTH as u16, HEIGHT as u16);
    lcdc.init(
        &[LayerConfig::new(
            LcdcLayerId::Base,
            fb1,
            dma_desc_addr_one,
            dma_desc_addr_one,
        )],
        || (),
    );
    lcdc.wait_for_sync_in_progress();
    lcdc.set_pwm_compare_value(0xff / 2);

    let mut console = uart;
    // writeln!(console, "running");

    // Timer for delays
    let mut pit = Pit::new();
    pit.set_interval(PIV_MAX);
    pit.set_enabled(true);
    pit.set_clock_speed(MASTER_CLOCK_SPEED);

    //writeln!(console, "Waiting for 30 seconds");
    //pit.busy_wait_us(MASTER_CLOCK_SPEED, 30 * 1000000);

    let uart4_tx = Pio::pb4();
    uart4_tx.set_func(Func::A);
    let uart4_rx = Pio::pb3();
    uart4_rx.set_func(Func::A);

    // writeln!(console, "here");

    const SE_BAUD: u32 = 230400;

    let mut swi_uart = Uart::<Uart4>::new();
    // swi_uart.set_baud(MASTER_CLOCK_SPEED, SE_BAUD / 2);
    swi_uart.set_parity(Parity::No);

    // Restore original baud rate and send calibration command
    swi_uart.set_baud(MASTER_CLOCK_SPEED, SE_BAUD);

    /*
    swi_uart.set_tx(true);
    swi_uart.set_rx(false);
    swi_uart.write_byte(0x00); // Wake-up call
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 3); // Closest to 2.5 ms

    swi_send(&mut swi_uart, &[0x88]);

    let mut response = [0u8; 4];
    swi_receive(&mut swi_uart, &mut response);

    let overrun = swi_uart.overrun();
    let parity_error = swi_uart.parity_error();
    let framing_error = swi_uart.framing_error();

    match response {
        [0x04, 0x11, 0x33, 0x43] => writeln!(
            console,
            "[+] Communication successful! overrun={overrun:?} parity={parity_error:?} \
             framing={framing_error:?}"
        )
        .ok(),
        [0x04, 0x07, 0xC4, 0x40] => writeln!(console, "[!] Self-test failed!").ok(),
        _ => writeln!(console, "Unexpected response").ok(),
    };*/
    // }

    // /*
    unsafe {
        g_atcab_device_ptr = core::ptr::null_mut();
        static mut CFG: ATCAIfaceCfg = ATCAIfaceCfg {
            iface_type: ATCAIfaceType_ATCA_SWI_IFACE,
            devtype: ATECC608B as u8,
            __bindgen_anon_1: ATCAIfaceCfg__bindgen_ty_1 {
                atcaswi: ATCAIfaceCfg__bindgen_ty_1__bindgen_ty_2 { address: 0, bus: 4 },
            },
            wake_delay: 2500,
            rx_retries: 10,
            cfg_data: core::ptr::null_mut(),
        };
        //writeln!(console, "calling atcab_init").ok();
        let status = cryptoauthlib::atcab_init(&mut CFG as _);

        assert_eq!(status, ATCA_SUCCESS as i32);
        let mut rev_num = [0u8; 4];

        let status = cryptoauthlib::atcab_info(rev_num.as_mut_ptr());
        writeln!(console, "atcab_info: {}, {:02x?}", status, rev_num).ok();
        assert_eq!(status, ATCA_SUCCESS as i32);
        assert_eq!(rev_num, [0x00, 0x00, 0x60, 0x03], "Not ATECC608b revision"); // verify 608b chip

        hal_delay_ms(100);

        writeln!(console, "Running self-test").ok();
        match self_test(false, false, false, true, false) {
            Ok(res) => {
                writeln!(
                    console,
                    "self-test result: {:08b} (should be all zeros)",
                    res
                )
                .ok();
            }
            Err(e) => {
                writeln!(console, "self-test failed: {}", e).ok();
                armv7::asm::bkpt();
            }
        }

        hal_delay_ms(100);

        let mut is_locked = false;
        let status = cryptoauthlib::atcab_is_config_locked(&mut is_locked as *mut bool);
        if status != 0 {
            writeln!(console, "ATECC is_config_locked() error: {}", status).ok();
            armv7::asm::bkpt();
        }
        writeln!(console, "is_config_locked: {}", is_locked).ok();

        hal_delay_ms(100);

        let data = [1, 2, 3, 4, 5, 6, 7, 8];
        let mut sha256 = [0u8; 32];
        let status =
            cryptoauthlib::atcab_hw_sha2_256(data.as_ptr(), data.len(), sha256.as_mut_ptr());
        if status != 0 {
            writeln!(console, "sha256 error: {}", status).ok();
        }
        writeln!(console, "sha256({:02x?}) -> {:02x?}", data, sha256).ok();
        assert_eq!(
            sha256,
            [
                0x66, 0x84, 0x0d, 0xda, 0x15, 0x4e, 0x8a, 0x11, 0x3c, 0x31, 0xdd, 0x0a, 0xd3, 0x2f,
                0x7f, 0x3a, 0x36, 0x6a, 0x80, 0xe8, 0x13, 0x69, 0x79, 0xd8, 0xf5, 0xa1, 0x01, 0xd3,
                0xd2, 0x9d, 0x6f, 0x72
            ],
            "invalid sha256 digest"
        );

        hal_delay_ms(100);

        let secrets = se_port::RomSecrets {
            pairing_secret: [0; 32],
            serial_number: [0; 9],
            otp_key: [0; 72],
            hash_cache_secret: [0; 32],
        };

        // TODO Some of the RomSecrets fields are unused, what's that about?
        if let Err(e) = se_port::setup_config(&secrets) {
            writeln!(console, "setup_config failed: {}", e.0).ok();
            panic!();
        } else {
            writeln!(console, "setup_config successful").ok();
        }

        match se_port::se_stretch_iter([3; 32], 10, &secrets) {
            Ok(v) => writeln!(console, "se_stretch_iter: {:02x?}", v).unwrap(),
            Err(e) => {
                writeln!(console, "se_stretch_iter failed: {}", e.0).ok();
                panic!();
            }
        }
    }
    // */
    fill_display_background();
    loop {
        armv7::asm::wfi();
    }
}

fn self_test(sha: bool, aes: bool, ecdh: bool, ecdsa: bool, rng: bool) -> Result<u8, ATCA_STATUS> {
    let mut res = [0u8; 1];
    let mut mode = 0u8;

    mode |= (sha as u8) << 5;
    mode |= (aes as u8) << 4;
    mode |= (ecdh as u8) << 3;
    mode |= (ecdsa as u8) << 2;
    mode |= rng as u8;

    let status = unsafe { cryptoauthlib::atcab_selftest(mode, 0x0000, res.as_mut_ptr()) };
    if status == 0 {
        Ok(res[0])
    } else {
        Err(status)
    }
}

static mut SWI_UART: Option<Uart<Uart4>> = None;

// TODO Test the delay functions

#[no_mangle]
extern "C" fn hal_delay_ms(delay: u32) {
    Pit::new().busy_wait_ms(MASTER_CLOCK_SPEED, delay);
}

#[no_mangle]
extern "C" fn hal_delay_us(delay: u32) {
    Pit::new().busy_wait_us(MASTER_CLOCK_SPEED, delay);
}

#[no_mangle]
extern "C" fn hal_uart_init(_iface: ATCAIface, _cfg: *mut ATCAIfaceCfg) -> ATCA_STATUS {
    let mut uart = Uart::<Uart4>::new();
    uart.set_parity(Parity::No);
    uart.set_tx(true);
    uart.set_rx(true);
    ATCA_SUCCESS as _
}

#[no_mangle]
extern "C" fn hal_uart_post_init(_iface: ATCAIface) -> ATCA_STATUS {
    ATCA_SUCCESS as _
}

#[no_mangle]
extern "C" fn hal_uart_send(
    _iface: ATCAIface,
    _word_address: u8,
    txdata: *mut u8,
    txlength: core::ffi::c_int,
) -> ATCA_STATUS {
    let mut uart = Uart::<Uart4>::new();
    uart.set_tx(true);
    uart.set_rx(true);
    let data = unsafe { core::slice::from_raw_parts(txdata, txlength as usize) };
    for byte in data {
        uart.write_byte(*byte | (1_u8 << 7));
    }
    ATCA_SUCCESS as _
}

#[no_mangle]
extern "C" fn hal_uart_receive(
    _iface: ATCAIface,
    _word_address: u8,
    rxdata: *mut u8,
    rxlength: *mut u16,
) -> ATCA_STATUS {
    let mut uart = Uart::<Uart4>::new();
    uart.set_tx(true);
    uart.set_rx(true);
    let data = unsafe { core::slice::from_raw_parts_mut(rxdata, *rxlength as usize) };
    for byte in data.iter_mut() {
        match uart.getc_timeout(UART_RX_TIMEOUT) {
            Some(c) => *byte = c & 0x7F,
            None => return ATCA_RX_TIMEOUT as _,
        }
    }
    ATCA_SUCCESS as _
}

#[no_mangle]
extern "C" fn hal_uart_control(
    _iface: ATCAIface,
    option: u8,
    param: *mut core::ffi::c_void,
    paramlen: usize,
) -> ATCA_STATUS {
    //writeln!(Uart::<Uart1>::new(), "hal_uart_control: option: {}", option);
    let mut uart = Uart::<Uart4>::new();
    match option as u32 {
        ATCA_HAL_CHANGE_BAUD => {
            let param = unsafe { &*(param as *mut u32) };
            uart.set_baud(MASTER_CLOCK_SPEED, *param);
            ATCA_SUCCESS as _
        }
        ATCA_HAL_FLUSH_BUFFER => {
            // TODO Is this sufficient? I think so
            let res = uart.getc_nonblocking();
            //writeln!(Uart::<Uart1>::new(), "Flushed buffer: {:?}", res).ok();
            ATCA_SUCCESS as _
        }
        ATCA_HAL_CONTROL_SELECT | ATCA_HAL_CONTROL_DESELECT => ATCA_SUCCESS as _,
        _ => ATCA_UNIMPLEMENTED as _,
    }
}

#[no_mangle]
extern "C" fn hal_uart_release(hal_data: *mut core::ffi::c_void) -> ATCA_STATUS {
    ATCA_SUCCESS as _
}

#[no_mangle]
extern "C" fn se_debug(s: *mut core::ffi::c_char) {
    let mut console = Uart::<Uart1>::new();
    let s = unsafe { core::ffi::CStr::from_ptr(s) };
    writeln!(console, "{}", s.to_str().unwrap()).ok();
}

#[no_mangle]
unsafe extern "C" fn aic_spurious_handler() {
    core::arch::asm!("bkpt");
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
