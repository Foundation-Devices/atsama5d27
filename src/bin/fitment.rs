#![no_std]
#![no_main]

use {
    atsama5d27::{
        aic::{Aic, InterruptEntry, SourceKind},
        display::FramebufDisplay,
        l2cc::{Counter, EventCounterKind, L2cc},
        lcdc::{LayerConfig, LcdDmaDesc, Lcdc, LcdcLayerId},
        lcdspi::LcdSpi,
        pio::{Direction, Event, Func, Pio, PioB, PioC, PioPort},
        pit::{Pit, PIV_MAX},
        pmc::{PeripheralId, Pmc},
        sfr::Sfr,
        spi::{ChipSelect, Spi},
        tc::Tc,
        twi::Twi,
        uart::{Uart, Uart1},
    },
    core::{
        arch::global_asm,
        fmt::Write,
        panic::PanicInfo,
        sync::atomic::{
            compiler_fence, AtomicBool,
            Ordering::{Relaxed, SeqCst},
        },
    },
    drv2605::{Drv2605, Effect},
    embedded_graphics::{
        mono_font::{ascii::FONT_9X18, MonoTextStyle},
        pixelcolor::Rgb888,
        prelude::*,
        primitives::{Circle, Line, PrimitiveStyleBuilder, Rectangle, StyledDrawable},
        text::Text,
    },
    ft3269::{Ft3269, Touch, TouchKind},
    is31fl32xx::{Is31fl32xx, OscillatorClock, PwmResolution, SoftwareShutdownMode, IS31FL3205},
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

static mut TWI0: Option<Twi> = None;
static mut HFB: Option<Drv2605<Twi>> = None;
static mut LEDS: Option<Is31fl32xx<IS31FL3205, Twi, Pio<PioC, 11>>> = None;

static mut DISPLAY: Option<FramebufDisplay> = None;

static mut BUTTONS: [Option<Button>; 9] = [None; 9];

const BRIGHTNESS_FULL: &str = "100%";
const BRIGHTNESS_75: &str = "75%";
const BRIGHTNESS_50: &str = "50%";
const BRIGHTNESS_25: &str = "25%";
const BRIGHTNESS_12_5: &str = "12.5%";

const CANVAS_X: u32 = 64;
const CANVAS_Y: u32 = 64;
const CANVAS_W: u32 = WIDTH as u32 - CANVAS_X * 2;
const CANVAS_H: u32 = HEIGHT as u32 / 2 - CANVAS_Y;
const CANVAS_BORDER_WIDTH: u32 = 4;
const PEN_SIZE: u32 = 8;
const RESET_CANVAS_X: u16 = (CANVAS_X + CANVAS_W + 8) as u16;
const RESET_CANVAS_Y: u16 = CANVAS_Y as u16;
const RESET_CANVAS_W: u16 = 32;
const RESET_CANVAS_H: u16 = 32;

static mut PEN_COLOR: Option<Rgb888> = None;

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
    pmc.enable_peripheral_clock(PeripheralId::Twi0);
    pmc.enable_peripheral_clock(PeripheralId::Spi0);

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

    let pio_irq_ptr = pioa_irq_handler as unsafe extern "C" fn() as usize;
    aic.set_interrupt_handler(InterruptEntry {
        peripheral_id: PeripheralId::Pioa,
        vector_fn_ptr: pio_irq_ptr,
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

    let dma_desc_addr_one = (unsafe { &mut DMA_DESC_ONE } as *const _) as usize;
    let fb1 = unsafe { FRAMEBUFFER_ONE.0.as_ptr() as usize };
    configure_lcdc_pins();
    pmc.enable_peripheral_clock(PeripheralId::Lcdc);
    let mut lcdc = Lcdc::new(WIDTH as u16, HEIGHT as u16);
    lcdc.init(&[LayerConfig::new(
        LcdcLayerId::Base,
        fb1,
        dma_desc_addr_one,
        dma_desc_addr_one,
    )]);
    lcdc.wait_for_sync_in_progress();
    lcdc.set_pwm_compare_value(0xff / 2);

    let mut console = uart;
    let display = FramebufDisplay::new(unsafe { &mut FRAMEBUFFER_ONE.0 }, WIDTH, HEIGHT);
    unsafe {
        DISPLAY = Some(display);
    }

    tc0.set_interrupt(true);

    // Do one clock cycle of SCL to reset all the possibly stuck slaves
    let mut scl = Pio::pc28();
    scl.set_func(Func::Gpio);
    scl.set_direction(Direction::Output);
    for _ in 0..1 {
        scl.set(false);
        for _ in 0..1000 {
            armv7::asm::nop();
        }
        scl.set(true);
    }

    let scl = Pio::pc28();
    scl.set_func(Func::E); // TWI
    let sda = Pio::pc27();
    sda.set_func(Func::E); // TWI
    let twi0 = Twi::twi0();

    writeln!(console, "TWI0: initializing master").ok();
    twi0.init_master(MASTER_CLOCK_SPEED as usize, 100_000);
    writeln!(console, "TWI0 status: {:?}", twi0.status()).ok();

    let mut hfb_en = Pio::pa21();
    hfb_en.set_func(Func::Gpio);
    hfb_en.set_direction(Direction::Output);
    hfb_en.set(true);

    let mut drv = Drv2605::new(unsafe { twi0.clone() });
    drv.init_open_loop_erm().expect("init vibration");
    drv.set_single_effect(Effect::ShortDoubleClickMediumOne100)
        .expect("set effect");
    drv.set_go(true).expect("set go");

    drv.set_single_effect(Effect::SharpClick60)
        .expect("set effect");
    unsafe {
        HFB = Some(drv);
    }

    haptic_click();

    unsafe { TWI0 = Some(twi0.clone()) };

    let mut touch_reset = Pio::pb2();
    touch_reset.set_func(Func::Gpio);
    touch_reset.set_direction(Direction::Output);
    touch_reset.set(false);
    for _ in 0..100_000 {
        armv7::asm::nop();
    }
    touch_reset.set(true);

    let touch_int = Pio::pa12();
    touch_int.set_direction(Direction::Input);
    touch_int.set_event_detection(Event::Falling);
    touch_int.set_interrupt(true);
    touch_int.set_func(Func::Gpio);
    aic.set_interrupt_enabled(PeripheralId::Pioa, true);

    let mut led_charge_pump_en = Pio::pc12();
    led_charge_pump_en.set_func(Func::Gpio);
    led_charge_pump_en.set_direction(Direction::Output);
    led_charge_pump_en.set(true); // Enable RGB LED 5v charge pump

    let led_shutdown = Pio::pc11();
    led_shutdown.set_func(Func::Gpio);
    led_shutdown.set_direction(Direction::Output);

    // Timer for delays
    let mut pit = Pit::new();
    pit.set_interval(PIV_MAX);
    pit.set_enabled(true);
    pit.set_clock_speed(MASTER_CLOCK_SPEED);
    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 500);

    // RGB LED driver
    let mut leds =
        Is31fl32xx::<IS31FL3205, _, _>::init_with_i2c(0x34, led_shutdown, unsafe { twi0.clone() });
    leds.enable_device(
        &mut pit,
        OscillatorClock::SixteenMHz,
        PwmResolution::Eightbit,
        SoftwareShutdownMode::Normal,
    )
    .expect("leds enable");
    // Set 50% LED scaling on all channels
    leds.set_all_led_scaling(0x80).expect("set led scaling");
    // Set max global current
    leds.set_global_current(0xff)
        .expect("set max global current");
    // Set default 0 brightness (turn off all LEDs)
    for ch in 0..12 {
        leds.set(ch, 0).expect("set led dark");
    }

    unsafe {
        LEDS = Some(leds);
    }

    // let ft3269 = Ft3269::new(twi0);
    // ft3269.dump_regs(&mut console);

    // let mut touches = [Touch::default(); 5];
    // ft3269.touches(&mut touches).expect("touches");

    // ft3269.set_dimensions(&Dimensions { x: 0, y: 0 }).expect("set dims");
    // ft3269.set_virt_key_pos(&Dimensions { x: 200, y: 820 }).expect("set virt key pos");
    // ft3269.set_virt_key_dimensions(&Dimensions { x: 100, y: 20 }).expect("set virt key
    // dims");

    // let dims = ft3269.dimensions().unwrap();
    // let virt_key_pos = ft3269.virt_key_pos().unwrap();
    // let virt_key_dim = ft3269.virt_key_dimensions().unwrap();
    // writeln!(console, "dims: ({}, {})", dims.x, dims.y).ok();
    // writeln!(console, "key pos: ({}, {})", virt_key_pos.x, virt_key_pos.y).ok();
    // writeln!(console, "key dim: ({}, {})", virt_key_dim.x, virt_key_dim.y).ok();

    // ft3269.dump_regs(&mut console);

    for (i, button) in [
        Button::new("", ButtonId::Led3, 64, 600, 64, 64, Rgb888::BLACK),
        Button::new("", ButtonId::Led2, 64 + 96, 600, 64, 64, Rgb888::BLACK),
        Button::new("", ButtonId::Led1, 64 + 96 * 2, 600, 64, 64, Rgb888::BLACK),
        Button::new("", ButtonId::Led0, 64 + 96 * 3, 600, 64, 64, Rgb888::BLACK),
        Button::new(
            BRIGHTNESS_50,
            ButtonId::BrightnessLed3,
            64,
            504,
            64,
            64,
            Rgb888::WHITE,
        ),
        Button::new(
            BRIGHTNESS_50,
            ButtonId::BrightnessLed2,
            64 + 96,
            504,
            64,
            64,
            Rgb888::WHITE,
        ),
        Button::new(
            BRIGHTNESS_50,
            ButtonId::BrightnessLed1,
            64 + 96 * 2,
            504,
            64,
            64,
            Rgb888::WHITE,
        ),
        Button::new(
            BRIGHTNESS_50,
            ButtonId::BrightnessLed0,
            64 + 96 * 3,
            504,
            64,
            64,
            Rgb888::WHITE,
        ),
        Button::new(
            "",
            ButtonId::ResetCanvas,
            RESET_CANVAS_X,
            RESET_CANVAS_Y,
            RESET_CANVAS_W,
            RESET_CANVAS_H,
            Rgb888::WHITE,
        ),
    ]
    .iter()
    .enumerate()
    {
        unsafe {
            BUTTONS[i] = Some(*button);
        }
    }
    unsafe {
        PEN_COLOR = Some(Rgb888::RED);
    }

    fill_display_background();
    redraw_canvas();
    loop {
        redraw_display();
        armv7::asm::wfi();
    }
}

#[no_mangle]
unsafe extern "C" fn pioa_irq_handler() {
    let ctp_irq_pin = Pio::pa12();
    if ctp_irq_pin.get_interrupt_status() {
        let mut ft3269 = Ft3269::new(unsafe { TWI0.as_ref().expect("twi0").clone() });
        let mut touch_buf: [Touch; 5] = [Touch::default(); 5];

        if ft3269.touches(&mut touch_buf).is_ok() {
            process_touches(&touch_buf);
        }
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

#[derive(Debug, Copy, Clone)]
struct Button {
    title: &'static str,
    id: ButtonId,
    x: u16,
    y: u16,
    width: u16,
    height: u16,
    color: Rgb888,
    inner: u8,
}

impl Button {
    const fn new(
        title: &'static str,
        id: ButtonId,
        x: u16,
        y: u16,
        width: u16,
        height: u16,
        color: Rgb888,
    ) -> Self {
        Self {
            title,
            id,
            x,
            y,
            width,
            height,
            color,
            inner: 0,
        }
    }

    fn is_within_button(&self, x: u16, y: u16) -> bool {
        let x_inside = x >= self.x && x < self.x + self.width;
        let y_inside = y >= self.y && y < self.y + self.height;
        x_inside && y_inside
    }

    pub fn as_rect(&self) -> Rectangle {
        Rectangle::new(
            Point::new(self.x as i32, self.y as i32),
            Size::new(self.width as u32, self.height as u32),
        )
    }
}

#[derive(Debug, Copy, Clone)]
enum ButtonId {
    Led0,
    Led1,
    Led2,
    Led3,
    BrightnessLed0,
    BrightnessLed1,
    BrightnessLed2,
    BrightnessLed3,
    ResetCanvas,
}

fn redraw_display() {
    if let Some(display) = unsafe { &mut DISPLAY } {
        for button in unsafe { BUTTONS.iter() }.flatten() {
            display
                .fill_solid(&button.as_rect(), button.color)
                .expect("draw button");
            if !button.title.is_empty() {
                let font = FONT_9X18;
                let pos = Point::new(
                    button.x as i32 + 4,
                    button.y as i32 + button.height as i32 / 2
                        - font.character_size.height as i32 / 2,
                );
                Text::new(button.title, pos, MonoTextStyle::new(&font, Rgb888::BLACK))
                    .draw(display)
                    .expect("can't draw line");
            }
        }

        let style = PrimitiveStyleBuilder::new()
            .stroke_color(Rgb888::BLACK)
            .stroke_width(2)
            .build();
        Line::new(
            Point::new(RESET_CANVAS_X as i32, RESET_CANVAS_Y as i32),
            Point::new(
                RESET_CANVAS_X as i32 + RESET_CANVAS_W as i32,
                RESET_CANVAS_Y as i32 + RESET_CANVAS_H as i32,
            ),
        )
        .draw_styled(&style, display)
        .expect("draw reset canvas cross");
        Line::new(
            Point::new(
                RESET_CANVAS_X as i32 + RESET_CANVAS_W as i32,
                RESET_CANVAS_Y as i32,
            ),
            Point::new(
                RESET_CANVAS_X as i32,
                RESET_CANVAS_Y as i32 + RESET_CANVAS_H as i32,
            ),
        )
        .draw_styled(&style, display)
        .expect("draw reset canvas cross");
    }
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

fn redraw_canvas() {
    if let Some(display) = unsafe { &mut DISPLAY } {
        Rectangle::new(
            Point::new(CANVAS_X as i32, CANVAS_Y as i32),
            Size::new(CANVAS_W, CANVAS_H),
        )
        .draw_styled(
            &PrimitiveStyleBuilder::new()
                .stroke_color(Rgb888::BLACK)
                .stroke_width(CANVAS_BORDER_WIDTH)
                .build(),
            display,
        )
        .expect("draw canvas");

        display
            .fill_solid(
                &Rectangle::new(
                    Point::new(
                        CANVAS_X as i32 + CANVAS_BORDER_WIDTH as i32,
                        CANVAS_Y as i32 + CANVAS_BORDER_WIDTH as i32,
                    ),
                    Size::new(
                        CANVAS_W - 2 * CANVAS_BORDER_WIDTH,
                        CANVAS_H - 2 * CANVAS_BORDER_WIDTH,
                    ),
                ),
                Rgb888::WHITE,
            )
            .expect("fill");
    }
}

fn process_touches(touches: &[Touch]) {
    for touch in touches {
        match touch.kind {
            TouchKind::Press => process_press(touch.x, touch.y),
            TouchKind::Drag => process_drag(touch.x, touch.y),
            TouchKind::Release => process_release(touch.x, touch.y),
            _ => (),
        }
    }
}

fn process_press(x: u16, y: u16) {
    for button in unsafe { BUTTONS.iter_mut() }.flatten() {
        if button.is_within_button(x, y) {
            haptic_click();
            process_button_pressed(button);
        }
    }

    if is_within_canvas(x, y) {
        haptic_click();
        if let Some(pen_color) = unsafe { &mut PEN_COLOR } {
            *pen_color = cycle_color(pen_color);
            if *pen_color == Rgb888::WHITE {
                // Skip white
                *pen_color = cycle_color(pen_color);
            }
            draw_canvas_pen(x, y);
        }
    }
}

fn process_button_pressed(button: &mut Button) {
    match button.id {
        ButtonId::Led0 => {
            button.color = cycle_color(&button.color);
            set_led_color(0, button.color);
        }
        ButtonId::Led1 => {
            button.color = cycle_color(&button.color);
            set_led_color(1, button.color);
        }
        ButtonId::Led2 => {
            button.color = cycle_color(&button.color);
            set_led_color(2, button.color);
        }
        ButtonId::Led3 => {
            button.color = cycle_color(&button.color);
            set_led_color(3, button.color);
        }
        ButtonId::BrightnessLed0 => {
            (button.title, button.inner) = cycle_brightness(button.inner);
            set_led_brightness(0, button.inner);
        }
        ButtonId::BrightnessLed1 => {
            (button.title, button.inner) = cycle_brightness(button.inner);
            set_led_brightness(1, button.inner);
        }
        ButtonId::BrightnessLed2 => {
            (button.title, button.inner) = cycle_brightness(button.inner);
            set_led_brightness(2, button.inner);
        }
        ButtonId::BrightnessLed3 => {
            (button.title, button.inner) = cycle_brightness(button.inner);
            set_led_brightness(3, button.inner);
        }
        ButtonId::ResetCanvas => {
            redraw_canvas();
        }
    }
}

fn cycle_color(prev_color: &Rgb888) -> Rgb888 {
    match *prev_color {
        Rgb888::BLACK => Rgb888::RED,
        Rgb888::RED => Rgb888::GREEN,
        Rgb888::GREEN => Rgb888::BLUE,
        Rgb888::BLUE => Rgb888::CSS_PURPLE,
        Rgb888::CSS_PURPLE => Rgb888::CYAN,
        Rgb888::CYAN => Rgb888::YELLOW,
        Rgb888::YELLOW => Rgb888::WHITE,
        Rgb888::WHITE => Rgb888::BLACK,

        _ => unreachable!(),
    }
}

fn is_within_canvas(x: u16, y: u16) -> bool {
    let canvas_x_min = (CANVAS_X + CANVAS_BORDER_WIDTH + PEN_SIZE) as u16;
    let canvas_x_max = (CANVAS_X + CANVAS_W - 2 * CANVAS_BORDER_WIDTH - PEN_SIZE) as u16;
    let canvas_y_min = (CANVAS_Y + CANVAS_BORDER_WIDTH + PEN_SIZE) as u16;
    let canvas_y_max = (CANVAS_Y + CANVAS_H - 2 * CANVAS_BORDER_WIDTH - PEN_SIZE) as u16;
    let x_inside = x >= canvas_x_min && x < canvas_x_max;
    let y_inside = y >= canvas_y_min && y < canvas_y_max;
    x_inside && y_inside
}

fn process_drag(x: u16, y: u16) {
    if is_within_canvas(x, y) {
        draw_canvas_pen(x, y);
    }
}

fn process_release(_x: u16, _y: u16) {}

fn draw_canvas_pen(x: u16, y: u16) {
    if let Some(display) = unsafe { &mut DISPLAY } {
        if let Some(pen_color) = unsafe { PEN_COLOR } {
            let pos = Point::new(x as i32, y as i32);
            let circle = Circle::with_center(pos, PEN_SIZE);
            let style = PrimitiveStyleBuilder::new()
                .stroke_color(pen_color)
                .fill_color(pen_color)
                .build();
            circle.draw_styled(&style, display).expect("draw pen");
        }
    }
}

fn cycle_brightness(prev_brightness: u8) -> (&'static str, u8) {
    match prev_brightness {
        0 => (BRIGHTNESS_75, 1),
        1 => (BRIGHTNESS_FULL, 2),
        2 => (BRIGHTNESS_12_5, 3),
        3 => (BRIGHTNESS_25, 4),
        4 => (BRIGHTNESS_50, 0),
        _ => unreachable!(),
    }
}

fn set_led_brightness(led_no: u8, brightness: u8) {
    let val = match brightness {
        0 => 0x80, // 50%
        1 => 0xc0, // 75%
        2 => 0xff, // 100%
        3 => 0x20, // 12.5%
        4 => 0x40, // 25%
        _ => unreachable!(),
    };

    if let Some(leds) = unsafe { &mut LEDS } {
        for ch in 0..3 {
            leds.set_led_scaling(led_no * 3 + ch, val)
                .expect("set led brightness");
        }
    }
}

fn set_led_color(led_no: u8, color: Rgb888) {
    if let Some(leds) = unsafe { &mut LEDS } {
        let r = color.r();
        let g = color.g();
        let b = color.b();

        leds.set(led_no * 3 + 2, r as u16).expect("set R");
        leds.set(led_no * 3 + 1, g as u16).expect("set R");
        leds.set(led_no * 3, b as u16).expect("set R");
    }
}

fn haptic_click() {
    if let Some(hfb) = unsafe { &mut HFB } {
        hfb.set_go(true).expect("vibration click");
    }
}
