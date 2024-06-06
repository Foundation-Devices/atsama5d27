// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

use {
    atsama5d27::{
        aic::{Aic, InterruptEntry, SourceKind},
        pit::{Pit, PIV_MAX},
        pmc::{PeripheralId, Pmc},
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
use atsama5d27::pio::{Direction, Func, Pio};
use atsama5d27::twi::Twi;

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
    pmc.enable_peripheral_clock(PeripheralId::Twi0);

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

    let mut console = UartType::new();
    console.set_rx_interrupt(true);
    console.set_rx(true);

    // Do 8 clock cycles of SCL to reset all the possibly stuck slaves
    let mut scl = Pio::pc28();
    scl.set_func(Func::Gpio);
    scl.set_direction(Direction::Output);
    for _ in 0..1 {
        scl.set(false);
        pit.busy_wait_ms(MASTER_CLOCK_SPEED, 1);
        scl.set(true);
        pit.busy_wait_ms(MASTER_CLOCK_SPEED, 1);
    }

    let scl = Pio::pc28();
    scl.set_func(Func::E); // TWI
    let sda = Pio::pc27();
    sda.set_func(Func::E); // TWI
    let twi0 = Twi::twi0();

    writeln!(console, "TWI0: initializing master").ok();
    twi0.init_master(MASTER_CLOCK_SPEED as usize, 100_000);
    writeln!(console, "TWI0 status: {:?}", twi0.status()).ok();

    //////////////////////////////////

    let mut wpt_en1 = Pio::pa24(); // WPT_EN1
    wpt_en1.set_func(Func::Gpio);
    wpt_en1.set_direction(Direction::Output);
    wpt_en1.set(true);
    let mut wpt_en2 = Pio::pd6(); // WPT_EN2
    wpt_en2.set_func(Func::Gpio);
    wpt_en2.set_direction(Direction::Output);
    wpt_en2.set(false);

    let mut bc_cd = Pio::pd20(); // BC_CD is battery charger disable pin
    bc_cd.set_func(Func::Gpio);
    bc_cd.set_direction(Direction::Output);

    let mut bc_otg = Pio::pa29(); // BC_OTG is battery charger disable pin
    bc_otg.set_func(Func::Gpio);
    bc_otg.set_direction(Direction::Output);
    bc_otg.set(false); // Disable OTG (boost) mode

    //////////////////////////////////

    let mut bq = bq24157::Bq24157::new(twi0);
    assert!(bq.verify_chip_id().unwrap(), "unexpected chip ID");
    writeln!(console, "BQ24517 chip ID verified").ok();

    //////////////////////////////////
    let status = bq.status().unwrap();
    writeln!(console, "fault chg: {:?}", status.charge_fault()).ok();

    let safety = bq.safety_limits().unwrap();
    writeln!(console, "Safety limits: {:?}", safety).ok();
    const V_CURR_SENSE: u8 = 0b1111;
    const VR_MAX: u8 = 0b1111;
    let mut safety = bq.safety_limits().unwrap();
    safety.set_v_curr_sense(V_CURR_SENSE);
    safety.set_vr_max(VR_MAX);
    writeln!(console, "Setting safety limits: {:?}", safety).ok();
    bq.set_safety_limits(safety).unwrap();
    writeln!(console, "Safety limits initialized: {:?}", bq.safety_limits()).ok();
    // assert_eq!(bq.safety_limits().unwrap().v_curr_sense(), V_CURR_SENSE, "safety is locked");

    //////////////////////////////////

    let status = bq.status().unwrap();
    writeln!(console, "fault chg: {:?}", status.charge_fault()).ok();

    writeln!(console, "Resetting charger").ok();
    bq.reset_charger().unwrap();

    //////////////////////////////////
    let status = bq.status().unwrap();
    writeln!(console, "fault chg: {:?}", status.charge_fault()).ok();

    writeln!(console, "Setting input current limit to 500 mA and enabling TE").ok();
    let mut control = bq.control().unwrap();
    writeln!(console, "before: {:?}", control).ok();
    control.set_te(true);
    control.set_i_lim(0b01);
    bq.set_control(control).unwrap();
    writeln!(console, "after: {:?}", bq.control().unwrap()).ok();

    //////////////////////////////////
    let status = bq.status().unwrap();
    writeln!(console, "fault chg: {:?}", status.charge_fault()).ok();

    writeln!(console, "Setting batt regulated voltage to 4.45V").ok();
    let mut batt_voltage = bq.batt_voltage().unwrap();
    writeln!(console, "before: {:?}", batt_voltage).ok();
    batt_voltage.set_bat_vreg(0b101111); // 3.5V + 0b101111 * 0.020mV = 4.44V (closest to 4.45V)
    bq.set_batt_voltage(batt_voltage).unwrap();
    writeln!(console, "after: {:?}", bq.batt_voltage().unwrap()).ok();

    //////////////////////////////////
    let status = bq.status().unwrap();
    writeln!(console, "fault chg: {:?}", status.charge_fault()).ok();

    writeln!(console, "Disabling low current").ok();
    let mut special_charger_voltage = bq.special_charger_voltage().unwrap();
    writeln!(console, "before: {:?}", special_charger_voltage).ok();

    special_charger_voltage.set_low_chg(false);
    bq.set_special_charger_voltage(special_charger_voltage).unwrap();
    writeln!(console, "after: {:?}", bq.special_charger_voltage().unwrap()).ok();

    //////////////////////////////////
    let status = bq.status().unwrap();
    writeln!(console, "fault chg: {:?}", status.charge_fault()).ok();

    writeln!(console, "Setting charger current to 550 mA with termination current at 100 mA").ok();
    let mut chg_curr = bq.charger_current().unwrap();
    writeln!(console, "before: {:?}", chg_curr).ok();
    chg_curr.set_v_iterm(0b011); // 50mA + 1 * 50mA + 0*100mA + 0*200mA = 100mA
    chg_curr.set_chr_curr_sense_v(0b000); // 550 mA
    bq.set_charge_current(chg_curr).unwrap();
    writeln!(console, "after: {:?}", bq.charger_current().unwrap()).ok();

    writeln!(console, "Setting CD to LOW to enable charging").ok();
    bc_cd.set(false);

    loop {
        let status = bq.status().unwrap();
        writeln!(console, "{:?}", status).ok();
        writeln!(console, "{:?}", status.state().unwrap()).ok();
        writeln!(console, "fault chg: {:?}", status.charge_fault().unwrap()).ok();
        writeln!(console, "fault boost: {:?}", status.boost_fault().unwrap()).ok();

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
