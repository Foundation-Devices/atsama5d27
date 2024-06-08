//! USB Host driver

use bitfield::bitfield;
use volatile_register::{RO, RW};

const UHPHS_BASE: u32 = 0x00500000;

pub struct UsbHost {
    caps: *const Caps,
    opregs: *mut OpRegs,
}

#[repr(C)]
pub struct Caps {
    caplength: RO<u8>,
    _reserved: u8,
    version: RO<u16>,
    structural_params: RO<u32>,
    capability_params: RO<u32>,
}

#[repr(C)]
pub struct OpRegs {
    cmd: RW<Command>,
    status: RW<Status>,
    interrupt_enable: RW<u32>,
    frame_index: RW<u32>,
    control_data_segment: RW<u32>,
    periodic_list: RW<u32>,
    async_list: RW<u32>,
    _reserved: [u32; 9],
    config: RW<u32>,
    ports: [RW<PortControl>; 3],
}

bitfield! {
    #[repr(C)]
    #[derive(Copy, Clone)]
    pub struct Command(u32);
    impl Debug;

    pub run, set_run: 0;
    pub hc_reset, set_hc_reset: 1;
    pub periodic_schedule_enable, set_periodic_schedule_enable: 4;
    pub async_schedule_enable, set_async_schedule_enable: 5;
}

bitfield! {
    #[repr(C)]
    #[derive(Copy, Clone)]
    pub struct Status(u32);
    impl Debug;

    pub interrupt, set_interrupt: 0;
    pub error_interrupt, set_error_interrupt: 1;
    pub port_change, set_port_change: 2;
    pub frame_list_rollover, set_frame_list_rollover: 3;
    pub host_system_error, set_host_system_error: 4;
    pub interrupt_on_async_advance, set_interrupt_on_async_advance: 5;
    pub halted, _: 12;
    pub reclamation, _: 13;
    pub periodic_schedule_status, _: 14;
    pub async_schedule_status, _: 15;
}

bitfield! {
    #[repr(C)]
    #[derive(Copy, Clone)]
    pub struct PortControl(u32);
    impl Debug;

    pub  connected,_: 0;
    pub  connected_changed,set_connected_changed: 1;
    pub  enabled,set_enabled: 2;
    pub  enabled_changed,set_enabled_changed: 3;
    pub  overcurrent,_: 4;
    pub  overcurrent_changed,set_overcurrent_changed: 5;
    pub  force_resume,set_force_resume: 6;
    pub  suspend,set_suspend: 7;
    pub  reset,set_reset: 8;
    pub  line_status,_: 11, 10;
    pub  owner,set_owner: 13;
}

impl UsbHost {
    pub fn new() -> Result<Self, &'static str> {
        let caps = UHPHS_BASE as *const Caps;
        let caps_len = unsafe { (*caps).caplength.read() };
        if caps_len < 12 {
            return Err("Usb caps too short. Is the usb clock enabled?");
        }
        Ok(Self {
            caps,
            opregs: (UHPHS_BASE + caps_len as u32) as *mut OpRegs,
        })
    }

    pub fn run(&mut self) {
        unsafe {
            (*self.opregs).cmd.modify(|mut v| {
                v.set_run(true);
                v
            })
        };
    }

    pub fn configure(&mut self) {
        unsafe { (*self.opregs).config.write(1) };
    }

    pub fn debug_info<W: core::fmt::Write>(&mut self, w: &mut W) {
        writeln!(w, "SParams:    {:x}", unsafe {
            (*self.caps).structural_params.read()
        })
        .ok();
        writeln!(w, "CParams:    {:x}", unsafe {
            (*self.caps).capability_params.read()
        })
        .ok();
        writeln!(w, "Cmd:        {:#x?}", unsafe {
            (*self.opregs).cmd.read()
        })
        .ok();
        writeln!(w, "Status:     {:#x?}", unsafe {
            (*self.opregs).status.read()
        })
        .ok();
        writeln!(w, "Interrupts: {:x}", unsafe {
            (*self.opregs).interrupt_enable.read()
        })
        .ok();
        writeln!(w, "PList:      {:x}", unsafe {
            (*self.opregs).periodic_list.read()
        })
        .ok();
        writeln!(w, "AList:      {:x}", unsafe {
            (*self.opregs).async_list.read()
        })
        .ok();
        writeln!(w, "Config:     {:x}", unsafe {
            (*self.opregs).config.read()
        })
        .ok();
        writeln!(w, "Ports[0]:   {:#x?}", unsafe {
            (*self.opregs).ports[0].read()
        })
        .ok();
    }
}

// For High-speed operations, the user has to perform the following:
// Enable UHP peripheral clock in PMC_PCER.
// Write PLLCOUNT field in CKGR_UCKR. (8)
// Enable UPLL with UPLLEN bit in CKGR_UCKR.
// Wait until UTMI_PLL is locked (LOCKU bit in PMC_SR).
// Enable BIAS with BIASEN bit in CKGR_UCKR.
// Select UPLLCK as Input clock of OHCI part (USBS bit in PMC_USB register).
// Program OHCI clocks (UHP48M and UHP12M) with USBDIV field in PMC_USB register. USBDIV must be 9 (division by 10) if UPLLCK is selected.
// Enable OHCI clocks with UHP bit in PMC_SCER.
