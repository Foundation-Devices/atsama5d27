//! USB Host driver

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
    cmd: RW<u32>,
    status: RW<u32>,
    interrupt_enable: RW<u32>,
    frame_index: RW<u32>,
    control_data_segment: RW<u32>,
    periodic_list: RW<u32>,
    async_list: RW<u32>,
    _reserved: [u32; 9],
    config: RW<u32>,
    ports: [RW<u32>; 3],
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
        unsafe { (*self.opregs).cmd.modify(|v| v | 1) };
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
        writeln!(w, "Cmd:        {:x}", unsafe { (*self.opregs).cmd.read() }).ok();
        writeln!(w, "Status:     {:x}", unsafe {
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
        writeln!(w, "Ports[0]:   {:x}", unsafe {
            (*self.opregs).ports[0].read()
        })
        .ok();
        writeln!(w, "Ports[1]:   {:x}", unsafe {
            (*self.opregs).ports[1].read()
        })
        .ok();
        writeln!(w, "Ports[2]:   {:x}", unsafe {
            (*self.opregs).ports[2].read()
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
