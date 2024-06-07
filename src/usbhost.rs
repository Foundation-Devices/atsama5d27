//! USB Host driver

const UDPHS_BASE: u32 = 0x00500000;

pub struct UsbHost {
    base_addr: u32,
}

#[derive(Debug)]
#[repr(C, packed)]
pub struct Caps {
    caplength: u8,
    _reserved: u8,
    version: u16,
    structural_params: u32,
    capability_params: u32,
    _padding: u32,
}

impl UsbHost {
    pub fn new() -> Self {
        Self {
            base_addr: UDPHS_BASE,
        }
    }

    pub fn caps(&self) -> Caps {
        let caps_ptr = UDPHS_BASE as *const Caps;
        unsafe {caps_ptr.read_volatile() }
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
