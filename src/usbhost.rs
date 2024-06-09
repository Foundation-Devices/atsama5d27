//! USB Host driver

use core::{
    marker::PhantomData,
    ptr::{null, null_mut},
};

use bitfield::bitfield;
use volatile_register::{RO, RW};

const UHPHS_BASE: u32 = 0x00500000;

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
    async_list: RW<QueueHeadPointer>,
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

#[derive(Debug)]
#[repr(C, align(32))]
pub struct QueueHead {
    // Hardware-managed part
    next: QueueHeadPointer,
    info1: QueueHeadInfo1,
    info2: QueueHeadInfo2,
    current_qtd: QtdPointer,
    next_qtd: QtdPointer,
    alternate_next_qtd: QtdPointer,
    token: QtdToken,
    buffers: [u32; 5],
    // Our part
    last_qtd: *mut Qtd,
}

impl QueueHead {
    pub fn new(address: u8, endpoint: u8, max_packet_len: u16) -> Self {
        let mut info1 = QueueHeadInfo1(0);
        info1.set_address(address);
        info1.set_endpoint(endpoint);
        info1.set_max_packet_length(max_packet_len);
        info1.set_endpoint_speed(2); // Hihg Speed, i.e. USB2.0
        info1.set_nak_count_reload(3);

        let mut info2 = QueueHeadInfo2(0);
        info2.set_multiplier(1);

        Self {
            next: QH_TERMINATE,
            info1,
            info2,
            current_qtd: QTD_TERMINATE,
            next_qtd: QTD_TERMINATE,
            alternate_next_qtd: QTD_TERMINATE,
            token: QtdToken(0),
            buffers: Default::default(),
            last_qtd: null_mut(),
        }
    }

    // TODO: Incredibly unsafe, lifetimes are going to suck with this
    // The struct should definitely be pinned after this.
    pub unsafe fn link_to_self(&mut self) {
        self.next = QueueHeadPointer::from_queue_head(self);
        self.info1.set_is_head(true);
    }

    // TODO: Incredibly unsafe, lifetimes are going to suck with this
    pub unsafe fn add_qtd(&mut self, qtd: &mut Qtd) {
        qtd.next = QTD_TERMINATE;
        match self.last_qtd.as_mut() {
            Some(last_qtd) => {
                last_qtd.next = QtdPointer::from_qtd(qtd);
            }
            None => {
                self.next_qtd = QtdPointer::from_qtd(qtd);
            }
        };
        self.last_qtd = qtd as *mut Qtd;
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(transparent)]
pub struct QueueHeadPointer(u32);

const QH_TERMINATE: QueueHeadPointer = QueueHeadPointer(1);

impl QueueHeadPointer {
    fn from_queue_head(qh: &QueueHead) -> Self {
        // bit 0: T=0
        // bit 2..1: Type=0b01 (QH)
        Self((qh as *const QueueHead) as u32 | 0b010)
    }
}

bitfield! {
    #[repr(C)]
    #[derive(Copy, Clone)]
    pub struct QueueHeadInfo1(u32);
    impl Debug;

    pub u8, address,set_address:6,0;
    pub inactivate_on_next,set_inactivate_on_next:7;
    pub u8, endpoint, set_endpoint: 11,8;
    pub endpoint_speed,set_endpoint_speed:13,12;
    pub data_toggle_control,set_data_toggle_control:14;
    pub is_head,set_is_head:15;
    pub u16, max_packet_length,set_max_packet_length:26,16;
    // Control endpoint ommitted, as it is for USB1.x
    pub nak_count_reload,set_nak_count_reload:31,28;

}

bitfield! {
    #[repr(C)]
    #[derive(Copy, Clone)]
    pub struct QueueHeadInfo2(u32);
    impl Debug;

    pub interrupt_schedule_mask,set_interrupt_schedule_mask:7,0;
    // USB1.x stuff and split transactions ommitted
    pub multiplier,set_multiplier:31,30;

}

#[derive(Debug)]
#[repr(C, align(32))]
pub struct Qtd {
    next: QtdPointer,
    alternate_next: QtdPointer,
    token: QtdToken,
    buffers: [u32; 5],
}

impl Qtd {
    unsafe fn new(pid: u8, mut data: &[u8]) -> Self {
        let mut token = QtdToken(0);
        token.set_pid(pid);
        token.set_total_bytes(data.len() as u16);
        token.set_error_counter(3);
        token.set_active(true);
        let mut buffers: [u32; 5] = Default::default();
        if !data.is_empty() {
            for b in &mut buffers {
                *b = data.as_ptr() as u32;
                let offset = *b as usize & 0xFFF;
                let len_to_subtract = 0x1000 - offset;
                if len_to_subtract >= data.len() {
                    break;
                }
                data = &data[len_to_subtract..];
            }
        }

        Self {
            next: QTD_TERMINATE,
            alternate_next: QTD_TERMINATE,
            token,
            buffers,
        }
    }

    pub unsafe fn new_out(data_buffer: &[u8]) -> Self {
        Self::new(0, data_buffer)
    }
    pub unsafe fn new_in(data_buffer: &mut [u8]) -> Self {
        Self::new(1, data_buffer)
    }
    pub unsafe fn new_setup(request: UsbRequest, data_buffer: &mut [u8; 8]) -> Self {
        *data_buffer = core::mem::transmute(request);
        Self::new(2, data_buffer)
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(transparent)]
pub struct QtdPointer(u32);

const QTD_TERMINATE: QtdPointer = QtdPointer(1);

impl QtdPointer {
    fn from_qtd(qtd: &Qtd) -> Self {
        Self((qtd as *const Qtd) as u32)
    }
}

bitfield! {
    #[repr(C)]
    #[derive(Copy, Clone)]
    pub struct QtdToken(u32);
    impl Debug;

    pub ping,set_ping:0;
    // split state and missed uframe ommitted
    pub xact_err,set_xact_err:3;
    pub babble,set_babble:4;
    pub buffer_error,set_buffer_error:5;
    pub halted,set_halted:6;
    pub active,set_active:7;
    pub u8, pid,set_pid:9,8;
    pub error_counter,set_error_counter:11,10;
    pub current_page,set_current_page:14,12;
    pub interrupt_on_complete,set_interrupt_on_complete:15;
    pub u16, total_bytes,set_total_bytes:30,16;
    pub data_toggle,set_data_toggle:31;
}

#[repr(C)]
pub struct UsbRequest {
    pub typ: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
}

pub struct UsbHost<'qh> {
    caps: *const Caps,
    opregs: *mut OpRegs,
    _qh: PhantomData<&'qh QueueHead>,
}

impl<'qh> UsbHost<'qh> {
    pub fn new() -> Result<Self, &'static str> {
        let caps = UHPHS_BASE as *const Caps;
        let caps_len = unsafe { (*caps).caplength.read() };
        if caps_len < 12 {
            return Err("Usb caps too short. Is the usb clock enabled?");
        }
        Ok(Self {
            caps,
            opregs: (UHPHS_BASE + caps_len as u32) as *mut OpRegs,
            _qh: PhantomData,
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

    pub fn auto_reset(&mut self, port: usize) {
        let mut port_status = unsafe { (*self.opregs).ports[port].read() };
        if !port_status.connected() {
            return;
        }
        if port_status.enabled() {
            port_status.set_reset(false);
        } else {
            // We should check the line status bit, and release the device if it's 1,
            // but we don't support low-speed devices, so whatever.

            // Toggle reset
            if !port_status.reset() {
                port_status.set_reset(true);
            } else {
                port_status.set_reset(false);
            }
        }
        unsafe { (*self.opregs).ports[port].write(port_status) }
    }

    pub fn is_enabled(&self, port: usize) -> bool {
        unsafe { (*self.opregs).ports[port].read().enabled() }
    }

    // TODO: 'qh lifetime on the head.
    pub fn set_async_queue_head(&mut self, qh: &mut QueueHead) {
        unsafe {
            (*self.opregs)
                .async_list
                .write(QueueHeadPointer::from_queue_head(qh));
            (*self.opregs).cmd.modify(|mut c| {
                c.set_async_schedule_enable(true);
                c
            });
        };
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
        writeln!(w, "Cmd:        {:x?}", unsafe { (*self.opregs).cmd.read() }).ok();
        writeln!(w, "Status:     {:x?}", unsafe {
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
        writeln!(w, "AList:      {:?}", unsafe {
            (*self.opregs).async_list.read()
        })
        .ok();
        writeln!(w, "Config:     {:x}", unsafe {
            (*self.opregs).config.read()
        })
        .ok();
        writeln!(w, "Ports[0]:   {:x?}", unsafe {
            (*self.opregs).ports[0].read()
        })
        .ok();
    }
}
