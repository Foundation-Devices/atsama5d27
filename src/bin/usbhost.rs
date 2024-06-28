// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

extern crate alloc;

use {
    alloc::vec::Vec,
    atsama5d27::{
        aic::{Aic, InterruptEntry, SourceKind},
        heap::init_heap,
        logging::init_logging,
        pio::{self, Pio},
        pit::Pit,
        pmc::{PeripheralId, Pmc},
        tc::Tc,
        twi::Twi,
        uart::{Uart, Uart1},
    },
    core::{
        arch::global_asm,
        cell::RefCell,
        fmt::Debug,
        panic::PanicInfo,
        ptr::addr_of_mut,
        sync::atomic::{compiler_fence, AtomicUsize, Ordering::SeqCst},
    },
    ehci::{
        descriptors::{DescriptorSet, EndpointType},
        EndpointDirection, QtdPool, QtdPoolElement, QueueHeadPool, QueueHeadPoolElement,
    },
    embedded_sdmmc::{BlockDevice, TimeSource, VolumeIdx, VolumeManager},
    log::{debug, error, info, trace, warn},
    mass_storage::{MassStorageError, MassStorageHost},
    utralib::HW_UHPHS_EHCI_BASE,
};

global_asm!(include_str!("../start.S"));

// MCK: 164MHz
// Clock frequency is divided by 2 because of the default `h32mxdiv` PMC setting
const MASTER_CLOCK_SPEED: u32 = 164000000 / 2;

type UartType = Uart<Uart1>;
const UART_PERIPH_ID: PeripheralId = PeripheralId::Uart1;

static TICK_COUNT: AtomicUsize = AtomicUsize::new(0);

static mut QH_POOL_BACKING: [QueueHeadPoolElement; 5] = unsafe { core::mem::zeroed() };
static mut QTD_POOL_BACKING: [QtdPoolElement; 10] = unsafe { core::mem::zeroed() };

// ----- Initialization functions -----

#[no_mangle]
fn _entry() -> ! {
    extern "C" {
        // These symbols come from `link.ld`
        static mut _sbss: u32;
        static mut _ebss: u32;
    }

    // Initialize RAM
    unsafe {
        r0::zero_bss(addr_of_mut!(_sbss), addr_of_mut!(_ebss));
    }

    init_heap();

    atsama5d27::l1cache::disable_dcache();

    let mut pmc = Pmc::new();
    pmc.enable_peripheral_clock(PeripheralId::Pit);
    pmc.enable_peripheral_clock(PeripheralId::Aic);
    pmc.enable_peripheral_clock(PeripheralId::Pioa);
    pmc.enable_peripheral_clock(PeripheralId::Piob);
    pmc.enable_peripheral_clock(PeripheralId::Pioc);
    pmc.enable_peripheral_clock(PeripheralId::Piod);
    pmc.enable_peripheral_clock(PeripheralId::Twi0);
    pmc.enable_peripheral_clock(PeripheralId::Uhphs);
    pmc.enable_usb_high_speed_host_clock();

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

    setup_tick_counter(&mut aic);

    let mut uart = UartType::new();
    uart.set_rx_interrupt(true);
    uart.set_rx(true);

    init_logging(uart, Some(&TICK_COUNT));
    log::set_max_level(log::LevelFilter::Info);

    // Enable interrupts
    unsafe {
        core::arch::asm!("cpsie if");
    }

    info!(" === Testing USB === ");

    init_otg_on_bq24157(init_twi0());

    test_usb();
}

fn setup_tick_counter(aic: &mut Aic) {
    let pit_irq_ptr = pit_irq_handler as unsafe extern "C" fn() as usize;
    aic.set_interrupt_handler(InterruptEntry {
        peripheral_id: PeripheralId::Pit,
        vector_fn_ptr: pit_irq_ptr,
        kind: SourceKind::LevelSensitive,
        priority: 0,
    });

    let mut pit = Pit::new();
    // Every 1 ms
    pit.set_interval(MASTER_CLOCK_SPEED / 1000 / 16);
    pit.reset();
    pit.set_interrupt(true);
    pit.set_enabled(true);
}

fn init_twi0() -> Twi {
    // Do a few clock cycles of SCL to reset all the possibly stuck slaves
    let mut scl = Pio::pc28();
    scl.set_func(pio::Func::Gpio);
    scl.set_direction(pio::Direction::Output);
    for _ in 0..1 {
        scl.set(false);
        sleep_ms(1);
        scl.set(true);
        sleep_ms(1);
    }

    let scl = Pio::pc28();
    scl.set_func(pio::Func::E); // TWI
    let sda = Pio::pc27();
    sda.set_func(pio::Func::E); // TWI
    let twi0 = Twi::twi0();

    trace!("TWI0: initializing master");
    twi0.init_master(MASTER_CLOCK_SPEED as usize, 100_000);
    trace!("TWI0 status: {:?}", twi0.status());
    twi0
}

fn init_otg_on_bq24157(twi0: Twi) {
    let mut bq = bq24157::Bq24157::new(twi0);
    assert!(bq.verify_chip_id().unwrap(), "unexpected chip ID");
    trace!("BQ24517 chip ID verified");

    debug!("Setting up OTG mode handling");
    let mut control_reg = bq.batt_voltage().unwrap();
    debug!("before: {:?}", control_reg);
    control_reg.set_otg_en(true);
    control_reg.set_otg_pl(false); // OTG_ID is low when boost is needed
    bq.set_batt_voltage(control_reg).unwrap();
    debug!("after: {:?}", bq.batt_voltage().unwrap());
}

// ----- USB handling: Transform event-driven interface to blocking ------

#[derive(Default)]
struct TransferResultListener {
    ran: bool,
    success: bool,
    data: Vec<u8>,
}

impl ehci::EventHandler<u32> for TransferResultListener {
    fn device_connected(
        &mut self,
        _controller: &mut ehci::Controller<u32>,
        _address: u8,
        _descriptor: DescriptorSet,
    ) {
        warn!("Device connected (inner)");
    }

    fn device_disconnected(&mut self, _controller: &mut ehci::Controller<u32>, _address: u8) {
        // We should still get a transfer_result callback
        info!("Device disconnected (inner)");
    }

    fn transfer_result(
        &mut self,
        _controller: &mut ehci::Controller<u32>,
        success: bool,
        data: Vec<u8>,
        _context: u32,
    ) {
        self.ran = true;
        self.success = success;
        self.data = data;
    }
}

struct BlockingUsbWrapper<'a> {
    address: u8,
    ep_in: u8,
    ep_out: u8,
    controller: &'a mut ehci::Controller<u32>,
}

impl<'a> BlockingUsbWrapper<'a> {
    pub fn wait_transfer_result(&mut self) -> Result<TransferResultListener, ehci::EhciError> {
        loop {
            let mut result = TransferResultListener::default();
            self.controller.work(TICK_COUNT.load(SeqCst), &mut result)?;
            if result.ran {
                break Ok(result);
            }
            sleep_ms(0);
        }
    }
}

// ----- USB handling: Wire to mass storage crate -----

impl<'a> mass_storage::UsbHostCommands for BlockingUsbWrapper<'a> {
    fn bulk_in(
        &mut self,
        data_len: usize,
    ) -> core::result::Result<Vec<u8>, mass_storage::UsbError> {
        self.controller
            .schedule_bulk_in(self.address, self.ep_in, data_len, 0)
            .map_err(|_| mass_storage::UsbError::Other)?;
        let result = self
            .wait_transfer_result()
            .map_err(|_| mass_storage::UsbError::Other)?;
        if result.success {
            Ok(result.data)
        } else {
            Err(mass_storage::UsbError::Stalled)
        }
    }

    fn bulk_out(&mut self, data: &[u8]) -> core::result::Result<usize, mass_storage::UsbError> {
        self.controller
            .schedule_bulk_out(self.address, self.ep_out, data, 0)
            .map_err(|_| mass_storage::UsbError::Other)?;
        let result = self
            .wait_transfer_result()
            .map_err(|_| mass_storage::UsbError::Other)?;
        if result.success {
            Ok(result.data.len())
        } else {
            Err(mass_storage::UsbError::Stalled)
        }
    }
}

// ----- USB handling: Device connection, main test logic in device_connected() ------

struct DeviceConnectionListener;

impl ehci::EventHandler<u32> for DeviceConnectionListener {
    fn device_connected(
        &mut self,
        controller: &mut ehci::Controller<u32>,
        address: u8,
        descriptor: DescriptorSet,
    ) {
        info!("Device connected");
        let Some(mass_storage) = create_mass_storage_device(controller, address, descriptor) else {
            warn!("Could not open mass storage device");
            return;
        };

        // List volume contents (if it's FAT formatted)
        if let Err(e) = list_files_on_disk(mass_storage) {
            error!("Couldn't read the FAT FS volume: {e:?}");
        }
    }

    fn device_disconnected(&mut self, _controller: &mut ehci::Controller<u32>, _address: u8) {
        info!("Device disconnected");
    }

    fn transfer_result(
        &mut self,
        _controller: &mut ehci::Controller<u32>,
        _success: bool,
        _data: Vec<u8>,
        _context: u32,
    ) {
        info!("Spurious transfer result");
    }
}

// ----- Mass storage -> embedded-emmc BlockDevice -----

struct MassStorageWrapper<'a>(RefCell<MassStorageHost<BlockingUsbWrapper<'a>>>);

impl<'a> Debug for MassStorageWrapper<'a> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MassStorageWrapper")
    }
}

impl<'a> BlockDevice for MassStorageWrapper<'a> {
    type Error = MassStorageError;

    fn read(
        &self,
        blocks: &mut [embedded_sdmmc::Block],
        start_block_idx: embedded_sdmmc::BlockIdx,
        _reason: &str,
    ) -> Result<(), Self::Error> {
        let data = self
            .0
            .borrow_mut()
            .read(start_block_idx.0 as u32, blocks.len() as u16)?;
        for (block, data_part) in blocks
            .iter_mut()
            .zip(data.chunks(embedded_sdmmc::Block::LEN))
        {
            block.contents.copy_from_slice(data_part);
        }
        Ok(())
    }

    fn write(
        &self,
        _blocks: &[embedded_sdmmc::Block],
        _start_block_idx: embedded_sdmmc::BlockIdx,
    ) -> Result<(), Self::Error> {
        return Err(MassStorageError::OtherError);
    }

    fn num_blocks(&self) -> Result<embedded_sdmmc::BlockCount, Self::Error> {
        Ok(embedded_sdmmc::BlockCount(
            self.0.borrow().block_count() as u32
        ))
    }
}

#[derive(Debug)]
struct FakeTimeSource;

impl TimeSource for FakeTimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp::from_fat(0, 0)
    }
}

// ----- USB handling: Main function -----

fn test_usb() -> ! {
    let otg_id = Pio::pa20();
    otg_id.set_func(pio::Func::Gpio);
    otg_id.set_direction(pio::Direction::Input);

    let mut bc_otg = Pio::pa29();
    bc_otg.set_func(pio::Func::Gpio);
    bc_otg.set_direction(pio::Direction::Output);
    bc_otg.set(true);

    let mut ehci_ctrl = ehci::Controller::new(
        HW_UHPHS_EHCI_BASE,
        unsafe { QueueHeadPool::new(QH_POOL_BACKING.as_mut_ptr_range(), virt_to_phys) },
        unsafe { QtdPool::new(QTD_POOL_BACKING.as_mut_ptr_range(), virt_to_phys) },
        flush_caches,
        virt_to_phys,
    )
    .unwrap();
    let mut otg_prev = false;

    loop {
        // Mirror OTG pin towards battery charger
        let otg = otg_id.get();
        bc_otg.set(otg);
        if otg != otg_prev {
            info!("OTG pin changed to {otg} (inverted)");
            otg_prev = otg;
        }

        ehci_ctrl
            .work(TICK_COUNT.load(SeqCst), &mut DeviceConnectionListener)
            .unwrap();

        sleep_ms(0);
    }
}

fn create_mass_storage_device(
    controller: &mut ehci::Controller<u32>,
    address: u8,
    descriptor: DescriptorSet,
) -> Option<MassStorageHost<BlockingUsbWrapper>> {
    let mut ep_in = 0;
    let mut ep_out = 0;
    let Some(configuration) = descriptor.configurations().next() else {
        warn!("No config descriptor");
        return None;
    };
    let Some(interface) = configuration.interfaces().find(|interface| {
        interface.interface_class == 8 /* Mass storage */
                && interface.interface_sub_class == 6
                && interface.interface_protocol == 0x50 /* Bulk only */
    }) else {
        warn!("Could not find mass storage interface");
        return None;
    };

    for endpoint in interface.endpoints() {
        if endpoint.get_endpoint_type() == Some(EndpointType::Bulk) {
            match endpoint.get_direction() {
                EndpointDirection::Out => ep_out = endpoint.get_endpoint_number(),
                EndpointDirection::In => ep_in = endpoint.get_endpoint_number(),
            }
            if let Err(e) = controller.open_endpoint(
                address,
                endpoint.get_endpoint_number(),
                endpoint.max_packet_size,
                endpoint.get_direction(),
            ) {
                warn!("Could not open endpoint {endpoint:?}: {e:?}");
                return None;
            }
        }
    }
    debug!("Using endpoints IN:{ep_in} OUT:{ep_out}");
    let wrapper = BlockingUsbWrapper {
        address,
        ep_in,
        ep_out,
        controller,
    };
    let mut mass_storage = match mass_storage::MassStorageHost::new(wrapper) {
        Ok(ms) => ms,
        Err(e) => {
            error!("Could not init mass storage device: {e:?}");
            return None;
        }
    };

    let blocks = match mass_storage.read(0, 4) {
        Ok(b) => b,
        Err(e) => {
            error!("Could not read 4 blocks from block 0: {e:?}");
            return None;
        }
    };
    debug!("First 4 blocks: {blocks:x?}");
    return Some(mass_storage);
}

fn list_files_on_disk(
    mass_storage: MassStorageHost<BlockingUsbWrapper>,
) -> Result<(), embedded_sdmmc::Error<MassStorageError>> {
    let mut volume_mgr = VolumeManager::new(
        MassStorageWrapper(RefCell::new(mass_storage)),
        FakeTimeSource,
    );
    let mut volume0 = volume_mgr.open_volume(VolumeIdx(0))?;
    let mut root_dir = volume0.open_root_dir()?;
    info!("Files in root:");
    root_dir.iterate_dir(|de| {
        if de.attributes.is_archive() {
            info!("{} {} {:?}", de.name, de.size, de.attributes)
        }
    })?;

    Ok(())
}

// ----- Interrupt handlers -----

#[no_mangle]
unsafe extern "C" fn aic_spurious_handler() {
    core::arch::asm!("bkpt");
}

#[no_mangle]
unsafe extern "C" fn uart_irq_handler() {
    let mut uart = UartType::new();
    let char = uart.getc() as char;
    info!("Received character: {}", char);
}

#[no_mangle]
unsafe extern "C" fn pit_irq_handler() {
    let mut pit = Pit::new();
    // Every 1 ms
    pit.reset();
    pit.set_enabled(true);
    TICK_COUNT.fetch_add(1, SeqCst);
}

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    compiler_fence(SeqCst);
    log::error!("{}", _info);

    loop {
        unsafe {
            core::arch::asm!("bkpt");
        }
    }
}

// ----- Utils -----

fn sleep_ms(ms: usize) {
    let ticks = TICK_COUNT.load(SeqCst);
    loop {
        armv7::asm::wfi();
        if ticks + ms <= TICK_COUNT.load(SeqCst) {
            break;
        }
    }
}

fn flush_caches(_: &[u8]) {
    /* No caches are used in this example */
}

fn virt_to_phys(p: *const u8) -> usize {
    /* No MMUs were harmed in this process */
    p as usize
}
