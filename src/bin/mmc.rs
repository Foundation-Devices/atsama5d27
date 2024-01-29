// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

use {
    atsama5d27::{
        aic::{Aic, InterruptEntry, SourceKind},
        pit::{Pit, PIV_MAX},
        pmc::{PeripheralId, Pmc},
        sdmmc::{
            ADMADesc,
            DataDirection,
            DmaParams,
            ErrorStatus,
            NormalStatus,
            SDCmd,
            SDCmdInner,
            SDCommand,
            SDData,
            SDRespType,
            SdMmcError,
            Sdmmc,
        },
        tc::Tc,
        uart::{Uart, Uart1},
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
    embedded_sdmmc::{
        Block,
        BlockCount,
        BlockDevice,
        BlockIdx,
        Mode,
        TimeSource,
        Timestamp,
        VolumeIdx,
        VolumeManager,
    },
};

global_asm!(include_str!("../start.S"));

type UartType = Uart<Uart1>;
const UART_PERIPH_ID: PeripheralId = PeripheralId::Uart1;
const SDMMC_PERIPH_ID: PeripheralId = PeripheralId::Sdmmc0;

// MCK: 164MHz
// Clock frequency is divided by 2 because of the default `h32mxdiv` PMC setting
const MASTER_CLOCK_SPEED: u32 = 164000000 / 2;

const NUM_DMA_DESC: usize = 16;
static mut DMA_DESC_TABLE: [ADMADesc; NUM_DMA_DESC] = [ADMADesc::new(); NUM_DMA_DESC];
const BLOCK_SIZE: usize = 512;
const NUM_BLOCKS: usize = 1;
static mut SD_BUF: [u8; NUM_BLOCKS * BLOCK_SIZE] = [0; NUM_BLOCKS * BLOCK_SIZE];

static SD_DMA_FINISHED: AtomicBool = AtomicBool::new(false);

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
        SD_BUF.fill(0);
        DMA_DESC_TABLE.fill(ADMADesc::new());
        SD_DMA_FINISHED.store(false, Relaxed);
    }

    atsama5d27::l1cache::disable_dcache();

    let mut pmc = Pmc::new();
    pmc.enable_peripheral_clock(PeripheralId::Pit);
    pmc.enable_peripheral_clock(PeripheralId::Aic);
    pmc.enable_peripheral_clock(PeripheralId::Pioa);
    pmc.enable_peripheral_clock(PeripheralId::Piob);
    pmc.enable_peripheral_clock(PeripheralId::Pioc);
    pmc.enable_peripheral_clock(PeripheralId::Piod);
    pmc.enable_peripheral_clock(SDMMC_PERIPH_ID);

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
    let sdmmc_irq_ptr = sdmmc_irq_handler as unsafe extern "C" fn() as usize;
    aic.set_interrupt_handler(InterruptEntry {
        peripheral_id: SDMMC_PERIPH_ID,
        vector_fn_ptr: sdmmc_irq_ptr,
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

    let mut uart = UartType::new();
    uart.set_rx_interrupt(true);
    uart.set_rx(true);

    let mut sdmmc0 = Sdmmc::sdmmc0();
    writeln!(uart, "SDMMC0 status: {:?}", sdmmc0.status()).ok();

    sdmmc0.enable_interrupt_signal(NormalStatus::TRFC | NormalStatus::CMDC);
    sdmmc0.enable_error_interrupt_signal(ErrorStatus::all());

    writeln!(uart, "Reading {} blocks", NUM_BLOCKS).ok();
    let mut dma_params = DmaParams {
        dma_desc_table_phys_addr: unsafe { DMA_DESC_TABLE.as_ptr() as u32 },
        dma_desc_table: unsafe { &mut DMA_DESC_TABLE },
    };
    let block_number = 0;
    let cmd = SDCommand::new(
        SDCmd::Sd(if NUM_BLOCKS == 1 { SDCmdInner::ReadSingleBlock } else { SDCmdInner::ReadMultipleBlocks }),
        SDRespType::R1,
        block_number,
    );
    let data = SDData {
        buf_phys_addr: unsafe { SD_BUF.as_ptr() as u32 },
        direction: DataDirection::Read,
        blocks: NUM_BLOCKS,
        block_size: BLOCK_SIZE as u16,
    };
    writeln!(uart, "sending async command").ok();
    let res = sdmmc0
        .send_async_command(&mut dma_params, cmd, Some(data), || ())
        .expect("send_async_command");
    writeln!(uart, "result: {:?}", res).ok();

    writeln!(uart, "waiting for DMA to finish").ok();
    wait_sdmmc_dma_finish();
    writeln!(uart, "DMA finished, getting the result").ok();

    write!(uart, "BUF:").ok();
    for (i, byte) in unsafe { SD_BUF }.iter().enumerate() {
        if i % 64 == 0 {
            writeln!(uart).ok();
        }
        write!(uart, "{:02x} ", byte).ok();
    }
    writeln!(uart).ok();
    writeln!(uart, "SDMMC0 status: {:?}", sdmmc0.status()).ok();
    writeln!(uart, "SDMMC0 n. status: {:?}", sdmmc0.normal_status()).ok();

    pit.busy_wait_ms(MASTER_CLOCK_SPEED, 500);

    let disk = Disk(sdmmc0);
    let mut volume_mgr = VolumeManager::new(disk, Clock);
    let volume = volume_mgr.open_volume(VolumeIdx(0)).expect("open volume");
    let root_dir = volume_mgr.open_root_dir(volume).expect("open root dir");

    writeln!(uart, "Files in /:").ok();
    volume_mgr
        .iterate_dir(root_dir, |file| {
            if !file.attributes.is_hidden() && !file.attributes.is_volume() {
                writeln!(uart, "- {} {:?}", file.name, file.attributes).ok();
            }
        })
        .expect("iterate dir");

    // Initialize and increase the run counter stored in a file
    const FILE_NAME: &str = "count.bin";
    let mut num_runs = [0u8; 4];
    let file = match volume_mgr.open_file_in_dir(root_dir, FILE_NAME, Mode::ReadWriteCreate) {
        Ok(file) => file,
        Err(embedded_sdmmc::Error::FileAlreadyExists) => volume_mgr
            .open_file_in_dir(root_dir, FILE_NAME, Mode::ReadOnly)
            .expect("open"),
        Err(e) => panic!("{:?}", e),
    };
    volume_mgr
        .read(file, &mut num_runs)
        .expect("read num launched");
    let num_launched = u32::from_le_bytes([num_runs[0], num_runs[1], num_runs[2], num_runs[3]]) + 1;
    volume_mgr.close_file(file).expect("close");

    let file = volume_mgr
        .open_file_in_dir(root_dir, FILE_NAME, Mode::ReadWriteTruncate)
        .expect("open");
    volume_mgr
        .write(file, &num_launched.to_le_bytes())
        .expect("write file");
    volume_mgr.close_file(file).expect("close file");

    if num_launched == 1 {
        writeln!(uart, "This demo was run for the first time!").ok();
    } else {
        writeln!(uart, "This demo was run {} times!", num_launched).ok();
    }

    volume_mgr.close_dir(root_dir).expect("close dir");
    volume_mgr.close_volume(volume).expect("close volume");

    loop {
        armv7::asm::wfi();
    }
}

fn wait_sdmmc_dma_finish() {
    while !SD_DMA_FINISHED.load(Relaxed) {
        armv7::asm::wfi();
    }
    SD_DMA_FINISHED.store(false, Relaxed);
}

struct Clock;

impl TimeSource for Clock {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp::from_calendar(2024, 1, 29, 4, 20, 00).unwrap()
    }
}

struct Disk(Sdmmc);

impl BlockDevice for Disk {
    type Error = SdMmcError;

    fn read(
        &self,
        blocks: &mut [Block],
        start_block_idx: BlockIdx,
        _reason: &str,
    ) -> Result<(), Self::Error> {
        let mut dma_params = DmaParams {
            dma_desc_table_phys_addr: unsafe { DMA_DESC_TABLE.as_ptr() as u32 },
            dma_desc_table: unsafe { &mut DMA_DESC_TABLE },
        };

        for (i, block) in blocks.iter_mut().enumerate() {
            let block_number = start_block_idx.0 + i as u32;
            let cmd = SDCommand::new(
                SDCmd::Sd(SDCmdInner::ReadSingleBlock),
                SDRespType::R1,
                block_number,
            );
            let data = SDData {
                buf_phys_addr: unsafe { SD_BUF.as_ptr() as u32 },
                direction: DataDirection::Read,
                blocks: 1,
                block_size: BLOCK_SIZE as u16,
            };
            self.0
                .send_async_command(&mut dma_params, cmd, Some(data), || ())?;
            wait_sdmmc_dma_finish();

            block
                .contents
                .copy_from_slice(unsafe { &SD_BUF[..BLOCK_SIZE] });
        }

        Ok(())
    }

    fn write(&self, blocks: &[Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        let mut dma_params = DmaParams {
            dma_desc_table_phys_addr: unsafe { DMA_DESC_TABLE.as_ptr() as u32 },
            dma_desc_table: unsafe { &mut DMA_DESC_TABLE },
        };

        for (i, block) in blocks.iter().enumerate() {
            let block_number = start_block_idx.0 + i as u32;
            let cmd = SDCommand::new(
                SDCmd::Sd(SDCmdInner::WriteSingleBlock),
                SDRespType::R1,
                block_number,
            );

            unsafe {
                SD_BUF[..BLOCK_SIZE].copy_from_slice(&block.contents);
            }

            let data = SDData {
                buf_phys_addr: unsafe { SD_BUF.as_ptr() as u32 },
                direction: DataDirection::Write,
                blocks: 1,
                block_size: BLOCK_SIZE as u16,
            };
            self.0
                .send_async_command(&mut dma_params, cmd, Some(data), || ())?;
            wait_sdmmc_dma_finish();
        }

        Ok(())
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        // TODO: determine the capacity
        Ok(BlockCount(128 * 4096))
    }
}

#[no_mangle]
unsafe extern "C" fn pioa_irq_handler() {}

#[no_mangle]
unsafe extern "C" fn sdmmc_irq_handler() {
    let mut uart = UartType::new();
    let mut sdmmc0 = Sdmmc::sdmmc0();

    let ns = sdmmc0.normal_status();

    // Prevent an interrupt storm
    sdmmc0.acknowledge_interrupt(ns);

    // A DMA transfer completed?
    if ns.contains(NormalStatus::TRFC) {
        SD_DMA_FINISHED.store(true, Relaxed);
    }

    // Got an error, inspect it
    if ns.contains(NormalStatus::ERRINT) {
        writeln!(uart, "SDMMC IRQ, normal status: {:?}", ns).ok();
        let es = sdmmc0.error_status();
        writeln!(uart, "SDMMC IRQ, error status:  {:?}", es).ok();
        sdmmc0.acknowledge_error_interrupt(es);
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
