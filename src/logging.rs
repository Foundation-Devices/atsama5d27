extern crate alloc;

use {
    alloc::boxed::Box,
    core::{cell::RefCell, sync::atomic::AtomicUsize},
};

struct UartLogger<UartType> {
    uart: critical_section::Mutex<RefCell<UartType>>,
    tick_count: Option<&'static AtomicUsize>,
}

impl<UartType> UartLogger<UartType> {
    fn new(uart: UartType, tick_count: Option<&'static AtomicUsize>) -> Self {
        Self {
            uart: critical_section::Mutex::new(RefCell::new(uart)),
            tick_count,
        }
    }
}

impl<UartType: Send + core::fmt::Write> log::Log for UartLogger<UartType> {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        critical_section::with(|cs| {
            writeln!(
                self.uart.borrow(cs).borrow_mut(),
                "{} {} [{}] {}",
                record.level(),
                self.tick_count
                    .map(|t| (t.load(core::sync::atomic::Ordering::SeqCst) as f32) / 1000.0)
                    .unwrap_or(0.0),
                record.module_path().unwrap_or("bin"),
                record.args()
            )
            .ok();
        })
    }

    fn flush(&self) {}
}

pub fn init_logging<UartType: Send + core::fmt::Write + 'static>(
    uart: UartType,
    tick_count: Option<&'static AtomicUsize>,
) {
    log::set_max_level(log::LevelFilter::Debug);
    log::set_logger(Box::leak(Box::new(UartLogger::new(uart, tick_count)))).ok();
}
