//! UART controller.

use core::fmt::Write;
use core::marker::PhantomData;

use utralib::utra::uart0::{
    CR_RXDIS, CR_RXEN, IER_RXRDY, IMR_RXRDY, RHR_RXCHR, SR_RXRDY, SR_TXRDY, THR_TXCHR,
};
use utralib::*;

pub struct Uart0 {}
pub struct Uart1 {}
pub struct Uart2 {}
pub struct Uart3 {}
pub struct Uart4 {}

// UART peripheral base addresses.
pub const UART_BASE_ADDRESS: [u32; 5] =
    [0xf801c000, 0xf8020000, 0xf8024000, 0xfc008000, 0xfc00c000];

mod sealed {
    use crate::uart::*;

    pub trait Sealed {}
    impl Sealed for Uart0 {}
    impl Sealed for Uart1 {}
    impl Sealed for Uart2 {}
    impl Sealed for Uart3 {}
    impl Sealed for Uart4 {}
}

pub trait UartPeriph: sealed::Sealed {
    const ID: usize;
}

impl UartPeriph for Uart0 {
    const ID: usize = 0;
}
impl UartPeriph for Uart1 {
    const ID: usize = 1;
}
impl UartPeriph for Uart2 {
    const ID: usize = 2;
}
impl UartPeriph for Uart3 {
    const ID: usize = 3;
}
impl UartPeriph for Uart4 {
    const ID: usize = 4;
}

#[derive(Default)]
pub struct Uart<U: UartPeriph> {
    base_addr: u32,
    inner: PhantomData<U>,
}

impl<U: UartPeriph> Uart<U> {
    pub fn new() -> Uart<U> {
        Uart {
            base_addr: UART_BASE_ADDRESS[U::ID],
            inner: PhantomData,
        }
    }

    /// Creates a driver instance with an alternative base address.
    /// Useful when the UART peripheral is remapped to some other virtual address by the MMU.
    pub fn with_alt_base_addr(base_addr: u32) -> Uart<U> {
        Uart {
            base_addr,
            inner: PhantomData,
        }
    }

    pub fn write_byte(&mut self, byte: u8) {
        let mut uart_csr = CSR::new(self.base_addr as *mut u32);

        // Wait for the previous transfer to complete
        while uart_csr.rf(SR_TXRDY) == 0 {
            armv7::asm::nop();
        }

        // Send the byte
        uart_csr.wfo(THR_TXCHR, byte as u32);
    }

    pub fn write_str(&mut self, s: &str) {
        for byte in s.as_bytes().iter() {
            self.write_byte(*byte);
        }
    }

    pub fn set_rx(&mut self, enabled: bool) {
        let mut uart_csr = CSR::new(self.base_addr as *mut u32);
        if enabled {
            uart_csr.wfo(CR_RXEN, 1);
        } else {
            uart_csr.wfo(CR_RXDIS, 1);
        }
    }

    pub fn set_rx_interrupt(&mut self, enabled: bool) {
        let mut uart_csr = CSR::new(self.base_addr as *mut u32);
        uart_csr.rmwf(IMR_RXRDY, enabled.into());
        uart_csr.rmwf(IER_RXRDY, enabled.into());
    }

    pub fn getc(&mut self) -> u8 {
        let uart_csr = CSR::new(self.base_addr as *mut u32);

        // Wait for the character reception to complete
        while uart_csr.rf(SR_RXRDY) == 0 {
            armv7::asm::nop();
        }

        uart_csr.rf(RHR_RXCHR) as u8
    }
}

impl<U: UartPeriph> Write for Uart<U> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        Uart::<U>::write_str(self, s);
        Ok(())
    }
}
