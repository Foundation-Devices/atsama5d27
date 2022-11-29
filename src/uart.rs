//! UART controller.

use core::fmt::Write;
use core::marker::PhantomData;

use utralib::utra::uart0::{SR_TXRDY, THR_TXCHR};
use utralib::*;

pub struct Uart0 {}
pub struct Uart1 {}
pub struct Uart2 {}
pub struct Uart3 {}
pub struct Uart4 {}

// UART peripheral base addresses.
const UART_BASE_ADDRESS: [u32; 5] = [0xf801c000, 0xf8020000, 0xf8024000, 0xfc008000, 0xfc00c000];

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
    inner: PhantomData<U>,
}

impl<U: UartPeriph> Uart<U> {
    pub fn new() -> Uart<U> {
        Uart { inner: PhantomData }
    }

    pub fn write_byte(&mut self, byte: u8) {
        let mut uart_csr = CSR::new(UART_BASE_ADDRESS[U::ID] as *mut u32);

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
}

impl<U: UartPeriph> Write for Uart<U> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        Uart::<U>::write_str(self, s);
        Ok(())
    }
}
