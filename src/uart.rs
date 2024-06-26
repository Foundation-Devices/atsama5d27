//! UART controller.

use {
    core::{fmt::Write, marker::PhantomData},
    utralib::{utra::uart0::*, *},
};

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

#[derive(Debug)]
pub enum Parity {
    Even = 0,
    Odd = 1,
    Space = 2,
    Mark = 3,
    No = 4,
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
    /// Useful when the UART peripheral is remapped to some other virtual address by the
    /// MMU.
    pub fn with_alt_base_addr(base_addr: u32) -> Uart<U> {
        Uart {
            base_addr,
            inner: PhantomData,
        }
    }

    pub fn init(&mut self, clock_speed: u32, baud_rate: u32, parity: Parity) {
        let mut csr = CSR::new(self.base_addr as *mut u32);

        // Reset everything
        csr.wfo(CR_RSTTX, 1);
        csr.wfo(CR_RSTRX, 1);
        csr.wfo(CR_RXDIS, 1);
        csr.wfo(CR_TXDIS, 1);
        csr.wfo(CR_RSTSTA, 1);

        // Set baud rate and parity
        csr.wo(BRGR, clock_speed / (16 * baud_rate));
        self.set_parity(parity);

        // Enable receiver and transmitter
        csr.wfo(CR_RXEN, 1);
        csr.wfo(CR_TXEN, 1);
    }

    pub fn set_baud(&mut self, clock_speed: u32, baud_rate: u32) {
        let mut csr = CSR::new(self.base_addr as *mut u32);

        csr.wfo(CR_RSTTX, 1);
        csr.wfo(CR_RSTRX, 1);
        self.set_tx(false);
        self.set_rx(false);

        csr.wo(BRGR, clock_speed / (16 * baud_rate));
    }

    pub fn set_parity(&mut self, parity: Parity) {
        let mut csr = CSR::new(self.base_addr as *mut u32);
        csr.rmwf(MR_PAR, parity as u32);
    }

    pub fn write_byte(&mut self, byte: u8) {
        let mut uart_csr = CSR::new(self.base_addr as *mut u32);

        // Wait for the previous transfer to complete
        while uart_csr.rf(SR_TXRDY) == 0 {
            armv7::asm::nop();
        }

        // Send the byte
        uart_csr.wfo(THR_TXCHR, byte as u32);

        // Wait for the current transfer to complete
        while uart_csr.rf(SR_TXEMPTY) == 0 {
            armv7::asm::nop();
        }
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

    pub fn set_tx(&mut self, enabled: bool) {
        let mut uart_csr = CSR::new(self.base_addr as *mut u32);
        if enabled {
            uart_csr.wfo(CR_TXEN, 1);
        } else {
            uart_csr.wfo(CR_TXDIS, 1);
        }
    }

    pub fn set_rx_interrupt(&mut self, enabled: bool) {
        let mut uart_csr = CSR::new(self.base_addr as *mut u32);
        uart_csr.rmwf(IMR_RXRDY, enabled.into());
        uart_csr.rmwf(IER_RXRDY, enabled.into());
    }

    pub fn getc_nonblocking(&mut self) -> Option<u8> {
        let uart_csr = CSR::new(self.base_addr as *mut u32);
        if uart_csr.rf(SR_RXRDY) != 0 {
            Some(uart_csr.rf(RHR_RXCHR) as u8)
        } else {
            None
        }
    }

    pub fn getc(&mut self) -> u8 {
        let uart_csr = CSR::new(self.base_addr as *mut u32);

        // Wait for the character reception to complete
        while uart_csr.rf(SR_RXRDY) == 0 {
            armv7::asm::nop();
        }

        uart_csr.rf(RHR_RXCHR) as u8
    }

    pub fn is_overrun(&self) -> bool {
        let uart_csr = CSR::new(self.base_addr as *mut u32);
        uart_csr.rf(SR_OVRE) != 0
    }

    pub fn getc_isr(&self) -> u8 {
        let uart_csr = CSR::new(self.base_addr as *mut u32);
        uart_csr.rf(RHR_RXCHR) as u8
    }
}

impl<U: UartPeriph> Write for Uart<U> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        Uart::<U>::write_str(self, s);
        Ok(())
    }
}
