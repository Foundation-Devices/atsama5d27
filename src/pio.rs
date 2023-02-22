//! Parallel I/O controller (GPIO).

use core::marker::PhantomData;
use utralib::utra::pio::{
    HW_PIO_BASE, PIO_CFGR0, PIO_CFGR0_DIR, PIO_CFGR0_FUNC, PIO_CODR0, PIO_MSKR0, PIO_ODSR0,
    PIO_SODR0,
};
use utralib::*;

use crate::pio::sealed::Sealed;

pub struct PioA {}
pub struct PioB {}
pub struct PioC {}
pub struct PioD {}

pub trait PioPort: Sealed {
    const ID: u32;

    fn configure_pins_by_mask(mask: u32, func: Func, dir: impl Into<Option<Direction>>) {
        let mut pio_csr = CSR::new(Self::get_base_address() as *mut u32);
        pio_csr.wo(PIO_MSKR0, mask);
        pio_csr.wo(PIO_CFGR0, func as u32);
        if let Some(dir) = dir.into() {
            pio_csr.rmwf(PIO_CFGR0_DIR, dir as u32);
        }
    }

    fn clear_all() {
        let mut pio_csr = CSR::new(Self::get_base_address() as *mut u32);
        pio_csr.wo(PIO_CODR0, 0x00);
    }

    /// Retrieves PIO peripheral base address for the specific PIO Port.
    fn get_base_address() -> u32 {
        HW_PIO_BASE as u32 + Self::ID * 0x40
    }
}

impl PioPort for PioA {
    const ID: u32 = 0;
}
impl PioPort for PioB {
    const ID: u32 = 1;
}
impl PioPort for PioC {
    const ID: u32 = 2;
}
impl PioPort for PioD {
    const ID: u32 = 3;
}

#[derive(Debug)]
pub enum Func {
    Gpio = 0,
    A,
    B,
    C,
    D,
    E,
    F,
}

#[derive(Debug)]
pub enum Direction {
    Input = 0,
    Output = 1,
}

#[derive(Default)]
pub struct Pio<P: PioPort, const PIN: u32> {
    port: PhantomData<P>,
}

impl<P: PioPort, const PIN: u32> Pio<P, PIN> {
    /// Sets the pin into HIGH or LOW logic level.
    pub fn set(&mut self, hi: bool) {
        let mut pio_csr = CSR::new(P::get_base_address() as *mut u32);
        let pin_bit = 1 << PIN;

        if hi {
            pio_csr.wo(PIO_SODR0, pin_bit);
        } else {
            pio_csr.wo(PIO_CODR0, pin_bit);
        }
    }

    /// Returns `true` if pin is in HIGH logic level.
    pub fn get(&self) -> bool {
        let pio_csr = CSR::new(P::get_base_address() as *mut u32);
        let pin_bit = 1 << PIN;

        pio_csr.r(PIO_ODSR0) & pin_bit != 0
    }

    pub fn set_func(&self, func: Func) {
        let mut pio_csr = CSR::new(P::get_base_address() as *mut u32);
        let pin_bit = 1 << PIN;

        pio_csr.wo(PIO_MSKR0, pin_bit);
        pio_csr.rmwf(PIO_CFGR0_FUNC, func as u32);
    }

    pub fn set_direction(&self, direction: Direction) {
        let mut pio_csr = CSR::new(P::get_base_address() as *mut u32);
        let pin_bit = 1 << PIN;

        pio_csr.wo(PIO_MSKR0, pin_bit);
        pio_csr.rmwf(PIO_CFGR0_DIR, direction as u32);
    }
}

/// Sealed trait machinery
mod sealed {
    use super::*;

    pub trait Sealed {}
    impl Sealed for PioA {}
    impl Sealed for PioB {}
    impl Sealed for PioC {}
    impl Sealed for PioD {}
}

// The following code implements pin constructors for each port.
// This way we exhaust all possible pins to allow for a compile-check of the pin number being correct.
// This shall be simplified in future when compile-checks for const-generic parameters are stabilized.

macro_rules! impl_pins {
    ($port:ty, [$($name:ident => $pin:expr),+]) => {
        $(
            impl Pio<$port, $pin> {
                pub fn $name() -> Self {
                    Self { port: PhantomData }
                }
            }
        )+
    }
}

impl_pins!(PioA, [
    pa0=>0,   pa1=>1,   pa2=>2,   pa3=>3,   pa4=>4,   pa5=>5,   pa6=>6,   pa7=>7,
    pa8=>8,   pa9=>9,   pa10=>10, pa11=>11, pa12=>12, pa13=>13, pa14=>14, pa15=>15,
    pa16=>16, pa17=>17, pa18=>18, pa19=>19, pa20=>20, pa21=>21, pa22=>22, pa23=>23,
    pa24=>24, pa25=>25, pa26=>26, pa27=>27, pa28=>28, pa29=>29, pa30=>30, pa31=>31
]);

impl_pins!(PioB, [
    pb0=>0,   pb1=>1,   pb2=>2,   pb3=>3,   pb4=>4,   pb5=>5,   pb6=>6,   pb7=>7,
    pb8=>8,   pb9=>9,   pb10=>10, pb11=>11, pb12=>12, pb13=>13, pb14=>14, pb15=>15,
    pb16=>16, pb17=>17, pb18=>18, pb19=>19, pb20=>20, pb21=>21, pb22=>22, pb23=>23,
    pb24=>24, pb25=>25, pb26=>26, pb27=>27, pb28=>28, pb29=>29, pb30=>30, pb31=>31
]);

impl_pins!(PioC, [
    pc0=>0,   pc1=>1,   pc2=>2,   pc3=>3,   pc4=>4,   pc5=>5,   pc6=>6,   pc7=>7,
    pc8=>8,   pc9=>9,   pc10=>10, pc11=>11, pc12=>12, pc13=>13, pc14=>14, pc15=>15,
    pc16=>16, pc17=>17, pc18=>18, pc19=>19, pc20=>20, pc21=>21, pc22=>22, pc23=>23,
    pc24=>24, pc25=>25, pc26=>26, pc27=>27, pc28=>28, pc29=>29, pc30=>30, pc31=>31
]);

impl_pins!(PioD, [
    pd0=>0,   pd1=>1,   pd2=>2,   pd3=>3,   pd4=>4,   pd5=>5,   pd6=>6,   pd7=>7,
    pd8=>8,   pd9=>9,   pd10=>10, pd11=>11, pd12=>12, pd13=>13, pd14=>14, pd15=>15,
    pd16=>16, pd17=>17, pd18=>18, pd19=>19, pd20=>20, pd21=>21, pd22=>22, pd23=>23,
    pd24=>24, pd25=>25, pd26=>26, pd27=>27, pd28=>28, pd29=>29, pd30=>30, pd31=>31
]);
