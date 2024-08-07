//! ATSAMA5D2 TRNG driver.

use {
    core::marker::PhantomData,
    utralib::{
        utra::trng::{CR, ISR_DATRDY, ODATA_ODATA},
        *,
    },
};

pub trait TrngState {}
pub struct Enabled;
pub struct Disabled;
pub struct Unknown;

impl TrngState for Enabled {}
impl TrngState for Disabled {}

const WA_KEY: u32 = 0x524E47; // b'RNG' write access key (see datasheet)

pub struct StatefulTrng<T: TrngState> {
    base_addr: u32,
    _state: PhantomData<T>,
}

pub struct Trng {}

impl Trng {
    /// Creates new `TRNG` instance that's in a disabled (unknown) state.
    #[allow(clippy::new_ret_no_self)]
    pub fn new() -> StatefulTrng<Disabled> {
        StatefulTrng {
            base_addr: HW_TRNG_BASE as u32,
            _state: PhantomData,
        }
    }

    pub fn with_alt_base_addr(base_addr: u32) -> StatefulTrng<Disabled> {
        StatefulTrng {
            base_addr,
            _state: PhantomData,
        }
    }
}

impl StatefulTrng<Disabled> {
    /// Enables the `TRNG`. Ensure that `TRNG` has clock source enabled via `PMC`.
    pub fn enable(self) -> StatefulTrng<Enabled> {
        let mut trng_csr = CSR::new(self.base_addr as *mut u32);
        let wa_key_and_enable: u32 = WA_KEY << 8 | 0b01; // Combine WA key and '1' bit to enable
        trng_csr.wo(CR, wa_key_and_enable);

        StatefulTrng {
            base_addr: self.base_addr,
            _state: PhantomData,
        }
    }
}

impl StatefulTrng<Enabled> {
    /// Disables the `TRNG`.
    pub fn disable(self) -> StatefulTrng<Disabled> {
        let mut trng_csr = CSR::new(self.base_addr as *mut u32);
        let wa_key_and_enable: u32 = WA_KEY << 8; // Write only WA key as 'enable' bit stays 0
        trng_csr.wo(CR, wa_key_and_enable);

        StatefulTrng {
            base_addr: self.base_addr,
            _state: PhantomData,
        }
    }

    /// Returns `true` if a new random `u32` value is available.
    pub fn data_ready(&self) -> bool {
        let trng_csr = CSR::new(self.base_addr as *mut u32);

        trng_csr.rf(ISR_DATRDY) & 0x1 != 0
    }

    /// Reads a random `u32` value generated by the `TRNG`.
    pub fn read_u32(&self) -> u32 {
        let trng_csr = CSR::new(self.base_addr as *mut u32);
        while !self.data_ready() {
            armv7::asm::nop();
        }

        trng_csr.rf(ODATA_ODATA)
    }
}
