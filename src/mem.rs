// SPDX-FileCopyrightText: 2024 Foundation Devices, Inc. <hello@foundationdevices.com>
// SPDX-License-Identifier: MIT OR Apache-2.0

/// Provides a wrapper around a physical location that can be created and accessed in
/// various ways, primarily via DMA directly by its physical address.
pub struct PhysLocation(usize);

impl PhysLocation {
    pub fn new_phys(addr: usize) -> PhysLocation {
        PhysLocation(addr)
    }

    pub fn addr(&self) -> usize {
        self.0
    }

    /// # Safety
    ///
    /// This function is as safe as `slice::from_raw_parts`. In other words,
    /// absolutely not safe.
    pub unsafe fn as_slice(&self, len: usize) -> &'static [u8] {
        core::slice::from_raw_parts(self.0 as *mut u8, len)
    }

    /// # Safety
    ///
    /// This function is as safe as `slice::from_raw_parts_mut`. In other words,
    /// absolutely not safe.
    pub unsafe fn as_slice_mut(&self, len: usize) -> &'static mut [u8] {
        core::slice::from_raw_parts_mut(self.0 as *mut u8, len)
    }

    /// Creates a physical memory location from a slice that *should* be accessible in the
    /// current memory model.
    ///
    /// If you want to create a physical location by an address, use
    /// [`new_phys`](Self::new_phys).
    pub fn from_slice_phys(slice: impl AsRef<[u8]>) -> Self {
        Self(slice.as_ref().as_ptr() as usize)
    }

    /// Creates a physical memory location from a slice that's mapped from virtual to
    /// physical memory by a user-defined `v2p` function.
    pub fn from_slice_virt(slice: impl AsRef<[u8]>, v2p: impl Fn(usize) -> usize) -> Self {
        let addr = slice.as_ref().as_ptr() as usize;
        Self(v2p(addr))
    }
}

impl From<&[u8]> for PhysLocation {
    fn from(value: &[u8]) -> Self {
        Self(value.as_ptr() as usize)
    }
}

impl From<&mut [u8]> for PhysLocation {
    fn from(value: &mut [u8]) -> Self {
        Self(value.as_ptr() as usize)
    }
}
