use crate::l1cache::{clean_region_dcache, invalidate_region_dcache, is_dcache_enabled};
use crate::l2cc::L2cc;

pub fn invalidate_region(l2cc: &mut L2cc, start: usize, length: usize) {
    let start = start as u32;
    let end_addr = start + length as u32;
    if is_dcache_enabled() {
        invalidate_region_dcache(start, end_addr);
        if l2cc.is_enabled() {
            l2cc.invalidate_region(start, end_addr);
        }
    }
}

pub fn clean_region(l2cc: &mut L2cc, start: usize, length: usize) {
    let start = start as u32;
    let end_addr = start + length as u32;
    if is_dcache_enabled() {
        clean_region_dcache(start, end_addr);
        if l2cc.is_enabled() {
            l2cc.clean_region(start, end_addr);
        }
    }
}

pub fn invalidate_slice<T>(l2cc: &mut L2cc, slice: &[T]) {
    let start_addr = slice.as_ptr() as usize;
    let end_addr = start_addr + slice.len() * core::mem::size_of::<T>();
    invalidate_region(l2cc, start_addr, end_addr);
}

pub fn clean_slice<T>(l2cc: &mut L2cc, slice: &[T]) {
    let start_addr = slice.as_ptr() as usize;
    let end_addr = start_addr + slice.len() * core::mem::size_of::<T>();
    clean_region(l2cc, start_addr, end_addr);
}
