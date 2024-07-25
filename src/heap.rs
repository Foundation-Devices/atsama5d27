use {core::mem::MaybeUninit, embedded_alloc::Heap};

#[global_allocator]
static mut HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 1024 * 1024;
static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

pub fn init_heap() {
    unsafe {
        HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
    }
}
