use core::ptr::NonNull;

#[cfg(feature = "zicbom")]
mod cbo {
    use core::ptr::NonNull;

    // SpacemiT K3 advertises zicbom with riscv,cbom-block-size = <64> in the
    // vendor Linux DTS. Keep this explicit until the OS grows DT-driven CBO sizing.
    const CBO_BLOCK_SIZE: usize = 64;

    #[inline(always)]
    unsafe fn cbo_inval(addr: usize) {
        unsafe {
            core::arch::asm!(
                ".insn i 15, 2, x0, {addr}, 0",
                addr = in(reg) addr,
                options(nostack)
            );
        }
    }

    #[inline(always)]
    unsafe fn cbo_clean(addr: usize) {
        unsafe {
            core::arch::asm!(
                ".insn i 15, 2, x0, {addr}, 1",
                addr = in(reg) addr,
                options(nostack)
            );
        }
    }

    #[inline(always)]
    unsafe fn cbo_flush(addr: usize) {
        unsafe {
            core::arch::asm!(
                ".insn i 15, 2, x0, {addr}, 2",
                addr = in(reg) addr,
                options(nostack)
            );
        }
    }

    #[inline(always)]
    fn align_down(addr: usize) -> usize {
        addr & !(CBO_BLOCK_SIZE - 1)
    }

    fn cache_op_range(addr: NonNull<u8>, size: usize, op: unsafe fn(usize)) {
        if size == 0 {
            return;
        }

        let start = align_down(addr.as_ptr() as usize);
        let end = (addr.as_ptr() as usize).saturating_add(size);
        let mut cur = start;

        while cur < end {
            unsafe { op(cur) };
            cur = cur.saturating_add(CBO_BLOCK_SIZE);
        }

        unsafe {
            core::arch::asm!("fence rw, rw", options(nostack));
        }
    }

    pub fn flush(addr: NonNull<u8>, size: usize) {
        cache_op_range(addr, size, cbo_clean);
    }

    pub fn invalidate(addr: NonNull<u8>, size: usize) {
        cache_op_range(addr, size, cbo_inval);
    }

    pub fn flush_invalidate(addr: NonNull<u8>, size: usize) {
        cache_op_range(addr, size, cbo_flush);
    }
}

pub fn flush(addr: NonNull<u8>, size: usize) {
    #[cfg(feature = "zicbom")]
    {
        cbo::flush(addr, size);
    }
    #[cfg(not(feature = "zicbom"))]
    {
        let _ = (addr, size);
    }
}

pub fn invalidate(addr: NonNull<u8>, size: usize) {
    #[cfg(feature = "zicbom")]
    {
        cbo::invalidate(addr, size);
    }
    #[cfg(not(feature = "zicbom"))]
    {
        let _ = (addr, size);
    }
}

pub fn flush_invalidate(addr: NonNull<u8>, size: usize) {
    #[cfg(feature = "zicbom")]
    {
        cbo::flush_invalidate(addr, size);
    }
    #[cfg(not(feature = "zicbom"))]
    {
        let _ = (addr, size);
    }
}
