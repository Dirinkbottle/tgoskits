//! Lifetime bookkeeping for K3 AI kernel aliases.
//!
//! The address-space refactor in `origin/dev` removed the old public
//! `SharedPages`/`Backend` interface. The K3 path therefore keeps the
//! alias' virtual range here and obtains physical pages through the
//! address-space translation API after materialising the user range.

use alloc::collections::BTreeMap;

use ax_memory_addr::VirtAddr;
use ax_sync::SpinLock;

/// A registered channel shared area.
pub(super) struct RegisteredChannelMemory {
    /// User-space start address.
    pub(super) user_va: usize,
    /// User-space byte size.
    pub(super) size_bytes: usize,
    /// Number of channels in the shared layout.
    pub(super) channel_count: u32,
    /// Kernel alias start address.
    pub(super) kernel_va: usize,
    /// Bytes covered by the kernel alias.
    pub(super) kernel_map_size: usize,
}

impl Drop for RegisteredChannelMemory {
    fn drop(&mut self) {
        if self.kernel_va != 0 && self.kernel_map_size != 0 {
            let _ = ax_mm::kernel_aspace().lock().unmap(
                VirtAddr::from_usize(self.kernel_va),
                self.kernel_map_size,
            );
        }
    }
}

/// A temporary tensor/blob alias indexed by the address returned to the
/// scheduler.
#[allow(dead_code)]
pub(super) struct RegisteredUserKernelMapping {
    /// Owning process id.
    pub(super) pid: u32,
    /// User-space start address.
    pub(super) user_va: usize,
    /// Requested, not page-aligned, length.
    pub(super) requested_len: usize,
    /// Kernel address returned to the scheduler.
    pub(super) kernel_va: usize,
    /// Page-aligned kernel alias start.
    pub(super) kernel_base: usize,
    /// Bytes covered by the kernel alias.
    pub(super) kernel_map_size: usize,
}

impl Drop for RegisteredUserKernelMapping {
    fn drop(&mut self) {
        if self.kernel_base != 0 && self.kernel_map_size != 0 {
            let _ = ax_mm::kernel_aspace().lock().unmap(
                VirtAddr::from_usize(self.kernel_base),
                self.kernel_map_size,
            );
        }
    }
}

/// Registered channel areas, keyed by process id.
pub(super) static CHANNEL_MEMORY_TABLE: SpinLock<Option<BTreeMap<u32, RegisteredChannelMemory>>> =
    SpinLock::new(None);

/// Temporary tensor/blob aliases, keyed by returned kernel address.
pub(super) static USER_KERNEL_MAPPING_TABLE: SpinLock<
    Option<BTreeMap<usize, RegisteredUserKernelMapping>>,
> = SpinLock::new(None);

/// Kernel aliases are installed in the kernel address space. Kernel worker
/// threads use that same root, so the old page-table-root cloning step is no
/// longer needed (and is deliberately not reproduced through private MM
/// fields).
pub(super) fn sync_kernel_alias_to_current_aspace(
    pid: u32,
    kernel_va: usize,
    kernel_map_size: usize,
) -> bool {
    if kernel_va == 0 || kernel_map_size == 0 {
        info!(
            "k3_airunner: reject empty kernel alias pid={}, va={:#x}, size={:#x}",
            pid, kernel_va, kernel_map_size
        );
        return false;
    }
    true
}
