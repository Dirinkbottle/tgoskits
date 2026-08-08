//! Channel / tensor 的 kernel alias 记账与生命周期管理。
//!
//! 只要这里还持有 [`SharedPages`] 的 `Arc`，底层共享物理页就不会被回收；对应的
//! `Drop` 实现会在记录被移除时撤销 kernel alias，避免 kernel VA 泄漏。

use alloc::{collections::btree_map::BTreeMap, sync::Arc};

use ax_kspin::SpinNoIrq;
use ax_log::warn;
use ax_memory_addr::VirtAddr;
use ax_task::current;

use crate::{mm::SharedPages, task::AsThread};

/// 记录某个进程已经注册好的 channel 共享区。
///
/// 现在先只保存 [`SharedPages`] 的强引用，保证用户态映射和后续 guard 路径都还能访问。
pub(super) struct RegisteredChannelMemory {
    /// 用户态共享区起始虚拟地址。
    pub(super) user_va: usize,
    /// 共享区字节大小。
    pub(super) size_bytes: usize,
    /// 通道数量。
    pub(super) channel_count: u32,
    /// 保活底层物理页的共享页引用。
    pub(super) shared_pages: Arc<SharedPages>,
    /// 内核连续 alias 的起始虚拟地址。
    pub(super) kernel_va: usize,
    /// 内核 alias 覆盖的字节数。
    pub(super) kernel_map_size: usize,
}

/// 释放共享内存页, 然后将内核alias清除,防止内存泄漏
impl Drop for RegisteredChannelMemory {
    fn drop(&mut self) {
        if self.kernel_va != 0 && self.kernel_map_size != 0 {
            let kspace = ax_mm::kernel_aspace();
            let _ = kspace
                .lock()
                .unmap(VirtAddr::from_usize(self.kernel_va), self.kernel_map_size);
        }
    }
}

/// 记录 scheduler 临时建立的用户页 -> kernel alias。
#[allow(dead_code)]
pub(super) struct RegisteredUserKernelMapping {
    /// 所属进程 pid。
    pub(super) pid: u32,
    /// 用户态起始虚拟地址。
    pub(super) user_va: usize,
    /// 请求映射的字节数（未页对齐）。
    pub(super) requested_len: usize,
    /// 保活底层物理页的共享页引用。
    pub(super) shared_pages: Arc<SharedPages>,
    /// 返回给调度器的内核虚拟地址（含页内偏移）。
    pub(super) kernel_va: usize,
    /// 内核 alias 的页对齐起始地址。
    pub(super) kernel_base: usize,
    /// 内核 alias 覆盖的字节数。
    pub(super) kernel_map_size: usize,
}

impl Drop for RegisteredUserKernelMapping {
    fn drop(&mut self) {
        if self.kernel_base != 0 && self.kernel_map_size != 0 {
            let kspace = ax_mm::kernel_aspace();
            let _ = kspace
                .lock()
                .unmap(VirtAddr::from_usize(self.kernel_base), self.kernel_map_size);
        }
    }
}

/// 已注册 channel 共享区表。
///
/// 当前先按 pid 维度记一份最小状态。只要这里还持有 `Arc`，底层共享物理页就不会被回收。
pub(super) static CHANNEL_MEMORY_TABLE: SpinNoIrq<Option<BTreeMap<u32, RegisteredChannelMemory>>> =
    SpinNoIrq::new(None);

/// scheduler 的 tensor/blob 映射表，按返回给调度器的 `kernel_va` 索引。
pub(super) static USER_KERNEL_MAPPING_TABLE: SpinNoIrq<
    Option<BTreeMap<usize, RegisteredUserKernelMapping>>,
> = SpinNoIrq::new(None);

/// 把 kernel alias 映射同步到当前线程的进程地址空间，保证后续调度器能直接访问。
pub(super) fn sync_kernel_alias_to_current_aspace(
    pid: u32,
    kernel_va: usize,
    kernel_map_size: usize,
) -> bool {
    if kernel_va == 0 || kernel_map_size == 0 {
        info!(
            "k3_airunner: sync kernel alias rejected pid={}, kernel_va={:#x}, map_size={:#x}",
            pid, kernel_va, kernel_map_size
        );
        return false;
    }

    let start = VirtAddr::from_usize(kernel_va);
    info!(
        "k3_airunner: sync kernel alias begin pid={}, kernel_va={:#x}, map_size={:#x}",
        pid, kernel_va, kernel_map_size
    );
    {
        let curr = current();
        let aspace_arc = curr.as_thread().proc_data.aspace();
        let mut aspace = aspace_arc.lock();
        let kspace = ax_mm::kernel_aspace();
        let kspace = kspace.lock();
        if aspace
            .page_table_mut()
            .clone_missing_root_entries_from(kspace.page_table(), start, kernel_map_size)
            .is_err()
        {
            warn!("k3_airunner: sync kernel alias failed pid={pid}");
            return false;
        }
    }
    let _ = crate::mm::flush_tlb_range_sync(start, kernel_map_size);
    info!(
        "k3_airunner: sync kernel alias done pid={}, kernel_va={:#x}, map_size={:#x}",
        pid, kernel_va, kernel_map_size
    );
    true
}
