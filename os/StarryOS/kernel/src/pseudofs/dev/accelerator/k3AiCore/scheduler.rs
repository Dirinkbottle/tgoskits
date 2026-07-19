//! `K3SchedulerOps` 运行时回调实现：worker 线程、用户内存拷贝、tensor 映射。

use alloc::collections::btree_map::BTreeMap;

use ax_memory_addr::{MemoryAddr, PhysAddr, VirtAddr, VirtAddrRange};
use ax_runtime::hal::{
    cpu::asm::user_copy,
    paging::{MappingFlags, PageSize},
};
use ax_task::{AxCpuMask, current, spawn};
use k3_aiScheduler::K3SchedulerOps;

use super::{
    registry::{
        RegisteredUserKernelMapping, USER_KERNEL_MAPPING_TABLE, sync_kernel_alias_to_current_aspace,
    },
    runner::K3AiRunner,
};
use crate::{
    mm::{Backend, access_user_memory},
    task::AsThread,
};

impl K3SchedulerOps for K3AiRunner {
    fn spawn_thread(&self, f: fn(usize), arg: usize) {
        spawn(move || {
            // 这里让 scheduler worker 线程固定在 CPU 10 上，避免和其他非 AI 线程混跑。
            let affinity_set = ax_task::set_current_affinity(AxCpuMask::one_shot(10));
            warn!(
                "k3_airunner: scheduler worker affinity target_cpu=10, set={}",
                affinity_set
            );
            f(arg);
        });
    }

    unsafe fn copy_from_user(&self, user_va: u64, buf: &mut [u8]) -> Result<(), ()> {
        if buf.is_empty() {
            return Ok(());
        }

        let user_va = usize::try_from(user_va).map_err(|_| ())?;
        if user_va == 0 {
            return Err(());
        }

        // user_copy 允许在访问用户页时处理缺页；失败返回非 0。
        let failed_at = access_user_memory(|| unsafe {
            user_copy(buf.as_mut_ptr(), user_va as *const u8, buf.len())
        });
        if failed_at == 0 { Ok(()) } else { Err(()) }
    }

    unsafe fn copy_to_user(&self, user_va: u64, buf: &[u8]) -> Result<(), ()> {
        if buf.is_empty() {
            return Ok(());
        }

        let user_va = usize::try_from(user_va).map_err(|_| ())?;
        if user_va == 0 {
            return Err(());
        }

        // 当前只做最小封装，具体安全性由调用者按 trait contract 保证。
        let failed_at = access_user_memory(|| unsafe {
            user_copy(user_va as *mut u8, buf.as_ptr(), buf.len())
        });
        if failed_at == 0 { Ok(()) } else { Err(()) }
    }

    // 将用户态连续 VA 映射到内核连续 VA，返回内核 VA。
    unsafe fn map_user_to_kernel(&self, user_va: u64, len: usize) -> Result<u64, ()> {
        if user_va == 0 || len == 0 {
            return Err(());
        }

        let user_va = usize::try_from(user_va).map_err(|_| ())?;
        let user_end = user_va.checked_add(len).ok_or(())?;
        let range_start = VirtAddr::from(user_va).align_down_4k();
        let range_end = VirtAddr::from(user_end).align_up_4k();
        let range_len = range_end - range_start;
        let range_offset = user_va - range_start.as_usize();
        if range_len == 0 {
            return Err(());
        }

        let curr = current();
        let pid = curr.as_thread().proc_data.proc.pid();
        let aspace_arc = curr.as_thread().proc_data.aspace();
        let aspace = aspace_arc.lock();

        // 当前 tensor allocator 走 MAP_SHARED mmap，这里只接受 SharedBackend。
        let area = aspace.find_area(VirtAddr::from(user_va)).ok_or(())?;
        if area.start() > range_start || area.end() < range_end {
            return Err(());
        }

        let page_offset = (range_start - area.start()) / PageSize::Size4K as usize;
        let shared_pages = match area.backend() {
            Backend::Shared(shared) => shared.pages().clone(),
            _ => {
                info!(
                    "k3_airunner: map_user_to_kernel rejected non-shared user memory pid={}, \
                     user_va={:#x}, len={:#x}",
                    pid, user_va, len
                );
                return Err(());
            }
        };
        drop(aspace);

        let required_pages = range_len / PageSize::Size4K as usize;
        let required_end = page_offset.checked_add(required_pages).ok_or(())?;
        if shared_pages.len() < required_end {
            info!(
                "k3_airunner: map_user_to_kernel rejected short SharedPages pid={}, pages={}, \
                 offset={}, required={}",
                pid,
                shared_pages.len(),
                page_offset,
                required_pages
            );
            return Err(());
        }

        let (kernel_base, kernel_va, kernel_map_size) = {
            let kspace = ax_mm::kernel_aspace();
            let mut guard = kspace.lock();
            // 用户 VA 连续只说明用户视角连续；kernel alias 仍然逐页重建。
            let mut virt_start = guard
                .find_free_area(
                    guard.base(),
                    range_len,
                    VirtAddrRange::new(guard.base(), guard.end()),
                )
                .ok_or(())?;
            let kernel_base = virt_start.as_usize();
            for paddr in shared_pages.iter().skip(page_offset).take(required_pages) {
                if guard
                    .map_linear(
                        virt_start,
                        PhysAddr::from_usize(paddr.as_usize()),
                        PageSize::Size4K as usize,
                        MappingFlags::READ | MappingFlags::WRITE,
                    )
                    .is_err()
                {
                    let mapped_len = virt_start.as_usize() - kernel_base;
                    if mapped_len != 0 {
                        let _ = guard.unmap(VirtAddr::from_usize(kernel_base), mapped_len);
                    }
                    return Err(());
                }
                virt_start += PageSize::Size4K as usize;
            }
            (kernel_base, kernel_base + range_offset, range_len)
        };
        if !sync_kernel_alias_to_current_aspace(pid, kernel_base, kernel_map_size) {
            let kspace = ax_mm::kernel_aspace();
            let _ = kspace
                .lock()
                .unmap(VirtAddr::from_usize(kernel_base), kernel_map_size);
            return Err(());
        }

        {
            let mut table = USER_KERNEL_MAPPING_TABLE.lock();
            let table = table.get_or_insert_with(BTreeMap::new);
            if table.contains_key(&kernel_va) {
                let kspace = ax_mm::kernel_aspace();
                let _ = kspace
                    .lock()
                    .unmap(VirtAddr::from_usize(kernel_base), kernel_map_size);
                return Err(());
            }
            table.insert(
                kernel_va,
                RegisteredUserKernelMapping {
                    pid,
                    user_va,
                    requested_len: len,
                    shared_pages,
                    kernel_va,
                    kernel_base,
                    kernel_map_size,
                },
            );
        }

        info!(
            "k3_airunner: map_user_to_kernel pid={}, user_va={:#x}, len={:#x}, kernel_va={:#x}, \
             map_size={:#x}",
            pid, user_va, len, kernel_va, kernel_map_size
        );
        Ok(kernel_va as u64)
    }

    unsafe fn unmap_user(&self, kernel_va: u64, len: usize) -> Result<(), ()> {
        if kernel_va == 0 || len == 0 {
            return Err(());
        }

        let kernel_va = usize::try_from(kernel_va).map_err(|_| ())?;
        let mut table = USER_KERNEL_MAPPING_TABLE.lock();
        let table = table.as_mut().ok_or(())?;
        let registered = table.get(&kernel_va).ok_or(())?;
        if registered.requested_len != len {
            info!(
                "k3_airunner: unmap_user length mismatch kernel_va={:#x}, expected={:#x}, \
                 got={:#x}",
                kernel_va, registered.requested_len, len
            );
            return Err(());
        }

        // remove 后 RegisteredUserKernelMapping::drop 会撤销 kernel alias。
        let _mapping = table.remove(&kernel_va).ok_or(())?;
        info!(
            "k3_airunner: unmap_user kernel_va={:#x}, len={:#x}",
            kernel_va, len
        );
        Ok(())
    }
}
