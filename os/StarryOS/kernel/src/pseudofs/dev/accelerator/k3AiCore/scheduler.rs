//! `K3SchedulerOps` adapter for StarryOS.

use alloc::{boxed::Box, collections::btree_map::BTreeMap, string::String, vec::Vec};

use ax_memory_addr::{MemoryAddr, PAGE_SIZE_4K, PhysAddr, VirtAddr, VirtAddrRange};
use ax_runtime::hal::{paging::MappingFlags, percpu::this_cpu_id};
use ax_task::{CpuId, CpuSet, ThreadBuilder, WaitQueue};
use k3_ai_scheduler::{K3SchedulerOps, K3SchedulerWaitQueue, K3WaitQueue};

use super::{
    registry::{
        RegisteredUserKernelMapping, USER_KERNEL_MAPPING_TABLE,
        sync_kernel_alias_to_current_aspace,
    },
    runner::K3AiRunner,
};
use crate::mm::{UserConstPtr, UserPtr};

/// StarryOS wait-queue adapter.
struct StarryK3WaitQueue(WaitQueue);

impl K3SchedulerWaitQueue for StarryK3WaitQueue {
    fn wait(&self) {
        self.0.wait();
    }

    fn wait_until(&self, condition: &dyn Fn() -> bool) {
        self.0.wait_until(condition);
    }

    fn notify_one(&self) {
        let _ = self.0.notify_one();
    }

    fn notify_all(&self) {
        self.0.notify_all();
    }
}

impl K3SchedulerOps for K3AiRunner {
    fn new_wait_queue(&self) -> K3WaitQueue {
        Box::new(StarryK3WaitQueue(WaitQueue::new()))
    }

    fn current_core_id(&self) -> u32 {
        this_cpu_id() as u32
    }

    fn spawn_thread_on_core(&self, core_id: u32, f: fn(usize), arg: usize) {
        let cpu_count = ax_task::cpu_topology_len().unwrap_or(core_id as usize + 1);
        let mut affinity = CpuSet::empty(cpu_count);
        if !affinity.insert(CpuId::new(core_id)) {
            warn!("k3_airunner: invalid worker affinity cpu={core_id}");
            return;
        }

        match ThreadBuilder::new(String::from("k3-ai-worker"))
            .affinity(affinity)
            .spawn(move || f(arg))
        {
            Ok(handle) => handle.detach_permanent(),
            Err(error) => error!(
                "k3_airunner: failed to spawn worker cpu={}, error={:?}",
                core_id, error
            ),
        }
    }

    unsafe fn copy_from_user(&self, user_va: u64, buf: &mut [u8]) -> Result<(), ()> {
        if buf.is_empty() {
            return Ok(());
        }
        let user_va = usize::try_from(user_va).map_err(|_| ())?;
        let Ok(Some(task)) = crate::task::try_current_user_task() else {
            return Err(());
        };
        let bytes = UserConstPtr::<u8>::from(user_va)
            .read_slice(&task, buf.len())
            .map_err(|_| ())?;
        buf.copy_from_slice(&bytes);
        Ok(())
    }

    unsafe fn copy_to_user(&self, user_va: u64, buf: &[u8]) -> Result<(), ()> {
        if buf.is_empty() {
            return Ok(());
        }
        let user_va = usize::try_from(user_va).map_err(|_| ())?;
        let Ok(Some(task)) = crate::task::try_current_user_task() else {
            return Err(());
        };
        UserPtr::<u8>::from(user_va)
            .write_slice(&task, buf)
            .map_err(|_| ())
    }

    /// Materialises the user range, translates each resident page and builds
    /// a contiguous kernel alias from the resulting physical pages.
    unsafe fn map_user_to_kernel(&self, user_va: u64, len: usize) -> Result<u64, ()> {
        if user_va == 0 || len == 0 {
            return Err(());
        }

        let user_va = usize::try_from(user_va).map_err(|_| ())?;
        let user_end = user_va.checked_add(len).ok_or(())?;
        let range_start = VirtAddr::from_usize(user_va).align_down_4k();
        let range_end = VirtAddr::from_usize(user_end).align_up_4k();
        let range_len = range_end.as_usize().checked_sub(range_start.as_usize()).ok_or(())?;
        let range_offset = user_va.checked_sub(range_start.as_usize()).ok_or(())?;
        let required_pages = range_len / PAGE_SIZE_4K;
        if required_pages == 0 {
            return Err(());
        }

        let task = crate::task::current_user_task();
        let pid = task.as_thread().proc_data.proc.pid().get();
        let aspace_arc = task.as_thread().proc_data.aspace();
        let mut aspace = aspace_arc.lock();
        if !aspace.contains_range(range_start, range_len) {
            return Err(());
        }
        aspace
            .populate_area(range_start, range_len, MappingFlags::READ | MappingFlags::WRITE)
            .map_err(|_| ())?;

        let mut physical_pages = Vec::with_capacity(required_pages);
        for index in 0..required_pages {
            let offset = index.checked_mul(PAGE_SIZE_4K).ok_or(())?;
            let va = range_start.checked_add(offset).ok_or(())?;
            let paddr = aspace.translate(va).map_err(|_| ())?;
            physical_pages.push(paddr);
        }
        drop(aspace);

        let (kernel_base, kernel_map_size) = {
            let kspace = ax_mm::kernel_aspace();
            let mut guard = kspace.lock();
            let kernel_base = guard
                .find_free_area(
                    guard.base(),
                    range_len,
                    VirtAddrRange::new(guard.base(), guard.end()),
                )
                .ok_or(())?
                .as_usize();
            let mut virt = VirtAddr::from_usize(kernel_base);
            for paddr in physical_pages {
                if guard
                    .map_linear(
                        virt,
                        PhysAddr::from_usize(paddr.as_usize()),
                        PAGE_SIZE_4K,
                        MappingFlags::READ | MappingFlags::WRITE,
                    )
                    .is_err()
                {
                    let mapped_len = virt.as_usize().saturating_sub(kernel_base);
                    if mapped_len != 0 {
                        let _ = guard.unmap(VirtAddr::from_usize(kernel_base), mapped_len);
                    }
                    return Err(());
                }
                virt += PAGE_SIZE_4K;
            }
            (kernel_base, range_len)
        };

        let kernel_va = kernel_base.checked_add(range_offset).ok_or(())?;
        if !sync_kernel_alias_to_current_aspace(pid, kernel_base, kernel_map_size) {
            let _ = ax_mm::kernel_aspace()
                .lock()
                .unmap(VirtAddr::from_usize(kernel_base), kernel_map_size);
            return Err(());
        }

        let mut table = USER_KERNEL_MAPPING_TABLE.lock();
        let table = table.get_or_insert_with(BTreeMap::new);
        if table.contains_key(&kernel_va) {
            let _ = ax_mm::kernel_aspace()
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
                kernel_va,
                kernel_base,
                kernel_map_size,
            },
        );
        Ok(kernel_va as u64)
    }

    unsafe fn unmap_user(&self, kernel_va: u64, len: usize) -> Result<(), ()> {
        let kernel_va = usize::try_from(kernel_va).map_err(|_| ())?;
        let mut table = USER_KERNEL_MAPPING_TABLE.lock();
        let table = table.as_mut().ok_or(())?;
        let registered = table.get(&kernel_va).ok_or(())?;
        if registered.requested_len != len {
            return Err(());
        }
        table.remove(&kernel_va);
        Ok(())
    }
}
