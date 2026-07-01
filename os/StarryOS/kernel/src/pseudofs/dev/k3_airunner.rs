use alloc::{boxed::Box, collections::btree_map::BTreeMap, sync::Arc};
use core::any::Any;

use ax_memory_addr::{MemoryAddr, PhysAddr, VirtAddr, VirtAddrRange, align_up_4k};
use ax_runtime::hal::{
    cpu::asm::user_copy,
    paging::{MappingFlags, PageSize},
};
use ax_sync::spin::SpinNoIrq;
use ax_task::{AxCpuMask, current, spawn};
use axfs_ng_vfs::{DeviceId, NodeFlags, VfsError, VfsResult};
use k3_aiScheduler::{K3SchedulerOps, scheduler::run_graph};
use k3_aiUabi::{AI_ABI_VERSION, AiGraphSubmitEntry, GraphSubmitKind};
use ov_channels::{ChannelId, SharedMemory};

use crate::{
    mm::{Backend, SharedPages, UserPtr, access_user_memory},
    pseudofs::DeviceOps,
    task::AsThread,
};

pub const K3_AI_IOC_BUILD_CHANNEL: u32 = 0x4B33_0001;
pub const K3_AI_IOC_SUBMIT_GRAPH: u32 = 0x4B33_0002;
pub const K3_AIRUNNER_DEVICE_ID: DeviceId = DeviceId::new(240, 10);
pub const K3_AIRUNNER_CHANNEL_COUNT: usize = 2;
// TODO(k3-hmp):
// K3 的 X100/A100 属于两类向量宽度不兼容的核心，后面需要在 Starry 补一层
// AI 线程分类和 cpumask 约束，而不是只做通用的 sched_setaffinity。
// 入口可以做成:
// 1. Linux 兼容的 /proc/set_ai_thread
// 2. 或者专用的 /dev/ai... / /dev/k3_airunner ioctl 控制面
// 但真正的约束点仍然要落在这里和任务调度路径里，避免线程在 X100/A100 之间错误迁移。
// TODO(k3-hmp-control):
// Linux K3 目前通过 /proc/set_ai_thread 把线程标记为 AI 类型，并把它限制到 A100 CPU 集合。
// Starry 后面也需要一个对应控制面，形式可以是:
// 1. 在这个设备上继续扩 ioctl
// 2. 或者单独拆一个 /dev/ai... 设备
// 这层控制不只是“设置亲和性”，还要把 X100/A100 的核心类型语义带进调度器，
// 防止已经使用某类向量/IME 状态的线程跨到另一类核心上运行。

// 必须和用户态 `k3_aiUabi::kd_uring::K3AiChannelBuildParam` 保持一致。
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct K3AiChannelBuildParam {
    pub user_va: u64,
    pub size_bytes: u64,
    pub channel_count: u32,
    pub flags: u32,
    pub owner_pid: u32,
    pub reserved0: u32,
    pub reserved1: u64,
}

// 记录某个进程已经注册好的 channel 共享区。
// 现在先只保存 SharedPages 的强引用，保证用户态映射和后续 guard 路径都还能访问。
pub struct RegisteredChannelMemory {
    pub user_va: usize,
    pub size_bytes: usize,
    pub channel_count: u32,
    pub shared_pages: Arc<SharedPages>,
    pub kernel_va: usize,
    pub kernel_map_size: usize,
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
pub struct RegisteredUserKernelMapping {
    pub pid: u32,
    pub user_va: usize,
    pub requested_len: usize,
    pub shared_pages: Arc<SharedPages>,
    pub kernel_va: usize,
    pub kernel_base: usize,
    pub kernel_map_size: usize,
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

// 当前先按 pid 维度记一份最小状态。
// 只要这里还持有 Arc，底层共享物理页就不会被回收。
static CHANNEL_MEMORY_TABLE: SpinNoIrq<Option<BTreeMap<u32, RegisteredChannelMemory>>> =
    SpinNoIrq::new(None);

// scheduler 的 tensor/blob 映射按返回给调度器的 kernel_va 索引。
static USER_KERNEL_MAPPING_TABLE: SpinNoIrq<Option<BTreeMap<usize, RegisteredUserKernelMapping>>> =
    SpinNoIrq::new(None);

pub struct K3AiRunner;

/// 把 kernel alias 映射同步到当前线程的进程地址空间，保证后续调度器能直接访问。
fn sync_kernel_alias_to_current_aspace(pid: u32, kernel_va: usize, kernel_map_size: usize) -> bool {
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
        aspace
            .page_table_mut()
            .cursor()
            .copy_from(kspace.page_table(), start, kernel_map_size);
    }
    crate::mm::flush_tlb_range_sync(start, kernel_map_size);
    info!(
        "k3_airunner: sync kernel alias done pid={}, kernel_va={:#x}, map_size={:#x}",
        pid, kernel_va, kernel_map_size
    );
    true
}

impl DeviceOps for K3AiRunner {
    fn read_at(&self, _buf: &mut [u8], _offset: u64) -> VfsResult<usize> {
        Err(VfsError::InvalidInput)
    }

    fn write_at(&self, _buf: &[u8], _offset: u64) -> VfsResult<usize> {
        Err(VfsError::InvalidInput)
    }

    fn ioctl(&self, cmd: u32, arg: usize) -> VfsResult<usize> {
        match cmd {
            K3_AI_IOC_BUILD_CHANNEL => {
                info!("k3_airunner: BUILD_CHANNEL ioctl reached, arg={arg:#x}");

                // 先从用户态读 build 参数。
                let build_param = UserPtr::<K3AiChannelBuildParam>::from(arg).get_as_mut()?;

                if build_param.user_va == 0 || build_param.size_bytes == 0 {
                    return Err(VfsError::InvalidInput);
                }

                // UAPI 使用固定宽度字段，这里先收窄成内核 usize。
                let user_va =
                    usize::try_from(build_param.user_va).map_err(|_| VfsError::InvalidInput)?;
                let size_bytes =
                    usize::try_from(build_param.size_bytes).map_err(|_| VfsError::InvalidInput)?;
                if size_bytes == 0 {
                    return Err(VfsError::InvalidInput);
                }

                // 当前实现只接受用户态通过 MAP_SHARED 建出来的共享区。
                // 这样这段内存背后一定有 SharedPages，可以直接抓到 Arc 保活。
                let range_start = VirtAddr::from(user_va).align_down_4k();
                let range_end = VirtAddr::from(user_va + size_bytes).align_up_4k();
                let range_len = range_end - range_start;

                // 以当前线程所属进程作为 channel 所有者。
                let curr = current();
                let pid = curr.as_thread().proc_data.proc.pid();
                let aspace_arc = curr.as_thread().proc_data.aspace();
                let aspace = aspace_arc.lock();

                // 用户给的是 user_va，先确认整段地址还落在同一个 VMA 中。
                let area = aspace
                    .find_area(VirtAddr::from(user_va))
                    .ok_or(VfsError::BadAddress)?;
                if area.start() > range_start || area.end() < range_end {
                    return Err(VfsError::InvalidInput);
                }

                // 只接受 SharedBackend，这样能拿到 SharedPages 的 Arc 保活物理页。
                let shared_pages = match area.backend() {
                    Backend::Shared(shared) => shared.pages().clone(),
                    _ => {
                        info!(
                            "k3_airunner: BUILD_CHANNEL rejected non-shared backend, pid={}, \
                             va={:#x}, size={:#x}",
                            pid, user_va, size_bytes
                        );
                        return Err(VfsError::InvalidInput);
                    }
                };
                drop(aspace);

                let shared_memory_size =
                    core::mem::size_of::<SharedMemory<K3_AIRUNNER_CHANNEL_COUNT>>();
                // 现在内核和用户态都约定 ovchannel 为 SharedMemory<2>。
                if build_param.channel_count != K3_AIRUNNER_CHANNEL_COUNT as u32
                    || size_bytes < shared_memory_size
                {
                    info!(
                        "k3_airunner: BUILD_CHANNEL rejected channel layout pid={}, channels={}, \
                         size={:#x}",
                        pid, build_param.channel_count, size_bytes
                    );
                    return Err(VfsError::InvalidInput);
                }

                // range_len 已经页对齐，alias 需要映射同样数量的 4K 页。
                let required_pages = range_len / PageSize::Size4K as usize;
                if shared_pages.len() < required_pages {
                    info!(
                        "k3_airunner: BUILD_CHANNEL rejected short SharedPages pid={}, pages={}, \
                         required={}",
                        pid,
                        shared_pages.len(),
                        required_pages
                    );
                    return Err(VfsError::InvalidInput);
                }

                {
                    // 目前还没有 DROP_CHANNEL，重复注册会泄漏 kernel alias，先直接拒绝。
                    let table = CHANNEL_MEMORY_TABLE.lock();
                    if table.as_ref().is_some_and(|table| table.contains_key(&pid)) {
                        info!(
                            "k3_airunner: BUILD_CHANNEL pid={} already registered channel memory",
                            pid
                        );
                        return Err(VfsError::AlreadyExists);
                    }
                }

                let (kernel_va, kernel_map_size) = {
                    let kspace = ax_mm::kernel_aspace();
                    let mut guard = kspace.lock();
                    // 用户 VA 连续不代表内核能直接连续访问；这里重新找一段连续 kernel VA。
                    let mut virt_start = guard
                        .find_free_area(
                            guard.base(),
                            range_len,
                            VirtAddrRange::new(guard.base(), guard.end()),
                        )
                        .ok_or(VfsError::NoMemory)?;
                    let kernel_va = virt_start.as_usize();
                    // 将不连续的 SharedPages 逐页拼到连续 kernel VA 上。
                    for paddr in shared_pages.iter().take(required_pages) {
                        if guard
                            .map_linear(
                                virt_start,
                                PhysAddr::from_usize(paddr.as_usize()),
                                PageSize::Size4K as usize,
                                MappingFlags::READ | MappingFlags::WRITE,
                            )
                            .is_err()
                        {
                            // 中途失败要撤掉已经映射的 alias，避免 kernel VA 泄漏。
                            let mapped_len = virt_start.as_usize() - kernel_va;
                            if mapped_len != 0 {
                                let _ = guard.unmap(VirtAddr::from_usize(kernel_va), mapped_len);
                            }
                            return Err(VfsError::InvalidInput);
                        }
                        virt_start += PageSize::Size4K as usize;
                    }
                    (kernel_va, range_len)
                };

                // kernel alias 建好后，必须同步到当前线程的进程地址空间，否则调度器无法直接访问。
                if !sync_kernel_alias_to_current_aspace(pid, kernel_va, kernel_map_size) {
                    let kspace = ax_mm::kernel_aspace();
                    let _ = kspace
                        .lock()
                        .unmap(VirtAddr::from_usize(kernel_va), kernel_map_size);
                    return Err(VfsError::BadAddress);
                }

                {
                    let mut table = CHANNEL_MEMORY_TABLE.lock();
                    let table = table.get_or_insert_with(BTreeMap::new);
                    // 从这里开始，SUBMIT_GRAPH 只信任 kernel_va，不再依赖用户 VA。
                    table.insert(
                        pid,
                        RegisteredChannelMemory {
                            user_va,
                            size_bytes,
                            channel_count: build_param.channel_count,
                            shared_pages,
                            kernel_va,
                            kernel_map_size,
                        },
                    );
                    if let Some(registered) = table.get(&pid) {
                        info!(
                            "k3_airunner: BUILD_CHANNEL keepalive pid={}, user_va={:#x}, \
                             size={:#x}, channels={}, kernel_va={:#x}, kernel_map_size={:#x}",
                            pid,
                            registered.user_va,
                            registered.size_bytes,
                            registered.channel_count,
                            registered.kernel_va,
                            registered.kernel_map_size
                        );
                        let _ = registered.shared_pages.len();
                    }
                }

                // 回填 owner pid，用户态后面可以拿它做日志或调试匹配。
                build_param.owner_pid = pid;

                // 这里先不真正创建 ov-channel sender/receiver，也不唤醒 guard。
                info!(
                    "k3_airunner: BUILD_CHANNEL registered pid={}, va={:#x}, size={:#x}, pages={}",
                    pid,
                    user_va,
                    range_len,
                    align_up_4k(size_bytes) >> 12
                );
                Ok(0)
            }
            K3_AI_IOC_SUBMIT_GRAPH => {
                info!("k3_airunner: SUBMIT_GRAPH ioctl reached, arg={arg:#x}");

                let curr = current();
                let pid = curr.as_thread().proc_data.proc.pid();
                // 取出 BUILD_CHANNEL 建好的 kernel alias；clone Arc 保证本次 submit 期间页仍存活。
                let (kernel_va, size_bytes, channel_count, _shared_pages) = {
                    let table = CHANNEL_MEMORY_TABLE.lock();
                    let registered = table
                        .as_ref()
                        .and_then(|table| table.get(&pid))
                        .ok_or_else(|| {
                            info!(
                                "k3_airunner: SUBMIT_GRAPH no registered channel memory pid={}",
                                pid
                            );
                            VfsError::InvalidInput
                        })?;
                    info!(
                        "k3_airunner: SUBMIT_GRAPH registered channel pid={}, user_va={:#x}, \
                         kernel_va={:#x}, size={:#x}, kernel_map_size={:#x}, channels={}, pages={}",
                        pid,
                        registered.user_va,
                        registered.kernel_va,
                        registered.size_bytes,
                        registered.kernel_map_size,
                        registered.channel_count,
                        registered.shared_pages.len()
                    );
                    (
                        registered.kernel_va,
                        registered.size_bytes,
                        registered.channel_count,
                        registered.shared_pages.clone(),
                    )
                };

                // 防御性检查，后面支持更多 channel 时这里再扩展。
                if channel_count != K3_AIRUNNER_CHANNEL_COUNT as u32
                    || size_bytes < core::mem::size_of::<SharedMemory<K3_AIRUNNER_CHANNEL_COUNT>>()
                {
                    info!(
                        "k3_airunner: SUBMIT_GRAPH rejected channel layout pid={}, channels={}, \
                         size={:#x}",
                        pid, channel_count, size_bytes
                    );
                    return Err(VfsError::InvalidInput);
                }

                // 当前 submit 只验证 ovchannel 通路：用户态 channel 0 发 notification，
                // BUILD_CHANNEL 已经为这批 SharedPages 建立连续 kernel VA alias。
                let shm = unsafe { SharedMemory::<K3_AIRUNNER_CHANNEL_COUNT>::at(kernel_va) };

                // magic/version 不对说明共享区没有初始化或已经被破坏。
                for channel_index in 0..K3_AIRUNNER_CHANNEL_COUNT {
                    info!(
                        "k3_airunner: SUBMIT_GRAPH channel {} is_valid begin pid={}",
                        channel_index, pid
                    );
                    let channel =
                        unsafe { shm.channel_unchecked(ChannelId::new(channel_index as u8)) };
                    info!(
                        "k3_airunner: SUBMIT_GRAPH channel {} ref acquired pid={}, ptr={:#x}",
                        channel_index, pid, channel as *const _ as usize
                    );
                    let channel_valid = channel.is_valid();
                    info!(
                        "k3_airunner: SUBMIT_GRAPH channel {} is_valid done pid={}, valid={}",
                        channel_index, pid, channel_valid
                    );
                    if !channel_valid {
                        info!(
                            "k3_airunner: SUBMIT_GRAPH rejected invalid shared memory pid={}, \
                             kernel_va={:#x}, channel={}",
                            pid, kernel_va, channel_index
                        );
                        return Err(VfsError::InvalidInput);
                    }
                }
                info!("k3_airunner: SUBMIT_GRAPH shared memory valid pid={pid}");

                // 最小闭环先固定读 channel 0。
                info!("k3_airunner: SUBMIT_GRAPH receiver channel 0 init begin pid={pid}");
                let receiver = shm.receiver(ChannelId::new(0)).map_err(|err| {
                    info!(
                        "k3_airunner: SUBMIT_GRAPH receiver channel 0 init failed pid={}, err={:?}",
                        pid, err
                    );
                    VfsError::InvalidInput
                })?;
                info!("k3_airunner: SUBMIT_GRAPH receiver channel 0 init done pid={pid}");

                // complete环
                info!("k3_airunner: SUBMIT_GRAPH sender channel 1 init begin pid={pid}");
                let complete_sender = shm.sender(ChannelId::new(1)).map_err(|err| {
                    info!(
                        "k3_airunner: SUBMIT_GRAPH sender channel 1 init failed pid={}, err={:?}",
                        pid, err
                    );
                    VfsError::InvalidInput
                })?;
                info!("k3_airunner: SUBMIT_GRAPH sender channel 1 init done pid={pid}");
                info!("k3_airunner: SUBMIT_GRAPH receiver channel 1 init begin pid={pid}");
                let _complete_reciver = shm.receiver(ChannelId::new(1)).map_err(|err| {
                    info!(
                        "k3_airunner: SUBMIT_GRAPH receiver channel 1 init failed pid={}, err={:?}",
                        pid, err
                    );
                    VfsError::InvalidInput
                })?;
                info!("k3_airunner: SUBMIT_GRAPH receiver channel 1 init done pid={pid}");

                // try_recv 非阻塞；空队列先返回 WouldBlock，后面再接 poll/async。
                let message = receiver.try_recv().ok_or_else(|| {
                    info!("k3_airunner: SUBMIT_GRAPH channel empty pid={}", pid);
                    VfsError::WouldBlock
                })?;

                // 用户发送 AiGraphSubmitEntry 的序列化数据。
                if let Some(payload) = message.as_data() {
                    let entry_size = core::mem::size_of::<AiGraphSubmitEntry>();
                    if payload.len() < entry_size {
                        return Err(VfsError::InvalidInput);
                    }

                    // 用户态按 repr(C) 直接发送 entry 字节，这里按相同 ABI 读回来。
                    let graph_entry = unsafe {
                        core::ptr::read_unaligned(payload.as_ptr().cast::<AiGraphSubmitEntry>())
                    };

                    // 验证内核 abi_version 与用户 abi_version 是否匹配。
                    if graph_entry.abi_version != AI_ABI_VERSION {
                        info!(
                            "k3_airunner: SUBMIT_GRAPH rejected abi mismatch pid={}, user={}, \
                             kernel={}",
                            pid, graph_entry.abi_version, AI_ABI_VERSION
                        );
                        return Err(VfsError::InvalidInput);
                    }

                    // 当前只接收真正的 graph submit，cancel/query 后面单独走分支。
                    if graph_entry.submit_kind != GraphSubmitKind::GRAPH_SUBMIT {
                        info!(
                            "k3_airunner: SUBMIT_GRAPH rejected submit kind pid={}, kind={}",
                            pid, graph_entry.submit_kind.0
                        );
                        return Err(VfsError::OperationNotSupported);
                    }

                    // scheduler 需要 graph blob 的用户 VA 和大小。
                    if graph_entry.graph_user_va == 0 || graph_entry.graph_size == 0 {
                        info!(
                            "k3_airunner: SUBMIT_GRAPH rejected empty graph blob pid={}, \
                             graph_va={:#x}, size={:#x}",
                            pid, graph_entry.graph_user_va, graph_entry.graph_size
                        );
                        return Err(VfsError::InvalidInput);
                    }

                    use alloc::{vec, vec::Vec};
                    // 从graph_user_va 和 graph_size反序列化出parsed graph
                    // 必须从user空间copy过来,防止后续被篡改
                    info!(
                        "k3_airunner: SUBMIT_GRAPH graph_size usize conversion begin pid={}, \
                         graph_size={:#x}",
                        pid, graph_entry.graph_size
                    );
                    let graph_size = usize::try_from(graph_entry.graph_size)
                        .expect("can't read usize from graph");
                    info!(
                        "k3_airunner: SUBMIT_GRAPH graph_size usize conversion done pid={}, \
                         graph_size={:#x}",
                        pid, graph_size
                    );
                    info!(
                        "k3_airunner: SUBMIT_GRAPH graph blob alloc begin pid={}, len={:#x}",
                        pid, graph_size
                    );
                    let mut blob_slice: Vec<u8> = vec![0_u8; graph_size];
                    info!(
                        "k3_airunner: SUBMIT_GRAPH graph blob alloc done pid={}, len={:#x}",
                        pid,
                        blob_slice.len()
                    );
                    info!(
                        "k3_airunner: SUBMIT_GRAPH copy graph blob begin pid={}, user_va={:#x}, \
                         len={:#x}",
                        pid,
                        graph_entry.graph_user_va,
                        blob_slice.len()
                    );
                    let copy_result = unsafe {
                        K3AiRunner.copy_from_user(graph_entry.graph_user_va, &mut blob_slice)
                    };
                    info!(
                        "k3_airunner: SUBMIT_GRAPH copy graph blob done pid={}, ok={}",
                        pid,
                        copy_result.is_ok()
                    );
                    if let Err(()) = copy_result {
                        info!("data copy fail!");
                    }

                    let parsed_graph =
                        k3_aiUabi::AiGraphParser::parse(&blob_slice).expect("parse fail");
                    use k3_aiScheduler::kd_kring::resolve_parsed_graph;
                    let task_link = resolve_parsed_graph(0, &parsed_graph).expect("parse fail");

                    // 我们在这里必须将用户的tensor映射为内核虚拟地址

                    use k3_aiUabi::AiTensorDesc;
                    // 遍历 AiGraphNode
                    for node in task_link.iter() {
                        info!("I received the node:");
                        info!("  node_id: {}", node.node_id);
                        info!("  op: {:?}", node.desc.op);
                        info!("  target_hint: {:?}", node.desc.target_hint);
                        info!(
                            "  input_count: {}, output_count: {}",
                            node.desc.input_count, node.desc.output_count
                        );

                        // 打印输入 tensors 信息
                        for i in 0..node.desc.input_count as usize {
                            let tensor = &node.desc.tensors[i];
                            info!(
                                "  input[{}]: dtype={:?}, ndim={}, shape={:?}",
                                i,
                                tensor.dtype,
                                tensor.ndim,
                                &tensor.shape[..tensor.ndim as usize]
                            );
                        }

                        // 打印输出 tensors 信息
                        for i in 0..node.desc.output_count as usize {
                            let tensor = &node.desc.tensors[node.desc.input_count as usize + i];
                            info!(
                                "  output[{}]: dtype={:?}, ndim={}, shape={:?}",
                                i,
                                tensor.dtype,
                                tensor.ndim,
                                &tensor.shape[..tensor.ndim as usize]
                            );
                        }

                        // 映射输入 tensors 到内核地址
                        for i in 0..node.desc.input_count as usize {
                            let tensor: &mut AiTensorDesc;
                            unsafe {
                                // SAFETY: node.desc.tensors[i] is parsed from the copied blob,
                                // so reborrowing it as a mutable tensor descriptor does not
                                // alias user memory.
                                let addr = (&node.desc.tensors[i]) as *const _ as usize + 1;
                                let trans_tensor = &mut *((addr - 1) as *mut AiTensorDesc);
                                tensor = trans_tensor;
                            }

                            // guard
                            if tensor.kernel_va != 0 {
                                // 非法参数,阻止
                                info!("kernel_va should writen by kernel!");
                            }

                            if tensor.user_va != 0 && tensor.size_bytes != 0 {
                                match unsafe {
                                    K3AiRunner.map_user_to_kernel(
                                        tensor.user_va,
                                        tensor.size_bytes as usize,
                                    )
                                } {
                                    Ok(kernel_va) => {
                                        // 回填kernel_va
                                        tensor.kernel_va = kernel_va;

                                        info!(
                                            "  input[{}] mapped: user_va={:#x}, size={:#x} -> \
                                             kernel_va={:#x}",
                                            i, tensor.user_va, tensor.size_bytes, kernel_va
                                        );
                                    }
                                    Err(_) => {
                                        info!(
                                            "  input[{}] map failed: user_va={:#x}, size={:#x}",
                                            i, tensor.user_va, tensor.size_bytes
                                        );
                                    }
                                }
                            }
                        }

                        // 映射输出 tensors 到内核地址
                        for i in 0..node.desc.output_count as usize {
                            let tensor: &mut AiTensorDesc;
                            unsafe {
                                // SAFETY: node.desc.tensors[i] is parsed from the copied blob,
                                // so reborrowing it as a mutable tensor descriptor does not
                                // alias user memory.
                                let addr = (&node.desc.tensors[node.desc.input_count as usize + i])
                                    as *const _ as usize
                                    + 1;
                                let trans_tensor = &mut *((addr - 1) as *mut AiTensorDesc);
                                tensor = trans_tensor;
                            }
                            if tensor.user_va != 0 && tensor.size_bytes != 0 {
                                match unsafe {
                                    K3AiRunner.map_user_to_kernel(
                                        tensor.user_va,
                                        tensor.size_bytes as usize,
                                    )
                                } {
                                    Ok(kernel_va) => {
                                        // 回填kernel_va
                                        tensor.kernel_va = kernel_va;

                                        info!(
                                            "  output[{}] mapped: user_va={:#x}, size={:#x} -> \
                                             kernel_va={:#x}",
                                            i, tensor.user_va, tensor.size_bytes, kernel_va
                                        );
                                    }
                                    Err(_) => {
                                        info!(
                                            "  output[{}] map failed: user_va={:#x}, size={:#x}",
                                            i, tensor.user_va, tensor.size_bytes
                                        );
                                    }
                                }
                            }
                        }

                        info!("  attr_size: {} bytes", node.desc.attr_size);
                    }

                    // Only enqueue from normal task context. Enqueuing through an
                    // IPI callback can interrupt the scheduler worker while it
                    // holds the queue spinlock, then re-enter the same lock from
                    // interrupt context and leave that CPU spinning forever.
                    info!(
                        "k3_airunner: scheduler run_graph begin pid={}, token={}",
                        pid, graph_entry.user_token
                    );
                    if run_graph(
                        graph_entry.user_token,
                        Box::new(K3AiRunner),
                        complete_sender,
                        task_link,
                    )
                    .is_err()
                    {
                        info!("k3_airunner: scheduler run_graph failed");
                        return Err(VfsError::InvalidInput);
                    }
                    info!(
                        "k3_airunner: scheduler run_graph done pid={}, token={}",
                        pid, graph_entry.user_token
                    );

                    info!(
                        "k3_airunner: SUBMIT_GRAPH recv AiGraphSubmitEntry pid={}, token={}, \
                         graph_va={:#x}, graph_size={:#x}",
                        pid,
                        graph_entry.user_token,
                        graph_entry.graph_user_va,
                        graph_entry.graph_size
                    );
                    Ok(0)
                } else {
                    info!(
                        "k3_airunner: SUBMIT_GRAPH recv non-notification pid={}, msg={:?}",
                        pid, message
                    );
                    Err(VfsError::InvalidInput)
                }
            }
            _ => Err(VfsError::OperationNotSupported),
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn flags(&self) -> NodeFlags {
        NodeFlags::NON_CACHEABLE
    }
}

impl K3SchedulerOps for K3AiRunner {
    fn spawn_thread(&self, f: fn(usize), arg: usize) {
        spawn(move || {
            // 这里让 scheduler worker 线程固定在 CPU 10 上，避免和其他非 AI 线程混跑。
            let affinity_set = ax_task::set_current_affinity(AxCpuMask::one_shot(10));
            info!(
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
