//! `DeviceOps` 控制面实现：`BUILD_CHANNEL` / `SUBMIT_GRAPH` ioctl。

use alloc::{boxed::Box, collections::btree_map::BTreeMap, vec, vec::Vec};
use core::any::Any;

use ax_memory_addr::{MemoryAddr, PAGE_SIZE_4K, PhysAddr, VirtAddr, VirtAddrRange, align_up_4k};
use ax_runtime::hal::paging::MappingFlags;
use ax_task::current;
use axfs_ng_vfs::{NodeFlags, VfsError, VfsResult};
use k3_ai_scheduler::{K3SchedulerOps, kd_kring::resolve_parsed_graph, scheduler::run_graph};
use k3_ai_uabi::{
    AI_ABI_VERSION, AiGraphParser, AiGraphSubmitEntry, AiTensorDesc, GraphSubmitKind,
    K3_AI_IOC_BUILD_CHANNEL, K3_AI_IOC_SUBMIT_GRAPH, K3_CHANNEL_COUNT, K3_CHANNEL_RECIVERID,
    K3_CHANNEL_SNEDERID, K3AiChannelBuildParam, KernelVa, MAX_DIM,
};
use ov_channels::{ChannelId, SharedMemory};

use super::{
    registry::{
        CHANNEL_MEMORY_TABLE, RegisteredChannelMemory, sync_kernel_alias_to_current_aspace,
    },
    runner::K3AiRunner,
};
use crate::{
    mm::{Backend, UserPtr},
    pseudofs::DeviceOps,
    task::AsThread,
};

impl K3AiRunner {
    /// `BUILD_CHANNEL` ioctl: 注册用户态 ovchannel 共享区并建立连续 kernel VA alias。
    fn build_channel(&self, arg: usize) -> VfsResult<usize> {
        info!("k3_airunner: BUILD_CHANNEL ioctl reached, arg={arg:#x}");

        // 先从用户态读 build 参数。
        let build_param = UserPtr::<K3AiChannelBuildParam>::from(arg).get_as_mut()?;

        // 以当前线程所属进程作为 channel 所有者。
        let curr = current();
        let pid = curr.as_thread().proc_data.proc.pid();

        if build_param.abi_version != AI_ABI_VERSION {
            error!(
                "k3_airunner: BUILD_CHANNEL rejected abi mismatch pid={}, user={}, kernel={}",
                pid, build_param.abi_version, AI_ABI_VERSION
            );
            return Err(VfsError::InvalidInput);
        }

        if build_param.user_va == 0 || build_param.size_bytes == 0 {
            return Err(VfsError::InvalidInput);
        }

        // UAPI 使用固定宽度字段，这里先收窄成内核 usize。
        let user_va = usize::try_from(build_param.user_va).map_err(|_| VfsError::InvalidInput)?;
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
                    "k3_airunner: BUILD_CHANNEL rejected non-shared backend, pid={}, va={:#x}, \
                     size={:#x}",
                    pid, user_va, size_bytes
                );
                return Err(VfsError::InvalidInput);
            }
        };
        drop(aspace);

        let shared_memory_size = core::mem::size_of::<SharedMemory<K3_CHANNEL_COUNT>>();
        // 现在内核和用户态都约定 ovchannel 为 SharedMemory<2>。
        if build_param.channel_count != K3_CHANNEL_COUNT as u32 || size_bytes < shared_memory_size {
            info!(
                "k3_airunner: BUILD_CHANNEL rejected channel layout pid={}, channels={}, \
                 size={:#x}",
                pid, build_param.channel_count, size_bytes
            );
            return Err(VfsError::InvalidInput);
        }

        // range_len 已经页对齐，alias 需要映射同样数量的 4K 页。
        let required_pages = range_len / PAGE_SIZE_4K;
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
            // 幂等：同一 pid 重复 BUILD_CHANNEL 时，检查参数是否与已注册的一致。
            // 一致则直接返回成功，不做重复的 kernel alias 映射，避免泄漏。
            // 参数不一致则拒绝。
            let table = CHANNEL_MEMORY_TABLE.lock();
            if let Some(table) = table.as_ref()
                && let Some(existing) = table.get(&pid)
            {
                if existing.user_va == user_va
                    && existing.size_bytes == size_bytes
                    && existing.channel_count == build_param.channel_count
                {
                    // 参数完全一致，幂等返回。
                    info!(
                        "k3_airunner: BUILD_CHANNEL pid={} already registered, idempotent return, \
                         user_va={:#x}, size={:#x}, channels={}",
                        pid, user_va, size_bytes, build_param.channel_count
                    );
                    build_param.owner_pid = pid;
                    return Ok(0);
                }
                // 参数不一致，打印差异后拒绝。
                if existing.user_va != user_va {
                    error!(
                        "k3_airunner: BUILD_CHANNEL pid={} user_va mismatch: existing={:#x}, \
                         new={:#x}",
                        pid, existing.user_va, user_va
                    );
                }
                if existing.size_bytes != size_bytes {
                    error!(
                        "k3_airunner: BUILD_CHANNEL pid={} size_bytes mismatch: existing={:#x}, \
                         new={:#x}",
                        pid, existing.size_bytes, size_bytes
                    );
                }
                if existing.channel_count != build_param.channel_count {
                    error!(
                        "k3_airunner: BUILD_CHANNEL pid={} channel_count mismatch: existing={}, \
                         new={}",
                        pid, existing.channel_count, build_param.channel_count
                    );
                }
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
                        PAGE_SIZE_4K,
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
                virt_start += PAGE_SIZE_4K;
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
                    "k3_airunner: BUILD_CHANNEL keepalive pid={}, user_va={:#x}, size={:#x}, \
                     channels={}, kernel_va={:#x}, kernel_map_size={:#x}",
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

    /// `SUBMIT_GRAPH` ioctl: 从 channel 读取 graph 提交项，反序列化并交给调度器执行。
    fn submit_graph(&self, arg: usize) -> VfsResult<usize> {
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
        if channel_count != K3_CHANNEL_COUNT as u32
            || size_bytes < core::mem::size_of::<SharedMemory<K3_CHANNEL_COUNT>>()
        {
            info!(
                "k3_airunner: SUBMIT_GRAPH rejected channel layout pid={}, channels={}, size={:#x}",
                pid, channel_count, size_bytes
            );
            return Err(VfsError::InvalidInput);
        }

        // 当前 submit 只验证 ovchannel 通路：用户态 channel 0 发 notification，
        // BUILD_CHANNEL 已经为这批 SharedPages 建立连续 kernel VA alias。
        let shm = unsafe { SharedMemory::<K3_CHANNEL_COUNT>::at(kernel_va) };

        // magic/version 不对说明共享区没有初始化或已经被破坏。
        for channel_index in 0..K3_CHANNEL_COUNT {
            info!(
                "k3_airunner: SUBMIT_GRAPH channel {} is_valid begin pid={}",
                channel_index, pid
            );
            let channel = unsafe { shm.channel_unchecked(ChannelId::new(channel_index as u8)) };
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
        let receiver = shm
            .receiver(ChannelId::new(K3_CHANNEL_SNEDERID))
            .map_err(|err| {
                info!(
                    "k3_airunner: SUBMIT_GRAPH receiver channel 0 init failed pid={}, err={:?}",
                    pid, err
                );
                VfsError::InvalidInput
            })?;
        info!("k3_airunner: SUBMIT_GRAPH receiver channel 0 init done pid={pid}");

        // complete环
        info!("k3_airunner: SUBMIT_GRAPH sender channel 1 init begin pid={pid}");
        let complete_sender = shm
            .sender(ChannelId::new(K3_CHANNEL_RECIVERID))
            .map_err(|err| {
                info!(
                    "k3_airunner: SUBMIT_GRAPH sender channel 1 init failed pid={}, err={:?}",
                    pid, err
                );
                VfsError::InvalidInput
            })?;
        info!("k3_airunner: SUBMIT_GRAPH sender channel 1 init done pid={pid}");
        info!("k3_airunner: SUBMIT_GRAPH receiver channel 1 init begin pid={pid}");
        let _complete_reciver =
            shm.receiver(ChannelId::new(K3_CHANNEL_RECIVERID))
                .map_err(|err| {
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
        let Some(payload) = message.as_data() else {
            info!(
                "k3_airunner: SUBMIT_GRAPH recv non-notification pid={}, msg={:?}",
                pid, message
            );
            return Err(VfsError::InvalidInput);
        };

        let entry_size = core::mem::size_of::<AiGraphSubmitEntry>();
        if payload.len() < entry_size {
            return Err(VfsError::InvalidInput);
        }

        // 用户态按 repr(C) 直接发送 entry 字节，这里按相同 ABI 读回来。
        let graph_entry =
            unsafe { core::ptr::read_unaligned(payload.as_ptr().cast::<AiGraphSubmitEntry>()) };

        // 验证内核 abi_version 与用户 abi_version 是否匹配。
        if graph_entry.abi_version != AI_ABI_VERSION {
            error!(
                "k3_airunner: SUBMIT_GRAPH rejected abi mismatch pid={}, user={}, kernel={}",
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
                "k3_airunner: SUBMIT_GRAPH rejected empty graph blob pid={}, graph_va={:#x}, \
                 size={:#x}",
                pid,
                graph_entry.graph_user_va.get(),
                graph_entry.graph_size.get()
            );
            return Err(VfsError::InvalidInput);
        }

        // 从graph_user_va 和 graph_size反序列化出parsed graph
        // 必须从user空间copy过来,防止后续被篡改
        info!(
            "k3_airunner: SUBMIT_GRAPH graph_size usize conversion begin pid={}, graph_size={:#x}",
            pid,
            graph_entry.graph_size.get()
        );
        let graph_size = graph_entry
            .graph_size
            .try_as_usize()
            .map_err(|_| VfsError::InvalidInput)?;
        info!(
            "k3_airunner: SUBMIT_GRAPH graph_size usize conversion done pid={}, graph_size={:#x}",
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
            "k3_airunner: SUBMIT_GRAPH copy graph blob begin pid={}, user_va={:#x}, len={:#x}",
            pid,
            graph_entry.graph_user_va.get(),
            blob_slice.len()
        );
        let copy_result =
            unsafe { K3AiRunner.copy_from_user(graph_entry.graph_user_va.get(), &mut blob_slice) };
        info!(
            "k3_airunner: SUBMIT_GRAPH copy graph blob done pid={}, ok={}",
            pid,
            copy_result.is_ok()
        );
        if copy_result.is_err() {
            error!("k3_airunner: SUBMIT_GRAPH copy graph blob failed pid={pid}");
            return Err(VfsError::BadAddress);
        }

        let parsed_graph = AiGraphParser::parse(&blob_slice).map_err(|err| {
            error!("k3_airunner: SUBMIT_GRAPH graph parse failed pid={pid}, err={err:?}");
            VfsError::InvalidInput
        })?;
        let task_link = resolve_parsed_graph(0, &parsed_graph).map_err(|err| {
            error!("k3_airunner: SUBMIT_GRAPH graph resolve failed pid={pid}, err={err:?}");
            VfsError::InvalidInput
        })?;

        // 我们在这里必须将用户的tensor映射为内核虚拟地址

        // 遍历 AiGraphNode
        for node in task_link.iter() {
            let input_count = node
                .desc
                .input_count
                .try_as_usize()
                .map_err(|_| VfsError::InvalidInput)?;
            let output_count = node
                .desc
                .output_count
                .try_as_usize()
                .map_err(|_| VfsError::InvalidInput)?;
            let total_count = node
                .desc
                .input_count
                .checked_total(node.desc.output_count)
                .map_err(|_| VfsError::InvalidInput)?;
            if total_count > node.desc.tensors.len() {
                return Err(VfsError::InvalidInput);
            }

            info!("I received the node:");
            info!("  node_id: {}", node.node_id);
            info!("  op: {:?}", node.desc.op);
            info!("  target_hint: {:?}", node.desc.target_hint);
            info!(
                "  input_count: {}, output_count: {}",
                node.desc.input_count, node.desc.output_count
            );

            // 打印输入 tensors 信息
            for i in 0..input_count {
                let tensor = &node.desc.tensors[i];
                let ndim = tensor
                    .ndim
                    .try_under_max(MAX_DIM)
                    .map_err(|_| VfsError::InvalidInput)?;
                info!(
                    "  input[{}]: dtype={:?}, ndim={}, shape={:?}",
                    i,
                    tensor.dtype,
                    tensor.ndim,
                    &tensor.shape[..ndim]
                );
            }

            // 打印输出 tensors 信息
            for i in 0..output_count {
                let tensor = &node.desc.tensors[input_count + i];
                let ndim = tensor
                    .ndim
                    .try_under_max(MAX_DIM)
                    .map_err(|_| VfsError::InvalidInput)?;
                info!(
                    "  output[{}]: dtype={:?}, ndim={}, shape={:?}",
                    i,
                    tensor.dtype,
                    tensor.ndim,
                    &tensor.shape[..ndim]
                );
            }

            // 映射输入 tensors 到内核地址
            for i in 0..input_count {
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
                    error!("kernel_va should writen by kernel!");
                    return Err(ax_errno::AxError::BadAddress);
                }

                if tensor.user_va != 0 && tensor.size_bytes != 0 {
                    let user_va = tensor.user_va.get();
                    let size_bytes = tensor
                        .size_bytes
                        .try_as_usize()
                        .map_err(|_| VfsError::InvalidInput)?;
                    match unsafe { K3AiRunner.map_user_to_kernel(user_va, size_bytes) } {
                        Ok(kernel_va) => {
                            // kernel_va不能为0
                            assert_ne!(
                                kernel_va, 0,
                                "When map user tensor to kernel, will mapped a null ptr!"
                            );

                            // 回填kernel_va
                            tensor.kernel_va = KernelVa::new(kernel_va);

                            info!(
                                "  input[{}] mapped: user_va={:#x}, size={:#x} -> kernel_va={:#x}",
                                i,
                                user_va,
                                tensor.size_bytes.get(),
                                kernel_va
                            );
                        }
                        Err(_) => {
                            error!(
                                "  input[{}] map failed: user_va={:#x}, size={:#x}",
                                i,
                                user_va,
                                tensor.size_bytes.get()
                            );
                            return Err(ax_errno::AxError::BadAddress);
                        }
                    }
                } else {
                    error!("Tensor va or tensor size can't be null ptr!");
                    return Err(ax_errno::AxError::BadAddress);
                }
            }

            // 映射输出 tensors 到内核地址
            for i in 0..output_count {
                let tensor: &mut AiTensorDesc;
                unsafe {
                    // SAFETY: node.desc.tensors[i] is parsed from the copied blob,
                    // so reborrowing it as a mutable tensor descriptor does not
                    // alias user memory.
                    let addr = (&node.desc.tensors[input_count + i]) as *const _ as usize + 1;
                    let trans_tensor = &mut *((addr - 1) as *mut AiTensorDesc);
                    tensor = trans_tensor;
                }
                if tensor.user_va != 0 && tensor.size_bytes != 0 {
                    let user_va = tensor.user_va.get();
                    let size_bytes = tensor
                        .size_bytes
                        .try_as_usize()
                        .map_err(|_| VfsError::InvalidInput)?;
                    match unsafe { K3AiRunner.map_user_to_kernel(user_va, size_bytes) } {
                        Ok(kernel_va) => {
                            // 回填kernel_va
                            tensor.kernel_va = KernelVa::new(kernel_va);

                            info!(
                                "  output[{}] mapped: user_va={:#x}, size={:#x} -> kernel_va={:#x}",
                                i,
                                user_va,
                                tensor.size_bytes.get(),
                                kernel_va
                            );
                        }
                        Err(_) => {
                            info!(
                                "  output[{}] map failed: user_va={:#x}, size={:#x}",
                                i,
                                user_va,
                                tensor.size_bytes.get()
                            );
                        }
                    }
                }
            }

            info!("  attr_size: {} bytes", node.desc.attr_size);
        }

        // 这只是入队，真正的调度器 worker 线程会在后台执行。
        info!(
            "k3_airunner: scheduler run_graph begin pid={}, token={}",
            pid, graph_entry.user_token
        );

        // release 屏障
        core::sync::atomic::fence(core::sync::atomic::Ordering::Release);

        if run_graph(
            graph_entry.user_token,
            Box::new(K3AiRunner),
            complete_sender,
            task_link,
        )
        .is_err()
        {
            error!("k3_airunner: scheduler run_graph failed");
            return Err(VfsError::InvalidInput);
        }

        // 到这里任务已经提交成功,等待worker进行消费

        info!(
            "k3_airunner: scheduler enqueue graph done pid={}, token={}",
            pid, graph_entry.user_token
        );

        info!(
            "k3_airunner: SUBMIT_GRAPH recv AiGraphSubmitEntry pid={}, token={}, graph_va={:#x}, \
             graph_size={:#x}",
            pid,
            graph_entry.user_token,
            graph_entry.graph_user_va.get(),
            graph_entry.graph_size.get()
        );

        Ok(0)
    }
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
            K3_AI_IOC_BUILD_CHANNEL => self.build_channel(arg),
            K3_AI_IOC_SUBMIT_GRAPH => self.submit_graph(arg),
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
