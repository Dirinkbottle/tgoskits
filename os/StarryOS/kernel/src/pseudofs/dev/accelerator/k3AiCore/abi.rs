//! 与用户态 `k3_aiUabi` 对齐的常量和 ioctl 参数结构体。

use axfs_ng_vfs::DeviceId;

/// `BUILD_CHANNEL` ioctl 命令号：注册用户态 ovchannel 共享区。
pub const K3_AI_IOC_BUILD_CHANNEL: u32 = 0x4B33_0001;
/// `SUBMIT_GRAPH` ioctl 命令号：提交一次 graph 执行。
pub const K3_AI_IOC_SUBMIT_GRAPH: u32 = 0x4B33_0002;
/// `/dev/k3_airunner` 的设备号 (major 240, minor 10)。
pub const K3_AIRUNNER_DEVICE_ID: DeviceId = DeviceId::new(240, 10);
/// 内核与用户态约定的 ovchannel 通道数量 (`SharedMemory<2>`)。
pub const K3_AIRUNNER_CHANNEL_COUNT: usize = 2;

/// ovchannel 共享区注册参数。
///
/// 必须和用户态 `k3_aiUabi::kd_uring::K3AiChannelBuildParam` 保持一致。
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct K3AiChannelBuildParam {
    /// 用户态共享区起始虚拟地址。
    pub user_va: u64,
    /// 共享区字节大小。
    pub size_bytes: u64,
    /// 通道数量，当前必须等于 [`K3_AIRUNNER_CHANNEL_COUNT`]。
    pub channel_count: u32,
    /// 预留标志位。
    pub flags: u32,
    /// 内核回填的 owner pid，用户态可用它做日志或调试匹配。
    pub owner_pid: u32,
    /// 预留字段 0。
    pub reserved0: u32,
    /// 预留字段 1。
    pub reserved1: u64,
}
