//! SpacemiT K3 (`k3_com260kit`) AI runner character device (`/dev/k3_airunner`).
//!
//! 这个设备是 Starry 侧 K3 AI 加速通路的最小闭环入口，负责:
//! - `BUILD_CHANNEL` ioctl: 把用户态通过 `MAP_SHARED` 建出来的 ovchannel 共享区注册进
//!   内核，并为其建立连续的 kernel VA alias；
//! - `SUBMIT_GRAPH` ioctl: 从 channel 读取 `AiGraphSubmitEntry`，反序列化 graph blob，
//!   映射 tensor，并把任务交给 `k3_aiScheduler` 调度执行。
//!
//! 模块按职责拆分:
//! - [`abi`]: 内核专属设备号；共享 UABI 直接来自 `k3_ai_uabi`；
//! - [`registry`]: channel / tensor 的 kernel alias 记账与生命周期管理；
//! - [`runner`]: 设备对象 [`K3AiRunner`];
//! - [`device`]: `DeviceOps` ioctl 控制面实现；
//! - [`scheduler`]: `K3SchedulerOps` 运行时回调实现。
#![deny(missing_docs)]

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

mod abi;
mod device;
mod registry;
mod runner;
mod scheduler;

// 设备节点注册需要设备号和设备对象；ioctl 命令号 / channel 布局常量 /
// `K3AiChannelBuildParam` 属于 `k3_ai_uabi`，避免内核手写镜像结构体。
pub use abi::K3_AIRUNNER_DEVICE_ID;
pub use runner::K3AiRunner;
