//! `/dev/k3_airunner` 设备对象。

/// K3 AI runner 字符设备。
///
/// 设备本身无内部状态：channel / tensor 的映射记录都放在 [`super::registry`] 的全局表
/// 中，因此设备对象可以随意构造 / clone。控制面（ioctl）实现见 [`super::device`]，
/// 运行时回调（`K3SchedulerOps`）实现见 [`super::scheduler`]。
pub struct K3AiRunner;
