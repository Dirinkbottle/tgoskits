//! K3 AI runner 内核专属 ABI 常量。

use axfs_ng_vfs::DeviceId;

/// `/dev/k3_airunner` 的设备号 (major 240, minor 10)。
pub const K3_AIRUNNER_DEVICE_ID: DeviceId = DeviceId::new(240, 10);
