# K3 SDHCI Driver Registration

按照 rdrive 规则正确注册驱动。

## 使用方法

在需要启用 K3 SDHCI 的地方（如平台初始化代码）调用：

```rust
use k3_sdhci::platform;

// 在 rdrive 初始化后注册驱动
rdrive::register_add(platform::register());
```

## 完整示例

```rust
// 在 axplat-dyn 或其他平台代码中
pub fn init_platform_drivers() {
    #[cfg(feature = "k3-sdhci")]
    {
        use k3_sdhci::platform;
        rdrive::register_add(platform::register());
    }
}

// 在系统启动流程中
fn main() {
    // 1. 初始化 rdrive
    rdrive::init(Platform::Fdt { addr: fdt_addr })?;
    
    // 2. 注册所有驱动
    init_platform_drivers();
    
    // 3. 探测设备（会自动匹配 "spacemit,k3-sdhci"）
    rdrive::probe_all(false)?;
}
```

## FDT 匹配规则

驱动会自动匹配设备树中的节点：

```dts
emmc: mmc@d4281000 {
    compatible = "spacemit,k3-sdhci";
    reg = <0xd4281000 0x1000>;
};
```

当 `rdrive::probe_all()` 执行时：
1. 扫描 FDT 找到所有 `compatible = "spacemit,k3-sdhci"` 的节点
2. 对每个节点调用 `k3_sdhci_probe()` 函数
3. 自动传入 `FdtInfo` 包含节点路径、寄存器地址等信息

## 集成到具体 OS

需要在 OS 的驱动初始化代码中添加注册调用，参考 `drivers/examples/enumerate/src/main.rs`。
