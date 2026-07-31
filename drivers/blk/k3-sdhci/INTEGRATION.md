# K3 SDHCI Platform Integration

完整的四层驱动架构实现。

## 架构层次

```
┌──────────────────────────────────────┐
│ 4. Runtime (OS/App)                  │  应用层调用
│    - Block device operations         │
└────────────┬─────────────────────────┘
             │
┌────────────▼─────────────────────────┐
│ 3. OS Glue (axplat-dyn)              │  平台胶水层
│    src/drivers/k3_sdhci.rs            │
│    - FDT probing                     │
│    - MMIO mapping                    │
│    - IRQ registration                │
└────────────┬─────────────────────────┘
             │
┌────────────▼─────────────────────────┐
│ 2. Capability Boundary               │  能力边界
│    src/platform.rs                   │
│    - rdrive Driver trait             │
│    - Device registration             │
│    - Compatible table                │
└────────────┬─────────────────────────┘
             │
┌────────────▼─────────────────────────┐
│ 1. Driver Core (k3-sdhci)            │  驱动核心
│    src/lib.rs                        │
│    - K3 寄存器操作                    │
│    - PHY/DLL 配置                    │
│    - Tuning 算法                     │
└──────────────────────────────────────┘
```

## 设备树配置

在设备树中添加 K3 SDHCI 节点：

```dts
sdhci@d4281000 {
    compatible = "spacemit,k3-sdhci";
    reg = <0xd4281000 0x1000>;
    interrupts = <GIC_SPI 42 IRQ_TYPE_LEVEL_HIGH>;
    clocks = <&sdhci_clk>;
    bus-width = <8>;
    non-removable;
};
```

## 编译和使用

### 1. 添加到工作区

在 `drivers/blk/Cargo.toml` 添加：

```toml
[workspace]
members = [
    "k3-sdhci",
    # ...
]
```

### 2. 启用平台特性

在 `platforms/axplat-dyn/Cargo.toml` 添加：

```toml
[dependencies]
k3-sdhci = { path = "../../drivers/blk/k3-sdhci", features = ["platform"] }

[features]
k3-sdhci = []
```

### 3. 构建

```bash
cargo xtask starry build --arch riscv64 --features k3-sdhci
```

## DTB 探测流程

1. **rdrive 初始化** - 系统启动时初始化 rdrive 框架
2. **驱动注册** - `init_k3_sdhci()` 注册 K3 驱动到 rdrive
3. **FDT 扫描** - rdrive 扫描设备树寻找 `compatible = "spacemit,k3-sdhci"`
4. **驱动绑定** - 调用 `K3SdhciDriver::probe()`
5. **设备初始化** - 创建 MMIO 访问器，初始化控制器
6. **块设备注册** - 向系统注册为块设备

## 下一步

- [ ] 实现 MMIO accessor 的 MmioOp trait
- [ ] 添加 IRQ 处理器注册
- [ ] 集成 sdhci-host 作为协议层
- [ ] 添加 DMA 支持（ADMA2）
- [ ] 实现块设备接口
