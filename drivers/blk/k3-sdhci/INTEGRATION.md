# K3 SDHCI Integration

Portable Driver Core with the OS glue implemented by the consumer.

## Architecture

```
┌──────────────────────────────────────┐
│ 4. Runtime (OS/App)                  │  application / block stack
│    - Block device operations         │
└────────────┬─────────────────────────┘
             │
┌────────────▼─────────────────────────┐
│ 3. OS Glue (consumer)                │  ax-driver or board layer
│    - FDT probe                       │
│    - ioremap via mmio-api            │
│    - IRQ registration                │
│    - Block registration              │
└────────────┬─────────────────────────┘
             │
┌────────────▼─────────────────────────┐
│ 2. Capability Boundary               │  mmio-api (MmioRaw / MmioOp)
│    - MMIO window mapping             │
└────────────┬─────────────────────────┘
             │
┌────────────▼─────────────────────────┐
│ 1. Driver Core (k3-sdhci)            │  this crate, OS-independent
│    - K3 寄存器操作                    │
│    - PHY/DLL 配置                     │
│    - Tuning 算法                      │
└──────────────────────────────────────┘
```

This crate is `#![no_std]` and only depends on `mmio-api` for the raw MMIO
window. All OS-specific work (FDT probe, `ioremap`, IRQ, block registration)
is performed by the consuming layer, which decides whether to drive the host
synchronously, from an IRQ thread, or through a blocking/async runtime.

## Driver Core API

```rust
use k3_sdhci::{ClockGate, Hs400Strobe, K3SdhciHost, SdMode, Timing};

let mut host = K3SdhciHost::new(mmio); // mmio: mmio_api::MmioRaw

host.reset(SdMode::Emmc);                 // PHY config for eMMC/SD
host.set_clock_gate(ClockGate::Auto);     // clock gating policy
host.set_timing(Timing::MmcHs200);
host.set_clock(Timing::MmcHs200);
host.execute_tuning(Timing::MmcHs200, |delay| test_delay(delay))?;

host.enable_hs400_strobe(Hs400Strobe::Enable)?; // HS400 enhanced strobe
host.prepare_hs400();
host.post_hs400_config()?;
host.hs400_to_hs200();                    // downgrade HS400 -> HS200
```

`K3SdhciHost` also implements the `SdhciVendorExt` trait
(`vendor_reset`, `vendor_set_timing`, `vendor_set_clock`,
`vendor_execute_tuning`), which lets a generic SDHCI stack call the vendor
sequences without depending on the concrete host type.

## Device Tree

The device-tree binding used by the reference Linux driver is
`spacemit,k1-sdhci` (older board trees may use `spacemit,k3-sdhci`):

```dts
mmc@d4281000 {
    compatible = "spacemit,k1-sdhci";
    reg = <0x0 0xd4281000 0x0 0x1000>;
    interrupts = <...>;
    clocks = <&sdhci_clk>;
    bus-width = <8>;
    non-removable;      /* eMMC: probe with SdMode::Emmc */
};
```

The consumer's FDT probe reads `compatible`, `reg`, `interrupts` and the
removability (`non-removable`) properties and drives the crate accordingly.

## Adding to the workspace

`k3-sdhci` is picked up by the workspace through the `drivers/blk/*` member
glob; no extra `[workspace]` entry is needed. To consume it, declare the
dependency in the OS glue crate:

```toml
[dependencies]
k3-sdhci = { path = "../../drivers/blk/k3-sdhci" }
```

## Building

```bash
cargo check -p k3-sdhci
cargo clippy -p k3-sdhci
cargo fmt -p k3-sdhci -- --check
```

## Next steps

- [ ] Implement `mmio-api` `MmioOp` in the OS glue and map the window
- [ ] Add IRQ handling in the OS glue (MSI / level-triggered)
- [ ] Wire the host into the block stack (`rdif-block` / `ax-driver`)
- [ ] Optionally add DMA support through `dma-api` for large transfers
