# k3-sdhci

SpacemiT K3/K1 SDHCI Host Controller Driver (Rust port)

## Overview

Rust implementation of the Linux `sdhci-of-k1.c` driver for SpacemiT K3 and K1 RISC-V SoCs.

## Features

- K3/K1 specific PHY configuration
- HS200/HS400/HS400ES timing modes
- Software RX tuning with delay line control
- Clock gating and power management
- `#![no_std]` compatible

## Architecture

Follows the tgoskits four-layer driver model:
1. **Driver Core** (this crate) - register definitions, PHY state machine and RX tuning
2. **Capability Boundary** - MMIO access through `mmio_api::MmioRaw` (raw mapped window)
3. **OS Glue** - FDT probe, `mmio-api` ioremap, IRQ and block registration (implemented by the consumer, e.g. `ax-driver` or a board layer)
4. **Runtime** - blocking/async wrappers

This crate is OS-independent: it only requires an already-mapped `MmioRaw` window and never calls `ioremap`/`MmioOp`/`DmaOp` itself.

## Usage

The MMIO window must be mapped by the OS glue layer before the host is created:

```rust
use k3_sdhci::{K3SdhciHost, SdMode, Timing};
use mmio_api::MmioAddr;

// In OS glue: ioremap the physical register window (size covers 0x178).
let mmio = unsafe { mmio_api::ioremap_raw(MmioAddr::from(0x0D42_0000u64), 0x200) }
    .expect("failed to map K3 SDHCI registers");
let mut host = K3SdhciHost::new(mmio);

// Initialize for eMMC (pass SdMode::Sd for removable SD cards)
host.reset(SdMode::Emmc);

// Set timing mode
host.set_timing(Timing::MmcHs200);

// Execute tuning
host.execute_tuning(Timing::MmcHs200, |delay| {
    // Test function: send tuning block and check result
    send_tuning_block() == Ok(())
})?;
```

For bare-metal or test setups without a registered `MmioOp`, construct the
window directly with `MmioRaw::new(phys, virt, size)` instead of
`ioremap_raw`.

## Register Map

- `0x108` - OP_EXT: Clock override control
- `0x114` - MMC_CTRL: Timing mode, enhanced strobe
- `0x118` - RX_CFG: RX clock select
- `0x11C` - TX_CFG: TX clock select
- `0x130` - DLINE_CTRL: Delay line control
- `0x160` - PHY_CTRL: PHY function enable
- `0x168` - PHY_DLLCFG: DLL configuration

## License

Apache-2.0 (port of GPL-2.0 Linux driver)
