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

Follows tgoskits four-layer driver model:
1. **Driver Core** (this crate) - register definitions and state machines
2. **Capability Boundary** - `mmio_api::MmioOp`, `dma_api::DmaOp`
3. **OS Glue** - FDT probe, IRQ registration (in platform layer)
4. **Runtime** - blocking/async wrappers

## Usage

```rust
use k3_sdhci::{K3SdhciHost, Timing};
use mmio_api::MmioOp;

// Create host with MMIO accessor
let mut host = K3SdhciHost::new(mmio, base_addr);

// Initialize for eMMC
host.reset(true);

// Set timing mode
host.set_timing(Timing::MmcHs200);

// Execute tuning
host.execute_tuning(Timing::MmcHs200, |delay| {
    // Test function: send tuning command and check result
    send_tuning_block() == Ok(())
})?;
```

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
