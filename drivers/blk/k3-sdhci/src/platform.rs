//! K3 SDHCI rdrive registration

extern crate alloc;

use log::info;
use rdrive::{
    probe::OnProbeError,
    register::{DriverRegister, ProbeFdt, ProbeKind, ProbeLevel, ProbePriority},
};

use crate::{K3SdhciHost, SdhciVendorExt};

fn k3_sdhci_probe(probe: ProbeFdt<'_>) -> Result<(), OnProbeError> {
    let info = probe.info();

    info!("K3 SDHCI probing device: {}", info.node.name());

    let base_reg = info
        .node
        .regs()
        .into_iter()
        .next()
        .ok_or_else(|| OnProbeError::other("no reg property"))?;

    let mmio_size = base_reg.size.unwrap_or(0x1000);
    info!(
        "K3 SDHCI base address: {:#x}, size: {:#x}",
        base_reg.address, mmio_size
    );

    let mmio = axklib::mmio::ioremap_raw((base_reg.address as usize).into(), mmio_size as usize)
        .map_err(|e| OnProbeError::other(alloc::format!("ioremap failed: {:?}", e)))?;

    let mut host = K3SdhciHost::new(mmio);

    let is_emmc = info.node.as_node().get_property("non-removable").is_some();
    host.vendor_reset(is_emmc);

    info!("K3 SDHCI initialized successfully");

    Ok(())
}

pub fn register() -> DriverRegister {
    DriverRegister {
        name: "k3-sdhci",
        level: ProbeLevel::PostKernel,
        priority: ProbePriority::DEFAULT,
        probe_kinds: &[ProbeKind::Fdt {
            compatibles: &["spacemit,k3-sdhci"],
            on_probe: k3_sdhci_probe,
        }],
    }
}
