//! SpacemiT K3 SDHCI driver registration

use rdrive::{probe::OnProbeError, register::*};

crate::model_register!(
    name: "k3-sdhci",
    level: ProbeLevel::PostKernel,
    priority: ProbePriority::DEFAULT,
    probe_kinds: &[ProbeKind::Fdt {
        compatibles: &["spacemit,k3-sdhci"],
        on_probe: probe,
    }],
);

fn probe(probe: ProbeFdt<'_>) -> Result<(), OnProbeError> {
    let info = probe.info();

    log::info!("[k3-sdhci] Probing device at {}", info.node.name());

    rdrive::register_add(k3_sdhci::platform::register());

    Ok(())
}
