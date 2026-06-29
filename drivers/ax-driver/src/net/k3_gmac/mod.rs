//! SpacemiT K3 GMAC network driver.
//!
//! The Linux STMMAC reference files copied for this rewrite live under
//! `linux/`.  The running Rust path is intentionally split into small modules:
//! generated register/descriptor ABI, MMIO core, SpacemiT syscon glue, MDIO,
//! and the `rd_net` queue adapter.

mod core;
mod desc;
mod generated;
mod mdio;
mod queue;
mod regs;
mod syscon;

use alloc::format;

use fdt_edit::Node;
use log::{info, warn};
use rdrive::{
    probe::OnProbeError,
    register::{FdtInfo, ProbeFdt},
};

use self::{
    core::{DMA_MASK, K3GmacConfig, K3GmacCore},
    queue::K3GmacNet,
    syscon::GlueConfig,
};
use crate::{binding_info_from_fdt, mmio::iomap, net::PlatformDeviceNet};

pub const DRIVER_NAME: &str = "spacemit,k3-gmac";

crate::model_register!(
    name: "SpacemiT K3 GMAC",
    level: ProbeLevel::PostKernel,
    priority: ProbePriority::DEFAULT,
    probe_kinds: &[
        ProbeKind::Fdt {
            compatibles: &["spacemit,k3-gmac", "snps,dwmac-5.10a"],
            on_probe: probe
        },
    ],
);

fn probe(probe: ProbeFdt<'_>) -> Result<(), OnProbeError> {
    let (info, plat_dev) = probe.into_parts();
    let node = info.node.as_node();
    let reg = info.node.regs().into_iter().next().ok_or_else(|| {
        OnProbeError::other(format!("k3-gmac: [{}] has no reg", info.node.name()))
    })?;
    let size = reg.size.unwrap_or(0x2000) as usize;

    info!(
        "k3-gmac: probing {} at {:#x} size={:#x}",
        info.node.name(),
        reg.address,
        size
    );
    log_interrupts(&info);

    let glue = GlueConfig::parse(node);
    if let Err(err) = glue.apply(&info) {
        warn!("k3-gmac: syscon glue failed: {err:?}");
    }

    let mmio_base = iomap(reg.address as usize, size)?;
    let dma = axklib::dma::device_with_mask(DMA_MASK);
    let mac = parse_mac(node).unwrap_or_else(|| generated_mac(reg.address));
    let config = K3GmacConfig {
        mac,
        tx_fifo_depth: prop_u32(node, "tx-fifo-depth").unwrap_or(4096),
        rx_fifo_depth: prop_u32(node, "rx-fifo-depth").unwrap_or(4096),
        checksum_offload: false,
    };
    let net = K3GmacNet::new(K3GmacCore::new(mmio_base, &dma, config).map_err(|err| {
        OnProbeError::other(format!("k3-gmac: failed to initialize core: {err:?}"))
    })?);

    let binding = binding_info_from_fdt(&info)?;
    let irq = plat_dev.register_net_with_info(DRIVER_NAME, net, binding);
    info!(
        "k3-gmac: registered {} mac={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} irq={irq:?}",
        info.node.name(),
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5]
    );
    Ok(())
}

fn parse_mac(node: &Node) -> Option<[u8; 6]> {
    prop_mac(node, "local-mac-address")
        .or_else(|| prop_mac(node, "mac-address"))
        .filter(|mac| !mac.iter().all(|byte| *byte == 0))
}

fn prop_mac(node: &Node, name: &str) -> Option<[u8; 6]> {
    let prop = node.get_property(name)?;
    if prop.data.len() < 6 {
        return None;
    }
    let mut mac = [0u8; 6];
    mac.copy_from_slice(&prop.data[..6]);
    Some(mac)
}

fn generated_mac(base: u64) -> [u8; 6] {
    [
        0x02,
        0x4b,
        0x33,
        ((base >> 16) & 0xff) as u8,
        ((base >> 8) & 0xff) as u8,
        (base & 0xff) as u8,
    ]
}

fn prop_u32(node: &Node, name: &str) -> Option<u32> {
    node.get_property(name).and_then(|prop| prop.get_u32())
}

fn log_interrupts(info: &FdtInfo<'_>) {
    for irq in info.interrupts() {
        info!("k3-gmac: interrupt ref {irq:?}");
    }
}
