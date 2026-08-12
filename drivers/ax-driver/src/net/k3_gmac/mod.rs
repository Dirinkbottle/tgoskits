//! SpacemiT K3 GMAC 网卡驱动（Synopsys DWMAC 5.10a）。
//!
//! 从设备树 probe ethernet@xxx 节点，经 syscon glue 配置 APMU（接口模式 +
//! DLINE 调相）后初始化 DWMAC5 核心（DMA/MAC/MTL），注册为 rd_net 网卡。
//! 首版单队列（queue0/channel0），速率按设备树 `max-speed` 静态配置。

mod core;
mod desc;
mod mdio;
mod queue;
mod regs;
mod syscon;

use alloc::format;

use fdt_edit::Node;
use log::info;
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

/// 驱动名（rdrive 注册用）。
pub const DRIVER_NAME: &str = "spacemit,k3-gmac";

crate::model_register!(
    name: "SpacemiT K3 GMAC",
    level: ProbeLevel::PostKernel,
    priority: ProbePriority::DEFAULT,
    probe_kinds: &[
        ProbeKind::Fdt {
            compatibles: &["spacemit,k3-gmac", "snps,dwmac-5.10a"],
            on_probe: probe,
        },
    ],
);

/// 设备树 probe 入口。
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

    // 1. syscon glue：写 APMU CTRL（RGMII 选通 + WoL）+ DLINE（延迟线 + 调相码）
    let glue = GlueConfig::parse(node);
    if let Err(err) = glue.apply(&info) {
        log::warn!("k3-gmac: syscon glue failed: {err:?}");
    }

    // 等 GMAC IP 充分退出复位（DMA 子块 AXI 通路就绪）。K3 DWMAC5 在 syscon
    // 释放复位后需要较长时间稳定；U-Boot eqos 驱动 swr_wait=500ms，这里保守等 100ms。
    for _ in 0..5_000_000 {
        // ::core 绝对路径（本模块有 super::core 子模块 shadow 了 core crate 名）
        ::core::hint::spin_loop();
    }

    // 2. 映射 GMAC MMIO + 构造 DMA 设备
    let mmio_base = iomap(reg.address as usize, size)?;
    let dma = axklib::dma::device_with_mask(DMA_MASK);

    // 3. 解析 MAC 地址（DTS local-mac-address，缺失则按 reg 地址生成）
    let mac = parse_mac(node).unwrap_or_else(|| generated_mac(reg.address));

    // 4. 解析速率（DTS max-speed，默认 1000）
    let speed_mbps = prop_u32(node, "max-speed").unwrap_or(1000).min(1000);

    let config = K3GmacConfig {
        mac,
        tx_fifo_depth: prop_u32(node, "tx-fifo-depth").unwrap_or(4096),
        rx_fifo_depth: prop_u32(node, "rx-fifo-depth").unwrap_or(4096),
        checksum_offload: false,
        speed_mbps,
        full_duplex: true,
    };

    let net = K3GmacNet::new(K3GmacCore::new(mmio_base, &dma, config).map_err(|err| {
        OnProbeError::other(format!("k3-gmac: failed to initialize core: {err:?}"))
    })?);

    let binding = binding_info_from_fdt(&info)?;
    let irq = plat_dev.register_net_with_info(DRIVER_NAME, net, binding);
    info!(
        "k3-gmac: registered {} mac={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} speed={}Mbps \
         irq={irq:?}",
        info.node.name(),
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5],
        speed_mbps,
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

/// 按 MMIO 基址生成一个本地管理 MAC（locally administered, unicast）。
fn generated_mac(base: u64) -> [u8; 6] {
    [
        0x02, // 本地管理，单播
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
