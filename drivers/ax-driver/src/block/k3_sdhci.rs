//! SpacemiT K3/K1 SDHCI FDT glue.
//!
//! Low-level vendor reset/clock bits live in `drivers/blk/k3-sdhci`.

use alloc::{format, vec::Vec};
use core::time::Duration;

use fdt_edit::Node;
use k3_sdhci::K3Sdhci;
use log::{info, warn};
use rdrive::{
    probe::OnProbeError,
    register::{FdtInfo, ProbeFdt},
};
use sdmmc_protocol::{
    Error, OperationPoll,
    error::{ErrorContext, Phase},
    sdio::{CardInfo, CardInitPreference, SdioInitScratch, SdioSdmmc},
};

use crate::{
    BindingInfo, binding_info_from_fdt,
    block::{
        PlatformDeviceBlock, SharedDriver,
        sdmmc::{SdmmcBlockConfig, SdmmcBlockDevice},
    },
    mmio::iomap,
};

// SDHCI 3.3 V power selector.
// Reference: /home/inkbottle/桌面/linux-6.18.35/drivers/mmc/host/sdhci.h:124-128.
const SDHCI_POWER_330: u8 = 0x0e;

type K3SdMmc = SdioSdmmc<K3Sdhci>;

crate::model_register!(
    name: "K3 SDHCI",
    level: ProbeLevel::PostKernel,
    priority: ProbePriority::DEFAULT,
    probe_kinds: &[
        ProbeKind::Fdt {
            compatibles: &["spacemit,k3-sdhci", "spacemit,k1-sdhci"],
            on_probe: probe
        }
    ],
);

fn probe(probe: ProbeFdt<'_>) -> Result<(), OnProbeError> {
    let (info, plat_dev) = probe.into_parts();
    let node = info.node.as_node();
    let base_reg = info
        .node
        .regs()
        .into_iter()
        .next()
        .ok_or(OnProbeError::other(format!(
            "[{}] has no reg",
            info.node.name()
        )))?;

    let mmio_size = base_reg.size.unwrap_or(0x1000);
    log_fdt_summary(&info, node, base_reg.address, mmio_size);

    info!(
        "k3-sdhci: MMIO map addr={:#x} size={:#x}",
        base_reg.address, mmio_size
    );
    let mmio_base = iomap(base_reg.address as usize, mmio_size as usize)?;

    let no_mmc = has_prop(node, "no-mmc");
    let allow_mmc = !no_mmc;
    // SAFETY: `iomap` returned an exclusive mapped MMIO base for this probed
    // FDT node; the wrapper owns all access for this controller instance.
    let mut host = unsafe { K3Sdhci::new(mmio_base, allow_mmc) };

    match prop_u32(node, "clock-frequency") {
        Some(hz) if hz != 0 => {
            info!(
                "k3-sdhci: reference clock from DT clock-frequency={} Hz",
                hz
            );
            host.set_reference_clock_hz(hz);
        }
        Some(_) => warn!("k3-sdhci: DT clock-frequency is 0"),
        None => info!("k3-sdhci: no DT clock-frequency; reading SDHCI capabilities"),
    }

    let base_clock = host.base_clock_hz();
    info!(
        "k3-sdhci: reference clock resolved to {} Hz, adma2={}",
        base_clock,
        host.supports_adma2()
    );
    if base_clock == 0 {
        host.dump_state("reference clock missing");
        return Err(init_error(
            base_reg.address,
            mmio_size,
            Error::BadResponse(ErrorContext::new(Phase::Init)),
        ));
    }

    info!("k3-sdhci: reset generic + SpacemiT vendor block");
    if let Err(err) = host.reset_all() {
        host.dump_state("reset failed");
        return Err(init_error(base_reg.address, mmio_size, err));
    }

    info!("k3-sdhci: enable 3.3V power, status interrupts, 32-bit DMA");
    host.set_power(SDHCI_POWER_330);
    host.enable_interrupts();
    host.set_dma(axklib::dma::device_with_mask(u32::MAX as u64));
    host.dump_state("after power/irq/dma");

    let preference = card_init_preference(&info);
    info!(
        "k3-sdhci: initialize SD/MMC card preference={:?}",
        preference
    );
    let mut card = SdioSdmmc::new(host);
    card.set_sd_uhs_selection_enabled(false);
    let card_info = match poll_card_init(&mut card, preference) {
        Ok(info) => info,
        Err(err) => {
            card.host().dump_state("card init failed");
            return Err(card_init_error(base_reg.address, mmio_size, err));
        }
    };
    info!(
        "k3-sdhci card: kind={:?} high_capacity={} rca={} ocr={:#010x} capacity_blocks={:?} \
         cid={} ext_csd={}",
        card_info.kind,
        card_info.high_capacity,
        card_info.rca,
        card_info.ocr,
        card_info.capacity_blocks,
        card_info.cid.is_some(),
        card_info.ext_csd.is_some()
    );

    let raw = SharedDriver::new(card);
    info!(
        "k3-sdhci: using FIFO block transfers for v1 rootfs bring-up; ADMA2 stays disabled until \
         K3 DMA/cache completion is validated"
    );
    let dev = SdmmcBlockDevice::new(
        raw,
        SdmmcBlockConfig::fifo("k3-sdhci", card_info.capacity_blocks.unwrap_or(0), false),
    );
    let binding_info = binding_info_or_polling(&info);
    let irq = binding_info.irq_num();
    plat_dev.register_block_with_info(dev, binding_info);
    info!("k3-sdhci block device registered irq={:?}", irq);
    Ok(())
}

fn binding_info_or_polling(info: &FdtInfo<'_>) -> BindingInfo {
    match binding_info_from_fdt(info) {
        Ok(binding_info) => {
            info!(
                "k3-sdhci: FDT IRQ resolved for block registration irq={:?}",
                binding_info.irq_num()
            );
            binding_info
        }
        Err(err) => {
            warn!(
                "k3-sdhci: failed to resolve FDT IRQ for {}; registering block device without IRQ \
                 so the polling path can still mount root: {}",
                info.node.path(),
                err
            );
            BindingInfo::empty()
        }
    }
}

fn poll_card_init(card: &mut K3SdMmc, preference: CardInitPreference) -> Result<CardInfo, Error> {
    let mut scratch = SdioInitScratch::new();
    let mut request = card.submit_init_with_preference(preference, &mut scratch)?;
    let mut pace_waits = 0u32;
    loop {
        match card.poll_init_request(&mut request)? {
            OperationPoll::Pending => {
                if request.take_needs_pace() {
                    pace_waits = pace_waits.saturating_add(1);
                    info!("k3-sdhci: ACMD41/CMD1 paced wait #{}", pace_waits);
                    axklib::time::busy_wait(Duration::from_millis(10));
                } else {
                    core::hint::spin_loop();
                }
            }
            OperationPoll::Complete(info) => return Ok(info),
            _ => return Err(Error::UnsupportedCommand),
        }
    }
}

fn card_init_preference(info: &FdtInfo<'_>) -> CardInitPreference {
    let node = info.node.as_node();
    if has_prop(node, "no-mmc") {
        CardInitPreference::SdOnly
    } else if has_prop(node, "no-sd") || has_prop(node, "non-removable") {
        CardInitPreference::MmcOnly
    } else {
        CardInitPreference::SdFirst
    }
}

fn log_fdt_summary(info: &FdtInfo<'_>, node: &Node, address: u64, size: u64) {
    let compatible = prop_str_list(node, "compatible");
    let clock_frequency = prop_u32(node, "clock-frequency");
    let tx_delaycode = prop_u32(node, "spacemit,tx_delaycode");
    let bus_width = prop_u32(node, "bus-width");
    let clocks = info.node.clocks();
    let interrupts = info.interrupts();
    let resets = prop_u32_list(node, "resets");
    let reset_names = prop_str_list(node, "reset-names");
    let clock_names = prop_str_list(node, "clock-names");
    let cd_gpios = prop_u32_list(node, "cd-gpios");
    let vmmc = prop_u32_list(node, "vmmc-supply");
    let vqmmc = prop_u32_list(node, "vqmmc-supply");

    info!(
        "k3-sdhci probe: node={} path={} compatible={:?} reg={:#x}+{:#x}",
        info.node.name(),
        info.node.path(),
        compatible,
        address,
        size
    );
    info!("k3-sdhci fdt: interrupts={:?}", interrupts);
    info!(
        "k3-sdhci fdt: clock-names={:?} clocks={:?}",
        clock_names, clocks
    );
    info!(
        "k3-sdhci fdt: reset-names={:?} resets={:#x?}",
        reset_names, resets
    );
    info!(
        "k3-sdhci fdt: flags no-mmc={} no-sdio={} no-sd={} non-removable={} bus-width={:?}",
        has_prop(node, "no-mmc"),
        has_prop(node, "no-sdio"),
        has_prop(node, "no-sd"),
        has_prop(node, "non-removable"),
        bus_width
    );
    info!(
        "k3-sdhci fdt: cd-gpios={:#x?} vmmc-supply={:#x?} vqmmc-supply={:#x?}",
        cd_gpios, vmmc, vqmmc
    );
    info!(
        "k3-sdhci fdt: clock-frequency={:?} spacemit,tx_delaycode={:?}",
        clock_frequency, tx_delaycode
    );
    info!(
        "k3-sdhci fdt: Linux clock/reset reference is K1-only, not writing unproven K3 APMU regs"
    );
}

fn init_error(address: u64, size: u64, err: Error) -> OnProbeError {
    OnProbeError::other(format!(
        "failed to initialize K3 SDHCI device at [PA:{:?}, SZ:0x{:x}): {err:?}",
        address, size
    ))
}

fn card_init_error(address: u64, size: u64, err: Error) -> OnProbeError {
    if is_absent_card_init_error(err) {
        warn!(
            "k3-sdhci: no responsive SD card at [PA:{:?}, SZ:0x{:x}); skipping controller: {err:?}",
            address, size
        );
        return OnProbeError::NotMatch;
    }

    init_error(address, size, err)
}

fn is_absent_card_init_error(err: Error) -> bool {
    match err {
        Error::NoCard => true,
        Error::Timeout(ctx) | Error::Crc(ctx) | Error::BadResponse(ctx) => {
            ctx.cmd.is_some()
                && matches!(
                    ctx.phase,
                    Phase::CommandSend | Phase::ResponseWait | Phase::Init
                )
        }
        _ => false,
    }
}

fn has_prop(node: &Node, name: &str) -> bool {
    node.get_property(name).is_some()
}

fn prop_u32(node: &Node, name: &str) -> Option<u32> {
    node.get_property(name).and_then(|prop| prop.get_u32())
}

fn prop_u32_list(node: &Node, name: &str) -> Vec<u32> {
    node.get_property(name)
        .map(|prop| prop.get_u32_iter().collect())
        .unwrap_or_default()
}

fn prop_str_list<'a>(node: &'a Node, name: &str) -> Vec<&'a str> {
    node.get_property(name)
        .map(|prop| prop.as_str_iter().collect())
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use sdmmc_protocol::error::ErrorContext;

    use super::*;

    #[test]
    fn command_timeout_during_card_init_is_absent_card() {
        let err = Error::Timeout(ErrorContext::for_cmd(Phase::ResponseWait, 41));

        assert!(is_absent_card_init_error(err));
    }

    #[test]
    fn data_timeout_after_card_init_is_not_absent_card() {
        let err = Error::Timeout(ErrorContext::for_cmd(Phase::DataRead, 17));

        assert!(!is_absent_card_init_error(err));
    }
}
