//! SpacemiT K3 GMAC syscon glue from `dwmac-spacemit-ethqos.c`.

use alloc::format;

use fdt_edit::{Node, Phandle};
use rdrive::{probe::OnProbeError, register::FdtInfo};

use super::regs;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhyMode {
    Mii,
    Rmii,
    Rgmii,
    RgmiiId,
    RgmiiRxId,
    RgmiiTxId,
}

impl PhyMode {
    pub fn parse(value: Option<&str>) -> Self {
        match value.unwrap_or("rgmii") {
            "mii" => Self::Mii,
            "rmii" => Self::Rmii,
            "rgmii-id" => Self::RgmiiId,
            "rgmii-rxid" => Self::RgmiiRxId,
            "rgmii-txid" => Self::RgmiiTxId,
            _ => Self::Rgmii,
        }
    }

    fn syscon_value(self) -> u32 {
        match self {
            Self::Mii => regs::PHY_INTF_MII,
            Self::Rmii => regs::PHY_INTF_RMII,
            Self::Rgmii | Self::RgmiiId | Self::RgmiiRxId | Self::RgmiiTxId => regs::PHY_INTF_RGMII,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum ClockTuning {
    Disabled,
    ByReg { tx_phase: u8, rx_phase: u8 },
    ByDelayLine { tx_phase: u8, rx_phase: u8 },
    ByClockRevert { tx_phase: u8, rx_phase: u8 },
}

#[derive(Debug, Clone, Copy)]
pub struct GlueConfig {
    pub phy_mode: PhyMode,
    pub apmu: Option<Phandle>,
    pub ctrl_offset: Option<u32>,
    pub dline_offset: Option<u32>,
    pub wake_irq_enable: bool,
    pub tuning: ClockTuning,
}

impl GlueConfig {
    pub fn parse(node: &Node) -> Self {
        let phy_mode = PhyMode::parse(prop_str(node, "phy-mode"));
        let tx_phase = prop_u32(node, "spacemit,tx-phase").unwrap_or(0).min(255) as u8;
        let rx_phase = prop_u32(node, "spacemit,rx-phase").unwrap_or(0).min(255) as u8;
        let tuning = if has_prop(node, "spacemit,clk-tuning-enable") {
            if has_prop(node, "spacemit,clk-tuning-by-delayline") {
                ClockTuning::ByDelayLine { tx_phase, rx_phase }
            } else if has_prop(node, "spacemit,clk-tuning-by-clk-revert") {
                ClockTuning::ByClockRevert { tx_phase, rx_phase }
            } else {
                ClockTuning::ByReg { tx_phase, rx_phase }
            }
        } else {
            ClockTuning::Disabled
        };

        Self {
            phy_mode,
            apmu: prop_phandle(node, "spacemit,apmu"),
            ctrl_offset: prop_u32(node, "spacemit,ctrl-offset"),
            dline_offset: prop_u32(node, "spacemit,dline-offset"),
            wake_irq_enable: has_prop(node, "spacemit,wake-irq-enable"),
            tuning,
        }
    }

    pub fn apply(self, info: &FdtInfo<'_>) -> Result<(), OnProbeError> {
        let Some(apmu) = self.apmu else {
            log::warn!("k3-gmac: missing spacemit,apmu; skip syscon glue");
            return Ok(());
        };
        let Some(ctrl_offset) = self.ctrl_offset else {
            log::warn!("k3-gmac: missing spacemit,ctrl-offset; skip syscon glue");
            return Ok(());
        };
        let syscon = info.get_by_phandle(apmu).ok_or_else(|| {
            OnProbeError::other(format!("k3-gmac: syscon phandle {apmu:?} not found"))
        })?;
        let reg =
            syscon.regs().into_iter().next().ok_or_else(|| {
                OnProbeError::other(format!("k3-gmac: {} has no reg", syscon.name()))
            })?;
        let size = reg.size.unwrap_or(0x1000) as usize;
        let base = crate::mmio::iomap(reg.address as usize, size)?;
        let syscon = unsafe { regs::Mmio::new(base) };

        update_bits(
            &syscon,
            ctrl_offset,
            regs::PHY_INTF_MODE_MASK,
            self.phy_mode.syscon_value(),
        );
        update_bits(
            &syscon,
            ctrl_offset,
            regs::WOL_WAKE_IRQ_EN,
            if self.wake_irq_enable {
                regs::WOL_WAKE_IRQ_EN
            } else {
                0
            },
        );

        if let Some(dline_offset) = self.dline_offset {
            update_bits(
                &syscon,
                dline_offset,
                regs::EMAC_TX_DLINE_EN
                    | regs::EMAC_RX_DLINE_EN
                    | regs::EMAC_TX_DLINE_CODE_MASK
                    | regs::EMAC_RX_DLINE_CODE_MASK,
                regs::EMAC_TX_DLINE_EN | regs::EMAC_RX_DLINE_EN,
            );
        } else {
            log::warn!("k3-gmac: missing spacemit,dline-offset; skip delayline init");
        }

        self.log_deferred_tuning();
        Ok(())
    }

    fn log_deferred_tuning(self) {
        match self.tuning {
            ClockTuning::Disabled => {}
            ClockTuning::ByReg { tx_phase, rx_phase } => {
                log::info!(
                    "k3-gmac: clk phase tuning parsed for {:?}: mode=reg tx_phase={} rx_phase={} \
                     (deferred until link speed is known)",
                    self.phy_mode,
                    tx_phase,
                    rx_phase
                );
            }
            ClockTuning::ByDelayLine { tx_phase, rx_phase } => {
                log::info!(
                    "k3-gmac: clk phase tuning parsed for {:?}: mode=delayline tx_phase={} \
                     rx_phase={} (deferred until link speed is known)",
                    self.phy_mode,
                    tx_phase,
                    rx_phase
                );
            }
            ClockTuning::ByClockRevert { tx_phase, rx_phase } => {
                log::info!(
                    "k3-gmac: clk phase tuning parsed for {:?}: mode=clk-revert tx_phase={} \
                     rx_phase={} (deferred until link speed is known)",
                    self.phy_mode,
                    tx_phase,
                    rx_phase
                );
            }
        }
    }
}

fn update_bits(mmio: &regs::Mmio, offset: u32, mask: u32, value: u32) {
    mmio.update(offset, mask, value & mask);
}

fn prop_u32(node: &Node, name: &str) -> Option<u32> {
    node.get_property(name).and_then(|prop| prop.get_u32())
}

fn prop_str<'a>(node: &'a Node, name: &str) -> Option<&'a str> {
    node.get_property(name).and_then(|prop| prop.as_str())
}

fn prop_phandle(node: &Node, name: &str) -> Option<Phandle> {
    node.get_property(name)
        .and_then(|prop| prop.get_u32())
        .map(Phandle::from)
}

fn has_prop(node: &Node, name: &str) -> bool {
    node.get_property(name).is_some()
}
