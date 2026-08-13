//! SpacemiT K3 GMAC syscon glue（APMU CTRL/DLINE 寄存器配置）。
//!
//! 来源：Linux `dwmac-spacemit-ethqos.c`。K3 的 GMAC 没有独立的时钟/复位寄存器，
//! 而是借用 APMU（或 RCPU_SYSCTRL）系统控制器的两个 32 位寄存器：
//! - CTRL：接口模式（RGMII/RMII/MII）+ AXI 总线时钟使能/复位 + WoL
//! - DLINE：RGMII TX/RX 延迟线（用于时钟调相）

use alloc::format;

use fdt_edit::{Node, Phandle};
use rdrive::{probe::OnProbeError, register::FdtInfo};

use super::regs;

/// PHY 接口模式（对应 DTS `phy-mode`）。
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

    /// 映射到 syscon CTRL 的 PHY_INTF_MODE 位段值。
    fn syscon_value(self) -> u32 {
        match self {
            Self::Mii => regs::PHY_INTF_MII,
            Self::Rmii => regs::PHY_INTF_RMII,
            Self::Rgmii | Self::RgmiiId | Self::RgmiiRxId | Self::RgmiiTxId => regs::PHY_INTF_RGMII,
        }
    }
}

/// RGMII 时钟调相策略（对应 DTS `spacemit,clk-tuning-*`）。
///
/// 首版仅实现 `ByDelayLine`（写 DLINE 延迟码）；`ByReg`/`ByClockRevert` 的相位
/// 字段为预留，待对应调相模式实现后使用。
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)] // 预留变体字段
pub enum ClockTuning {
    Disabled,
    ByReg { tx_phase: u8, rx_phase: u8 },
    ByDelayLine { tx_phase: u8, rx_phase: u8 },
    ByClockRevert { tx_phase: u8, rx_phase: u8 },
}

/// PHY 硬件复位 GPIO 配置（对应 DTS `snps,reset-gpios` + `snps,reset-delays-us`）。
///
/// K3 的 RTL8211F PHY 复位脚接在 GPIO1_5。若驱动不复位 PHY，MDIO 会探测不到
/// 设备（PHYID 读出 0xffff）。U-Boot eqos 驱动在 `k3_eqos_phy_reset` 做同样的事。
#[derive(Debug, Clone, Copy)]
pub struct GpioReset {
    /// GPIO 控制器 phandle（指向 `gpio@d4019000`/`syscon-gpio`）。
    pub gpio_phandle: Phandle,
    /// bank 编号（K3 GPIO 控制器有 4 个 bank，stride=0x40）。
    pub bank: u32,
    /// bank 内 pin 编号（0..32）。
    pub pin: u32,
    /// 是否低电平有效（DTS gpio spec flags bit0 = 1 表示 active low）。
    pub active_low: bool,
    /// 复位序列时序（微秒）：(pre, assert, post-deassert)。
    pub delays_us: (u32, u32, u32),
}

impl GpioReset {
    /// 从 FDT 节点解析 `snps,reset-gpios`（标准 stmmac 属性）。
    ///
    /// 格式：`<phandle bank pin flags>`（`#gpio-cells = 3`）。
    /// 配合 `snps,reset-delays-us = <pre assert deassert>`。
    fn parse(node: &Node) -> Option<Self> {
        let prop = node.get_property("snps,reset-gpios")?;
        let mut cells = prop.get_u32_iter();
        let gpio_phandle = Phandle::from(cells.next()?);
        let bank = cells.next()?;
        let pin = cells.next()?;
        let flags = cells.next().unwrap_or(0);
        // active low 时，"assert" = 输出低（写 0），"deassert" = 输出高（写 1）。
        // active high 反之。RTL8211F 复位脚是 active low（flags bit0=1）。
        let active_low = flags & 1 != 0;

        let delays_us = prop_u32_array_const(node, "snps,reset-delays-us", 3)
            .map(|d| (d[0], d[1], d[2]))
            .unwrap_or((0, 20_000, 100_000));

        Some(Self {
            gpio_phandle,
            bank,
            pin,
            active_low,
            delays_us,
        })
    }

    /// 执行 PHY 硬件复位：deassert → 等 pre → assert（拉低）→ 等 assert → deassert → 等 post。
    ///
    /// 直接 MMIO 操作 K3 GPIO 控制器（基址 0xd4019000），避免引入完整 GPIO 驱动
    /// 框架。复位序列匹配 U-Boot `k3_eqos_phy_reset` 与 Linux `stmmac_mdio_reset`。
    fn apply(self, info: &FdtInfo<'_>) -> Result<(), OnProbeError> {
        let gpio_node = info.get_by_phandle(self.gpio_phandle).ok_or_else(|| {
            OnProbeError::other(format!(
                "k3-gmac: reset GPIO phandle {:?} not found",
                self.gpio_phandle
            ))
        })?;
        let reg = gpio_node.regs().into_iter().next().ok_or_else(|| {
            OnProbeError::other(format!("k3-gmac: {} has no reg", gpio_node.name()))
        })?;
        let base = crate::mmio::iomap(reg.address as usize, reg.size.unwrap_or(0x800) as usize)?;
        // SAFETY: iomap 返回的指针指向已映射的 K3 GPIO MMIO 区域。
        let gpio = unsafe { regs::Mmio::new(base) };

        let bank_off = self.bank * regs::K3_GPIO_BANK_STRIDE;
        let bit = 1u32 << self.pin;
        // 设为 output：写 GSDR 对应 bit（K3 GPIO 用 set/clear 方向寄存器）
        gpio.write(bank_off + regs::K3_GPIO_GSDR, bit);

        // 1. 先 deassert（稳定到当前态）
        self.write_level(&gpio, bank_off, bit, false);
        delay_us(self.delays_us.0.max(2));

        // 2. assert（进入复位）
        self.write_level(&gpio, bank_off, bit, true);
        delay_us(self.delays_us.1);

        // 3. deassert（退出复位）
        self.write_level(&gpio, bank_off, bit, false);
        delay_us(self.delays_us.2);

        log::info!(
            "k3-gmac: PHY reset via GPIO{}_{} (active_low={}) delays=({},{},{})us",
            self.bank,
            self.pin,
            self.active_low,
            self.delays_us.0,
            self.delays_us.1,
            self.delays_us.2
        );
        Ok(())
    }

    /// 写 GPIO 输出电平。
    /// `assert` = true 表示进入复位态（active_low 时输出 0，active_high 时输出 1）。
    fn write_level(&self, gpio: &regs::Mmio, bank_off: u32, bit: u32, assert: bool) {
        let physical_high = if self.active_low { !assert } else { assert };
        if physical_high {
            gpio.write(bank_off + regs::K3_GPIO_GPSR, bit); // 置高
        } else {
            gpio.write(bank_off + regs::K3_GPIO_GPCR, bit); // 置低
        }
    }
}

/// K3 syscon glue 配置（从 DTS GMAC 节点解析）。
#[derive(Debug, Clone, Copy)]
pub struct GlueConfig {
    pub phy_mode: PhyMode,
    pub apmu: Option<Phandle>,
    pub ctrl_offset: Option<u32>,
    pub dline_offset: Option<u32>,
    pub wake_irq_enable: bool,
    pub tuning: ClockTuning,
    /// PHY 硬件复位 GPIO（`snps,reset-gpios`）。缺失则跳过复位（依赖 U-Boot 已复位）。
    pub phy_reset: Option<GpioReset>,
}

impl GlueConfig {
    /// 从 FDT 节点解析 glue 配置。
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
            phy_reset: GpioReset::parse(node),
        }
    }

    /// 应用 glue 配置：写 APMU CTRL（总线时钟/复位 + 接口模式 + WoL）
    /// + DLINE（延迟线使能 + 延迟码）。
    ///
    /// 在 probe 阶段调用一次。CTRL 必须先置 BUS_CLK_EN | BUS_RST_DEASSERT，
    /// 否则 GMAC DMA 寄存器无时钟，DMA 软复位永不完成。
    pub fn apply(self, info: &FdtInfo<'_>) -> Result<(), OnProbeError> {
        let Some(apmu) = self.apmu else {
            log::warn!("k3-gmac: missing spacemit,apmu; skip syscon glue");
            return Ok(());
        };
        let Some(ctrl_offset) = self.ctrl_offset else {
            log::warn!("k3-gmac: missing spacemit,ctrl-offset; skip syscon glue");
            return Ok(());
        };
        let syscon_node = info.get_by_phandle(apmu).ok_or_else(|| {
            OnProbeError::other(format!("k3-gmac: syscon phandle {apmu:?} not found"))
        })?;
        let reg = syscon_node.regs().into_iter().next().ok_or_else(|| {
            OnProbeError::other(format!("k3-gmac: {} has no reg", syscon_node.name()))
        })?;
        let size = reg.size.unwrap_or(0x1000) as usize;
        let base = crate::mmio::iomap(reg.address as usize, size)?;
        // SAFETY: iomap 返回的指针指向已映射的 syscon MMIO 区域，生命周期与设备相同。
        let syscon = unsafe { regs::Mmio::new(base) };

        // CTRL：总线时钟使能 + 释放复位 + 接口模式 + WoL
        //
        // 序列严格匹配 U-Boot eqos 驱动（dwc_eth_qos_spacemit.c）：
        //   step 1: 开总线时钟（bit0=1）+ 接口模式 → udelay
        //   step 2: 释放复位（bit1=1）+ WoL → udelay
        // 关键：不先关时钟（U-Boot 从默认态直接开时钟再释放复位；先前
        // "先关→开→释放"的三步脉冲反而让 DMA 子块 SFT_RESET 永不清零）。

        // step 1: 开总线时钟 + 接口模式
        update_bits(
            &syscon,
            ctrl_offset,
            regs::EMAC_BUS_CLK_EN | regs::PHY_INTF_MODE_MASK,
            regs::EMAC_BUS_CLK_EN | self.phy_mode.syscon_value(),
        );
        delay_us(100); // 等时钟稳定

        // step 2: 释放复位 + WoL
        update_bits(
            &syscon,
            ctrl_offset,
            regs::EMAC_BUS_RST_DEASSERT | regs::WOL_WAKE_IRQ_EN,
            regs::EMAC_BUS_RST_DEASSERT
                | if self.wake_irq_enable {
                    regs::WOL_WAKE_IRQ_EN
                } else {
                    0
                },
        );
        delay_us(100);

        log::info!(
            "k3-gmac: syscon CTRL applied phy_mode={:?} wol={} readback={:#010x}",
            self.phy_mode,
            self.wake_irq_enable,
            syscon.read(ctrl_offset)
        );

        // DLINE：延迟线使能 + 延迟码（ByDelayLine 模式）
        if let Some(dline_offset) = self.dline_offset {
            let mut set = regs::EMAC_TX_DLINE_EN | regs::EMAC_RX_DLINE_EN;
            let mut mask = set;
            if let ClockTuning::ByDelayLine { tx_phase, rx_phase } = self.tuning {
                set |= (u32::from(tx_phase) << 24) | (u32::from(rx_phase) << 8);
                mask |= regs::EMAC_TX_DLINE_CODE_MASK | regs::EMAC_RX_DLINE_CODE_MASK;
                log::info!(
                    "k3-gmac: DLINE delayline tx_phase={} rx_phase={}",
                    tx_phase,
                    rx_phase
                );
            }
            update_bits(&syscon, dline_offset, mask, set);
        } else {
            log::warn!("k3-gmac: missing spacemit,dline-offset; skip delayline init");
        }

        // PHY 硬件复位（snps,reset-gpios）。必须在 GMAC 时钟就绪后做，否则 PHY
        // 退出复位时 MDIO 主机还没时钟，PHY 的自协商启动会失败。
        //
        // 注：RGMII/MDC/MDIO 引脚 mux 由 rdrive 框架在本驱动 probe 前自动应用
        // （pinctrl 驱动 priority=CLK 先于 GMAC 的 DEFAULT probe，框架调
        // apply_default_pinctrl 解析 pinctrl-0）。
        if let Some(reset) = self.phy_reset {
            if let Err(err) = reset.apply(info) {
                log::warn!("k3-gmac: PHY GPIO reset failed: {err:?}");
            }
        } else {
            log::debug!("k3-gmac: no snps,reset-gpios; assuming PHY pre-reset by bootloader");
        }
        Ok(())
    }
}

/// 读-改-写一个 syscon 寄存器：`(old & !mask) | (value & mask)`。
fn update_bits(mmio: &regs::Mmio, offset: u32, mask: u32, value: u32) {
    mmio.update(offset, mask, value & mask);
}

/// 粗粒度微秒延时（probe 阶段用，无定时器依赖）。
fn delay_us(us: u32) {
    // spin-wait：每个循环约几 ns，按保守估计迭代 us * 50 次
    // （假设 ~20ns/迭代，避免依赖定时器子系统）。
    for _ in 0..us.saturating_mul(50) {
        core::hint::spin_loop();
    }
}

fn prop_u32(node: &Node, name: &str) -> Option<u32> {
    node.get_property(name).and_then(|prop| prop.get_u32())
}

/// 读一个定长 u32 数组属性（如 `snps,reset-delays-us` 需恰好 3 个）。
fn prop_u32_array_const(node: &Node, name: &str, len: usize) -> Option<alloc::vec::Vec<u32>> {
    let prop = node.get_property(name)?;
    let v: alloc::vec::Vec<u32> = prop.get_u32_iter().take(len).collect();
    (v.len() == len).then_some(v)
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
