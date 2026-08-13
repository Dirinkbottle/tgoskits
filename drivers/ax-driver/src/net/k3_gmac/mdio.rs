//! DWMAC4 MDIO Clause 22 读写辅助。
//!
//! CSR 时钟分频由调用方传入：K3 的 stmmaceth CSR 时钟在 250-300MHz 范围，
//! 对应 CR=5（来源：U-Boot eqos_spacemit_k3_config）。

use super::regs;

const MDIO_TIMEOUT: usize = 10_000;

/// GMAC 内置 MDIO 主机控制器，通过 GMAC_MDIO_ADDR/DATA 寄存器读写 PHY。
pub struct Mdio<'a> {
    mmio: &'a regs::Mmio,
    /// MDC 时钟分频值（已移位到 bit11:8），由 probe 按 CSR 时钟频率决定。
    csr_clock_range: u32,
}

impl<'a> Mdio<'a> {
    pub const fn new(mmio: &'a regs::Mmio, csr_clock_range: u32) -> Self {
        Self {
            mmio,
            csr_clock_range,
        }
    }

    /// Clause 22 读：读 phy 的 reg 寄存器，返回 16 位值。
    pub fn read_c22(&self, phy: u8, reg: u8) -> Option<u16> {
        self.wait_idle()?;
        let cmd = self.command(phy, reg, regs::MII_GMAC4_READ);
        self.mmio.write(regs::GMAC_MDIO_ADDR, cmd);
        self.wait_idle()?;
        Some((self.mmio.read(regs::GMAC_MDIO_DATA) & 0xffff) as u16)
    }

    /// Clause 22 写：向 phy 的 reg 寄存器写 16 位值。
    /// 来源：stmmac_mdio.c `stmmac_mdio_write_c22` → `stmmac_mdio_access`。
    pub fn write_c22(&self, phy: u8, reg: u8, val: u16) -> Option<()> {
        self.wait_idle()?;
        self.mmio.write(regs::GMAC_MDIO_DATA, val as u32);
        let cmd = self.command(phy, reg, regs::MII_GMAC4_WRITE);
        self.mmio.write(regs::GMAC_MDIO_ADDR, cmd);
        self.wait_idle()?;
        Some(())
    }

    // -----------------------------------------------------------------------
    // IEEE 802.3 clause 22 寄存器号与位掩码（MII 标准）
    // -----------------------------------------------------------------------
    const MII_BMCR: u8 = 0;
    const MII_BMSR: u8 = 1;
    const MII_ADVERTISE: u8 = 4;
    const MII_LPA: u8 = 5;
    const MII_CTRL1000: u8 = 9;
    const MII_STAT1000: u8 = 10;

    // BMCR 位（init_phy 用到的）
    const BMCR_RESET: u16 = 1 << 15;
    const BMCR_ANENABLE: u16 = 1 << 12;
    const BMCR_ANRESTART: u16 = 1 << 9;

    // BMSR 位
    const BMSR_ANEGCOMPLETE: u16 = 1 << 5;
    const BMSR_LSTATUS: u16 = 1 << 2;
    const BMSR_ESTATEN: u16 = 1 << 8;

    // ADVERTISE 位（10/100 能力）
    const ADV_10HALF: u16 = 1 << 5; // 0x0020
    const ADV_10FULL: u16 = 1 << 6; // 0x0040
    const ADV_100HALF: u16 = 1 << 7; // 0x0080
    const ADV_100FULL: u16 = 1 << 8; // 0x0100
    const ADV_PAUSE_CAP: u16 = 1 << 10; // 0x0400
    const ADV_PAUSE_ASYM: u16 = 1 << 11; // 0x0800

    // CTRL1000 位（1000M 能力通告）
    const ADV_1000FULL: u16 = 1 << 9; // 0x0200

    /// PHY 完整 bring-up（genphy 路径，参照 Linux `genphy_config_aneg` +
    /// `genphy_update_link` + U-Boot `genphy_startup`）。
    ///
    /// 步骤：① 软复位 BMCR.RESET 并等待自清 → ② 写 ADVERTISE（10/100 FD+HD + pause）
    ///    → ③ 写 CTRL1000（1000FD） → ④ 重启自协商 BMCR.ANENABLE|ANRESTART
    ///    → ⑤ 轮询 BMSR.ANEGCOMPLETE（最长 ~5s） → ⑥ 轮询 BMSR.LSTATUS。
    ///
    /// 用 spin_loop 做等待（probe 上下文，与 syscon.rs delay_us 一致）。
    /// 返回协商结果，便于上层按实际速率配 MAC。
    pub fn init_phy(&self, phy: u8, target_speed_mbps: u32) -> Option<PhyLinkState> {
        log::info!("k3-gmac: PHY{phy} init_phy begin (target_speed={target_speed_mbps}M)");

        // ① 软复位 BMCR.RESET，轮询自清（最长 ~500ms）
        self.write_c22(phy, Self::MII_BMCR, Self::BMCR_RESET)?;
        if !self.wait_bmcr_reset_clear(phy) {
            log::warn!("k3-gmac: PHY{phy} BMCR.RESET stuck; continuing");
        }

        // ② 写 ADVERTISE：10/100 全双工+半双工 + 非对称暂停（0x0DE1 标准值）
        let adv = Self::ADV_10HALF
            | Self::ADV_10FULL
            | Self::ADV_100HALF
            | Self::ADV_100FULL
            | Self::ADV_PAUSE_CAP
            | Self::ADV_PAUSE_ASYM;
        self.write_c22(phy, Self::MII_ADVERTISE, adv)?;
        log::info!("k3-gmac: PHY{phy} ADVERTISE={adv:#06x} written");

        // ③ 写 CTRL1000：仅当 PHY 支持 1000T（BMSR_ESTATEN）且目标是 1000M
        let bmsr = self.read_c22(phy, Self::MII_BMSR).unwrap_or(0);
        if target_speed_mbps >= 1000 && (bmsr & Self::BMSR_ESTATEN) != 0 {
            self.write_c22(phy, Self::MII_CTRL1000, Self::ADV_1000FULL)?;
            log::info!("k3-gmac: PHY{phy} CTRL1000=0x0200 (1000FD) written");
        } else {
            // 关闭 1000M 通告
            self.write_c22(phy, Self::MII_CTRL1000, 0)?;
            log::info!("k3-gmac: PHY{phy} CTRL1000=0 (1000M disabled, target<1000 or no ESTATEN)");
        }

        // ④ 重启自协商：写 BMCR = ANENABLE | ANRESTART（保留 1000M 速率位由协商决定）
        self.write_c22(
            phy,
            Self::MII_BMCR,
            Self::BMCR_ANENABLE | Self::BMCR_ANRESTART,
        )?;
        log::info!("k3-gmac: PHY{phy} autoneg restarted");

        // ⑤ 轮询 BMSR.ANEGCOMPLETE（每 100ms，最长 5s）
        let aneg_done = self.wait_autoneg_complete(phy);
        if aneg_done {
            log::info!("k3-gmac: PHY{phy} autoneg COMPLETE");
        } else {
            log::warn!("k3-gmac: PHY{phy} autoneg NOT complete after timeout; continuing anyway");
        }

        // ⑥ 读 BMSR.LSTATUS（两次去锁存）确认链路
        let _ = self.read_c22(phy, Self::MII_BMSR);
        let bmsr_final = self.read_c22(phy, Self::MII_BMSR).unwrap_or(0);
        let link = bmsr_final & Self::BMSR_LSTATUS != 0;

        // 读协商结果（STAT1000 + LPA）确定速率/双工
        let stat1000 = self.read_c22(phy, Self::MII_STAT1000).unwrap_or(0);
        let lpa = self.read_c22(phy, Self::MII_LPA).unwrap_or(0);
        let ctrl1000 = self.read_c22(phy, Self::MII_CTRL1000).unwrap_or(0);
        let (speed, full_duplex) =
            if stat1000 & (Self::ADV_1000FULL << 2) != 0 && ctrl1000 & Self::ADV_1000FULL != 0 {
                (1000, true)
            } else if lpa & Self::ADV_100FULL != 0 {
                (100, true)
            } else if lpa & Self::ADV_100HALF != 0 {
                (100, false)
            } else if lpa & Self::ADV_10FULL != 0 {
                (10, true)
            } else {
                (10, false)
            };

        let state = PhyLinkState {
            up: link,
            aneg_complete: aneg_done,
            speed_mbps: speed,
            full_duplex,
        };
        log::info!(
            "k3-gmac: PHY{phy} link state: up={} speed={}Mbps duplex={} (LPA={lpa:#06x} \
             CTRL1000={ctrl1000:#06x} STAT1000={stat1000:#06x})",
            state.up,
            state.speed_mbps,
            if state.full_duplex { "full" } else { "half" },
        );
        Some(state)
    }

    /// 轮询 BMCR.RESET 自清（最长 ~500ms）。
    fn wait_bmcr_reset_clear(&self, phy: u8) -> bool {
        for _ in 0..50 {
            // 每 10ms 检查一次（spin_loop 估计，5_000_000 迭代 ≈ 10ms @ 2GHz）
            for _ in 0..50_000 {
                core::hint::spin_loop();
            }
            let bmcr = self.read_c22(phy, Self::MII_BMCR).unwrap_or(0xffff);
            if bmcr & Self::BMCR_RESET == 0 {
                return true;
            }
        }
        false
    }

    /// 轮询 BMSR.ANEGCOMPLETE（每 ~100ms，最长 5s = 50 次迭代）。
    /// 用 spin_loop 延时（probe 上下文，与 syscon.rs delay_us 一致，无额外依赖）。
    fn wait_autoneg_complete(&self, phy: u8) -> bool {
        for i in 0..50u32 {
            // 第一次读 BMSR 可能返回旧锁存值，多读几次去抖
            let _ = self.read_c22(phy, Self::MII_BMSR);
            let bmsr = self.read_c22(phy, Self::MII_BMSR).unwrap_or(0);
            if bmsr & Self::BMSR_ANEGCOMPLETE != 0 {
                log::info!("k3-gmac: PHY{phy} autoneg complete after {}*100ms", i + 1);
                return true;
            }
            // spin ~100ms（5_000_000 迭代 ≈ 10ms @ ~2GHz，×10 = 100ms）
            for _ in 0..50_000_000 {
                core::hint::spin_loop();
            }
        }
        false
    }

    /// 扫描 MDIO 总线（addr 0..8），返回第一个有合法 PHYID1 的地址。
    /// 只扫 0..8 而非 0..32：标准 MDIO PHY 地址范围，覆盖实际 PHY。
    pub fn find_phy(&self) -> Option<u8> {
        for addr in 0..8u8 {
            let id1 = self.read_c22(addr, 2).unwrap_or(0xffff);
            if id1 != 0x0000 && id1 != 0xffff {
                log::info!("k3-gmac: MDIO scan addr {} PHYID1={:#06x}", addr, id1);
                return Some(addr);
            }
        }
        None
    }

    fn command(&self, phy: u8, reg: u8, op: u32) -> u32 {
        regs::MII_ADDR_GBUSY
            | op
            | (self.csr_clock_range << regs::MDIO_CSR_CLK_SHIFT)
            | (u32::from(phy) << 21)
            | (u32::from(reg) << regs::MII_GMAC4_REG_ADDR_SHIFT)
    }

    fn wait_idle(&self) -> Option<()> {
        for _ in 0..MDIO_TIMEOUT {
            if (self.mmio.read(regs::GMAC_MDIO_ADDR) & regs::MII_ADDR_GBUSY) == 0 {
                return Some(());
            }
            core::hint::spin_loop();
        }
        None
    }
}

/// PHY 协商结果：链路状态 + 协商到的速率/双工。
#[derive(Debug, Clone, Copy)]
pub struct PhyLinkState {
    pub up: bool,
    pub aneg_complete: bool,
    pub speed_mbps: u32,
    pub full_duplex: bool,
}
