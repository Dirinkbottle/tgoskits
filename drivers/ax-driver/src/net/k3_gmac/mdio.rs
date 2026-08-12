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

    /// 扫描 MDIO 总线（addr 0..31），返回第一个有合法 PHYID1 的地址。
    /// 同时记录扫描到的非空 PHY（info 级），便于排查 PHY 地址与时序问题。
    pub fn find_phy(&self) -> Option<u8> {
        // 首次扫描前 dump MDIO 寄存器初始态，定位 wait_idle / GBUSY 卡死问题
        let addr_before = self.mmio.read(regs::GMAC_MDIO_ADDR);
        let data_before = self.mmio.read(regs::GMAC_MDIO_DATA);
        log::info!(
            "k3-gmac: MDIO init state GMAC_MDIO_ADDR={:#010x} GMAC_MDIO_DATA={:#010x} (GBUSY={})",
            addr_before,
            data_before,
            (addr_before & regs::MII_ADDR_GBUSY) != 0,
        );

        let mut found: Option<u8> = None;
        for addr in 0..32u8 {
            // 对 addr 0/1 做详细诊断（PHY 最可能在这两个地址）
            let id1 = if addr < 2 {
                self.read_c22_verbose(addr, 2).unwrap_or(0xffff)
            } else {
                self.read_c22(addr, 2).unwrap_or(0xffff)
            };
            if id1 != 0x0000 && id1 != 0xffff {
                log::info!("k3-gmac: MDIO scan addr {} PHYID1={:#06x}", addr, id1);
                if found.is_none() {
                    found = Some(addr);
                }
            }
        }
        found
    }

    /// 带 verbose 日志的 C22 读：记录 wait_idle 入口/出口与最终 data，
    /// 用于定位 MDIO 控制器是否响应（区分"无 PHY"与"MDIO 不工作"）。
    fn read_c22_verbose(&self, phy: u8, reg: u8) -> Option<u16> {
        let entry_busy = self.mmio.read(regs::GMAC_MDIO_ADDR) & regs::MII_ADDR_GBUSY;
        let idle_entry = self.wait_idle();
        let cmd = self.command(phy, reg, regs::MII_GMAC4_READ);
        self.mmio.write(regs::GMAC_MDIO_ADDR, cmd);
        let written_back = self.mmio.read(regs::GMAC_MDIO_ADDR);
        let idle_exit = self.wait_idle();
        let data = self.mmio.read(regs::GMAC_MDIO_DATA) & 0xffff;
        log::info!(
            "k3-gmac: MDIO read phy={} reg={}: entry_busy={} idle_entry={} cmd={:#010x} \
             written_back={:#010x} idle_exit={} data={:#06x}",
            phy,
            reg,
            entry_busy != 0,
            idle_entry.is_some(),
            cmd,
            written_back,
            idle_exit.is_some(),
            data,
        );
        idle_exit?;
        Some(data as u16)
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
