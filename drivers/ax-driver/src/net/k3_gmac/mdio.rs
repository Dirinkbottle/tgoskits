//! Minimal GMAC4 MDIO helpers.

use super::regs;

const MDIO_TIMEOUT: usize = 10_000;

pub struct Mdio<'a> {
    mmio: &'a regs::Mmio,
    csr_clock_range: u32,
}

impl<'a> Mdio<'a> {
    pub const fn new(mmio: &'a regs::Mmio) -> Self {
        Self {
            mmio,
            csr_clock_range: 0,
        }
    }

    pub fn read_c22(&self, phy: u8, reg: u8) -> Option<u16> {
        self.wait_idle()?;
        let cmd = self.command(phy, reg, regs::MII_GMAC4_READ);
        self.mmio.write(regs::GMAC_MDIO_ADDR, cmd);
        self.wait_idle()?;
        Some((self.mmio.read(regs::GMAC_MDIO_DATA) & 0xffff) as u16)
    }

    pub fn find_phy(&self) -> Option<u8> {
        (0..32).find(|&addr| {
            let id1 = self.read_c22(addr, 2).unwrap_or(0xffff);
            id1 != 0x0000 && id1 != 0xffff
        })
    }

    fn command(&self, phy: u8, reg: u8, op: u32) -> u32 {
        regs::MII_ADDR_GBUSY
            | op
            | self.csr_clock_range
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
