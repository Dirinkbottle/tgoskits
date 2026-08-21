//! SpacemiT K3 SDHCI Host Controller Driver
//!
//! Rust port of the Linux sdhci-of-k1.c driver for K3/K1 SoCs.

#![no_std]

use log::{debug, warn};
use mmio_api::MmioRaw;

mod vendor_ext;
pub use vendor_ext::SdhciVendorExt;

#[cfg(feature = "platform")]
pub mod platform;

/// K3 SDHCI register offsets
mod regs {
    pub const OP_EXT: usize = 0x108;
    pub const MMC_CTRL: usize = 0x114;
    pub const RX_CFG: usize = 0x118;
    pub const TX_CFG: usize = 0x11C;
    pub const DLINE_CTRL: usize = 0x130;
    pub const DLINE_CFG: usize = 0x134;
    pub const PHY_CTRL: usize = 0x160;
    pub const PHY_FUNC: usize = 0x164;
    pub const PHY_DLLCFG: usize = 0x168;
    pub const PHY_DLLCFG1: usize = 0x16C;
    pub const PHY_DLLSTS: usize = 0x170;
    pub const PHY_PADCFG: usize = 0x178;
}

bitflags::bitflags! {
    #[derive(Copy, Clone, Debug)]
    pub struct OpExtFlags: u32 {
        const OVRRD_CLK_OEN = 1 << 11;
        const FORCE_CLK_ON = 1 << 12;
    }

    #[derive(Copy, Clone, Debug)]
    pub struct MmcCtrlFlags: u32 {
        const MISC_INT_EN = 1 << 1;
        const MISC_INT = 1 << 2;
        const ENHANCE_STROBE_EN = 1 << 8;
        const MMC_HS400 = 1 << 9;
        const MMC_HS200 = 1 << 10;
        const MMC_CARD_MODE = 1 << 12;
    }

    #[derive(Copy, Clone, Debug)]
    pub struct TxCfgFlags: u32 {
        const TX_INT_CLK_SEL = 1 << 30;
        const TX_MUX_SEL = 1 << 31;
    }

    #[derive(Copy, Clone, Debug)]
    pub struct PhyCtrlFlags: u32 {
        const PHY_FUNC_EN = 1 << 0;
        const PHY_PLL_LOCK = 1 << 1;
        const HOST_LEGACY_MODE = 1 << 31;
    }

    #[derive(Copy, Clone, Debug)]
    pub struct PhyFuncFlags: u32 {
        const PHY_TEST_EN = 1 << 7;
        const HS200_USE_RFIFO = 1 << 15;
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Timing {
    Legacy,
    MmcHs,
    SdHs,
    UhsSdr12,
    UhsSdr25,
    UhsSdr50,
    UhsSdr104,
    MmcHs200,
    MmcHs400,
}

#[derive(Debug, Copy, Clone)]
pub struct TuningWindow {
    min_delay: u8,
    max_delay: u8,
}

pub struct RxTuning {
    tx_delaycode: u8,
    tx_dline_reg: u8,
    rx_dline_reg: u8,
    windows: TuningWindow,
    select_delay: u8,
    window_limit: u8,
}

impl Default for RxTuning {
    fn default() -> Self {
        Self {
            tx_delaycode: 0x7F,
            tx_dline_reg: 0,
            rx_dline_reg: 0,
            windows: TuningWindow {
                min_delay: 0,
                max_delay: 0,
            },
            select_delay: 0,
            window_limit: 50,
        }
    }
}

pub struct K3SdhciHost {
    mmio: MmioRaw,
    phy_driver_sel: u8,
    rxtuning: RxTuning,
}

impl K3SdhciHost {
    pub fn new(mmio: MmioRaw) -> Self {
        Self {
            mmio,
            phy_driver_sel: 4,
            rxtuning: RxTuning::default(),
        }
    }

    #[inline]
    fn read32(&self, offset: usize) -> u32 {
        self.mmio.read(offset)
    }

    #[inline]
    fn write32(&self, offset: usize, val: u32) {
        self.mmio.write(offset, val);
    }

    #[inline]
    fn setbits(&mut self, offset: usize, bits: u32) {
        let val = self.read32(offset);
        self.write32(offset, val | bits);
    }

    #[inline]
    fn clrbits(&mut self, offset: usize, bits: u32) {
        let val = self.read32(offset);
        self.write32(offset, val & !bits);
    }

    #[inline]
    fn clrsetbits(&mut self, offset: usize, clr: u32, set: u32) {
        let val = self.read32(offset);
        self.write32(offset, (val & !clr) | set);
    }

    pub fn reset(&mut self, is_emmc: bool) {
        if is_emmc {
            self.setbits(
                regs::PHY_CTRL,
                PhyCtrlFlags::PHY_FUNC_EN.bits() | PhyCtrlFlags::PHY_PLL_LOCK.bits(),
            );
            self.clrsetbits(
                regs::PHY_PADCFG,
                0x7,
                (1 << 5) | (self.phy_driver_sel as u32 & 0x7),
            );
            self.setbits(regs::MMC_CTRL, MmcCtrlFlags::MMC_CARD_MODE.bits());
        } else {
            self.setbits(regs::TX_CFG, TxCfgFlags::TX_INT_CLK_SEL.bits());
        }
    }

    pub fn set_timing(&mut self, timing: Timing) {
        match timing {
            Timing::MmcHs200 => {
                self.setbits(regs::MMC_CTRL, MmcCtrlFlags::MMC_HS200.bits());
            }
            Timing::MmcHs400 => {
                self.setbits(regs::MMC_CTRL, MmcCtrlFlags::MMC_HS400.bits());
            }
            _ => {}
        }
    }

    pub fn set_clock_gate(&mut self, auto_gate: bool) {
        let flags = OpExtFlags::OVRRD_CLK_OEN | OpExtFlags::FORCE_CLK_ON;
        if auto_gate {
            self.clrbits(regs::OP_EXT, flags.bits());
        } else {
            self.setbits(regs::OP_EXT, flags.bits());
        }
    }

    pub fn set_clock(&mut self, timing: Timing) {
        if matches!(
            timing,
            Timing::Legacy
                | Timing::MmcHs
                | Timing::SdHs
                | Timing::UhsSdr12
                | Timing::UhsSdr25
                | Timing::UhsSdr50
        ) {
            self.setbits(regs::TX_CFG, TxCfgFlags::TX_INT_CLK_SEL.bits());
        } else {
            self.clrbits(regs::TX_CFG, TxCfgFlags::TX_INT_CLK_SEL.bits());
        }
    }

    fn phy_dll_init(&mut self) -> Result<(), ()> {
        self.clrsetbits(regs::PHY_DLLCFG, 0xFC, (1 << 2) | (1 << 4) | (1 << 6));
        self.clrsetbits(regs::PHY_DLLCFG1, 0xFF, 0x92);
        self.setbits(regs::PHY_DLLCFG, 1 << 31);

        for _ in 0..50 {
            if self.read32(regs::PHY_DLLSTS) & 1 != 0 {
                return Ok(());
            }
            // 2us delay
        }
        warn!("PHY DLL lock timeout");
        Err(())
    }

    pub fn enable_hs400_strobe(&mut self, enable: bool) -> Result<(), ()> {
        if enable {
            self.setbits(regs::MMC_CTRL, MmcCtrlFlags::ENHANCE_STROBE_EN.bits());
            self.phy_dll_init()?;
        } else {
            self.clrbits(regs::MMC_CTRL, MmcCtrlFlags::ENHANCE_STROBE_EN.bits());
        }
        Ok(())
    }

    pub fn prepare_hs400(&mut self) {
        self.setbits(regs::MMC_CTRL, MmcCtrlFlags::MMC_HS400.bits());
    }

    pub fn post_hs400_config(&mut self) -> Result<(), ()> {
        self.phy_dll_init()
    }

    pub fn hs400_to_hs200(&mut self) {
        self.clrbits(
            regs::PHY_CTRL,
            PhyCtrlFlags::PHY_FUNC_EN.bits() | PhyCtrlFlags::PHY_PLL_LOCK.bits(),
        );
        self.clrbits(
            regs::MMC_CTRL,
            MmcCtrlFlags::MMC_HS400.bits()
                | MmcCtrlFlags::MMC_HS200.bits()
                | MmcCtrlFlags::ENHANCE_STROBE_EN.bits(),
        );
        self.clrbits(regs::PHY_FUNC, PhyFuncFlags::HS200_USE_RFIFO.bits());
        // 5us delay
        self.setbits(
            regs::PHY_CTRL,
            PhyCtrlFlags::PHY_FUNC_EN.bits() | PhyCtrlFlags::PHY_PLL_LOCK.bits(),
        );
    }

    fn sw_rx_tuning_prepare(&mut self, dline_reg: u8) {
        self.clrsetbits(regs::DLINE_CFG, 0xFF, dline_reg as u32);
        self.setbits(regs::DLINE_CTRL, 1);
        // 5us delay
        self.clrsetbits(regs::RX_CFG, 0xC, 1 << 2);
    }

    fn sw_rx_set_delaycode(&mut self, delay: u8) {
        self.clrsetbits(regs::DLINE_CTRL, 0xFF << 16, (delay as u32) << 16);
    }

    pub fn execute_tuning<F>(&mut self, timing: Timing, mut test_fn: F) -> Result<(), ()>
    where
        F: FnMut(u8) -> bool,
    {
        if timing != Timing::MmcHs200 && timing != Timing::UhsSdr50 && timing != Timing::UhsSdr104 {
            return Ok(());
        }

        self.sw_rx_tuning_prepare(self.rxtuning.rx_dline_reg);

        let mut max_windows = 0;
        let mut cur_windows = 0;
        let mut min = 0;
        let mut max = 0;

        for delay in 0..=0xFF {
            self.sw_rx_set_delaycode(delay);

            if test_fn(delay) {
                if cur_windows == 0 {
                    min = delay;
                }
                cur_windows += 1;
                if cur_windows > max_windows {
                    max_windows = cur_windows;
                    max = delay;
                }
            } else {
                cur_windows = 0;
            }
        }

        if max_windows < self.rxtuning.window_limit {
            warn!("Tuning failed: max_windows={}", max_windows);
            return Err(());
        }

        self.rxtuning.windows.min_delay = max.saturating_sub(max_windows - 1);
        self.rxtuning.windows.max_delay = max;
        self.rxtuning.select_delay = self.rxtuning.windows.min_delay + max_windows / 2;

        self.sw_rx_set_delaycode(self.rxtuning.select_delay);
        debug!("Tuning done: delay={}", self.rxtuning.select_delay);

        Ok(())
    }
}
