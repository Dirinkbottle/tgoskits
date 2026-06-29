//! SpacemiT K3/K1 SDHCI register offsets.
//!
//! Vendor offsets and bits are copied from the official Linux K1 driver:
//! `/home/inkbottle/桌面/linux-6.18.35/drivers/mmc/host/sdhci-of-k1.c:23-61`.
//! Standard SDHCI diagnostic offsets are from Linux `sdhci.h:80-164`.

#![allow(dead_code)]

// Standard SDHCI diagnostic registers.
// Reference: drivers/mmc/host/sdhci.h:80-164.
pub const SDHCI_PRESENT_STATE: usize = 0x24;
pub const SDHCI_HOST_CONTROL1: usize = 0x28;
pub const SDHCI_POWER_CONTROL: usize = 0x29;
pub const SDHCI_CLOCK_CONTROL: usize = 0x2c;
pub const SDHCI_NORMAL_INT_STATUS: usize = 0x30;
pub const SDHCI_ERROR_INT_STATUS: usize = 0x32;
pub const SDHCI_HOST_CONTROL2: usize = 0x3e;

// SpacemiT vendor control registers.
// Reference: drivers/mmc/host/sdhci-of-k1.c:23-61.
pub const MMC_CTRL: usize = 0x114;
pub const TX_CFG: usize = 0x11c;
pub const PHY_CTRL: usize = 0x160;
pub const PHY_FUNC: usize = 0x164;
pub const PHY_DLLCFG: usize = 0x168;
pub const PHY_DLLCFG1: usize = 0x16c;
pub const PHY_DLLSTS: usize = 0x170;
pub const PHY_PADCFG: usize = 0x178;

// MMC_CTRL bits.
// Reference: drivers/mmc/host/sdhci-of-k1.c:23-29.
pub const MISC_INT_EN: u32 = 1 << 1;
pub const MISC_INT: u32 = 1 << 2;
pub const ENHANCE_STROBE_EN: u32 = 1 << 8;
pub const MMC_HS400: u32 = 1 << 9;
pub const MMC_HS200: u32 = 1 << 10;
pub const MMC_CARD_MODE: u32 = 1 << 12;

// TX_CFG bits.
// Reference: drivers/mmc/host/sdhci-of-k1.c:31-33.
pub const TX_INT_CLK_SEL: u32 = 1 << 30;
pub const TX_MUX_SEL: u32 = 1 << 31;

// PHY_CTRL bits.
// Reference: drivers/mmc/host/sdhci-of-k1.c:35-38.
pub const PHY_FUNC_EN: u32 = 1 << 0;
pub const PHY_PLL_LOCK: u32 = 1 << 1;
pub const HOST_LEGACY_MODE: u32 = 1 << 31;

// PHY_FUNC bits.
// Reference: drivers/mmc/host/sdhci-of-k1.c:40-42.
pub const PHY_TEST_EN: u32 = 1 << 7;
pub const HS200_USE_RFIFO: u32 = 1 << 15;

// PHY_DLLCFG bits.
// Reference: drivers/mmc/host/sdhci-of-k1.c:44-48.
pub const DLL_PREDLY_NUM_MASK: u32 = 0b11 << 2;
pub const DLL_FULLDLY_RANGE_MASK: u32 = 0b11 << 4;
pub const DLL_VREG_CTRL_MASK: u32 = 0b11 << 6;
pub const DLL_ENABLE: u32 = 1 << 31;

// PHY_DLLCFG1 fields.
// Reference: drivers/mmc/host/sdhci-of-k1.c:50-54.
pub const DLL_REG1_CTRL_MASK: u32 = 0xff;
pub const DLL_REG2_CTRL_MASK: u32 = 0xff << 8;
pub const DLL_REG3_CTRL_MASK: u32 = 0xff << 16;
pub const DLL_REG4_CTRL_MASK: u32 = 0xff << 24;

// PHY_DLLSTS bits.
// Reference: drivers/mmc/host/sdhci-of-k1.c:56-57.
pub const DLL_LOCK_STATE: u32 = 1 << 0;

// PHY_PADCFG fields.
// Reference: drivers/mmc/host/sdhci-of-k1.c:59-61 and reset sequence lines 97-99.
pub const PHY_DRIVE_SEL_MASK: u32 = 0b111;
pub const PHY_DRIVE_SEL_VALUE_4: u32 = 4;
pub const RX_BIAS_CTRL: u32 = 1 << 5;

pub const fn phy_drive_sel(value: u32) -> u32 {
    value & PHY_DRIVE_SEL_MASK
}
