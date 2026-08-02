//! K3 UFS host register map and hardware constants.
//!
//! Register offsets are relative to the mapped UFSHCI base; the host's
//! `read32`/`write32` methods add the mapped base pointer. Values come from
//! the SpacemiT K3 UFS datasheet and from Linux (`include/ufs/ufshci.h`,
//! `include/ufs/unipro.h`, `drivers/ufs/host/ufs-spacemit.c`).

/// K3 UFS vendor registers (Linux: ufs-spacemit.c UFS_SYS1CLK_1US).
pub(super) const UFS_SYS1CLK_1US: usize = 0xC0;
pub(super) const UFS_TX_SYMBOL_CLK_NS_US: usize = 0xC4;
pub(super) const UFS_PA_LINK_STARTUP_TIMER: usize = 0xD8;
/// TX symbol clock value written to `UFS_TX_SYMBOL_CLK_NS_US`
/// (Linux: ufs-spacemit.h UFS_TX_SYMBO_CLK).
pub(super) const UFS_TX_SYMBO_CLK: u32 = 0x800;

/// MPHY control block.
pub(super) const UFS_PHY_MNG_BASE: usize = 0x1B00;
pub(super) const UFS_MPHY_PU_CTRL: usize = 0x4;
pub(super) const UFS_MPHY_BKDR_CTRL: usize = 0x8;
pub(super) const UFS_DEVICE_IO_CTRL: usize = 0xC;

/// ATOP (Analog Top) register block.
pub(super) const UFS_ATOP_BASE: usize = 0x1C00;

/// MPHY power-up register values (Linux: ufs-spacemit.c mphy_init).
pub(super) const MPHY_PU_ALL: u32 = 0x87f;
pub(super) const MPHY_PU_WITH_HB8_RESET: u32 = 0xb7f;
pub(super) const MPHY_DEVICE_RESET_DEASSERT: u32 = 0x101;
/// PLL lock status bit in the MPHY power-up register.
pub(super) const MPHY_PLL_LOCK_BIT: u32 = 1 << 31;

/// UFSHCI host controller registers.
pub(super) const REG_CONTROLLER_CAPABILITIES: usize = 0x00;
pub(super) const REG_UFS_VERSION: usize = 0x08;
pub(super) const REG_INTERRUPT_STATUS: usize = 0x20;
pub(super) const REG_INTERRUPT_ENABLE: usize = 0x24;
pub(super) const REG_CONTROLLER_STATUS: usize = 0x30;
pub(super) const REG_CONTROLLER_ENABLE: usize = 0x34;
pub(super) const REG_UTP_TRANSFER_REQ_INT_AGG_CONTROL: usize = 0x4C;
pub(super) const REG_UTP_TRANSFER_REQ_LIST_BASE_L: usize = 0x50;
pub(super) const REG_UTP_TRANSFER_REQ_LIST_BASE_H: usize = 0x54;
pub(super) const REG_UTP_TRANSFER_REQ_DOOR_BELL: usize = 0x58;
pub(super) const REG_UTP_TRANSFER_REQ_LIST_RUN_STOP: usize = 0x60;
pub(super) const REG_UTP_TASK_REQ_LIST_BASE_L: usize = 0x70;
pub(super) const REG_UTP_TASK_REQ_LIST_BASE_H: usize = 0x74;
pub(super) const REG_UTP_TASK_REQ_DOOR_BELL: usize = 0x78;
pub(super) const REG_UTP_TASK_REQ_LIST_RUN_STOP: usize = 0x80;
pub(super) const REG_UIC_COMMAND: usize = 0x90;
pub(super) const REG_UIC_COMMAND_ARG1: usize = 0x94;
pub(super) const REG_UIC_COMMAND_ARG2: usize = 0x98;
pub(super) const REG_UIC_COMMAND_ARG3: usize = 0x9C;

/// UIC error code registers (Linux: include/ufs/ufshci.h). These are sticky
/// status registers that are consumed (cleared) by a read.
pub(super) const REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER: usize = 0x38;
pub(super) const REG_UIC_ERROR_CODE_DATA_LINK_LAYER: usize = 0x3C;
pub(super) const REG_UIC_ERROR_CODE_NETWORK_LAYER: usize = 0x40;
pub(super) const REG_UIC_ERROR_CODE_TRANSPORT_LAYER: usize = 0x44;
pub(super) const REG_UIC_ERROR_CODE_DME: usize = 0x48;

/// UIC commands (Linux: include/ufs/ufshci.h).
pub(super) const UIC_CMD_DME_GET: u32 = 0x01;
pub(super) const UIC_CMD_DME_SET: u32 = 0x02;
pub(super) const UIC_CMD_DME_PEER_GET: u32 = 0x03;
pub(super) const UIC_CMD_DME_LINK_STARTUP: u32 = 0x16;
/// UIC command bit: generate an interrupt on command completion
/// (Linux: UIC_COMMAND_CGE).
pub(super) const UIC_CMD_CGE: u32 = 1 << 8;
/// Low-byte mask of the UIC command result in `REG_UIC_COMMAND_ARG2`
/// (Linux: MASK_UIC_COMMAND_RESULT, 0 = success).
pub(super) const MASK_UIC_COMMAND_RESULT: u32 = 0xFF;

/// UIC MIB attributes (from Linux ufs-spacemit.c).
pub(super) const PA_TXHSG1SYNCLENGTH: u32 = 0x1552;
pub(super) const PA_TXHSG1PREPARELENGTH: u32 = 0x1553;
pub(super) const PA_TXHSG2SYNCLENGTH: u32 = 0x1554;
pub(super) const PA_TXHSG2PREPARELENGTH: u32 = 0x1555;
pub(super) const PA_TXHSG3SYNCLENGTH: u32 = 0x1556;
pub(super) const PA_TXHSG3PREPARELENGTH: u32 = 0x1557;
pub(super) const PA_TXMK2EXTENSION: u32 = 0x155A;
pub(super) const PA_PEERSCRAMBLING: u32 = 0x155B;
pub(super) const PA_TXSKIP: u32 = 0x155C;
pub(super) const PA_TXSKIPPERIOD: u32 = 0x155D;
pub(super) const PA_LOCAL_TX_LCC_ENABLE: u32 = 0x155E;
pub(super) const PA_PEER_TX_LCC_ENABLE: u32 = 0x155F;
pub(super) const PA_SCRAMBLING: u32 = 0x1585;
pub(super) const PA_GRANULARITY: u32 = 0x15AA;
pub(super) const PA_MK2EXTENSIONGUARDBAND: u32 = 0x15AB;
pub(super) const PA_STALLNOCONFIGTIME: u32 = 0x15A3;
pub(super) const PA_TACTIVATE: u32 = 0x15A8;
pub(super) const PA_TXTRAILINGCLOCKS: u32 = 0x1564;

/// RX/TX lane-specific attributes.
pub(super) const RX_LS_PRE_LEN_CAP: u32 = 0x008D;
pub(super) const RX_LANE_HB8_BKDOOR_ATTR: u32 = 0x00F4;
pub(super) const RX_PWRM_CLOSURE_LEN_CAP: u32 = 0x008E;
pub(super) const RX_MIN_STALL_CAP: u32 = 0x0088;
pub(super) const TX_HIBERN8TIME_CAP: u32 = 0x000F;
pub(super) const RX_HIBERN8TIME_CAP: u32 = 0x0092;
pub(super) const ANA_EQ_CTRL_REG_ATTR: u32 = 0x00CD;
pub(super) const RX_GARBAGE_COUNT_OFFSET: u32 = 0x00F2;

/// Special analog / M-PHY attributes (Linux: drivers/ufs/host/ufs-spacemit.c).
///
/// `ANA_HSGEAR_CTRL_ATTR` is the SpacemiT "backdoor" analog register that
/// pre-sets TX rate/gear so the M-PHY PLL can lock at the target HS rate
/// before the PA power-mode change (`ufs_spacemit_apply_dev_quirks`).
pub(super) const ANA_HSGEAR_CTRL_ATTR: u32 = 0x00C1;
/// M-TX attributes programmed during `apply_dev_quirks` (Linux unipro.h).
pub(super) const TX_LCC_ENABLE: u32 = 0x002C;
pub(super) const TX_MIN_ACTIVATETIME: u32 = 0x0033;
/// Vendor MIB attribute written after link startup to make a UFS2.1 device
/// run at GEAR3 + 2 lanes (Linux: "add 0xe8 make UFS2.1 run GEAR3 + 2Lane@409M").
pub(super) const UFS_SPACEMIT_GEAR3_ATTR: u32 = 0x00E8;

/// PA power-mode attributes (Linux: include/ufs/unipro.h).
///
/// These configure the target gear/lane/termination before the PA power-mode
/// change is triggered by writing [`PA_PWRMODE`] (Linux:
/// `ufshcd_dme_change_power_mode`).
pub(super) const PA_ACTIVETXDATALANES: u32 = 0x1560;
pub(super) const PA_CONNECTEDTXDATALANES: u32 = 0x1561;
pub(super) const PA_TXGEAR: u32 = 0x1568;
pub(super) const PA_TXTERMINATION: u32 = 0x1569;
pub(super) const PA_HSSERIES: u32 = 0x156A;
pub(super) const PA_PWRMODE: u32 = 0x1571;
pub(super) const PA_AVAILTXDATALANES: u32 = 0x1520;
pub(super) const PA_AVAILRXDATALANES: u32 = 0x1540;
pub(super) const PA_ACTIVERXDATALANES: u32 = 0x1580;
pub(super) const PA_CONNECTEDRXDATALANES: u32 = 0x1581;
pub(super) const PA_RXGEAR: u32 = 0x1583;
pub(super) const PA_RXTERMINATION: u32 = 0x1584;
pub(super) const PA_MAXRXPWMGEAR: u32 = 0x1586;
pub(super) const PA_MAXRXHSGEAR: u32 = 0x1587;

/// UniPro power-mode user data attributes, programmed with the DL-layer
/// timeout defaults before a power-mode change (Linux: include/ufs/unipro.h).
pub(super) const PA_PWRMODEUSERDATA0: u32 = 0x15B0;
pub(super) const PA_PWRMODEUSERDATA1: u32 = 0x15B1;
pub(super) const PA_PWRMODEUSERDATA2: u32 = 0x15B2;
pub(super) const PA_PWRMODEUSERDATA3: u32 = 0x15B3;
pub(super) const PA_PWRMODEUSERDATA4: u32 = 0x15B4;
pub(super) const PA_PWRMODEUSERDATA5: u32 = 0x15B5;

/// DME local (host-side) DL timeout attributes (Linux: include/ufs/unipro.h).
pub(super) const DME_LOCAL_FC0_PROTECTION_TIMEOUT: u32 = 0xD041;
pub(super) const DME_LOCAL_TC0_REPLAY_TIMEOUT: u32 = 0xD042;
pub(super) const DME_LOCAL_AFC0_REQ_TIMEOUT: u32 = 0xD043;

/// PA power-mode encoding (Linux: include/ufs/unipro.h).
pub(super) const UFS_HS_G1: u32 = 1;
pub(super) const UFS_HS_G3: u32 = 3;
/// HS Rate Series A/B (Linux: `enum ufs_hs_gear_rate`).
pub(super) const PA_HS_MODE_A: u32 = 1;
pub(super) const PA_HS_MODE_B: u32 = 2;
/// PA power modes (Linux: `enum ufs_pa_pwr_mode`).
pub(super) const PA_PWR_FAST_MODE: u32 = 1;
pub(super) const PA_PWR_SLOW_MODE: u32 = 2;
/// Maximum number of M-PHY data lanes (Linux: PA_MAXDATALANES).
pub(super) const PA_MAXDATALANES: u32 = 4;

/// Data Link layer attributes (Linux: include/ufs/unipro.h).
pub(super) const DL_AFC0REQTIMEOUTVAL: u32 = 0x2043;
pub(super) const UFS_DL_AFC0REQTIMEOUTVAL_MAX: u32 = 0xFFFF;

/// Interrupt status bits (from Linux ufshci.h).
pub(super) const UTP_TRANSFER_REQ_COMPL: u32 = 0x1;
pub(super) const UIC_ERROR: u32 = 0x4;
pub(super) const UIC_LINK_LOST: u32 = 0x80;
pub(super) const DEVICE_FATAL_ERROR: u32 = 0x800;
pub(super) const UTP_ERROR: u32 = 0x1000;
pub(super) const CONTROLLER_FATAL_ERROR: u32 = 0x10000;
pub(super) const SYSTEM_BUS_FATAL_ERROR: u32 = 0x20000;
pub(super) const CRYPTO_ENGINE_FATAL_ERROR: u32 = 0x40000;
pub(super) const UIC_COMMAND_COMPL: u32 = 1 << 10;

/// Error interrupt bits that force a transfer to abort (Linux UFSHCD_ERROR_MASK).
pub(super) const INT_FATAL_ERRORS: u32 = DEVICE_FATAL_ERROR
    | CONTROLLER_FATAL_ERROR
    | SYSTEM_BUS_FATAL_ERROR
    | CRYPTO_ENGINE_FATAL_ERROR
    | UIC_LINK_LOST
    | UTP_ERROR;
pub(super) const UFSHCD_ERROR_MASK: u32 = UIC_ERROR | INT_FATAL_ERRORS;

/// Controller status bits.
pub(super) const DEVICE_PRESENT: u32 = 1 << 0;
pub(super) const UTP_TRANSFER_REQ_LIST_READY: u32 = 1 << 1;
pub(super) const UTP_TASK_REQ_LIST_READY: u32 = 1 << 2;
pub(super) const UIC_COMMAND_READY: u32 = 1 << 3;
pub(super) const UFSHCD_STATUS_READY: u32 =
    UTP_TRANSFER_REQ_LIST_READY | UTP_TASK_REQ_LIST_READY | UIC_COMMAND_READY;

/// RISC-V barrier helpers (Linux: arch/riscv/include/asm/barrier.h).
///
/// `dma_wmb()` = `wmb()` = `fence ow,ow` (Linux: ufs_spacemit_setup_xfer_req).
/// `dma_rmb()` = `rmb()` = `fence ir,ir` (Linux: __ufshcd_transfer_req_compl K3 path).
#[inline(always)]
pub(super) fn dma_wmb() {
    // SAFETY: The inline asm has no memory operand side effects: `fence ow,ow`
    // acts purely as a compiler barrier plus a RISC-V write-write device
    // fence, and `options(nostack)` guarantees no stack frame is used, so this
    // is safe in any context.
    unsafe { core::arch::asm!("fence ow, ow", options(nostack)) };
}

#[inline(always)]
pub(super) fn dma_rmb() {
    // SAFETY: The inline asm has no memory operand side effects: `fence ir,ir`
    // acts purely as a compiler barrier plus a RISC-V read-read device fence,
    // and `options(nostack)` guarantees no stack frame is used, so this is
    // safe in any context.
    unsafe { core::arch::asm!("fence ir, ir", options(nostack)) };
}

/// Helper to build a UIC MIB selector (attribute in the high word).
#[inline]
pub(super) const fn uic_arg_mib(attr: u32) -> u32 {
    (attr & 0xFFFF) << 16
}

/// Helper to build a UIC MIB selector with a lane selector in the low word.
#[inline]
pub(super) const fn uic_arg_mib_sel(attr: u32, sel: u32) -> u32 {
    ((attr & 0xFFFF) << 16) | (sel & 0xFFFF)
}

/// M-PHY RX lane selector: RX lane `lane` maps to MIB selector
/// `PA_MAXDATALANES + lane` (Linux: `UIC_ARG_MPHY_RX_GEN_SEL_INDEX(lane) =
/// PA_MAXDATALANES + lane`). TX lane `lane` keeps selector `lane`.
#[inline]
pub(super) const fn rx_lane_sel(lane: u32) -> u32 {
    PA_MAXDATALANES + lane
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mib_selector_places_attribute_in_high_word() {
        assert_eq!(uic_arg_mib(0x1552), 0x1552_0000);
        assert_eq!(uic_arg_mib(0x2043), 0x2043_0000);
    }

    #[test]
    fn mib_selector_combines_lane_selector_in_low_word() {
        assert_eq!(uic_arg_mib_sel(0x008D, 1), 0x008D_0001);
        assert_eq!(uic_arg_mib_sel(0x155B, 0), 0x155B_0000);
    }

    #[test]
    fn rx_lane_selector_is_pa_maxdatalanes_plus_lane() {
        // Linux: UIC_ARG_MPHY_RX_GEN_SEL_INDEX(lane) = PA_MAXDATALANES + lane.
        assert_eq!(rx_lane_sel(0), 4);
        assert_eq!(rx_lane_sel(1), 5);
        assert_eq!(uic_arg_mib_sel(0x008D, rx_lane_sel(1)), 0x008D_0005);
    }
}
