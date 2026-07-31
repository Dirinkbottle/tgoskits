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
pub(super) const UFS_HCLKDIV_REG: usize = 0xFC;

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
pub(super) const UIC_CMD_DME_SET: u32 = 0x02;
pub(super) const UIC_CMD_DME_LINK_STARTUP: u32 = 0x16;

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
}
