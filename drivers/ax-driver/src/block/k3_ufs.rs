//! SpacemiT K3 UFS host controller driver - Phase 3: SCSI commands

use alloc::{format, vec::Vec};
use core::ptr::NonNull;

use dma_api::{ContiguousArray, DeviceDma, DmaDirection, DmaOp};
use log::{info, warn};
use rdrive::{probe::OnProbeError, register::*};

use crate::mmio::iomap;

/// K3 UFS vendor registers
const UFS_SYS1CLK_1US: usize = 0xC0;
const UFS_TX_SYMBOL_CLK_NS_US: usize = 0xC4;
const UFS_PA_LINK_STARTUP_TIMER: usize = 0xD8;
const UFS_HCLKDIV_REG: usize = 0xFC;

/// MPHY control
const UFS_PHY_MNG_BASE: usize = 0x1B00;
const UFS_MPHY_PU_CTRL: usize = 0x4;
const UFS_MPHY_BKDR_CTRL: usize = 0x8;
const UFS_DEVICE_IO_CTRL: usize = 0xC;

/// ATOP (Analog Top) registers
const UFS_ATOP_BASE: usize = 0x1C00;

const MPHY_PU_ALL: u32 = 0x87f;
const MPHY_PU_WITH_HB8_RESET: u32 = 0xb7f;
const MPHY_DEVICE_RESET_DEASSERT: u32 = 0x101;
const MPHY_PLL_LOCK_BIT: u32 = 1 << 31;

/// UFSHCI registers
const REG_CONTROLLER_CAPABILITIES: usize = 0x00;
const REG_UFS_VERSION: usize = 0x08;
const REG_INTERRUPT_STATUS: usize = 0x20;
const REG_INTERRUPT_ENABLE: usize = 0x24;
const REG_CONTROLLER_STATUS: usize = 0x30;
const REG_CONTROLLER_ENABLE: usize = 0x34;
const REG_UTP_TRANSFER_REQ_INT_AGG_CONTROL: usize = 0x4C;
const REG_UTP_TRANSFER_REQ_LIST_BASE_L: usize = 0x50;
const REG_UTP_TRANSFER_REQ_LIST_BASE_H: usize = 0x54;
const REG_UTP_TRANSFER_REQ_DOOR_BELL: usize = 0x58;
const REG_UTP_TRANSFER_REQ_LIST_RUN_STOP: usize = 0x60;
const REG_UTP_TASK_REQ_LIST_BASE_L: usize = 0x70;
const REG_UTP_TASK_REQ_LIST_BASE_H: usize = 0x74;
const REG_UTP_TASK_REQ_LIST_RUN_STOP: usize = 0x80;
const REG_UIC_COMMAND: usize = 0x90;
const REG_UIC_COMMAND_ARG1: usize = 0x94;
const REG_UIC_COMMAND_ARG2: usize = 0x98;
const REG_UIC_COMMAND_ARG3: usize = 0x9C;

/// UIC Commands
const UIC_CMD_DME_SET: u32 = 0x02;
const UIC_CMD_DME_LINK_STARTUP: u32 = 0x16;

/// UIC attributes (from Linux ufs-spacemit.c)
const PA_TXHSG1SYNCLENGTH: u32 = 0x1552;
const PA_TXHSG1PREPARELENGTH: u32 = 0x1553;
const PA_TXHSG2SYNCLENGTH: u32 = 0x1554;
const PA_TXHSG2PREPARELENGTH: u32 = 0x1555;
const PA_TXHSG3SYNCLENGTH: u32 = 0x1556;
const PA_TXHSG3PREPARELENGTH: u32 = 0x1557;
const PA_TXMK2EXTENSION: u32 = 0x155A;
const PA_PEERSCRAMBLING: u32 = 0x155B;
const PA_TXSKIP: u32 = 0x155C;
const PA_TXSKIPPERIOD: u32 = 0x155D;
const PA_LOCAL_TX_LCC_ENABLE: u32 = 0x155E;
const PA_PEER_TX_LCC_ENABLE: u32 = 0x155F;
const PA_SCRAMBLING: u32 = 0x1585;
const PA_GRANULARITY: u32 = 0x15AA;
const PA_MK2EXTENSIONGUARDBAND: u32 = 0x15AB;
const PA_STALLNOCONFIGTIME: u32 = 0x15A3;
const PA_TACTIVATE: u32 = 0x15A8;
const PA_TXTRAILINGCLOCKS: u32 = 0x1564;

/// RX/TX lane-specific attributes
const RX_LS_PRE_LEN_CAP: u32 = 0x008D;
const RX_LANE_HB8_BKDOOR_ATTR: u32 = 0x00F4;
const RX_PWRM_CLOSURE_LEN_CAP: u32 = 0x008E;
const RX_MIN_STALL_CAP: u32 = 0x0088;
const TX_HIBERN8TIME_CAP: u32 = 0x000F;
const RX_HIBERN8TIME_CAP: u32 = 0x0092;
const ANA_EQ_CTRL_REG_ATTR: u32 = 0x00CD;
const RX_GARBAGE_COUNT_OFFSET: u32 = 0x00F2;

/// Data Link layer attributes (Linux: include/ufs/unipro.h)
const DL_AFC0REQTIMEOUTVAL: u32 = 0x2043;
const UFS_DL_AFC0REQTIMEOUTVAL_MAX: u32 = 0xFFFF;

/// RISC-V barrier helpers (Linux: arch/riscv/include/asm/barrier.h)
/// dma_wmb() = wmb() = fence ow,ow  (Linux: ufs_spacemit_setup_xfer_req)
/// dma_rmb() = rmb() = fence ir,ir  (Linux: __ufshcd_transfer_req_compl K3 path)
#[inline(always)]
fn dma_wmb() {
    unsafe { core::arch::asm!("fence ow, ow", options(nostack)) };
}
#[inline(always)]
fn dma_rmb() {
    unsafe { core::arch::asm!("fence ir, ir", options(nostack)) };
}

/// Helper to build UIC MIB selector
#[inline]
const fn uic_arg_mib(attr: u32) -> u32 {
    ((attr & 0xFFFF) << 16) | 0
}

#[inline]
const fn uic_arg_mib_sel(attr: u32, sel: u32) -> u32 {
    ((attr & 0xFFFF) << 16) | (sel & 0xFFFF)
}

/// Interrupt status bits (from Linux ufshci.h)
const UTP_TRANSFER_REQ_COMPL: u32 = 0x1;
const UTP_TASK_REQ_COMPL: u32 = 0x200;
const UIC_ERROR: u32 = 0x4;
const UIC_LINK_LOST: u32 = 0x80;
const DEVICE_FATAL_ERROR: u32 = 0x800;
const UTP_ERROR: u32 = 0x1000;
const CONTROLLER_FATAL_ERROR: u32 = 0x10000;

/// Decodes the K3 UFS FDT interrupt specifier into a Linux-style global IRQ.
///
/// The interrupt parent decides the specifier width through `#interrupt-cells`.
/// Linux's OF parser first reads that width from the parent before interpreting
/// the cells. K3 boards currently use the common one-cell, two-cell, or GIC
/// three-cell forms:
/// - one cell: already a global IRQ number;
/// - two cells: `<irq, flags>`;
/// - three cells: GIC `<kind, irq, flags>`, where SPI starts at global IRQ 32
///   and PPI starts at global IRQ 16.
///
/// 参考: /home/inkbottle/桌面/linux-5.4.29/drivers/of/irq.c:108-127
/// 参考: /home/inkbottle/桌面/linux-5.4.29/Documentation/devicetree/booting-without-of.txt:1300-1313
fn decode_fdt_irq(interrupts: &[rdrive::probe::fdt::InterruptRef]) -> Option<usize> {
    let interrupt = interrupts.first()?;
    match interrupt.specifier.as_slice() {
        [irq] => Some(*irq as usize),
        [irq, _flags] => Some(*irq as usize),
        [kind, irq, _flags] => match *kind {
            0 => Some(*irq as usize + 32),
            1 => Some(*irq as usize + 16),
            _ => Some(*irq as usize),
        },
        _ => None,
    }
}
const SYSTEM_BUS_FATAL_ERROR: u32 = 0x20000;
const CRYPTO_ENGINE_FATAL_ERROR: u32 = 0x40000;
const UIC_COMMAND_COMPL: u32 = 1 << 10;

const INT_FATAL_ERRORS: u32 = DEVICE_FATAL_ERROR
    | CONTROLLER_FATAL_ERROR
    | SYSTEM_BUS_FATAL_ERROR
    | CRYPTO_ENGINE_FATAL_ERROR
    | UIC_LINK_LOST
    | UTP_ERROR;
const UFSHCD_ERROR_MASK: u32 = UIC_ERROR | INT_FATAL_ERRORS;
const UFSHCD_ENABLE_INTRS: u32 = UTP_TRANSFER_REQ_COMPL | UTP_TASK_REQ_COMPL | UFSHCD_ERROR_MASK;

/// Controller status bits
const DEVICE_PRESENT: u32 = 1 << 0;
const UTP_TRANSFER_REQ_LIST_READY: u32 = 1 << 1;
const UTP_TASK_REQ_LIST_READY: u32 = 1 << 2;
const UIC_COMMAND_READY: u32 = 1 << 3;
const UFSHCD_STATUS_READY: u32 =
    UTP_TRANSFER_REQ_LIST_READY | UTP_TASK_REQ_LIST_READY | UIC_COMMAND_READY;

/// SCSI commands
const SCSI_TEST_UNIT_READY: u8 = 0x00;
const SCSI_REQUEST_SENSE: u8 = 0x03;
const SCSI_INQUIRY: u8 = 0x12;
const SCSI_READ_CAPACITY_10: u8 = 0x25;
const SCSI_READ_10: u8 = 0x28;
const SCSI_WRITE_10: u8 = 0x2A;
const SCSI_SERVICE_ACTION_IN_16: u8 = 0x9E;
const SCSI_REPORT_LUNS: u8 = 0xA0;
const SAI_READ_CAPACITY_16: u8 = 0x10;

/// SCSI status codes (Linux: include/scsi/scsi_proto.h)
const SAM_STAT_GOOD: u8 = 0x00;
const SAM_STAT_CHECK_CONDITION: u8 = 0x02;

/// SCSI sense keys (Linux: include/scsi/scsi_proto.h)
const SCSI_SENSE_NOT_READY: u8 = 0x02;
const SCSI_SENSE_ILLEGAL_REQUEST: u8 = 0x05;
const SCSI_SENSE_UNIT_ATTENTION: u8 = 0x06;

/// SCSI well-known LUN encoding (Linux: include/scsi/scsi.h)
const SCSI_W_LUN_BASE: u64 = 0xC100;
const SCSI_REPORT_LUNS_ALLOC_LEN: usize = 4096;

/// UPIU (UFS Protocol Information Unit) types (Linux: include/ufs/ufs.h)
const UPIU_TRANSACTION_NOP_OUT: u8 = 0x00;
const UPIU_TRANSACTION_COMMAND: u8 = 0x01;
const UPIU_TRANSACTION_NOP_IN: u8 = 0x20;
const UPIU_TRANSACTION_RESPONSE: u8 = 0x21;
const UPIU_TRANSACTION_QUERY_REQ: u8 = 0x16;
const UPIU_TRANSACTION_QUERY_RSP: u8 = 0x36;

const UFS_UPIU_MAX_UNIT_NUM_ID: u8 = 0x7F;
const UFS_UPIU_WLUN_ID: u8 = 1 << 7;

/// UPIU flags
const UPIU_CMD_FLAGS_NONE: u8 = 0x00;
const UPIU_CMD_FLAGS_READ: u8 = 0x40;
const UPIU_CMD_FLAGS_WRITE: u8 = 0x20;

/// QUERY opcodes (Linux: include/ufs/ufs.h enum query_opcode)
const UPIU_QUERY_OPCODE_READ_FLAG: u8 = 0x5;
const UPIU_QUERY_OPCODE_SET_FLAG: u8 = 0x6;

/// QUERY function codes (Linux: include/ufs/ufs.h)
const UPIU_QUERY_FUNC_STANDARD_READ_REQUEST: u8 = 0x01;
const UPIU_QUERY_FUNC_STANDARD_WRITE_REQUEST: u8 = 0x81;

/// Flag idn (Linux: include/ufs/ufs.h enum flag_idn)
const QUERY_FLAG_IDN_FDEVICEINIT: u8 = 0x01;

/// Retry and timeout constants (Linux: drivers/ufs/core/ufshcd.c)
const NOP_OUT_RETRIES: usize = 10;
const QUERY_REQ_RETRIES: usize = 3;
const FDEVICEINIT_COMPL_TIMEOUT_MS: u64 = 10000;

/// Transfer Request Descriptor command type.
///
/// Linux 6.18 uses UTP_CMD_TYPE_UFS_STORAGE for all transfer request
/// descriptors, including NOP OUT and QUERY device commands. Do not use the
/// older UTP_DEVICE_MANAGEMENT_FUNCTION full-DW value here; that produces
/// DW0=0x21000000 and K3 completes the slot without writing OCS/response.
const UTP_CMD_TYPE_UFS_STORAGE: u8 = 0x01;
const UTP_DATA_DIR_TO_DEVICE: u8 = 0x01;
const UTP_DATA_DIR_TO_HOST: u8 = 0x02;
const OCS_INVALID_COMMAND_STATUS: u8 = 0x0F;
const UFSHCI_NUM_SLOTS: usize = 32;
const UCD_COMMAND_UPIU_SIZE: usize = 512;
const UCD_RESPONSE_UPIU_SIZE: usize = 512;
const UCD_PRDT_OFFSET: usize = UCD_COMMAND_UPIU_SIZE + UCD_RESPONSE_UPIU_SIZE;
const UCD_PRDT_ENTRIES: usize = 128; // Linux SG_ALL default.
const UCD_SLOT_SIZE: usize = UCD_PRDT_OFFSET + UCD_PRDT_ENTRIES * core::mem::size_of::<Prdt>();

/// UTP Transfer Request Descriptor
#[repr(C)]
struct Utrd {
    dw0: u32,
    dw1: u32,
    dw2: u32,
    dw3: u32,
    ucdba: u32,
    ucdbau: u32,
    rul: u16,
    ruo: u16,
    prdtl: u16,
    prdto: u16,
}

/// UPIU header
#[repr(C)]
struct UpiuHeader {
    trans_type: u8,
    flags: u8,
    lun: u8,
    task_tag: u8,
    cmd_set_type: u8,
    reserved: [u8; 3],
    total_ehs_len: u8,
    reserved2: u8,
    data_segment_len: u16,
}

/// Physical Region Description Table Entry
#[repr(C, align(4))]
struct Prdt {
    dba: u32,
    dbau: u32,
    reserved: u32,
    dbc: u32,
}

/// Command UPIU
#[repr(C, align(4))]
struct CommandUpiu {
    header: UpiuHeader,
    exp_data_len: u32,
    cdb: [u8; 16],
}

/// Response UPIU
#[repr(C, align(4))]
struct ResponseUpiu {
    header: UpiuHeader,
    residual_len: u32,
    reserved: [u32; 4],
    sense_data_len: u16,
    sense_data: [u8; 18],
}

/// Query UPIU structure (Linux: include/uapi/scsi/scsi_bsg_ufs.h struct utp_upiu_query)
#[repr(C)]
struct QueryUpiu {
    opcode: u8,
    idn: u8,
    index: u8,
    selector: u8,
    reserved_osf: u16, // big-endian in protocol
    length: u16,       // big-endian in protocol
    value: u32,        // big-endian in protocol
    reserved: [u32; 2],
}

/// UFS Command Descriptor (UCD) - matches Linux ALIGNED_UPIU_SIZE = 512
#[repr(C, align(128))]
struct Ucd {
    command_upiu: [u8; 512],
    response_upiu: [u8; 512],
    prdt: [Prdt; UCD_PRDT_ENTRIES],
}

crate::model_register!(
    name: "K3 UFS",
    level: ProbeLevel::PostKernel,
    priority: ProbePriority::DEFAULT,
    probe_kinds: &[
        ProbeKind::Fdt {
            compatibles: &["spacemit,k3-ufshcd"],
            on_probe: probe
        }
    ],
);

struct K3UfsHost {
    mmio_base: NonNull<u8>,
    clock_freq: u32,
    nutrs: usize,
    active_lun: u8,
    num_blocks: u64,
    block_size: usize,
    dma: DeviceDma,
    utrd_list: Option<ContiguousArray<u8>>,
    utmrd_list: Option<ContiguousArray<u8>>,
    ucd_buf: Option<ContiguousArray<u8>>,
}

// SAFETY: MMIO register access is thread-safe for this hardware
unsafe impl Send for K3UfsHost {}

impl K3UfsHost {
    unsafe fn read32(&self, offset: usize) -> u32 {
        let v =
            unsafe { core::ptr::read_volatile(self.mmio_base.as_ptr().add(offset) as *const u32) };
        // Linux: __io_ar() = fence i,ir (arch/riscv/include/asm/mmio.h)
        unsafe { core::arch::asm!("fence i, ir", options(nostack)) };
        v
    }

    unsafe fn write32(&self, offset: usize, value: u32) {
        unsafe {
            // Linux: __io_bw() = fence w,o (arch/riscv/include/asm/mmio.h)
            core::arch::asm!("fence w, o", options(nostack));
            core::ptr::write_volatile(self.mmio_base.as_ptr().add(offset) as *mut u32, value);
        }
    }

    unsafe fn read_utrd_ocs(utrd: *const Utrd) -> u32 {
        unsafe { core::ptr::read_volatile(core::ptr::addr_of!((*utrd).dw2)) & 0x0F }
    }

    /// Linux: ufshcd_scsi_to_upiu_lun() in drivers/ufs/core/ufshcd-priv.h.
    #[inline]
    fn scsi_to_upiu_lun(scsi_lun: u64) -> u8 {
        if (scsi_lun & 0xFF00) == SCSI_W_LUN_BASE {
            ((scsi_lun as u8) & UFS_UPIU_MAX_UNIT_NUM_ID) | UFS_UPIU_WLUN_ID
        } else {
            (scsi_lun as u8) & UFS_UPIU_MAX_UNIT_NUM_ID
        }
    }

    /// Linux: scsilun_to_int() in drivers/scsi/scsi_common.c.
    fn scsilun_to_int(lun: &[u8]) -> u64 {
        let mut value = 0u64;
        let mut i = 0usize;
        while i + 1 < 8 && i + 1 < lun.len() {
            value |= (lun[i] as u64) << ((i + 1) * 8);
            value |= (lun[i + 1] as u64) << (i * 8);
            i += 2;
        }
        value
    }

    fn mphy_init(&self) -> Result<(), &'static str> {
        info!("[k3-ufs] Initializing MPHY...");

        unsafe {
            // Reset all MPHY logical
            self.write32(UFS_PHY_MNG_BASE, 0x003);
            axklib::time::busy_wait(core::time::Duration::from_millis(1));

            // Power up all
            self.write32(UFS_PHY_MNG_BASE + UFS_MPHY_PU_CTRL, MPHY_PU_ALL);
            axklib::time::busy_wait(core::time::Duration::from_millis(1));

            // Assert ana_rx_hb8_reset
            self.write32(UFS_PHY_MNG_BASE + UFS_MPHY_PU_CTRL, MPHY_PU_WITH_HB8_RESET);
            axklib::time::busy_wait(core::time::Duration::from_millis(1));

            // Deassert ana_rx_hb8_reset
            self.write32(UFS_PHY_MNG_BASE + UFS_MPHY_PU_CTRL, MPHY_PU_ALL);
            axklib::time::busy_wait(core::time::Duration::from_millis(1));

            // Deassert UFS device reset & enable reference clock output
            self.write32(
                UFS_PHY_MNG_BASE + UFS_DEVICE_IO_CTRL,
                MPHY_DEVICE_RESET_DEASSERT,
            );
            axklib::time::busy_wait(core::time::Duration::from_millis(1));

            // Wait for PLL lock
            for _ in 0..10000 {
                let pu_ctrl = self.read32(UFS_PHY_MNG_BASE + UFS_MPHY_PU_CTRL);
                if pu_ctrl & MPHY_PLL_LOCK_BIT != 0 {
                    info!("[k3-ufs] MPHY PLL locked: 0x{:08x}", pu_ctrl);

                    // Configure ATOP registers via backdoor
                    self.write32(UFS_PHY_MNG_BASE + UFS_MPHY_BKDR_CTRL, 0x1);
                    axklib::time::busy_wait(core::time::Duration::from_micros(20));

                    self.write32(UFS_ATOP_BASE + (0xC1 << 2), 0x00);
                    self.write32(UFS_ATOP_BASE + (0xC2 << 2), 0x00);
                    axklib::time::busy_wait(core::time::Duration::from_micros(20));

                    self.write32(UFS_PHY_MNG_BASE + UFS_MPHY_BKDR_CTRL, 0x0);
                    axklib::time::busy_wait(core::time::Duration::from_micros(20));

                    return Ok(());
                }
                axklib::time::busy_wait(core::time::Duration::from_micros(1));
            }

            Err("MPHY PLL lock timeout")
        }
    }

    fn host_init(&self) -> Result<(), &'static str> {
        info!("[k3-ufs] Initializing host controller...");

        unsafe {
            let cap = self.read32(REG_CONTROLLER_CAPABILITIES);
            let version = self.read32(REG_UFS_VERSION);
            info!("[k3-ufs] CAP: 0x{:08x}, VER: 0x{:08x}", cap, version);

            self.write32(REG_CONTROLLER_ENABLE, 0);
            axklib::time::busy_wait(core::time::Duration::from_millis(1));

            let sys1clk = self.clock_freq / 1_000_000;
            self.write32(UFS_SYS1CLK_1US, sys1clk);

            let tx_clk = 1000 / (self.clock_freq / 1_000_000);
            self.write32(UFS_TX_SYMBOL_CLK_NS_US, tx_clk << 10);

            self.write32(UFS_PA_LINK_STARTUP_TIMER, 0xFFFFFFFF);

            self.write32(REG_CONTROLLER_ENABLE, 1);
            axklib::time::busy_wait(core::time::Duration::from_millis(1));

            let hce = self.read32(REG_CONTROLLER_ENABLE);
            if hce & 1 == 0 {
                return Err("Controller enable failed");
            }

            info!("[k3-ufs] Host controller enabled");
        }

        Ok(())
    }

    fn uic_cmd(&self, cmd: u32, arg1: u32, arg2: u32, arg3: u32) -> Result<u32, &'static str> {
        unsafe {
            self.write32(REG_INTERRUPT_STATUS, UIC_COMMAND_COMPL);
            self.write32(REG_UIC_COMMAND_ARG1, arg1);
            self.write32(REG_UIC_COMMAND_ARG2, arg2);
            self.write32(REG_UIC_COMMAND_ARG3, arg3);
            self.write32(REG_UIC_COMMAND, cmd);

            for _ in 0..5000 {
                let is = self.read32(REG_INTERRUPT_STATUS);
                if is & UIC_COMMAND_COMPL != 0 {
                    self.write32(REG_INTERRUPT_STATUS, UIC_COMMAND_COMPL);
                    return Ok(self.read32(REG_UIC_COMMAND_ARG2));
                }
                axklib::time::busy_wait(core::time::Duration::from_micros(100));
            }

            Err("UIC command timeout")
        }
    }

    fn dme_set(&self, attr: u32, value: u32) -> Result<(), &'static str> {
        let result = self.uic_cmd(UIC_CMD_DME_SET, attr, 0, value)?;
        if result != 0 {
            warn!(
                "[k3-ufs] DME_SET(0x{:04x})={} failed: 0x{:08x}",
                attr, value, result
            );
            return Err("DME_SET failed");
        }
        Ok(())
    }

    /// UNIPRO v1.6 initialization - critical for link startup
    fn unipro_init(&self) -> Result<(), &'static str> {
        info!("[k3-ufs] Initializing UNIPRO v1.6...");

        // PA layer attributes
        self.dme_set(uic_arg_mib(PA_TXHSG1SYNCLENGTH), 0x4f)?;
        self.dme_set(uic_arg_mib(PA_TXHSG1PREPARELENGTH), 0x0f)?;
        self.dme_set(uic_arg_mib(PA_TXHSG2SYNCLENGTH), 0x4f)?;
        self.dme_set(uic_arg_mib(PA_TXHSG2PREPARELENGTH), 0x0f)?;
        self.dme_set(uic_arg_mib(PA_TXHSG3SYNCLENGTH), 0x4f)?;
        self.dme_set(uic_arg_mib(PA_TXHSG3PREPARELENGTH), 0x0f)?;
        self.dme_set(uic_arg_mib(PA_TXMK2EXTENSION), 0x0)?;
        self.dme_set(uic_arg_mib(PA_PEERSCRAMBLING), 0x1)?;
        self.dme_set(uic_arg_mib(PA_TXSKIP), 0x1)?;
        self.dme_set(uic_arg_mib(PA_TXSKIPPERIOD), 250)?;
        self.dme_set(uic_arg_mib(PA_LOCAL_TX_LCC_ENABLE), 0x0)?;
        self.dme_set(uic_arg_mib(PA_PEER_TX_LCC_ENABLE), 0x0)?;
        self.dme_set(uic_arg_mib(PA_SCRAMBLING), 0x1)?;
        self.dme_set(uic_arg_mib(PA_GRANULARITY), 0x1)?;
        self.dme_set(uic_arg_mib(PA_MK2EXTENSIONGUARDBAND), 0x0)?;
        self.dme_set(uic_arg_mib(PA_STALLNOCONFIGTIME), 15)?;
        self.dme_set(uic_arg_mib(PA_TACTIVATE), 0x64)?;
        self.dme_set(uic_arg_mib(PA_TXTRAILINGCLOCKS), 0x64)?;

        // RX lane 0 & 1 attributes
        self.dme_set(uic_arg_mib_sel(RX_LS_PRE_LEN_CAP, 0), 0x0b)?;
        self.dme_set(uic_arg_mib_sel(RX_LS_PRE_LEN_CAP, 1), 0x0b)?;
        self.dme_set(uic_arg_mib_sel(RX_LANE_HB8_BKDOOR_ATTR, 0), 0x9f)?;
        self.dme_set(uic_arg_mib_sel(RX_LANE_HB8_BKDOOR_ATTR, 1), 0x9f)?;
        self.dme_set(uic_arg_mib_sel(RX_PWRM_CLOSURE_LEN_CAP, 0), 15)?;
        self.dme_set(uic_arg_mib_sel(RX_PWRM_CLOSURE_LEN_CAP, 1), 15)?;
        self.dme_set(uic_arg_mib_sel(RX_MIN_STALL_CAP, 0), 0xff)?;
        self.dme_set(uic_arg_mib_sel(RX_MIN_STALL_CAP, 1), 0xff)?;

        // TX lane 0 & 1 hibernate time
        self.dme_set(uic_arg_mib_sel(TX_HIBERN8TIME_CAP, 0), 0x64)?;
        self.dme_set(uic_arg_mib_sel(TX_HIBERN8TIME_CAP, 1), 0x64)?;

        // RX lane 0 & 1 hibernate time
        self.dme_set(uic_arg_mib_sel(RX_HIBERN8TIME_CAP, 0), 0x64)?;
        self.dme_set(uic_arg_mib_sel(RX_HIBERN8TIME_CAP, 1), 0x64)?;

        // TX EQ and RX garbage count
        self.dme_set(uic_arg_mib_sel(ANA_EQ_CTRL_REG_ATTR, 0), 0x5)?;
        self.dme_set(uic_arg_mib_sel(RX_GARBAGE_COUNT_OFFSET, 0), 0x9f)?;
        self.dme_set(uic_arg_mib_sel(RX_GARBAGE_COUNT_OFFSET, 1), 0x9f)?;

        // HCLKDIV register (via DME, not direct register write)
        self.dme_set(uic_arg_mib(UFS_HCLKDIV_REG as u32), 0xfc)?;

        info!("[k3-ufs] UNIPRO v1.6 init completed");
        Ok(())
    }

    fn link_startup(&self) -> Result<(), &'static str> {
        info!("[k3-ufs] Starting UFS link...");

        let result = self.uic_cmd(UIC_CMD_DME_LINK_STARTUP, 0, 0, 0)?;

        if result != 0 {
            warn!(
                "[k3-ufs] Link startup command failed: result=0x{:08x}",
                result
            );
            return Err("Link startup failed");
        }

        unsafe {
            for _ in 0..1000 {
                let status = self.read32(REG_CONTROLLER_STATUS);
                if status & DEVICE_PRESENT != 0 {
                    info!(
                        "[k3-ufs] Link active, device present. Status=0x{:08x}",
                        status
                    );
                    return Ok(());
                }
                axklib::time::busy_wait(core::time::Duration::from_millis(1));
            }
        }

        Err("Device not present after link startup")
    }

    /// Link startup post processing (Linux: ufs_spacemit_link_startup_post_change)
    fn link_startup_post(&self) -> Result<(), &'static str> {
        // Set DL_AFC0REQTIMEOUTVAL_MAX (required by Linux driver)
        self.dme_set(
            uic_arg_mib(DL_AFC0REQTIMEOUTVAL),
            UFS_DL_AFC0REQTIMEOUTVAL_MAX,
        )?;

        // Clear UECPA due to LINERESET during LINK_STARTUP (Linux: ufshcd.c after POST_CHANGE)
        unsafe {
            let _ = self.read32(0x38); // REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER
        }

        info!("[k3-ufs] Link startup post processing completed");
        Ok(())
    }

    fn dump_regs(&self) {
        unsafe {
            info!("[k3-ufs] Register dump:");
            info!("  CAP:     0x{:08x}", self.read32(0x00));
            info!("  VER:     0x{:08x}", self.read32(0x08));
            info!("  HCS:     0x{:08x}", self.read32(0x30));
            info!("  HCE:     0x{:08x}", self.read32(0x34));
        }
    }

    fn setup_transfer_lists(&mut self) -> Result<(), &'static str> {
        // Linux: nutrs = (CAP & 0x1f) + 1 (ufshcd_get_transfer_req_mgmt_max_slots)
        let cap = unsafe { self.read32(REG_CONTROLLER_CAPABILITIES) };
        self.nutrs = ((cap & 0x1f) + 1) as usize;
        let reserved_slot = self.nutrs - 1;
        info!(
            "[k3-ufs] CAP=0x{:08x}, nutrs={}, reserved_slot={}",
            cap, self.nutrs, reserved_slot
        );

        // Linux ufshcd_memory_alloc(): UTRDL/UTMRDL require 1 KiB alignment,
        // while UCD requires 128-byte alignment.
        let utrd_list = self
            .dma
            .contiguous_array_zero_with_align::<u8>(
                UFSHCI_NUM_SLOTS * 32,
                1024,
                DmaDirection::Bidirectional,
            )
            .map_err(|_| "Failed to allocate UTRD list")?;
        let utmrd_list = self
            .dma
            .contiguous_array_zero_with_align::<u8>(8 * 80, 1024, DmaDirection::Bidirectional)
            .map_err(|_| "Failed to allocate UTMRD list")?;

        // Allocate one UCD per transfer slot. Linux lays out UCD as an array
        // indexed by task_tag.
        let ucd_buf = self
            .dma
            .contiguous_array_zero_with_align::<u8>(
                UFSHCI_NUM_SLOTS * UCD_SLOT_SIZE,
                128,
                DmaDirection::Bidirectional,
            )
            .map_err(|_| "Failed to allocate UCD buffer")?;

        // Linux: ufshcd_make_hba_operational sequence
        let utrd_phys = utrd_list.dma_addr().as_u64();
        let utmrd_phys = utmrd_list.dma_addr().as_u64();
        let ucd_phys = ucd_buf.dma_addr().as_u64();
        info!(
            "[k3-ufs] DMA layout: utrd_base=0x{:x}, utmrd_base=0x{:x}, ucd_base=0x{:x}, \
             ucd_slot_size={}",
            utrd_phys, utmrd_phys, ucd_phys, UCD_SLOT_SIZE
        );
        unsafe {
            // 1. Enable required interrupts (Linux: ufshcd_enable_intr)
            let old_ie = self.read32(REG_INTERRUPT_ENABLE);
            self.write32(REG_INTERRUPT_ENABLE, old_ie | UFSHCD_ENABLE_INTRS);

            // 2. Disable interrupt aggregation (Linux: ufshcd_disable_intr_aggr)
            self.write32(REG_UTP_TRANSFER_REQ_INT_AGG_CONTROL, 0);

            // 3. Configure UTRL and UTMRL base address registers
            self.write32(
                REG_UTP_TRANSFER_REQ_LIST_BASE_L,
                (utrd_phys & 0xFFFFFFFF) as u32,
            );
            self.write32(REG_UTP_TRANSFER_REQ_LIST_BASE_H, (utrd_phys >> 32) as u32);
            self.write32(
                REG_UTP_TASK_REQ_LIST_BASE_L,
                (utmrd_phys & 0xFFFFFFFF) as u32,
            );
            self.write32(REG_UTP_TASK_REQ_LIST_BASE_H, (utmrd_phys >> 32) as u32);

            // Flush posted base-address writes, as Linux does around these registers.
            let _ = self.read32(REG_UTP_TRANSFER_REQ_LIST_BASE_H);
            let _ = self.read32(REG_UTP_TASK_REQ_LIST_BASE_H);

            // UCRDY, UTMRLDY and UTRLRDY must be set before run-stop.
            let hcs = self.read32(REG_CONTROLLER_STATUS);
            if (hcs & UFSHCD_STATUS_READY) != UFSHCD_STATUS_READY {
                warn!("[k3-ufs] Host lists not ready: HCS=0x{:08x}", hcs);
                return Err("Host lists not ready");
            }

            // 4. Enable run-stop registers (Linux: ufshcd_enable_run_stop_reg)
            self.write32(REG_UTP_TASK_REQ_LIST_RUN_STOP, 1);
            self.write32(REG_UTP_TRANSFER_REQ_LIST_RUN_STOP, 1);
        }

        self.utrd_list = Some(utrd_list);
        self.utmrd_list = Some(utmrd_list);
        self.ucd_buf = Some(ucd_buf);

        info!("[k3-ufs] Transfer lists configured (Linux sequence)");
        Ok(())
    }

    /// Dump UTRD for debugging (Linux: ufshcd_print_tr)
    fn dump_transfer_state(&self, slot: usize, msg: &str) {
        let utrd_list = self.utrd_list.as_ref().unwrap();
        let ucd_buf = self.ucd_buf.as_ref().unwrap();
        let utrd_ptr = unsafe { (utrd_list.as_ptr().as_ptr() as *const Utrd).add(slot) };
        let ucd_ptr = unsafe { ucd_buf.as_ptr().as_ptr().add(slot * UCD_SLOT_SIZE) } as *const Ucd;

        unsafe {
            let utrd_bytes = core::slice::from_raw_parts(utrd_ptr as *const u8, 32);
            let req_upiu = core::slice::from_raw_parts((*ucd_ptr).command_upiu.as_ptr(), 32);
            let rsp_upiu = core::slice::from_raw_parts((*ucd_ptr).response_upiu.as_ptr(), 64);

            let db = self.read32(REG_UTP_TRANSFER_REQ_DOOR_BELL);
            let is = self.read32(REG_INTERRUPT_STATUS);
            let ie = self.read32(REG_INTERRUPT_ENABLE);
            let hcs = self.read32(REG_CONTROLLER_STATUS);
            let run_stop = self.read32(REG_UTP_TRANSFER_REQ_LIST_RUN_STOP);
            let uecpa = self.read32(0x38);
            let uecdl = self.read32(0x3C);
            let uecn = self.read32(0x40);
            let uect = self.read32(0x44);
            let uecdme = self.read32(0x48);

            warn!("[k3-ufs] {} slot={}", msg, slot);
            warn!("  UTRD[32]: {:02x?}", utrd_bytes);
            warn!("  REQ[32]:  {:02x?}", req_upiu);
            warn!("  RSP[64]:  {:02x?}", rsp_upiu);
            warn!(
                "  DB=0x{:x}, IS=0x{:x}, IE=0x{:x}, HCS=0x{:x}",
                db, is, ie, hcs
            );
            warn!("  RUN_STOP=0x{:x}", run_stop);
            warn!(
                "  UECPA=0x{:x}, UECDL=0x{:x}, UECN=0x{:x}, UECT=0x{:x}, UECDME=0x{:x}",
                uecpa, uecdl, uecn, uect, uecdme
            );
        }
    }

    /// Build NOP OUT UPIU (Linux: ufshcd_prepare_utp_nop_upiu, ufshcd.c:2851)
    fn build_nop_upiu(&self, upiu: &mut [u8; 512], task_tag: u8) {
        upiu.fill(0);
        upiu[0] = UPIU_TRANSACTION_NOP_OUT;
        upiu[3] = task_tag;
    }

    /// Build QUERY UPIU (Linux: ufshcd_prepare_utp_query_req_upiu, ufshcd.c:2820)
    /// Note: request value is always 0, only response contains the actual value
    fn build_query_upiu(
        &self,
        upiu: &mut [u8; 512],
        task_tag: u8,
        query_func: u8,
        opcode: u8,
        idn: u8,
    ) {
        upiu.fill(0);
        // Header bytes 0-11
        upiu[0] = UPIU_TRANSACTION_QUERY_REQ;
        upiu[1] = 0; // flags
        upiu[2] = 0; // lun
        upiu[3] = task_tag;
        upiu[4] = 0; // cmd_set_type
        upiu[5] = query_func; // query_function (Linux line 2833)
        // bytes 6-11 are 0

        // Query structure bytes 12-31 (Linux: memcpy(&ucd_req_ptr->qr, &query->request.upiu_req, QUERY_OSF_SIZE))
        upiu[12] = opcode;
        upiu[13] = idn;
        upiu[14] = 0; // index
        upiu[15] = 0; // selector
        // bytes 16-23: reserved_osf, length, value all 0 in request
        // bytes 24-31: reserved = 0
    }

    /// Build SCSI command UPIU (Linux: ufshcd_prepare_utp_scsi_cmd_upiu, ufshcd.c:2780)
    fn build_scsi_upiu(
        &self,
        upiu: &mut [u8; 512],
        task_tag: u8,
        lun: u8,
        flags: u8,
        cdb: &[u8],
        exp_len: u32,
    ) {
        upiu.fill(0);
        // Header bytes 0-11
        upiu[0] = UPIU_TRANSACTION_COMMAND;
        upiu[1] = flags;
        upiu[2] = lun;
        upiu[3] = task_tag;
        upiu[4] = 0; // cmd_set_type for SCSI
        // bytes 5-11 are 0

        // SCSI command structure bytes 12-31
        upiu[12..16].copy_from_slice(&exp_len.to_be_bytes()); // exp_data_transfer_len
        let cdb_len = cdb.len().min(16);
        upiu[16..16 + cdb_len].copy_from_slice(&cdb[..cdb_len]);
    }

    /// Unified UPIU submission using reserved slot (Linux: ufshcd_exec_dev_cmd)
    fn submit_upiu(
        &mut self,
        upiu: &[u8; 512],
        mut data_buf: Option<&mut ContiguousArray<u8>>,
        data_len: u32,
        data_dir: u8,
    ) -> Result<[u8; 512], &'static str> {
        // Linux: reserved_slot = nutrs - 1 (ufshcd_exec_dev_cmd)
        let slot = if self.nutrs > 0 { self.nutrs - 1 } else { 0 };
        let slot_mask = 1u32 << slot;

        {
            let utrd_list = self.utrd_list.as_mut().ok_or("UTRD not initialized")?;
            let ucd_buf = self.ucd_buf.as_mut().ok_or("UCD not initialized")?;
            let ucd_ptr =
                unsafe { ucd_buf.as_ptr().as_ptr().add(slot * UCD_SLOT_SIZE) } as *mut Ucd;
            let ucd = unsafe { &mut *ucd_ptr };

            ucd.command_upiu.copy_from_slice(upiu);
            ucd.command_upiu[3] = slot as u8; // task_tag must equal slot (Linux: lrbp->task_tag)
            ucd.response_upiu.fill(0);
            for prdt in ucd.prdt.iter_mut() {
                prdt.dba = 0;
                prdt.dbau = 0;
                prdt.reserved = 0;
                prdt.dbc = 0;
            }

            let utrd_ptr = unsafe { (utrd_list.as_ptr().as_ptr() as *mut Utrd).add(slot) };
            let utrd = unsafe { &mut *utrd_ptr };
            let ucd_phys = ucd_buf.dma_addr().as_u64() + (slot * UCD_SLOT_SIZE) as u64;
            let utrd_phys = utrd_list.dma_addr().as_u64() + (slot * 32) as u64;
            let rsp_dma = ucd_phys + UCD_COMMAND_UPIU_SIZE as u64;
            // info!(
            //     "[k3-ufs] submit: slot={} utrd_dma=0x{:x} ucd_dma=0x{:x} rsp_dma=0x{:x} db_mask=0x{:x}",
            //     slot, utrd_phys, ucd_phys, rsp_dma, slot_mask
            // );

            utrd.dw0 = 0;
            utrd.dw1 = 0;
            utrd.dw2 = OCS_INVALID_COMMAND_STATUS as u32;
            utrd.dw3 = 0;
            utrd.ucdba = (ucd_phys & 0xFFFFFFFF) as u32;
            utrd.ucdbau = (ucd_phys >> 32) as u32;
            utrd.rul = (UCD_RESPONSE_UPIU_SIZE / 4) as u16;
            utrd.ruo = (UCD_COMMAND_UPIU_SIZE / 4) as u16;
            utrd.prdtl = 0;
            utrd.prdto = (UCD_PRDT_OFFSET / 4) as u16;

            utrd.dw0 =
                (UTP_CMD_TYPE_UFS_STORAGE as u32) << 28 | (data_dir as u32) << 25 | (1u32 << 24);

            if let Some(buf) = data_buf.as_mut() {
                let data_phys = buf.dma_addr().as_u64();
                ucd.prdt[0].dba = (data_phys & 0xFFFFFFFF) as u32;
                ucd.prdt[0].dbau = (data_phys >> 32) as u32;
                ucd.prdt[0].reserved = 0;
                ucd.prdt[0].dbc = data_len - 1;
                utrd.prdtl = 1;
                buf.prepare_for_device(0, data_len as usize);
            }

            ucd_buf.prepare_for_device(slot * UCD_SLOT_SIZE, UCD_SLOT_SIZE);
            utrd_list.prepare_for_device(slot * 32, 32);
        }

        unsafe {
            self.write32(REG_INTERRUPT_STATUS, 0xFFFFFFFF);
            // dma_wmb() = fence ow,ow (Linux: ufs_spacemit_setup_xfer_req before doorbell)
            dma_wmb();
            self.write32(REG_UTP_TRANSFER_REQ_DOOR_BELL, slot_mask);
            // K3: flush posted doorbell write (Linux: ufshcd_send_command K3 readback)
            let _ = self.read32(REG_UTP_TRANSFER_REQ_DOOR_BELL);
        }

        // Poll for completion: Linux uses doorbell-clear as the sole signal.
        // (Linux: completed_reqs = ~tr_doorbell & outstanding_reqs)
        for i in 0..10000 {
            let db = unsafe { self.read32(REG_UTP_TRANSFER_REQ_DOOR_BELL) };

            if i % 500 == 0 {
                let is = unsafe { self.read32(REG_INTERRUPT_STATUS) };
                let ocs = {
                    let utrd_list = self.utrd_list.as_mut().ok_or("UTRD not initialized")?;
                    utrd_list.complete_for_cpu(slot * 32, 32);
                    let utrd_ptr =
                        unsafe { (utrd_list.as_ptr().as_ptr() as *const Utrd).add(slot) };
                    unsafe { Self::read_utrd_ocs(utrd_ptr) }
                };
                // info!("[k3-ufs] Poll {}: DB=0x{:x}, IS=0x{:x}, OCS=0x{:x}",
                //       i, db, is, ocs);
            }

            if db & slot_mask == 0 {
                // SpacemiT K3: dma_rmb() before reading UTRD OCS and response UPIU.
                // (Linux: __ufshcd_transfer_req_compl → dma_rmb() under CONFIG_SCSI_UFS_SPACEMIT_K3)
                dma_rmb();
                let (ocs, response) = {
                    let utrd_list = self.utrd_list.as_mut().ok_or("UTRD not initialized")?;
                    let ucd_buf = self.ucd_buf.as_mut().ok_or("UCD not initialized")?;
                    utrd_list.complete_for_cpu(slot * 32, 32);
                    ucd_buf.complete_for_cpu(slot * UCD_SLOT_SIZE, UCD_SLOT_SIZE);

                    let utrd_ptr =
                        unsafe { (utrd_list.as_ptr().as_ptr() as *const Utrd).add(slot) };
                    let ucd_ptr = unsafe { ucd_buf.as_ptr().as_ptr().add(slot * UCD_SLOT_SIZE) }
                        as *const Ucd;
                    let ocs = unsafe { Self::read_utrd_ocs(utrd_ptr) };
                    let mut response = [0u8; 512];
                    if ocs == 0 {
                        unsafe {
                            let rsp = (*ucd_ptr).response_upiu.as_ptr();
                            for j in 0..512 {
                                response[j] = core::ptr::read_volatile(rsp.add(j));
                            }
                        }
                    }
                    (ocs, response)
                };

                if ocs != 0 {
                    self.dump_transfer_state(slot, "OCS ERROR");
                    unsafe { self.write32(REG_INTERRUPT_STATUS, UTP_TRANSFER_REQ_COMPL) };
                    return Err("OCS error");
                }

                if let Some(buf) = data_buf.as_mut() {
                    buf.complete_for_cpu(0, data_len as usize);
                }

                unsafe { self.write32(REG_INTERRUPT_STATUS, UTP_TRANSFER_REQ_COMPL) };
                return Ok(response);
            }

            axklib::time::busy_wait(core::time::Duration::from_micros(100));
        }

        self.dump_transfer_state(slot, "TIMEOUT");
        Err("Timeout")
    }

    /// NOP OUT command (Linux: ufshcd_prepare_utp_nop_upiu, ufshcd.c:2851)
    fn nop_out(&mut self) -> Result<(), &'static str> {
        info!("[k3-ufs] Sending NOP OUT...");

        for retry in 0..NOP_OUT_RETRIES {
            let mut upiu = [0u8; 512];
            self.build_nop_upiu(&mut upiu, 0);

            match self.submit_upiu(&upiu, None, 0, 0) {
                Ok(response) => {
                    if response[0] == UPIU_TRANSACTION_NOP_IN {
                        info!("[k3-ufs] NOP IN received");
                        return Ok(());
                    }
                    warn!("[k3-ufs] NOP: unexpected response 0x{:02x}", response[0]);
                }
                Err(e) => {
                    warn!(
                        "[k3-ufs] NOP OUT retry {}/{}: {}",
                        retry + 1,
                        NOP_OUT_RETRIES,
                        e
                    );
                }
            }
        }
        Err("NOP OUT failed")
    }

    /// QUERY FLAG operation (Linux: ufshcd_query_flag, ufshcd.c:3419)
    /// Request value is always 0; SET_FLAG sets by opcode, READ_FLAG returns value in response
    fn query_flag(&mut self, opcode: u8, idn: u8) -> Result<bool, &'static str> {
        let query_func = if opcode == UPIU_QUERY_OPCODE_SET_FLAG {
            UPIU_QUERY_FUNC_STANDARD_WRITE_REQUEST
        } else {
            UPIU_QUERY_FUNC_STANDARD_READ_REQUEST
        };

        let mut upiu = [0u8; 512];
        self.build_query_upiu(&mut upiu, 0, query_func, opcode, idn);

        let response = self.submit_upiu(&upiu, None, 0, 0)?;

        if response[0] != UPIU_TRANSACTION_QUERY_RSP {
            warn!("[k3-ufs] QUERY: unexpected response 0x{:02x}", response[0]);
            return Err("Invalid QUERY response");
        }

        let value_bytes = [response[20], response[21], response[22], response[23]];
        let value = u32::from_be_bytes(value_bytes);
        Ok((value & 1) != 0)
    }

    /// Complete device init (Linux: ufshcd_complete_dev_init, ufshcd.c:4812)
    fn complete_dev_init(&mut self) -> Result<(), &'static str> {
        info!("[k3-ufs] Setting fDeviceInit flag...");

        for retry in 0..QUERY_REQ_RETRIES {
            match self.query_flag(UPIU_QUERY_OPCODE_SET_FLAG, QUERY_FLAG_IDN_FDEVICEINIT) {
                Ok(_) => break,
                Err(e) if retry < QUERY_REQ_RETRIES - 1 => {
                    warn!("[k3-ufs] SET_FLAG retry {}: {}", retry + 1, e);
                    continue;
                }
                Err(e) => return Err(e),
            }
        }

        info!("[k3-ufs] Polling fDeviceInit completion...");
        for _ in 0..(FDEVICEINIT_COMPL_TIMEOUT_MS / 10) {
            match self.query_flag(UPIU_QUERY_OPCODE_READ_FLAG, QUERY_FLAG_IDN_FDEVICEINIT) {
                Ok(false) => {
                    info!("[k3-ufs] fDeviceInit cleared by device");
                    return Ok(());
                }
                Ok(true) => {}
                Err(e) => return Err(e),
            }
            axklib::time::busy_wait(core::time::Duration::from_millis(10));
        }
        Err("fDeviceInit timeout")
    }

    fn scsi_read_command(
        &mut self,
        name: &'static str,
        cdb: &[u8],
        data_len: usize,
    ) -> Result<([u8; 512], ContiguousArray<u8>), &'static str> {
        let mut data_buf = self
            .dma
            .contiguous_array_zero_with_align::<u8>(data_len, 64, DmaDirection::FromDevice)
            .map_err(|_| "Failed to allocate SCSI data buffer")?;

        let mut upiu = [0u8; 512];
        self.build_scsi_upiu(
            &mut upiu,
            0,
            self.active_lun,
            UPIU_CMD_FLAGS_READ,
            cdb,
            data_len as u32,
        );

        let response = self.submit_upiu(
            &upiu,
            Some(&mut data_buf),
            data_len as u32,
            UTP_DATA_DIR_TO_HOST,
        )?;

        if response[0] != UPIU_TRANSACTION_RESPONSE {
            warn!(
                "[k3-ufs] {} invalid response transaction=0x{:02x}, rsp[0..32]={:02x?}",
                name,
                response[0],
                &response[..32]
            );
            return Err("Invalid SCSI response");
        }

        data_buf.complete_for_cpu(0, data_len);
        Ok((response, data_buf))
    }

    fn scsi_nodata_command(
        &mut self,
        name: &'static str,
        cdb: &[u8],
    ) -> Result<[u8; 512], &'static str> {
        let mut upiu = [0u8; 512];
        self.build_scsi_upiu(&mut upiu, 0, self.active_lun, UPIU_CMD_FLAGS_NONE, cdb, 0);

        let response = self.submit_upiu(&upiu, None, 0, 0)?;
        if response[0] != UPIU_TRANSACTION_RESPONSE {
            warn!(
                "[k3-ufs] {} invalid response transaction=0x{:02x}, rsp[0..32]={:02x?}",
                name,
                response[0],
                &response[..32]
            );
            return Err("Invalid SCSI response");
        }
        Ok(response)
    }

    fn log_scsi_failure(&self, name: &str, response: &[u8; 512]) {
        let data_seg_len = u16::from_be_bytes([response[10], response[11]]);
        let sense_len = u16::from_be_bytes([response[32], response[33]]) as usize;
        warn!(
            "[k3-ufs] {} failed: rsp=0x{:02x}, response=0x{:02x}, status=0x{:02x}, \
             flags=0x{:02x}, data_seg_len={}, sense_len={}, rsp[0..64]={:02x?}",
            name,
            response[0],
            response[6],
            response[7],
            response[1],
            data_seg_len,
            sense_len,
            &response[..64]
        );

        if sense_len > 0 {
            let sense_end = (34 + sense_len).min(64);
            warn!(
                "[k3-ufs] {} sense in UPIU: {:02x?}",
                name,
                &response[34..sense_end]
            );
            if let Some((key, asc, ascq)) = Self::decode_fixed_sense(&response[34..sense_end]) {
                warn!(
                    "[k3-ufs] {} sense decoded: key=0x{:02x}, asc=0x{:02x}, ascq=0x{:02x}",
                    name, key, asc, ascq
                );
            }
        }
    }

    fn sense_from_response(response: &[u8; 512]) -> Option<(u8, u8, u8)> {
        let sense_len = u16::from_be_bytes([response[32], response[33]]) as usize;
        if sense_len == 0 {
            return None;
        }
        let sense_end = (34 + sense_len).min(response.len());
        Self::decode_fixed_sense(&response[34..sense_end])
    }

    fn decode_fixed_sense(sense: &[u8]) -> Option<(u8, u8, u8)> {
        if sense.len() < 14 {
            return None;
        }
        let response_code = sense[0] & 0x7f;
        if response_code != 0x70 && response_code != 0x71 {
            return None;
        }
        Some((sense[2] & 0x0f, sense[12], sense[13]))
    }

    fn request_sense(&mut self, reason: &'static str) -> Option<(u8, u8, u8)> {
        info!(
            "[k3-ufs] Sending REQUEST_SENSE after {} on UPIU LUN 0x{:02x}...",
            reason, self.active_lun
        );
        let cdb = [
            SCSI_REQUEST_SENSE,
            0,
            0,
            0,
            18,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];
        let (response, data_buf) = match self.scsi_read_command("REQUEST_SENSE", &cdb, 18) {
            Ok(ok) => ok,
            Err(e) => {
                warn!("[k3-ufs] REQUEST_SENSE transport failed: {}", e);
                return None;
            }
        };

        if response[7] != SAM_STAT_GOOD {
            self.log_scsi_failure("REQUEST_SENSE", &response);
            return None;
        }

        let sense = unsafe { core::slice::from_raw_parts(data_buf.as_ptr().as_ptr(), 18) };
        info!("[k3-ufs] REQUEST_SENSE data: {:02x?}", sense);
        Self::decode_fixed_sense(sense)
    }

    fn test_unit_ready(&mut self) -> Result<(), &'static str> {
        info!(
            "[k3-ufs] Sending TEST_UNIT_READY on UPIU LUN 0x{:02x}...",
            self.active_lun
        );
        let cdb = [
            SCSI_TEST_UNIT_READY,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];

        for retry in 0..10 {
            let response = self.scsi_nodata_command("TEST_UNIT_READY", &cdb)?;
            if response[7] == SAM_STAT_GOOD {
                info!("[k3-ufs] TEST_UNIT_READY OK");
                return Ok(());
            }

            self.log_scsi_failure("TEST_UNIT_READY", &response);
            let sense = if response[7] == SAM_STAT_CHECK_CONDITION {
                Self::sense_from_response(&response)
                    .or_else(|| self.request_sense("TEST_UNIT_READY"))
            } else {
                None
            };

            if let Some((key, asc, ascq)) = sense {
                if key == SCSI_SENSE_UNIT_ATTENTION || key == SCSI_SENSE_NOT_READY {
                    warn!(
                        "[k3-ufs] TEST_UNIT_READY retry {} due to sense key=0x{:02x}, \
                         asc=0x{:02x}, ascq=0x{:02x}",
                        retry + 1,
                        key,
                        asc,
                        ascq
                    );
                    axklib::time::busy_wait(core::time::Duration::from_millis(100));
                    continue;
                }
            }

            return Err("TEST_UNIT_READY failed");
        }

        Err("TEST_UNIT_READY timeout")
    }

    /// SCSI INQUIRY (Linux SCSI layer)
    fn scsi_inquiry(&mut self) -> Result<(), &'static str> {
        info!(
            "[k3-ufs] Sending SCSI INQUIRY on UPIU LUN 0x{:02x}...",
            self.active_lun
        );

        let cdb = [SCSI_INQUIRY, 0, 0, 0, 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let (response, data_buf) = self.scsi_read_command("INQUIRY", &cdb, 36)?;

        if response[0] != UPIU_TRANSACTION_RESPONSE {
            return Err("Invalid INQUIRY response");
        }

        let status = response[7];
        if status != SAM_STAT_GOOD {
            self.log_scsi_failure("INQUIRY", &response);
            return Err("INQUIRY failed");
        }

        let inq_data = unsafe { core::slice::from_raw_parts(data_buf.as_ptr().as_ptr(), 36) };
        let vendor = core::str::from_utf8(&inq_data[8..16]).unwrap_or("?");
        let product = core::str::from_utf8(&inq_data[16..32]).unwrap_or("?");
        info!(
            "[k3-ufs] LUN 0x{:02x} device: {} {}",
            self.active_lun,
            vendor.trim(),
            product.trim()
        );

        Ok(())
    }

    /// SCSI READ_CAPACITY(10) (Linux SCSI layer)
    fn scsi_read_capacity(&mut self) -> Result<(u64, u32), &'static str> {
        let (blocks, block_size) = match self.scsi_read_capacity_10() {
            Ok((blocks, block_size)) if blocks != 0x1_0000_0000 => Ok((blocks, block_size)),
            Ok((_blocks, _block_size)) => {
                warn!(
                    "[k3-ufs] READ_CAPACITY(10) returned 0xffffffff last LBA, trying \
                     READ_CAPACITY(16)"
                );
                self.scsi_read_capacity_16()
            }
            Err(e) => {
                warn!(
                    "[k3-ufs] READ_CAPACITY(10) failed: {}, trying READ_CAPACITY(16)",
                    e
                );
                self.scsi_read_capacity_16()
            }
        }?;

        let block_size = if block_size == 0 {
            warn!("[k3-ufs] Sector size 0 reported, assuming 512");
            512
        } else {
            block_size
        };

        if !matches!(block_size, 512 | 1024 | 2048 | 4096) {
            warn!("[k3-ufs] Unsupported sector size {}", block_size);
            return Err("Unsupported sector size");
        }

        Ok((blocks, block_size))
    }

    /// SCSI REPORT LUNS (Linux: scsi_report_lun_scan in drivers/scsi/scsi_scan.c).
    fn report_luns(&mut self) -> Result<Vec<u64>, &'static str> {
        info!("[k3-ufs] Sending REPORT_LUNS...");

        let mut cdb = [0u8; 16];
        cdb[0] = SCSI_REPORT_LUNS;
        cdb[6..10].copy_from_slice(&(SCSI_REPORT_LUNS_ALLOC_LEN as u32).to_be_bytes());

        let (response, data_buf) =
            self.scsi_read_command("REPORT_LUNS", &cdb, SCSI_REPORT_LUNS_ALLOC_LEN)?;
        if response[7] != SAM_STAT_GOOD {
            self.log_scsi_failure("REPORT_LUNS", &response);
            return Err("REPORT_LUNS failed");
        }

        let data = unsafe {
            core::slice::from_raw_parts(data_buf.as_ptr().as_ptr(), SCSI_REPORT_LUNS_ALLOC_LEN)
        };
        let list_len = u32::from_be_bytes([data[0], data[1], data[2], data[3]]) as usize;
        if list_len == 0 || list_len + 8 > SCSI_REPORT_LUNS_ALLOC_LEN || list_len % 8 != 0 {
            warn!("[k3-ufs] REPORT_LUNS invalid list length {}", list_len);
            return Err("Invalid REPORT_LUNS data");
        }

        let mut luns = Vec::new();
        for entry in data[8..8 + list_len].chunks_exact(8) {
            let scsi_lun = Self::scsilun_to_int(entry);
            let upiu_lun = Self::scsi_to_upiu_lun(scsi_lun);
            info!(
                "[k3-ufs] REPORT_LUNS entry: scsi_lun=0x{:x}, upiu_lun=0x{:02x}",
                scsi_lun, upiu_lun
            );
            luns.push(scsi_lun);
        }

        Ok(luns)
    }

    fn is_wlun(scsi_lun: u64) -> bool {
        (scsi_lun & 0xFF00) == SCSI_W_LUN_BASE
    }

    fn has_partition_signature(block0: &[u8]) -> bool {
        block0.get(510..512) == Some([0x55, 0xaa].as_slice())
    }

    fn configure_lun(&mut self, scsi_lun: u64) -> Result<(u64, u32), &'static str> {
        self.active_lun = Self::scsi_to_upiu_lun(scsi_lun);
        info!(
            "[k3-ufs] Probing SCSI LUN 0x{:x} as UPIU LUN 0x{:02x}",
            scsi_lun, self.active_lun
        );

        self.scsi_inquiry()?;
        self.test_unit_ready()?;
        let (num_blocks, block_size) = self.scsi_read_capacity()?;
        self.num_blocks = num_blocks;
        self.block_size = block_size as usize;
        Ok((num_blocks, block_size))
    }

    fn select_data_lun(&mut self) -> Result<(u64, u64, u32), &'static str> {
        let mut luns = match self.report_luns() {
            Ok(luns) if !luns.is_empty() => luns,
            Ok(_) => {
                warn!("[k3-ufs] REPORT_LUNS returned no LUNs, falling back to LUN 0");
                Vec::from([0])
            }
            Err(e) => {
                warn!("[k3-ufs] REPORT_LUNS failed: {}, falling back to LUN 0", e);
                Vec::from([0])
            }
        };

        if !luns.contains(&0) {
            luns.push(0);
        }

        let mut first_usable = None;
        for scsi_lun in luns {
            if Self::is_wlun(scsi_lun) {
                info!("[k3-ufs] Skipping well-known SCSI LUN 0x{:x}", scsi_lun);
                continue;
            }

            let Ok((num_blocks, block_size)) = self.configure_lun(scsi_lun) else {
                warn!("[k3-ufs] LUN 0x{:x} probe failed, skipping", scsi_lun);
                continue;
            };

            if first_usable.is_none() {
                first_usable = Some((scsi_lun, num_blocks, block_size));
            }

            let mut test_buf = [0u8; 4096];
            let test_len = self.block_size.min(test_buf.len());
            if self.scsi_read_10(0, 1, &mut test_buf[..test_len]).is_err() {
                warn!("[k3-ufs] LUN 0x{:x} LBA0 read failed, skipping", scsi_lun);
                continue;
            }

            let mbr_sig = test_buf.get(510..512).unwrap_or(&[]);
            let gpt_sig = if self.block_size >= 4096 && self.num_blocks > 1 {
                let mut gpt_buf = [0u8; 4096];
                if self.scsi_read_10(1, 1, &mut gpt_buf[..test_len]).is_ok() {
                    let sig = gpt_buf.get(0..8).unwrap_or(&[]);
                    info!(
                        "[k3-ufs] LUN 0x{:x} GPT header signature at LBA1: {:02x?}",
                        scsi_lun, sig
                    );
                    sig == b"EFI PART"
                } else {
                    false
                }
            } else {
                false
            };

            info!(
                "[k3-ufs] LUN 0x{:x} LBA0[0..64]={:02x?}, MBR sig={:02x?}",
                scsi_lun,
                &test_buf[..64.min(test_len)],
                mbr_sig
            );

            if Self::has_partition_signature(&test_buf[..test_len]) || gpt_sig {
                info!("[k3-ufs] Selecting LUN 0x{:x} as block device", scsi_lun);
                return Ok((scsi_lun, num_blocks, block_size));
            }

            warn!(
                "[k3-ufs] LUN 0x{:x} has no MBR/GPT signature at probed locations",
                scsi_lun
            );
        }

        if let Some((scsi_lun, num_blocks, block_size)) = first_usable {
            warn!(
                "[k3-ufs] No LUN with partition signature found; using first usable LUN 0x{:x}",
                scsi_lun
            );
            self.active_lun = Self::scsi_to_upiu_lun(scsi_lun);
            self.num_blocks = num_blocks;
            self.block_size = block_size as usize;
            return Ok((scsi_lun, num_blocks, block_size));
        }

        Err("No usable LUN found")
    }

    fn scsi_read_capacity_10(&mut self) -> Result<(u64, u32), &'static str> {
        info!(
            "[k3-ufs] Sending READ_CAPACITY(10) on UPIU LUN 0x{:02x}...",
            self.active_lun
        );
        let cdb = [
            SCSI_READ_CAPACITY_10,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];

        for retry in 0..10 {
            let (response, data_buf) = self.scsi_read_command("READ_CAPACITY(10)", &cdb, 8)?;
            if response[7] == SAM_STAT_GOOD {
                let cap_data =
                    unsafe { core::slice::from_raw_parts(data_buf.as_ptr().as_ptr(), 8) };
                info!("[k3-ufs] READ_CAPACITY(10) data: {:02x?}", cap_data);
                let last_lba =
                    u32::from_be_bytes([cap_data[0], cap_data[1], cap_data[2], cap_data[3]]);
                let block_size =
                    u32::from_be_bytes([cap_data[4], cap_data[5], cap_data[6], cap_data[7]]);
                let blocks = last_lba as u64 + 1;
                info!(
                    "[k3-ufs] Capacity(10): {} blocks x {} bytes",
                    blocks, block_size
                );
                return Ok((blocks, block_size));
            }

            self.log_scsi_failure("READ_CAPACITY(10)", &response);
            let sense = if response[7] == SAM_STAT_CHECK_CONDITION {
                Self::sense_from_response(&response)
                    .or_else(|| self.request_sense("READ_CAPACITY(10)"))
            } else {
                None
            };

            if let Some((key, asc, ascq)) = sense {
                if key == SCSI_SENSE_UNIT_ATTENTION || key == SCSI_SENSE_NOT_READY {
                    warn!(
                        "[k3-ufs] READ_CAPACITY(10) retry {} due to sense key=0x{:02x}, \
                         asc=0x{:02x}, ascq=0x{:02x}",
                        retry + 1,
                        key,
                        asc,
                        ascq
                    );
                    axklib::time::busy_wait(core::time::Duration::from_millis(100));
                    continue;
                }

                if key == SCSI_SENSE_ILLEGAL_REQUEST {
                    return Err("READ_CAPACITY(10) illegal request");
                }
            }

            axklib::time::busy_wait(core::time::Duration::from_millis(50));
        }

        Err("READ_CAPACITY(10) retry exhausted")
    }

    fn scsi_read_capacity_16(&mut self) -> Result<(u64, u32), &'static str> {
        info!(
            "[k3-ufs] Sending READ_CAPACITY(16) on UPIU LUN 0x{:02x}...",
            self.active_lun
        );
        let mut cdb = [0u8; 16];
        cdb[0] = SCSI_SERVICE_ACTION_IN_16;
        cdb[1] = SAI_READ_CAPACITY_16;
        cdb[10..14].copy_from_slice(&32u32.to_be_bytes());

        for retry in 0..10 {
            let (response, data_buf) = self.scsi_read_command("READ_CAPACITY(16)", &cdb, 32)?;
            if response[7] == SAM_STAT_GOOD {
                let cap_data =
                    unsafe { core::slice::from_raw_parts(data_buf.as_ptr().as_ptr(), 32) };
                info!("[k3-ufs] READ_CAPACITY(16) data: {:02x?}", cap_data);
                let last_lba = u64::from_be_bytes([
                    cap_data[0],
                    cap_data[1],
                    cap_data[2],
                    cap_data[3],
                    cap_data[4],
                    cap_data[5],
                    cap_data[6],
                    cap_data[7],
                ]);
                let block_size =
                    u32::from_be_bytes([cap_data[8], cap_data[9], cap_data[10], cap_data[11]]);
                let blocks = last_lba + 1;
                info!(
                    "[k3-ufs] Capacity(16): {} blocks x {} bytes",
                    blocks, block_size
                );
                return Ok((blocks, block_size));
            }

            self.log_scsi_failure("READ_CAPACITY(16)", &response);
            let sense = if response[7] == SAM_STAT_CHECK_CONDITION {
                Self::sense_from_response(&response)
                    .or_else(|| self.request_sense("READ_CAPACITY(16)"))
            } else {
                None
            };

            if let Some((key, asc, ascq)) = sense {
                if key == SCSI_SENSE_UNIT_ATTENTION || key == SCSI_SENSE_NOT_READY {
                    warn!(
                        "[k3-ufs] READ_CAPACITY(16) retry {} due to sense key=0x{:02x}, \
                         asc=0x{:02x}, ascq=0x{:02x}",
                        retry + 1,
                        key,
                        asc,
                        ascq
                    );
                    axklib::time::busy_wait(core::time::Duration::from_millis(100));
                    continue;
                }

                if key == SCSI_SENSE_ILLEGAL_REQUEST {
                    return Err("READ_CAPACITY(16) illegal request");
                }
            }

            axklib::time::busy_wait(core::time::Duration::from_millis(50));
        }

        Err("READ_CAPACITY(16) retry exhausted")
    }

    fn scsi_read_10(
        &mut self,
        lba: u32,
        num_blocks: u16,
        buffer: &mut [u8],
    ) -> Result<(), &'static str> {
        let transfer_len = num_blocks as u32 * self.block_size as u32;
        if buffer.len() < transfer_len as usize {
            return Err("Buffer too small");
        }

        // info!(
        //     "[k3-ufs] READ_10: lun=0x{:02x}, lba={}, blocks={}, block_size={}, len={}",
        //     self.active_lun, lba, num_blocks, self.block_size, transfer_len
        // );

        let mut data_buf = self
            .dma
            .contiguous_array_zero_with_align::<u8>(
                transfer_len as usize,
                64,
                DmaDirection::FromDevice,
            )
            .map_err(|_| "Failed to allocate data buffer")?;

        let mut cdb = [0u8; 16];
        cdb[0] = SCSI_READ_10;
        cdb[2..6].copy_from_slice(&lba.to_be_bytes());
        cdb[7..9].copy_from_slice(&num_blocks.to_be_bytes());

        let mut upiu = [0u8; 512];
        self.build_scsi_upiu(
            &mut upiu,
            0,
            self.active_lun,
            UPIU_CMD_FLAGS_READ,
            &cdb,
            transfer_len,
        );

        let response = self.submit_upiu(
            &upiu,
            Some(&mut data_buf),
            transfer_len,
            UTP_DATA_DIR_TO_HOST,
        )?;

        if response[0] != UPIU_TRANSACTION_RESPONSE || response[7] != SAM_STAT_GOOD {
            self.log_scsi_failure("READ_10", &response);
            return Err("READ_10 failed");
        }

        data_buf.read_from_device(transfer_len as usize, |data| {
            buffer[..transfer_len as usize].copy_from_slice(data);
        });

        Ok(())
    }

    fn scsi_write_10(
        &mut self,
        lba: u32,
        num_blocks: u16,
        buffer: &[u8],
    ) -> Result<(), &'static str> {
        let transfer_len = num_blocks as u32 * self.block_size as u32;
        if buffer.len() < transfer_len as usize {
            return Err("Buffer too small");
        }

        // info!(
        //     "[k3-ufs] WRITE_10: lun=0x{:02x}, lba={}, blocks={}, block_size={}, len={}",
        //     self.active_lun,
        //     lba,
        //     num_blocks,
        //     self.block_size,
        //     transfer_len
        // );

        let mut data_buf = self
            .dma
            .contiguous_array_zero_with_align::<u8>(
                transfer_len as usize,
                64,
                DmaDirection::ToDevice,
            )
            .map_err(|_| "Failed to allocate data buffer")?;
        data_buf.copy_to_device_from_slice(&buffer[..transfer_len as usize]);

        let mut cdb = [0u8; 16];
        cdb[0] = SCSI_WRITE_10;
        cdb[2..6].copy_from_slice(&lba.to_be_bytes());
        cdb[7..9].copy_from_slice(&num_blocks.to_be_bytes());

        let mut upiu = [0u8; 512];
        self.build_scsi_upiu(
            &mut upiu,
            0,
            self.active_lun,
            UPIU_CMD_FLAGS_WRITE,
            &cdb,
            transfer_len,
        );

        let response = self.submit_upiu(
            &upiu,
            Some(&mut data_buf),
            transfer_len,
            UTP_DATA_DIR_TO_DEVICE,
        )?;

        if response[0] != UPIU_TRANSACTION_RESPONSE || response[7] != SAM_STAT_GOOD {
            self.log_scsi_failure("WRITE_10", &response);
            return Err("WRITE_10 failed");
        }

        Ok(())
    }
}

impl crate::block::SyncBlockOps for K3UfsHost {
    fn name(&self) -> &'static str {
        "k3-ufs"
    }

    fn num_blocks(&self) -> u64 {
        self.num_blocks
    }

    fn block_size(&self) -> usize {
        self.block_size
    }

    fn read_blocks(&mut self, block_id: u64, buf: &mut [u8]) -> Result<(), rdif_block::BlkError> {
        let num_blocks = buf.len() / self.block_size;
        if num_blocks == 0 || buf.len() % self.block_size != 0 {
            return Err(rdif_block::BlkError::InvalidRequest);
        }
        if block_id > u32::MAX as u64 || num_blocks > u16::MAX as usize {
            return Err(rdif_block::BlkError::InvalidRequest);
        }

        match self.scsi_read_10(block_id as u32, num_blocks as u16, buf) {
            Ok(()) => {
                // K3 UFS fills the SyncBlockOps request buffer through the CPU
                // mapping. The generic Block wrapper later treats this same
                // buffer as a DMA-from-device bounce buffer, so clean these CPU
                // writes before the wrapper invalidates and copies it.
                if let Some(ptr) = NonNull::new(buf.as_mut_ptr()) {
                    axklib::dma::op().flush(ptr, buf.len());
                }
                Ok(())
            }
            Err(e) => {
                warn!(
                    "[k3-ufs] read_blocks({}, {} bytes) failed: {}",
                    block_id,
                    buf.len(),
                    e
                );
                Err(rdif_block::BlkError::Io)
            }
        }
    }

    fn write_blocks(&mut self, block_id: u64, buf: &[u8]) -> Result<(), rdif_block::BlkError> {
        let num_blocks = buf.len() / self.block_size;
        if num_blocks == 0 || buf.len() % self.block_size != 0 {
            return Err(rdif_block::BlkError::InvalidRequest);
        }
        if block_id > u32::MAX as u64 || num_blocks > u16::MAX as usize {
            return Err(rdif_block::BlkError::InvalidRequest);
        }

        self.scsi_write_10(block_id as u32, num_blocks as u16, buf)
            .map_err(|e| {
                warn!(
                    "[k3-ufs] write_blocks({}, {} bytes) failed: {}",
                    block_id,
                    buf.len(),
                    e
                );
                rdif_block::BlkError::Io
            })
    }
}

fn probe(probe: ProbeFdt<'_>) -> Result<(), OnProbeError> {
    let (info, plat_dev) = probe.into_parts();

    info!("[k3-ufs] ============================================");
    info!("[k3-ufs] Probing SpacemiT K3 UFS controller");

    let base_reg = info
        .node
        .regs()
        .into_iter()
        .next()
        .ok_or_else(|| OnProbeError::other("no reg property"))?;

    let mmio_size = base_reg.size.unwrap_or(0x40000) as usize;
    info!(
        "[k3-ufs] MMIO: base=0x{:x}, size=0x{:x}",
        base_reg.address, mmio_size
    );

    let irq_num = decode_fdt_irq(&info.interrupts());
    if let Some(irq) = irq_num {
        info!("[k3-ufs] IRQ: {}", irq);
    }

    let mmio_base = iomap(base_reg.address as usize, mmio_size)?;

    let dma = axklib::dma::device_with_mask(0xFFFFFFFFFFFFFFFF);

    let mut host = K3UfsHost {
        mmio_base,
        clock_freq: 491_520_000,
        nutrs: 0,
        active_lun: 0,
        num_blocks: 0,
        block_size: 512,
        dma,
        utrd_list: None,
        utmrd_list: None,
        ucd_buf: None,
    };

    host.mphy_init()
        .map_err(|e| OnProbeError::other(format!("MPHY init failed: {}", e)))?;

    host.host_init()
        .map_err(|e| OnProbeError::other(format!("Host init failed: {}", e)))?;

    host.unipro_init()
        .map_err(|e| OnProbeError::other(format!("UNIPRO init failed: {}", e)))?;

    host.link_startup()
        .map_err(|e| OnProbeError::other(format!("Link startup failed: {}", e)))?;

    host.link_startup_post()
        .map_err(|e| OnProbeError::other(format!("Link startup post failed: {}", e)))?;

    host.dump_regs();

    info!("[k3-ufs] *** PHASE 2 COMPLETE: Link startup OK ***");

    // Phase 3: Setup transfer lists and test read
    host.setup_transfer_lists()
        .map_err(|e| OnProbeError::other(format!("Transfer setup failed: {}", e)))?;

    info!("[k3-ufs] *** PHASE 3: Device initialization chain ***");

    // Step 1: NOP OUT
    host.nop_out()
        .map_err(|e| OnProbeError::other(format!("NOP OUT failed: {}", e)))?;

    // Step 2: Complete device init (fDeviceInit)
    host.complete_dev_init()
        .map_err(|e| OnProbeError::other(format!("Device init failed: {}", e)))?;

    // Step 3: Linux registers WLUNs and then calls scsi_scan_host(), which
    // uses REPORT_LUNS and probes each regular LU. Do the same minimal scan
    // here and select the first LU that looks like a data disk.
    let (scsi_lun, num_blocks, block_size) = host
        .select_data_lun()
        .map_err(|e| OnProbeError::other(format!("LUN scan failed: {}", e)))?;

    // Register block device
    crate::block::register_sync_block(plat_dev, host);
    info!("[k3-ufs] Block device registered");
    info!(
        "[k3-ufs] *** DEVICE READY: SCSI LUN 0x{:x}, {} blocks x {} bytes ***",
        scsi_lun, num_blocks, block_size
    );
    info!("[k3-ufs] ============================================");

    Ok(())
}