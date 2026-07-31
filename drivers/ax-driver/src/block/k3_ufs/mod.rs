//! SpacemiT K3 UFS host controller driver.
//!
//! The driver probes the `spacemit,k3-ufshcd` FDT node, brings up the MPHY
//! and UNIPRO link, programs the UTP transfer lists, scans for a data LUN,
//! and registers a synchronous block device through [`crate::block`].
//!
//! Module layout:
//! - `regs`: register map and hardware constants
//! - `error`: the `UfsError` driver error type
//! - `desc`: UTRD/PRDT/UCD descriptors and UPIU build helpers
//! - `init`: MPHY / UNIPRO / link-startup initialization sequence
//! - `transfer`: UPIU submission, completion polling, controller recovery
//! - `scsi`: SCSI command layer and LUN selection

mod desc;
mod error;
mod init;
mod regs;
mod scsi;
mod transfer;

use alloc::format;
use core::ptr::NonNull;

use dma_api::{ContiguousArray, DeviceDma, DmaOp};
use log::{info, warn};
use rdrive::{
    probe::{OnProbeError, fdt::InterruptRef},
    register::*,
};

use crate::mmio::iomap;

/// Reference clock frequency of the K3 UFS MPHY (491.52 MHz), used to derive
/// the 1 us and symbol-clock timer values programmed in [`K3UfsHost::host_init`].
const DEFAULT_K3_UFS_CLOCK_HZ: u32 = 491_520_000;
/// Fallback MMIO window size when the FDT node's `reg` property carries no
/// size cell (Linux ufs-spacemit.dtsi uses 0x40000).
const DEFAULT_K3_UFS_MMIO_SIZE: u64 = 0x40000;

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

/// FDT probe entry: map MMIO, run the init chain, scan LUNs, and register a
/// synchronous block device.
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

    let mmio_size = base_reg.size.unwrap_or(DEFAULT_K3_UFS_MMIO_SIZE) as usize;
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
        clock_freq: DEFAULT_K3_UFS_CLOCK_HZ,
        nutrs: 0,
        active_lun: 0,
        num_blocks: 0,
        block_size: 512,
        dma,
        utrd_list: None,
        utmrd_list: None,
        ucd_buf: None,
        next_io_slot: 0,
        fatal: false,
    };

    // Bring up the PHY and link before any descriptor work can happen.
    host.mphy_init()
        .map_err(|e| OnProbeError::other(format!("MPHY init failed: {e}")))?;
    host.host_init()
        .map_err(|e| OnProbeError::other(format!("Host init failed: {e}")))?;
    host.unipro_init()
        .map_err(|e| OnProbeError::other(format!("UNIPRO init failed: {e}")))?;
    host.link_startup()
        .map_err(|e| OnProbeError::other(format!("Link startup failed: {e}")))?;
    host.link_startup_post()
        .map_err(|e| OnProbeError::other(format!("Link startup post failed: {e}")))?;
    host.dump_regs();

    // Setup the transfer lists and start the device initialization chain.
    host.setup_transfer_lists()
        .map_err(|e| OnProbeError::other(format!("Transfer setup failed: {e}")))?;

    info!("[k3-ufs] Starting device initialization chain");

    // Step 1: NOP OUT
    host.nop_out()
        .map_err(|e| OnProbeError::other(format!("NOP OUT failed: {e}")))?;

    // Step 2: Complete device init (fDeviceInit)
    host.complete_dev_init()
        .map_err(|e| OnProbeError::other(format!("Device init failed: {e}")))?;

    // Step 3: Linux registers WLUNs and then calls scsi_scan_host(), which
    // uses REPORT_LUNS and probes each regular LU. Do the same minimal scan
    // here and select the first LU that looks like a data disk.
    let (scsi_lun, num_blocks, block_size) = host
        .select_data_lun()
        .map_err(|e| OnProbeError::other(format!("LUN scan failed: {e}")))?;

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
/// Reference: linux-5.4.29 drivers/of/irq.c:108-127
/// Reference: linux-5.4.29 Documentation/devicetree/booting-without-of.txt:1300-1313
fn decode_fdt_irq(interrupts: &[InterruptRef]) -> Option<usize> {
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

/// SpacemiT K3 UFS host controller state.
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
    /// Rotating slot counter for SCSI I/O (Linux: blk-mq tag for ufshcd_queue_command).
    /// Device management commands always use the reserved slot `nutrs - 1`.
    next_io_slot: usize,
    /// Set when controller recovery itself fails; the controller is left in
    /// an unknown state and further transfers are refused until re-probe.
    fatal: bool,
}

/// Which doorbell slot class an UPIU uses.
///
/// Linux keeps the last slot (`nutrs - 1`) reserved for device management
/// commands (`ufshcd_exec_dev_cmd`) and issues regular SCSI I/O on the
/// remaining slots (`ufshcd_queue_command`). Keeping the two classes apart
/// prevents a stuck SCSI transfer from blocking NOP/QUERY recovery traffic.
#[derive(Clone, Copy, PartialEq, Eq)]
enum UpiuSlotKind {
    DevCmd,
    ScsiIo,
}

impl K3UfsHost {
    /// Read a 32-bit MMIO register.
    ///
    /// # Safety
    ///
    /// - `mmio_base` must point to a mapped MMIO window and `offset` must fall
    ///   inside that window such that `offset..offset + 4` is fully covered.
    /// - `offset` must be 4-byte aligned.
    /// - The caller must guarantee that no concurrent aliasing access to the
    ///   same location races with this volatile read.
    unsafe fn read32(&self, offset: usize) -> u32 {
        // SAFETY: the `# Safety` contract of `read32` (valid mapped window,
        // 4-byte aligned offset, no aliasing race) applies to this volatile read.
        let v =
            unsafe { core::ptr::read_volatile(self.mmio_base.as_ptr().add(offset) as *const u32) };
        // SAFETY: inline asm has no memory operands and `options(nostack)`; it
        // is only a read-read device fence, safe in any context.
        unsafe { core::arch::asm!("fence i, ir", options(nostack)) };
        v
    }

    /// Write a 32-bit MMIO register.
    ///
    /// # Safety
    ///
    /// - `mmio_base` must point to a mapped MMIO window and `offset` must fall
    ///   inside that window such that `offset..offset + 4` is fully covered.
    /// - `offset` must be 4-byte aligned.
    /// - The caller must guarantee that no concurrent aliasing access to the
    ///   same location races with this volatile write.
    unsafe fn write32(&self, offset: usize, value: u32) {
        // SAFETY: the `# Safety` contract of `write32` (valid mapped window,
        // 4-byte aligned offset, no aliasing race) applies to this volatile write.
        unsafe {
            // Linux: __io_bw() = fence w,o (arch/riscv/include/asm/mmio.h)
            core::arch::asm!("fence w, o", options(nostack));
            core::ptr::write_volatile(self.mmio_base.as_ptr().add(offset) as *mut u32, value);
        }
    }
}

// SAFETY: K3UfsHost is exclusively accessed through `SyncBlockOps` methods,
// which the block adapter serializes behind `Arc<Mutex<D>>` (block/mod.rs).
// The driver is poll-driven and never registers an IRQ handler, so no second
// context touches the MMIO window while a transfer is in flight.
unsafe impl Send for K3UfsHost {}

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
        if num_blocks == 0 || !buf.len().is_multiple_of(self.block_size) {
            return Err(rdif_block::BlkError::InvalidRequest);
        }
        if block_id > u32::MAX as u64 || num_blocks > u16::MAX as usize {
            return Err(rdif_block::BlkError::InvalidRequest);
        }
        if block_id + num_blocks as u64 > self.num_blocks {
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
        if num_blocks == 0 || !buf.len().is_multiple_of(self.block_size) {
            return Err(rdif_block::BlkError::InvalidRequest);
        }
        if block_id > u32::MAX as u64 || num_blocks > u16::MAX as usize {
            return Err(rdif_block::BlkError::InvalidRequest);
        }
        if block_id + num_blocks as u64 > self.num_blocks {
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
