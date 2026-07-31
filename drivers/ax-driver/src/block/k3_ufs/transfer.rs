//! UTP transfer submission, completion polling and controller recovery.
//!
//! The host submits device-management (NOP OUT / QUERY) and SCSI I/O
//! commands through the UTP transfer request list: [`K3UfsHost::submit_upiu`]
//! prepares a slot, rings the doorbell, polls for completion and, on a
//! wedged or fatal transfer, recovers the controller before retrying once.

use core::time::Duration;

use dma_api::{ContiguousArray, DmaDirection};
use log::{info, warn};

use super::{
    K3UfsHost, UpiuSlotKind,
    desc::{
        DataDirection, OCS_INVALID_COMMAND_STATUS, QUERY_FLAG_IDN_FDEVICEINIT,
        UCD_COMMAND_UPIU_SIZE, UCD_PRDT_OFFSET, UCD_RESPONSE_UPIU_SIZE, UCD_SLOT_SIZE,
        UFSHCI_NUM_SLOTS, UPIU_QUERY_FUNC_STANDARD_READ_REQUEST,
        UPIU_QUERY_FUNC_STANDARD_WRITE_REQUEST, UPIU_QUERY_OPCODE_READ_FLAG,
        UPIU_QUERY_OPCODE_SET_FLAG, UPIU_TRANSACTION_NOP_IN, UPIU_TRANSACTION_QUERY_RSP, Ucd, Utrd,
        build_nop_upiu, build_query_upiu, fill_prdt, read_utrd_ocs,
    },
    error::UfsError,
    regs::{
        REG_CONTROLLER_CAPABILITIES, REG_CONTROLLER_ENABLE, REG_CONTROLLER_STATUS,
        REG_INTERRUPT_ENABLE, REG_INTERRUPT_STATUS, REG_UIC_ERROR_CODE_DATA_LINK_LAYER,
        REG_UIC_ERROR_CODE_DME, REG_UIC_ERROR_CODE_NETWORK_LAYER,
        REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER, REG_UIC_ERROR_CODE_TRANSPORT_LAYER,
        REG_UTP_TASK_REQ_DOOR_BELL, REG_UTP_TASK_REQ_LIST_BASE_H, REG_UTP_TASK_REQ_LIST_BASE_L,
        REG_UTP_TASK_REQ_LIST_RUN_STOP, REG_UTP_TRANSFER_REQ_DOOR_BELL,
        REG_UTP_TRANSFER_REQ_INT_AGG_CONTROL, REG_UTP_TRANSFER_REQ_LIST_BASE_H,
        REG_UTP_TRANSFER_REQ_LIST_BASE_L, REG_UTP_TRANSFER_REQ_LIST_RUN_STOP, UFSHCD_ERROR_MASK,
        UFSHCD_STATUS_READY, UTP_TRANSFER_REQ_COMPL, dma_rmb, dma_wmb,
    },
};

/// Retry and timeout constants (Linux: drivers/ufs/core/ufshcd.c).
const NOP_OUT_RETRIES: usize = 10;
const QUERY_REQ_RETRIES: usize = 3;
const FDEVICEINIT_COMPL_TIMEOUT_MS: u64 = 10000;

impl K3UfsHost {
    /// Allocate the UTRD/UTMRD/UCD DMA buffers and program the list base
    /// registers (Linux: ufshcd_memory_alloc + ufshcd_make_hba_operational).
    pub(super) fn setup_transfer_lists(&mut self) -> Result<(), UfsError> {
        // Linux: nutrs = (CAP & 0x1f) + 1 (ufshcd_get_transfer_req_mgmt_max_slots)
        // SAFETY: `REG_CONTROLLER_CAPABILITIES` is a UFSHCI register inside
        // the mapped MMIO window, and access is exclusive to this host.
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
            .map_err(|_| UfsError::Other("Failed to allocate UTRD list"))?;
        let utmrd_list = self
            .dma
            .contiguous_array_zero_with_align::<u8>(8 * 80, 1024, DmaDirection::Bidirectional)
            .map_err(|_| UfsError::Other("Failed to allocate UTMRD list"))?;

        // Allocate one UCD per transfer slot. Linux lays out UCD as an array
        // indexed by task_tag.
        let ucd_buf = self
            .dma
            .contiguous_array_zero_with_align::<u8>(
                UFSHCI_NUM_SLOTS * UCD_SLOT_SIZE,
                128,
                DmaDirection::Bidirectional,
            )
            .map_err(|_| UfsError::Other("Failed to allocate UCD buffer"))?;

        // Linux: ufshcd_make_hba_operational sequence
        let utrd_phys = utrd_list.dma_addr().as_u64();
        let utmrd_phys = utmrd_list.dma_addr().as_u64();
        let ucd_phys = ucd_buf.dma_addr().as_u64();
        info!(
            "[k3-ufs] DMA layout: utrd_base=0x{:x}, utmrd_base=0x{:x}, ucd_base=0x{:x}, \
             ucd_slot_size={}",
            utrd_phys, utmrd_phys, ucd_phys, UCD_SLOT_SIZE
        );

        self.utrd_list = Some(utrd_list);
        self.utmrd_list = Some(utmrd_list);
        self.ucd_buf = Some(ucd_buf);
        self.program_transfer_lists()?;

        info!("[k3-ufs] Transfer lists configured (Linux sequence)");
        Ok(())
    }

    /// Program UTRL/UTMRL base addresses, interrupt enable, and run-stop
    /// registers. The DMA buffers must already be allocated. An HCE reset
    /// (controller recovery) clears these registers, so this sequence is
    /// replayed after recovery without reallocating the DMA buffers.
    fn program_transfer_lists(&mut self) -> Result<(), UfsError> {
        let utrd_list = self
            .utrd_list
            .as_ref()
            .ok_or(UfsError::Other("UTRD list not allocated"))?;
        let utmrd_list = self
            .utmrd_list
            .as_ref()
            .ok_or(UfsError::Other("UTMRD list not allocated"))?;
        let utrd_phys = utrd_list.dma_addr().as_u64();
        let utmrd_phys = utmrd_list.dma_addr().as_u64();

        // SAFETY: every register below is a UFSHCI constant inside the mapped
        // MMIO window, and access is exclusive to this host instance; see
        // `read32`/`write32`.
        unsafe {
            // This driver polls completion and never registers an IRQ handler,
            // so no interrupt-enable bits are set: the controller would
            // otherwise keep the interrupt line asserted with nobody to
            // service it. Interrupt *status* bits still latch and are read
            // directly by `poll_completion`.

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
                return Err(UfsError::Init("Host lists not ready"));
            }

            // 4. Enable run-stop registers (Linux: ufshcd_enable_run_stop_reg)
            self.write32(REG_UTP_TASK_REQ_LIST_RUN_STOP, 1);
            self.write32(REG_UTP_TRANSFER_REQ_LIST_RUN_STOP, 1);
        }

        // A non-zero transfer doorbell after (re)enable means stale bits
        // survived; such a slot would be silently ignored again.
        // SAFETY: `REG_UTP_TRANSFER_REQ_DOOR_BELL` is a UFSHCI register inside
        // the mapped MMIO window, and access is exclusive to this host.
        let db = unsafe { self.read32(REG_UTP_TRANSFER_REQ_DOOR_BELL) };
        if db != 0 {
            warn!(
                "[k3-ufs] doorbell not clear after controller (re)enable: DB=0x{:08x}",
                db
            );
        }

        Ok(())
    }

    /// Compute the next rotating SCSI I/O slot, returning `(slot, next counter)`.
    ///
    /// Slots rotate through `0..n` where `n = max(1, nutrs - 1)`, so ordinary
    /// commands never collide with the reserved device-management slot
    /// `nutrs - 1`. Transfers are submitted and awaited synchronously, so a
    /// rotating counter is sufficient; there is never an outstanding request
    /// to collide with.
    fn next_io_slot(nutrs: usize, counter: usize) -> (usize, usize) {
        let n = nutrs.saturating_sub(1).max(1);
        let slot = counter % n;
        (slot, counter.wrapping_add(1))
    }

    /// Allocate a doorbell slot for SCSI I/O.
    fn alloc_io_slot(&mut self) -> usize {
        debug_assert!(
            self.nutrs > 1,
            "k3-ufs: nutrs={} leaves no distinct SCSI I/O slot",
            self.nutrs
        );
        let (slot, next) = Self::next_io_slot(self.nutrs, self.next_io_slot);
        self.next_io_slot = next;
        slot
    }

    /// Dump the UTRD/UCD and controller state for a slot (Linux: ufshcd_print_tr).
    fn dump_transfer_state(&self, slot: usize, msg: &str) {
        let Some(utrd_list) = self.utrd_list.as_ref() else {
            warn!("[k3-ufs] {} slot={} (UTRD list not allocated)", msg, slot);
            return;
        };
        let Some(ucd_buf) = self.ucd_buf.as_ref() else {
            warn!("[k3-ufs] {} slot={} (UCD buffer not allocated)", msg, slot);
            return;
        };
        // SAFETY: `slot` is always below `nutrs <= UFSHCI_NUM_SLOTS`: it is
        // either the reserved device-management slot (`nutrs - 1`) or an I/O
        // slot from `alloc_io_slot()`. Both DMA buffers are allocated for
        // `UFSHCI_NUM_SLOTS` slots, so the address arithmetic stays within
        // each mapped region, and the buffers have been synced back to the CPU
        // by the caller before this dump is reached.
        let utrd_ptr = unsafe { (utrd_list.as_ptr().as_ptr() as *const Utrd).add(slot) };
        let ucd_ptr = unsafe { ucd_buf.as_ptr().as_ptr().add(slot * UCD_SLOT_SIZE) } as *const Ucd;

        // SAFETY: `utrd_ptr` points at a live 32-byte UTRD slot inside the
        // UTRD list and `ucd_ptr` at a live `Ucd` slot inside the UCD
        // buffer; both pointers are aligned and valid for the byte ranges
        // sliced below, and all register reads are inside the mapped MMIO
        // window with no aliasing access.
        unsafe {
            let utrd_bytes = core::slice::from_raw_parts(utrd_ptr as *const u8, 32);
            let req_upiu = core::slice::from_raw_parts((*ucd_ptr).command_upiu.as_ptr(), 32);
            let rsp_upiu = core::slice::from_raw_parts((*ucd_ptr).response_upiu.as_ptr(), 64);

            let db = self.read32(REG_UTP_TRANSFER_REQ_DOOR_BELL);
            let is = self.read32(REG_INTERRUPT_STATUS);
            let ie = self.read32(REG_INTERRUPT_ENABLE);
            let hcs = self.read32(REG_CONTROLLER_STATUS);
            let run_stop = self.read32(REG_UTP_TRANSFER_REQ_LIST_RUN_STOP);
            let uecpa = self.read32(REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER);
            let uecdl = self.read32(REG_UIC_ERROR_CODE_DATA_LINK_LAYER);
            let uecn = self.read32(REG_UIC_ERROR_CODE_NETWORK_LAYER);
            let uect = self.read32(REG_UIC_ERROR_CODE_TRANSPORT_LAYER);
            let uecdme = self.read32(REG_UIC_ERROR_CODE_DME);

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

    /// Submit an UPIU and wait for completion.
    ///
    /// Device management commands (NOP OUT, QUERY) use the reserved slot
    /// `nutrs - 1`; SCSI I/O rotates through slots 0..nutrs-2. On timeout or
    /// controller error the host is fully recovered (HCE reset + link
    /// startup, Linux: ufshcd_err_handler) and the command is resubmitted
    /// once. Without this recovery a wedged slot keeps its doorbell bit set
    /// forever and every later submission to it is silently ignored.
    pub(super) fn submit_upiu(
        &mut self,
        upiu: &[u8; 512],
        mut data_buf: Option<&mut ContiguousArray<u8>>,
        data_len: u32,
        data_dir: DataDirection,
        kind: UpiuSlotKind,
    ) -> Result<[u8; 512], UfsError> {
        if self.fatal {
            return Err(UfsError::ControllerFatal);
        }
        let slot = match kind {
            UpiuSlotKind::DevCmd => self.nutrs.saturating_sub(1),
            UpiuSlotKind::ScsiIo => self.alloc_io_slot(),
        };
        // `Option<&mut T>` is not Copy, so reborrow the buffer for each
        // attempt instead of moving it into submit_upiu_once.
        match self.submit_upiu_once(upiu, data_buf.as_deref_mut(), data_len, data_dir, slot) {
            Err(e)
                if matches!(
                    e,
                    UfsError::Timeout | UfsError::OcsError | UfsError::ControllerFatal
                ) =>
            {
                warn!(
                    "[k3-ufs] slot {} failed ({e}); recovering controller and retrying once",
                    slot
                );
                if self.recover_controller().is_ok() {
                    let slot = match kind {
                        UpiuSlotKind::DevCmd => self.nutrs.saturating_sub(1),
                        UpiuSlotKind::ScsiIo => self.alloc_io_slot(),
                    };
                    self.submit_upiu_once(upiu, data_buf, data_len, data_dir, slot)
                } else {
                    // Recovery failed: the controller may be half-reset (HCE
                    // down, lists cleared). Latch the fatal state so every
                    // later submission fails fast instead of re-entering the
                    // expensive recovery path repeatedly.
                    self.fatal = true;
                    Err(UfsError::Other("controller recovery failed"))
                }
            }
            other => other,
        }
    }

    /// Single-shot UPIU submission on an explicit slot (Linux:
    /// ufshcd_send_command + __ufshcd_transfer_req_compl). Split into
    /// [`Self::prepare_slot`], [`Self::ring_doorbell`] and
    /// [`Self::poll_completion`] steps so each phase has one owner.
    fn submit_upiu_once(
        &mut self,
        upiu: &[u8; 512],
        mut data_buf: Option<&mut ContiguousArray<u8>>,
        data_len: u32,
        data_dir: DataDirection,
        slot: usize,
    ) -> Result<[u8; 512], UfsError> {
        self.prepare_slot(upiu, data_buf.as_deref_mut(), data_len, data_dir, slot)?;
        self.ring_doorbell(slot);
        // `data_buf` is the last use here, so move it into the poll step.
        self.poll_completion(slot, data_buf, data_len)
    }

    /// Fill the UTRD/UCD for `slot` and flush the DMA buffers to the device.
    ///
    /// The command UPIU is copied, the task tag is pinned to the slot, the
    /// PRDT is cleared and, for data transfers, linked to `data_buf`.
    fn prepare_slot(
        &mut self,
        upiu: &[u8; 512],
        mut data_buf: Option<&mut ContiguousArray<u8>>,
        data_len: u32,
        data_dir: DataDirection,
        slot: usize,
    ) -> Result<(), UfsError> {
        let utrd_list = self
            .utrd_list
            .as_mut()
            .ok_or(UfsError::Other("UTRD list not allocated"))?;
        let ucd_buf = self
            .ucd_buf
            .as_mut()
            .ok_or(UfsError::Other("UCD buffer not allocated"))?;
        // SAFETY: `slot` is below `nutrs <= UFSHCI_NUM_SLOTS` (reserved
        // device-management slot or an `alloc_io_slot()` I/O slot), the
        // UCD buffer is allocated for `UFSHCI_NUM_SLOTS * UCD_SLOT_SIZE`
        // bytes and its lifetime covers the whole submit+poll cycle, and
        // the slot is not concurrently reused.
        let ucd_ptr = unsafe { ucd_buf.as_ptr().as_ptr().add(slot * UCD_SLOT_SIZE) } as *mut Ucd;
        // SAFETY: `ucd_ptr` is derived from the UCD buffer base and points
        // at a valid `Ucd` slot within it; the reference is exclusive.
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

        // SAFETY: `slot` is below `nutrs <= UFSHCI_NUM_SLOTS`, and the
        // UTRD list is allocated for `UFSHCI_NUM_SLOTS * 32` bytes whose
        // lifetime covers the whole submit+poll cycle; the slot is not
        // concurrently reused.
        let utrd_ptr = unsafe { (utrd_list.as_ptr().as_ptr() as *mut Utrd).add(slot) };
        // SAFETY: `utrd_ptr` is derived from the UTRD list base and points
        // at a valid `Utrd` slot within it; the reference is exclusive.
        let utrd = unsafe { &mut *utrd_ptr };
        let ucd_phys = ucd_buf.dma_addr().as_u64() + (slot * UCD_SLOT_SIZE) as u64;

        utrd.dw0 = data_dir.dw0();
        utrd.dw1 = 0;
        utrd.dw2 = OCS_INVALID_COMMAND_STATUS;
        utrd.dw3 = 0;
        utrd.ucdba = (ucd_phys & 0xFFFFFFFF) as u32;
        utrd.ucdbau = (ucd_phys >> 32) as u32;
        utrd.rul = (UCD_RESPONSE_UPIU_SIZE / 4) as u16;
        utrd.ruo = (UCD_COMMAND_UPIU_SIZE / 4) as u16;
        utrd.prdtl = 0;
        utrd.prdto = (UCD_PRDT_OFFSET / 4) as u16;

        if let Some(buf) = data_buf.as_mut() {
            fill_prdt(&mut ucd.prdt[0], buf.dma_addr().as_u64(), data_len);
            utrd.prdtl = 1;
            buf.prepare_for_device(0, data_len as usize);
        }

        ucd_buf.prepare_for_device(slot * UCD_SLOT_SIZE, UCD_SLOT_SIZE);
        utrd_list.prepare_for_device(slot * 32, 32);
        Ok(())
    }

    /// Clear the interrupt status, then ring the doorbell for `slot` and
    /// flush the posted write (Linux: ufs_spacemit_setup_xfer_req before
    /// doorbell + ufshcd_send_command K3 readback).
    fn ring_doorbell(&self, slot: usize) {
        let slot_mask = 1u32 << slot;
        // SAFETY: `read32`/`write32` are used with offsets inside the mapped
        // MMIO window; the UTRD/UCD contents have been prepared and flushed
        // for the device (`prepare_for_device`), and the doorbell write is
        // ordered by the `dma_wmb()` barrier with no aliasing access.
        unsafe {
            self.write32(REG_INTERRUPT_STATUS, 0xFFFFFFFF);
            // dma_wmb() = fence ow,ow (Linux: ufs_spacemit_setup_xfer_req before doorbell)
            dma_wmb();
            self.write32(REG_UTP_TRANSFER_REQ_DOOR_BELL, slot_mask);
            // K3: flush posted doorbell write (Linux: ufshcd_send_command K3 readback)
            let _ = self.read32(REG_UTP_TRANSFER_REQ_DOOR_BELL);
        }
    }

    /// Poll the doorbell for `slot` completion, drain the descriptors and
    /// return the response UPIU.
    ///
    /// Returns `ControllerFatal` on a fatal error interrupt, `OcsError` when
    /// the device reports a bad Overall Command Status, and `Timeout` when
    /// the doorbell never clears.
    fn poll_completion(
        &mut self,
        slot: usize,
        mut data_buf: Option<&mut ContiguousArray<u8>>,
        data_len: u32,
    ) -> Result<[u8; 512], UfsError> {
        let slot_mask = 1u32 << slot;

        // Poll for completion: Linux uses doorbell-clear as the sole signal.
        // (Linux: completed_reqs = ~tr_doorbell & outstanding_reqs)
        for i in 0..10000 {
            // SAFETY: `REG_UTP_TRANSFER_REQ_DOOR_BELL`/`REG_INTERRUPT_STATUS`
            // are UFSHCI registers inside the mapped MMIO window, and access
            // is exclusive to this host instance.
            let db = unsafe { self.read32(REG_UTP_TRANSFER_REQ_DOOR_BELL) };

            if i % 500 == 0 {
                // Surface controller fatal errors instead of spinning to the
                // full 1 s timeout (Linux: ufshcd_check_errors + err_handler).
                let is = unsafe { self.read32(REG_INTERRUPT_STATUS) };
                if is & UFSHCD_ERROR_MASK != 0 {
                    warn!("[k3-ufs] Fatal error interrupt: IS=0x{:08x}", is);
                    self.dump_transfer_state(slot, "FATAL ERROR");
                    return Err(UfsError::ControllerFatal);
                }
            }

            if db & slot_mask == 0 {
                // SpacemiT K3: dma_rmb() before reading UTRD OCS and response UPIU.
                // (Linux: __ufshcd_transfer_req_compl → dma_rmb() under CONFIG_SCSI_UFS_SPACEMIT_K3)
                dma_rmb();
                let (ocs, response) = {
                    let utrd_list = self
                        .utrd_list
                        .as_mut()
                        .ok_or(UfsError::Other("UTRD list not allocated"))?;
                    let ucd_buf = self
                        .ucd_buf
                        .as_mut()
                        .ok_or(UfsError::Other("UCD buffer not allocated"))?;
                    utrd_list.complete_for_cpu(slot * 32, 32);
                    ucd_buf.complete_for_cpu(slot * UCD_SLOT_SIZE, UCD_SLOT_SIZE);

                    // SAFETY: `slot` is below `nutrs <= UFSHCI_NUM_SLOTS`; the
                    // UTRD list and UCD buffer are allocated for
                    // `UFSHCI_NUM_SLOTS` slots, their lifetime covers the whole
                    // submit+poll cycle, and the slot is not concurrently
                    // reused.
                    let utrd_ptr =
                        unsafe { (utrd_list.as_ptr().as_ptr() as *const Utrd).add(slot) };
                    let ucd_ptr = unsafe { ucd_buf.as_ptr().as_ptr().add(slot * UCD_SLOT_SIZE) }
                        as *const Ucd;
                    // SAFETY: `utrd_ptr`/`ucd_ptr` point at valid, aligned
                    // descriptors whose DMA memory has just been synced to the
                    // CPU with `complete_for_cpu`; no other access races here.
                    let ocs = unsafe { read_utrd_ocs(utrd_ptr) };
                    let mut response = [0u8; 512];
                    if ocs == 0 {
                        // SAFETY: `rsp` points into the response UPIU of a
                        // live `Ucd` slot; `read_volatile` of each byte is
                        // within the 512-byte response buffer.
                        unsafe {
                            let rsp = (*ucd_ptr).response_upiu.as_ptr();
                            for (j, byte) in response.iter_mut().enumerate() {
                                *byte = core::ptr::read_volatile(rsp.add(j));
                            }
                        }
                    }
                    (ocs, response)
                };

                if ocs != 0 {
                    self.dump_transfer_state(slot, "OCS ERROR");
                    // SAFETY: clearing the transfer-complete interrupt status
                    // is a plain MMIO write inside the mapped window; see
                    // `write32`.
                    unsafe { self.write32(REG_INTERRUPT_STATUS, UTP_TRANSFER_REQ_COMPL) };
                    return Err(UfsError::OcsError);
                }

                if let Some(buf) = data_buf.as_mut() {
                    buf.complete_for_cpu(0, data_len as usize);
                }

                // SAFETY: acknowledging the transfer-complete interrupt status
                // is a plain MMIO write inside the mapped window; see `write32`.
                unsafe { self.write32(REG_INTERRUPT_STATUS, UTP_TRANSFER_REQ_COMPL) };
                return Ok(response);
            }

            axklib::time::busy_wait(Duration::from_micros(100));
        }

        self.dump_transfer_state(slot, "TIMEOUT");
        Err(UfsError::Timeout)
    }

    /// Recover the UFS host after a transfer timeout or fatal error.
    ///
    /// Mirrors Linux ufshcd_err_handler(): stop the transfer/task lists,
    /// reset the host controller (HCE toggle), re-run link startup, and
    /// reprogram the list base addresses. An HCE reset clears the
    /// UTRLBA/UTMRLBA/IE/run-stop registers, so the whole re-enable sequence
    /// is replayed; the DMA buffers themselves are reused.
    fn recover_controller(&mut self) -> Result<(), UfsError> {
        info!("[k3-ufs] Recovering UFS controller after transfer error");

        // Stop both list run-stop registers (Linux: ufshcd_hba_stop).
        // SAFETY: all offsets are UFSHCI register constants inside the mapped
        // MMIO window, and access is exclusive to this host instance; see
        // `read32`/`write32`.
        unsafe {
            self.write32(REG_UTP_TRANSFER_REQ_LIST_RUN_STOP, 0);
            self.write32(REG_UTP_TASK_REQ_LIST_RUN_STOP, 0);
            self.write32(REG_INTERRUPT_STATUS, 0xFFFFFFFF);
        }

        // ufshcd_hba_stop: HCE=0 and wait for it to take effect.
        // SAFETY: `REG_CONTROLLER_ENABLE` is a UFSHCI register inside the
        // mapped MMIO window; see `write32`.
        unsafe { self.write32(REG_CONTROLLER_ENABLE, 0) };
        for _ in 0..100 {
            // SAFETY: `REG_CONTROLLER_ENABLE` is a UFSHCI register inside the
            // mapped MMIO window; see `read32`.
            let hce = unsafe { self.read32(REG_CONTROLLER_ENABLE) };
            if hce & 1 == 0 {
                break;
            }
            axklib::time::busy_wait(Duration::from_millis(1));
        }

        // Clear both doorbell registers once the controller is confirmed
        // disabled (Linux: ufshcd_hba_stop). An HCE reset does NOT clear the
        // doorbell bits by itself, and a stale bit left set would be silently
        // ignored again once the controller re-enables - the exact wedge this
        // recovery exists to break. Clearing avoids a later ghost re-scan of
        // the failed slot's UTRD (whose PRDT may point at a freed buffer).
        // SAFETY: the doorbell registers are UFSHCI registers inside the
        // mapped MMIO window; see `write32`.
        unsafe {
            self.write32(REG_UTP_TRANSFER_REQ_DOOR_BELL, 0);
            self.write32(REG_UTP_TASK_REQ_DOOR_BELL, 0);
        }

        // ufshcd_hba_start + link startup: re-enable the host, re-run the
        // UNIPRO/PA attribute programming and link startup, then reprogram
        // the transfer/task list base addresses and run-stop.
        self.host_init()?;
        self.unipro_init()?;
        self.link_startup()?;
        self.link_startup_post()?;

        // HCE reset and link startup may latch fresh error interrupt bits;
        // clear them before program_transfer_lists re-enables the interrupt
        // enable register, or a stale IS bit would raise a spurious IRQ.
        // SAFETY: `REG_INTERRUPT_STATUS` is a UFSHCI register inside the
        // mapped MMIO window; see `write32`.
        unsafe { self.write32(REG_INTERRUPT_STATUS, 0xFFFFFFFF) };
        self.program_transfer_lists()?;

        // Verify the link with a NOP before retrying the failed command
        // (Linux: ufshcd_verify_dev_init). Uses the raw single-shot path so a
        // failed NOP cannot recursively re-enter recover_controller.
        self.verify_link_with_nop()?;

        info!("[k3-ufs] UFS controller recovered");
        Ok(())
    }

    /// Send a NOP OUT on the reserved slot without the recovery wrapper, used
    /// to verify the link after controller recovery. Returns an error when
    /// the device does not answer, which fails the recovery attempt.
    fn verify_link_with_nop(&mut self) -> Result<(), UfsError> {
        let mut upiu = [0u8; 512];
        build_nop_upiu(&mut upiu, 0);
        let slot = self.nutrs.saturating_sub(1);
        match self.submit_upiu_once(&upiu, None, 0, DataDirection::NoData, slot) {
            Ok(response) if response[0] == UPIU_TRANSACTION_NOP_IN => Ok(()),
            Ok(_) => Err(UfsError::Other("NOP verification: unexpected response")),
            Err(e) => Err(e),
        }
    }

    /// NOP OUT command (Linux: ufshcd_prepare_utp_nop_upiu, ufshcd.c:2851).
    pub(super) fn nop_out(&mut self) -> Result<(), UfsError> {
        info!("[k3-ufs] Sending NOP OUT...");

        for retry in 0..NOP_OUT_RETRIES {
            let mut upiu = [0u8; 512];
            build_nop_upiu(&mut upiu, 0);

            match self.submit_upiu(&upiu, None, 0, DataDirection::NoData, UpiuSlotKind::DevCmd) {
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
        Err(UfsError::Other("NOP OUT failed"))
    }

    /// QUERY FLAG operation (Linux: ufshcd_query_flag, ufshcd.c:3419).
    ///
    /// Request value is always 0; SET_FLAG sets by opcode, READ_FLAG returns
    /// the value in the response.
    fn query_flag(&mut self, opcode: u8, idn: u8) -> Result<bool, UfsError> {
        let query_func = if opcode == UPIU_QUERY_OPCODE_SET_FLAG {
            UPIU_QUERY_FUNC_STANDARD_WRITE_REQUEST
        } else {
            UPIU_QUERY_FUNC_STANDARD_READ_REQUEST
        };

        let mut upiu = [0u8; 512];
        build_query_upiu(&mut upiu, 0, query_func, opcode, idn);

        let response =
            self.submit_upiu(&upiu, None, 0, DataDirection::NoData, UpiuSlotKind::DevCmd)?;

        if response[0] != UPIU_TRANSACTION_QUERY_RSP {
            warn!("[k3-ufs] QUERY: unexpected response 0x{:02x}", response[0]);
            return Err(UfsError::Other("Invalid QUERY response"));
        }

        let value_bytes = [response[20], response[21], response[22], response[23]];
        let value = u32::from_be_bytes(value_bytes);
        Ok((value & 1) != 0)
    }

    /// Complete device init (Linux: ufshcd_complete_dev_init, ufshcd.c:4812).
    pub(super) fn complete_dev_init(&mut self) -> Result<(), UfsError> {
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
            axklib::time::busy_wait(Duration::from_millis(10));
        }
        Err(UfsError::Other("fDeviceInit timeout"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn next_io_slot_rotates_through_io_slots_only() {
        // nutrs=32: I/O slots are 0..=30, the reserved device-management slot
        // is 31 and must never be handed out.
        assert_eq!(K3UfsHost::next_io_slot(32, 0), (0, 1));
        assert_eq!(K3UfsHost::next_io_slot(32, 30), (30, 31));
        assert_eq!(K3UfsHost::next_io_slot(32, 31), (0, 32)); // wraps over the reserved slot
        assert_eq!(K3UfsHost::next_io_slot(32, 62), (31 % 31, 63));
    }

    #[test]
    fn next_io_slot_handles_small_slot_counts() {
        // nutrs=2 leaves exactly one I/O slot: n = max(1, 1) = 1.
        assert_eq!(K3UfsHost::next_io_slot(2, 0), (0, 1));
        assert_eq!(K3UfsHost::next_io_slot(2, 5), (0, 6));
        // nutrs=1 (malformed CAP) degrades to a single slot as well.
        assert_eq!(K3UfsHost::next_io_slot(1, 0), (0, 1));
    }
}
