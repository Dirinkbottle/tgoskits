//! UIC command transport for the K3 UFS host controller.
//!
//! A single synchronous command channel shared by all UniPro DME access
//! (DME_GET / DME_SET / DME_PEER_GET) and the link-startup command, with
//! polling-based completion (the driver has no UIC interrupt handler).
//! The channel semantics follow Linux `ufshcd_uic_cmd` / `ufshcd_dme_get_attr`
//! (`drivers/ufs/core/ufshcd.c`): the command result lives in the low byte of
//! ARG2 (`MASK_UIC_COMMAND_RESULT`, 0 = success) and the attribute value of a
//! DME_GET is returned in ARG3.

use log::warn;

use super::{
    K3UfsHost,
    error::UfsError,
    regs::{
        MASK_UIC_COMMAND_RESULT, REG_INTERRUPT_STATUS, REG_UIC_COMMAND, REG_UIC_COMMAND_ARG1,
        REG_UIC_COMMAND_ARG2, REG_UIC_COMMAND_ARG3, REG_UIC_ERROR_CODE_DATA_LINK_LAYER,
        REG_UIC_ERROR_CODE_DME, REG_UIC_ERROR_CODE_NETWORK_LAYER,
        REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER, REG_UIC_ERROR_CODE_TRANSPORT_LAYER, UIC_CMD_CGE,
        UIC_CMD_DME_GET, UIC_CMD_DME_PEER_GET, UIC_CMD_DME_SET, UIC_COMMAND_COMPL,
    },
};

impl K3UfsHost {
    /// Issue a UIC command and wait for UIC_COMMAND_COMPL.
    ///
    /// On completion ARG2 holds the command result (Linux:
    /// `MASK_UIC_COMMAND_RESULT`, 0 = success) and ARG3 holds the attribute
    /// value for DME_GET/DME_PEER_GET; this returns the ARG3 value and errors
    /// on a non-zero result.
    pub(super) fn uic_cmd(
        &self,
        cmd: u32,
        arg1: u32,
        arg2: u32,
        arg3: u32,
    ) -> Result<u32, UfsError> {
        // SAFETY: all offsets are UFSHCI register constants inside the mapped
        // MMIO window, and access is exclusive to this host instance; see
        // `read32`/`write32`.
        unsafe {
            self.write32(REG_INTERRUPT_STATUS, UIC_COMMAND_COMPL);
            self.write32(REG_UIC_COMMAND_ARG1, arg1);
            self.write32(REG_UIC_COMMAND_ARG2, arg2);
            self.write32(REG_UIC_COMMAND_ARG3, arg3);
            // Set CGE so the controller asserts IS.UCCS and we can poll for
            // completion without a registered UIC interrupt handler.
            self.write32(REG_UIC_COMMAND, cmd | UIC_CMD_CGE);

            for _ in 0..5000 {
                let is = self.read32(REG_INTERRUPT_STATUS);
                if is & UIC_COMMAND_COMPL != 0 {
                    self.write32(REG_INTERRUPT_STATUS, UIC_COMMAND_COMPL);
                    let result = self.read32(REG_UIC_COMMAND_ARG2);
                    if result & MASK_UIC_COMMAND_RESULT != 0 {
                        self.log_uic_error(cmd, result);
                        return Err(UfsError::Init("UIC command failed"));
                    }
                    return Ok(self.read32(REG_UIC_COMMAND_ARG3));
                }
                axklib::time::busy_wait(core::time::Duration::from_micros(100));
            }

            Err(UfsError::Init("UIC command timeout"))
        }
    }

    /// Read and log the sticky UIC error code registers on a failed command
    /// (Linux: ufshcd_uic_cmd error path reads `REG_UIC_ERROR_CODE_*`).
    fn log_uic_error(&self, cmd: u32, result: u32) {
        // SAFETY: all offsets are UFSHCI register constants inside the mapped
        // MMIO window, and access is exclusive to this host instance.
        unsafe {
            let pa = self.read32(REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER);
            let dl = self.read32(REG_UIC_ERROR_CODE_DATA_LINK_LAYER);
            let nw = self.read32(REG_UIC_ERROR_CODE_NETWORK_LAYER);
            let tr = self.read32(REG_UIC_ERROR_CODE_TRANSPORT_LAYER);
            let dme = self.read32(REG_UIC_ERROR_CODE_DME);
            warn!(
                "[k3-ufs] UIC cmd 0x{:02x} failed: result=0x{:08x} (pa=0x{:02x} dl=0x{:02x} \
                 nw=0x{:02x} tr=0x{:02x} dme=0x{:02x})",
                cmd, result, pa, dl, nw, tr, dme
            );
        }
    }

    /// DME_SET a UNIPRO attribute.
    pub(super) fn dme_set(&self, attr: u32, value: u32) -> Result<(), UfsError> {
        let _ = self.uic_cmd(UIC_CMD_DME_SET, attr, 0, value)?;
        Ok(())
    }

    /// DME_GET a UNIPRO attribute; the attribute value is returned in ARG3.
    pub(super) fn dme_get(&self, attr: u32) -> Result<u32, UfsError> {
        let value = self.uic_cmd(UIC_CMD_DME_GET, attr, 0, 0)?;
        Ok(value)
    }

    /// DME_PEER_GET a remote (device-side) UNIPRO attribute.
    pub(super) fn dme_peer_get(&self, attr: u32) -> Result<u32, UfsError> {
        let value = self.uic_cmd(UIC_CMD_DME_PEER_GET, attr, 0, 0)?;
        Ok(value)
    }
}
