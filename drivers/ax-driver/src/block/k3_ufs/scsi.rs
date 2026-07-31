//! SCSI command layer for the K3 UFS host.
//!
//! Probe-time commands (TEST UNIT READY, INQUIRY, READ CAPACITY, REPORT LUNS)
//! and the READ/WRITE (10) data path, plus LUN selection that mirrors the
//! Linux `scsi_scan_host()` flow: report LUNs, probe each regular LU, and
//! pick the first one that carries a partition signature.

use alloc::vec::Vec;
use core::time::Duration;

use dma_api::{ContiguousArray, DmaDirection};
use log::{info, warn};

use super::{
    K3UfsHost, UpiuSlotKind,
    desc::{
        DataDirection, UFS_UPIU_MAX_UNIT_NUM_ID, UFS_UPIU_WLUN_ID, UPIU_CMD_FLAGS_NONE,
        UPIU_CMD_FLAGS_READ, UPIU_CMD_FLAGS_WRITE, UPIU_TRANSACTION_RESPONSE, build_scsi_upiu,
    },
    error::UfsError,
};

/// SCSI commands (Linux: include/scsi/scsi.h).
const SCSI_TEST_UNIT_READY: u8 = 0x00;
const SCSI_REQUEST_SENSE: u8 = 0x03;
const SCSI_INQUIRY: u8 = 0x12;
const SCSI_READ_CAPACITY_10: u8 = 0x25;
const SCSI_READ_10: u8 = 0x28;
const SCSI_WRITE_10: u8 = 0x2A;
const SCSI_SERVICE_ACTION_IN_16: u8 = 0x9E;
const SCSI_REPORT_LUNS: u8 = 0xA0;
const SAI_READ_CAPACITY_16: u8 = 0x10;

/// SCSI status codes (Linux: include/scsi/scsi_proto.h).
const SAM_STAT_GOOD: u8 = 0x00;
const SAM_STAT_CHECK_CONDITION: u8 = 0x02;

/// SCSI sense keys (Linux: include/scsi/scsi_proto.h).
const SCSI_SENSE_NOT_READY: u8 = 0x02;
const SCSI_SENSE_ILLEGAL_REQUEST: u8 = 0x05;
const SCSI_SENSE_UNIT_ATTENTION: u8 = 0x06;

/// SCSI well-known LUN encoding (Linux: include/scsi/scsi.h).
const SCSI_W_LUN_BASE: u64 = 0xC100;
const SCSI_REPORT_LUNS_ALLOC_LEN: usize = 4096;

/// Allocation lengths and retry policy for probe-time SCSI commands.
const SCSI_INQUIRY_ALLOC_LEN: u8 = 36;
const REQUEST_SENSE_ALLOC_LEN: u8 = 18;
const SCSI_CMD_RETRIES: usize = 10;
const SCSI_RETRY_BACKOFF_MS: u64 = 100;

/// Build a READ_CAPACITY CDB.
///
/// `service_action` is SAI_READ_CAPACITY_16 for the (16) variant and 0 for
/// the (10) variant; the allocation length is placed in the CDB fields each
/// variant requires.
fn build_read_capacity_cdb(opcode: u8, service_action: u8, alloc_len: u32) -> [u8; 16] {
    let mut cdb = [0u8; 16];
    cdb[0] = opcode;
    if opcode == SCSI_SERVICE_ACTION_IN_16 {
        cdb[1] = service_action;
        cdb[10..14].copy_from_slice(&alloc_len.to_be_bytes());
    }
    cdb
}

/// Parse a READ_CAPACITY response into `(num_blocks, logical_block_size)`.
///
/// `data` is the zero-padded response buffer; `alloc_len >= 32` selects the
/// (16) layout (8-byte last LBA at offset 0, block size at offset 8) and the
/// (10) layout otherwise (4-byte last LBA, block size at offset 4). The
/// returned block count is `last_lba + 1`, saturated at `u64::MAX` so a
/// malicious all-ones reply cannot overflow.
fn parse_capacity(data: &[u8; 32], alloc_len: u32) -> (u64, u32) {
    let last_lba = if alloc_len >= 32 {
        u64::from_be_bytes([
            data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
        ])
    } else {
        u64::from(u32::from_be_bytes([data[0], data[1], data[2], data[3]]))
    };
    let block_size = if alloc_len >= 32 {
        u32::from_be_bytes([data[8], data[9], data[10], data[11]])
    } else {
        u32::from_be_bytes([data[4], data[5], data[6], data[7]])
    };
    (last_lba.saturating_add(1), block_size)
}

impl K3UfsHost {
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

    fn is_wlun(scsi_lun: u64) -> bool {
        (scsi_lun & 0xFF00) == SCSI_W_LUN_BASE
    }

    fn has_partition_signature(block0: &[u8]) -> bool {
        block0.get(510..512) == Some([0x55, 0xaa].as_slice())
    }

    /// Decode fixed-format sense data into `(key, asc, ascq)`.
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

    /// Extract the sense data embedded in a response UPIU, if any.
    fn sense_from_response(response: &[u8; 512]) -> Option<(u8, u8, u8)> {
        let sense_len = u16::from_be_bytes([response[32], response[33]]) as usize;
        if sense_len == 0 {
            return None;
        }
        let sense_end = (34 + sense_len).min(response.len());
        Self::decode_fixed_sense(&response[34..sense_end])
    }

    /// Whether a CHECK CONDITION sense should be retried instead of failing.
    ///
    /// UNIT_ATTENTION and NOT_READY are transient (device still powering up
    /// or reporting an async event); everything else is treated as fatal for
    /// the probe-time commands.
    fn sense_is_retryable(sense: Option<(u8, u8, u8)>) -> bool {
        matches!(
            sense,
            Some((key, _, _)) if key == SCSI_SENSE_UNIT_ATTENTION || key == SCSI_SENSE_NOT_READY
        )
    }

    /// Fetch the sense data for a CHECK CONDITION response, issuing a
    /// REQUEST_SENSE when the response UPIU carries no sense bytes.
    fn check_condition_sense(
        &mut self,
        name: &'static str,
        response: &[u8; 512],
    ) -> Option<(u8, u8, u8)> {
        if response[7] == SAM_STAT_CHECK_CONDITION {
            Self::sense_from_response(response).or_else(|| self.request_sense(name))
        } else {
            None
        }
    }

    /// Log a failed SCSI response together with its decoded sense data.
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

    /// Issue REQUEST_SENSE and decode the returned fixed sense data.
    fn request_sense(&mut self, reason: &'static str) -> Option<(u8, u8, u8)> {
        info!(
            "[k3-ufs] Sending REQUEST_SENSE after {} on UPIU LUN 0x{:02x}...",
            reason, self.active_lun
        );
        let mut cdb = [0u8; 16];
        cdb[0] = SCSI_REQUEST_SENSE;
        cdb[4] = REQUEST_SENSE_ALLOC_LEN;

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

        // SAFETY: `data_buf` is a DMA allocation of exactly `REQUEST_SENSE_ALLOC_LEN`
        // (18) bytes, already synced to the CPU by `scsi_read_command`; the slice
        // length matches the allocation.
        let sense = unsafe { core::slice::from_raw_parts(data_buf.as_ptr().as_ptr(), 18) };
        info!("[k3-ufs] REQUEST_SENSE data: {:02x?}", sense);
        Self::decode_fixed_sense(sense)
    }

    /// Issue a SCSI command with a read data phase and return the response
    /// UPIU plus the DMA data buffer (already synced to the CPU).
    fn scsi_read_command(
        &mut self,
        name: &'static str,
        cdb: &[u8],
        data_len: usize,
    ) -> Result<([u8; 512], ContiguousArray<u8>), UfsError> {
        let mut data_buf = self
            .dma
            .contiguous_array_zero_with_align::<u8>(data_len, 64, DmaDirection::FromDevice)
            .map_err(|_| UfsError::Other("Failed to allocate SCSI data buffer"))?;

        let mut upiu = [0u8; 512];
        build_scsi_upiu(
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
            DataDirection::ToHost,
            UpiuSlotKind::ScsiIo,
        )?;

        if response[0] != UPIU_TRANSACTION_RESPONSE {
            warn!(
                "[k3-ufs] {} invalid response transaction=0x{:02x}, rsp[0..32]={:02x?}",
                name,
                response[0],
                &response[..32]
            );
            return Err(UfsError::Other("Invalid SCSI response"));
        }

        data_buf.complete_for_cpu(0, data_len);
        Ok((response, data_buf))
    }

    /// Issue a SCSI command without a data phase and return the response UPIU.
    fn scsi_nodata_command(
        &mut self,
        name: &'static str,
        cdb: &[u8],
    ) -> Result<[u8; 512], UfsError> {
        let mut upiu = [0u8; 512];
        build_scsi_upiu(&mut upiu, 0, self.active_lun, UPIU_CMD_FLAGS_NONE, cdb, 0);

        let response =
            self.submit_upiu(&upiu, None, 0, DataDirection::NoData, UpiuSlotKind::ScsiIo)?;
        if response[0] != UPIU_TRANSACTION_RESPONSE {
            warn!(
                "[k3-ufs] {} invalid response transaction=0x{:02x}, rsp[0..32]={:02x?}",
                name,
                response[0],
                &response[..32]
            );
            return Err(UfsError::Other("Invalid SCSI response"));
        }
        Ok(response)
    }

    fn test_unit_ready(&mut self) -> Result<(), UfsError> {
        info!(
            "[k3-ufs] Sending TEST_UNIT_READY on UPIU LUN 0x{:02x}...",
            self.active_lun
        );
        let mut cdb = [0u8; 16];
        cdb[0] = SCSI_TEST_UNIT_READY;

        for retry in 0..SCSI_CMD_RETRIES {
            let response = self.scsi_nodata_command("TEST_UNIT_READY", &cdb)?;
            if response[7] == SAM_STAT_GOOD {
                info!("[k3-ufs] TEST_UNIT_READY OK");
                return Ok(());
            }

            self.log_scsi_failure("TEST_UNIT_READY", &response);
            let sense = self.check_condition_sense("TEST_UNIT_READY", &response);

            if let Some((key, asc, ascq)) = sense
                && Self::sense_is_retryable(Some((key, asc, ascq)))
            {
                warn!(
                    "[k3-ufs] TEST_UNIT_READY retry {} due to sense key=0x{:02x}, asc=0x{:02x}, \
                     ascq=0x{:02x}",
                    retry + 1,
                    key,
                    asc,
                    ascq
                );
                axklib::time::busy_wait(Duration::from_millis(SCSI_RETRY_BACKOFF_MS));
                continue;
            }

            return Err(UfsError::Other("TEST_UNIT_READY failed"));
        }

        Err(UfsError::Other("TEST_UNIT_READY timeout"))
    }

    /// SCSI INQUIRY (Linux SCSI layer).
    fn scsi_inquiry(&mut self) -> Result<(), UfsError> {
        info!(
            "[k3-ufs] Sending SCSI INQUIRY on UPIU LUN 0x{:02x}...",
            self.active_lun
        );

        let mut cdb = [0u8; 16];
        cdb[0] = SCSI_INQUIRY;
        cdb[4] = SCSI_INQUIRY_ALLOC_LEN;
        let (response, data_buf) = self.scsi_read_command("INQUIRY", &cdb, 36)?;

        if response[0] != UPIU_TRANSACTION_RESPONSE {
            return Err(UfsError::Other("Invalid INQUIRY response"));
        }

        let status = response[7];
        if status != SAM_STAT_GOOD {
            self.log_scsi_failure("INQUIRY", &response);
            return Err(UfsError::Other("INQUIRY failed"));
        }

        // SAFETY: `data_buf` is a DMA allocation of exactly 36 bytes, already
        // synced to the CPU by `scsi_read_command`; the slice length matches
        // the allocation.
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

    /// READ CAPACITY: try (10), fall back to (16) when the device reports a
    /// saturated last LBA, then normalize and validate the block size.
    fn scsi_read_capacity(&mut self) -> Result<(u64, u32), UfsError> {
        let (blocks, block_size) = match self.read_capacity(0, 8) {
            Ok((blocks, block_size)) if blocks != 0x1_0000_0000 => Ok((blocks, block_size)),
            Ok((_blocks, _block_size)) => {
                warn!(
                    "[k3-ufs] READ_CAPACITY(10) returned 0xffffffff last LBA, trying \
                     READ_CAPACITY(16)"
                );
                self.read_capacity(SAI_READ_CAPACITY_16, 32)
            }
            Err(e) => {
                warn!(
                    "[k3-ufs] READ_CAPACITY(10) failed: {}, trying READ_CAPACITY(16)",
                    e
                );
                self.read_capacity(SAI_READ_CAPACITY_16, 32)
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
            return Err(UfsError::Other("Unsupported sector size"));
        }

        Ok((blocks, block_size))
    }

    /// Issue READ_CAPACITY and return `(num_blocks, logical_block_size)`.
    ///
    /// `service_action` selects the variant: 0 runs READ_CAPACITY(10),
    /// otherwise READ_CAPACITY(16) with the given service action. `alloc_len`
    /// is both the CDB allocation length and the expected response size.
    fn read_capacity(
        &mut self,
        service_action: u8,
        alloc_len: u32,
    ) -> Result<(u64, u32), UfsError> {
        let (opcode, name) = if service_action == 0 {
            (SCSI_READ_CAPACITY_10, "READ_CAPACITY(10)")
        } else {
            (SCSI_SERVICE_ACTION_IN_16, "READ_CAPACITY(16)")
        };
        info!(
            "[k3-ufs] Sending {} on UPIU LUN 0x{:02x}...",
            name, self.active_lun
        );

        let cdb = build_read_capacity_cdb(opcode, service_action, alloc_len);
        let data = self.read_capacity_data(name, &cdb, alloc_len as usize)?;
        let (blocks, block_size) = parse_capacity(&data, alloc_len);
        info!(
            "[k3-ufs] {}: {} blocks x {} bytes",
            name, blocks, block_size
        );
        Ok((blocks, block_size))
    }

    /// Shared READ_CAPACITY fetch with a sense-driven retry loop.
    ///
    /// Retries on UNIT_ATTENTION / NOT_READY senses with a backoff, fails
    /// fast on ILLEGAL_REQUEST, and returns the response bytes zero-padded
    /// to 32.
    fn read_capacity_data(
        &mut self,
        name: &'static str,
        cdb: &[u8],
        alloc_len: usize,
    ) -> Result<[u8; 32], UfsError> {
        for retry in 0..SCSI_CMD_RETRIES {
            let (response, data_buf) = self.scsi_read_command(name, cdb, alloc_len)?;
            if response[7] == SAM_STAT_GOOD {
                let mut data = [0u8; 32];
                // SAFETY: `data_buf` is a DMA allocation of `alloc_len` bytes,
                // already synced to the CPU by `scsi_read_command`; the slice
                // length matches the allocation.
                let raw =
                    unsafe { core::slice::from_raw_parts(data_buf.as_ptr().as_ptr(), alloc_len) };
                data[..alloc_len].copy_from_slice(raw);
                return Ok(data);
            }

            self.log_scsi_failure(name, &response);
            let sense = self.check_condition_sense(name, &response);

            if let Some((key, asc, ascq)) = sense {
                if Self::sense_is_retryable(Some((key, asc, ascq))) {
                    warn!(
                        "[k3-ufs] {} retry {}/{} due to sense key=0x{:02x}, asc=0x{:02x}, \
                         ascq=0x{:02x}",
                        name,
                        retry + 1,
                        SCSI_CMD_RETRIES,
                        key,
                        asc,
                        ascq
                    );
                    axklib::time::busy_wait(Duration::from_millis(SCSI_RETRY_BACKOFF_MS));
                    continue;
                }

                if key == SCSI_SENSE_ILLEGAL_REQUEST {
                    return Err(UfsError::Other("READ_CAPACITY illegal request"));
                }
            }

            axklib::time::busy_wait(Duration::from_millis(50));
        }

        Err(UfsError::Other("READ_CAPACITY retry exhausted"))
    }

    /// SCSI REPORT LUNS (Linux: scsi_report_lun_scan in drivers/scsi/scsi_scan.c).
    fn report_luns(&mut self) -> Result<Vec<u64>, UfsError> {
        info!("[k3-ufs] Sending REPORT_LUNS...");

        let mut cdb = [0u8; 16];
        cdb[0] = SCSI_REPORT_LUNS;
        cdb[6..10].copy_from_slice(&(SCSI_REPORT_LUNS_ALLOC_LEN as u32).to_be_bytes());

        let (response, data_buf) =
            self.scsi_read_command("REPORT_LUNS", &cdb, SCSI_REPORT_LUNS_ALLOC_LEN)?;
        if response[7] != SAM_STAT_GOOD {
            self.log_scsi_failure("REPORT_LUNS", &response);
            return Err(UfsError::Other("REPORT_LUNS failed"));
        }

        // SAFETY: `data_buf` is a DMA allocation of `SCSI_REPORT_LUNS_ALLOC_LEN`
        // bytes, already synced to the CPU by `scsi_read_command`; the slice
        // length matches the allocation.
        let data = unsafe {
            core::slice::from_raw_parts(data_buf.as_ptr().as_ptr(), SCSI_REPORT_LUNS_ALLOC_LEN)
        };
        let list_len = u32::from_be_bytes([data[0], data[1], data[2], data[3]]) as usize;
        if list_len == 0 || list_len + 8 > SCSI_REPORT_LUNS_ALLOC_LEN || !list_len.is_multiple_of(8)
        {
            warn!("[k3-ufs] REPORT_LUNS invalid list length {}", list_len);
            return Err(UfsError::Other("Invalid REPORT_LUNS data"));
        }

        let mut luns = Vec::new();
        for entry in data[8..8 + list_len].as_chunks::<8>().0 {
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

    /// Probe one LUN and remember its capacity in the host state.
    fn configure_lun(&mut self, scsi_lun: u64) -> Result<(u64, u32), UfsError> {
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

    /// Select the data LUN: REPORT_LUNS, probe each regular LU, and prefer
    /// the first one that carries an MBR or GPT partition signature.
    pub(super) fn select_data_lun(&mut self) -> Result<(u64, u64, u32), UfsError> {
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

        Err(UfsError::Other("No usable LUN found"))
    }

    pub(super) fn scsi_read_10(
        &mut self,
        lba: u32,
        num_blocks: u16,
        buffer: &mut [u8],
    ) -> Result<(), UfsError> {
        let transfer_len = num_blocks as u32 * self.block_size as u32;
        if buffer.len() < transfer_len as usize {
            return Err(UfsError::Other("Buffer too small"));
        }

        let mut data_buf = self
            .dma
            .contiguous_array_zero_with_align::<u8>(
                transfer_len as usize,
                64,
                DmaDirection::FromDevice,
            )
            .map_err(|_| UfsError::Other("Failed to allocate data buffer"))?;

        let mut cdb = [0u8; 16];
        cdb[0] = SCSI_READ_10;
        cdb[2..6].copy_from_slice(&lba.to_be_bytes());
        cdb[7..9].copy_from_slice(&num_blocks.to_be_bytes());

        let mut upiu = [0u8; 512];
        build_scsi_upiu(
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
            DataDirection::ToHost,
            UpiuSlotKind::ScsiIo,
        )?;

        if response[0] != UPIU_TRANSACTION_RESPONSE || response[7] != SAM_STAT_GOOD {
            self.log_scsi_failure("READ_10", &response);
            return Err(UfsError::Other("READ_10 failed"));
        }

        data_buf.read_from_device(transfer_len as usize, |data| {
            buffer[..transfer_len as usize].copy_from_slice(data);
        });

        Ok(())
    }

    pub(super) fn scsi_write_10(
        &mut self,
        lba: u32,
        num_blocks: u16,
        buffer: &[u8],
    ) -> Result<(), UfsError> {
        let transfer_len = num_blocks as u32 * self.block_size as u32;
        if buffer.len() < transfer_len as usize {
            return Err(UfsError::Other("Buffer too small"));
        }

        let mut data_buf = self
            .dma
            .contiguous_array_zero_with_align::<u8>(
                transfer_len as usize,
                64,
                DmaDirection::ToDevice,
            )
            .map_err(|_| UfsError::Other("Failed to allocate data buffer"))?;
        data_buf.copy_to_device_from_slice(&buffer[..transfer_len as usize]);

        let mut cdb = [0u8; 16];
        cdb[0] = SCSI_WRITE_10;
        cdb[2..6].copy_from_slice(&lba.to_be_bytes());
        cdb[7..9].copy_from_slice(&num_blocks.to_be_bytes());

        let mut upiu = [0u8; 512];
        build_scsi_upiu(
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
            DataDirection::ToDevice,
            UpiuSlotKind::ScsiIo,
        )?;

        if response[0] != UPIU_TRANSACTION_RESPONSE || response[7] != SAM_STAT_GOOD {
            self.log_scsi_failure("WRITE_10", &response);
            return Err(UfsError::Other("WRITE_10 failed"));
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scsi_to_upiu_lun_keeps_regular_lun_number() {
        assert_eq!(K3UfsHost::scsi_to_upiu_lun(0), 0);
        assert_eq!(K3UfsHost::scsi_to_upiu_lun(5), 5);
        assert_eq!(K3UfsHost::scsi_to_upiu_lun(0x20), 0x20);
    }

    #[test]
    fn scsi_to_upiu_lun_sets_wlun_bit_for_well_known_luns() {
        // REPORT LUNS reports the report-luns well-known LU as scsi_lun 0xC101.
        assert_eq!(K3UfsHost::scsi_to_upiu_lun(0xC101), 0x81);
    }

    #[test]
    fn scsilun_to_int_reverses_each_byte_pair() {
        // The 8-byte SCSI LUN structure stores each 16-bit field with the low
        // byte last, so the Linux decoder swaps each pair.
        let lun = [0x00, 0x12, 0x00, 0x34, 0x00, 0x00, 0x00, 0x00];
        assert_eq!(K3UfsHost::scsilun_to_int(&lun), 0x340012);
    }

    #[test]
    fn decode_fixed_sense_parses_valid_fixed_sense() {
        let sense = [0x70, 0, 0x06, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x3f, 0x03];
        assert_eq!(
            K3UfsHost::decode_fixed_sense(&sense),
            Some((0x06, 0x3f, 0x03))
        );
    }

    #[test]
    fn decode_fixed_sense_rejects_short_and_descriptor_sense() {
        assert_eq!(K3UfsHost::decode_fixed_sense(&[0x70; 13]), None);
        // 0x72 is a descriptor-format response code, not fixed format.
        assert_eq!(K3UfsHost::decode_fixed_sense(&[0x72; 14]), None);
    }

    #[test]
    fn sense_retry_policy_retries_transient_keys_only() {
        assert!(K3UfsHost::sense_is_retryable(Some((
            SCSI_SENSE_UNIT_ATTENTION,
            0,
            0
        ))));
        assert!(K3UfsHost::sense_is_retryable(Some((
            SCSI_SENSE_NOT_READY,
            0,
            0
        ))));
        assert!(!K3UfsHost::sense_is_retryable(Some((
            SCSI_SENSE_ILLEGAL_REQUEST,
            0,
            0
        ))));
        assert!(!K3UfsHost::sense_is_retryable(None));
    }

    #[test]
    fn parse_capacity_reads_read_capacity_10_layout() {
        let mut data = [0u8; 32];
        data[0..4].copy_from_slice(&0x0000_FFFFu32.to_be_bytes()); // last LBA
        data[4..8].copy_from_slice(&512u32.to_be_bytes());
        assert_eq!(parse_capacity(&data, 8), (0x1_0000, 512));
    }

    #[test]
    fn parse_capacity_reads_read_capacity_16_layout() {
        let mut data = [0u8; 32];
        data[0..8].copy_from_slice(&0x0000_0001_0000_0000u64.to_be_bytes());
        data[8..12].copy_from_slice(&4096u32.to_be_bytes());
        assert_eq!(parse_capacity(&data, 32), (0x0000_0001_0000_0001, 4096));
    }

    #[test]
    fn read_capacity_16_cdb_places_service_action_and_alloc_len() {
        let cdb = build_read_capacity_cdb(SCSI_SERVICE_ACTION_IN_16, SAI_READ_CAPACITY_16, 32);
        assert_eq!(cdb[0], 0x9E);
        assert_eq!(cdb[1], 0x10);
        assert_eq!(&cdb[10..14], &32u32.to_be_bytes());
    }

    #[test]
    fn read_capacity_10_cdb_uses_plain_opcode() {
        let cdb = build_read_capacity_cdb(SCSI_READ_CAPACITY_10, 0, 8);
        assert_eq!(cdb[0], 0x25);
        assert_eq!(cdb[1], 0);
        assert_eq!(&cdb[10..14], &[0u8; 4]);
    }
}
