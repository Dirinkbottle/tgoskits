//! UTP transfer descriptors and UPIU build helpers.
//!
//! These plain data types and constructors describe the UTP Transfer Request
//! Descriptor (UTRD), Physical Region Description Table (PRDT) entry and UFS
//! Command Descriptor (UCD) layout, plus the UPIU header bytes that fill each
//! command descriptor. Everything here is pure data layout: MMIO access and
//! DMA ownership stay in the host (`K3UfsHost`) and its transfer path.

/// Transfer Request Descriptor command type.
///
/// Linux 6.18 uses UTP_CMD_TYPE_UFS_STORAGE for all transfer request
/// descriptors, including NOP OUT and QUERY device commands. Do not use the
/// older UTP_DEVICE_MANAGEMENT_FUNCTION full-DW value here; that produces
/// DW0=0x21000000 and K3 completes the slot without writing OCS/response.
pub(super) const UTP_CMD_TYPE_UFS_STORAGE: u8 = 0x01;

/// UTRD DW0 field layout (UFSHCI JEDEC JESD223C, Table 38).
/// The command type occupies bits 31:28, the data direction bits 26:25, and
/// the transfer-request interrupt bit 24.
pub(super) const UTP_CMD_TYPE_SHIFT: u32 = 28;
pub(super) const UTP_TRANSFER_REQ_DATA_DIR_SHIFT: u32 = 25;
pub(super) const UTP_TRANSFER_REQ_INT_SHIFT: u32 = 24;

/// UTRD DW0 data direction field values (Table 38).
pub(super) const UTP_DATA_DIR_NONE: u32 = 0x00;
pub(super) const UTP_DATA_DIR_TO_DEVICE: u32 = 0x01;
pub(super) const UTP_DATA_DIR_TO_HOST: u32 = 0x02;

/// UTRD DW2 Overall Command Status: field width and the "invalid" sentinel
/// written before submission.
pub(super) const OCS_MASK: u32 = 0x0F;
pub(super) const OCS_INVALID_COMMAND_STATUS: u32 = 0x0F;

/// Number of UTP transfer request slots this host programs (Linux UFSHCI_MAX_QUEUE 32).
pub(super) const UFSHCI_NUM_SLOTS: usize = 32;

/// Width (in bits) of the PRDT Data Byte Count field.
///
/// UFSHCI Table 42 defines `DBC` as a 20-bit field holding `byte_count - 1`,
/// so a single PRDT entry can describe at most 1 MiB of contiguous DMA.
pub(super) const PRDT_DBC_BITS: u32 = 20;
/// Maximum byte count a single PRDT entry can describe (`2^20`).
pub(super) const PRDT_MAX_BYTES: u32 = 1 << PRDT_DBC_BITS;

/// UCD (UFS Command Descriptor) layout, matching Linux ALIGNED_UPIU_SIZE = 512.
pub(super) const UCD_COMMAND_UPIU_SIZE: usize = 512;
pub(super) const UCD_RESPONSE_UPIU_SIZE: usize = 512;
pub(super) const UCD_PRDT_OFFSET: usize = UCD_COMMAND_UPIU_SIZE + UCD_RESPONSE_UPIU_SIZE;
pub(super) const UCD_PRDT_ENTRIES: usize = 128; // Linux SG_ALL default.
pub(super) const UCD_SLOT_SIZE: usize =
    UCD_PRDT_OFFSET + UCD_PRDT_ENTRIES * core::mem::size_of::<Prdt>();

/// Data direction of a UTP transfer (UFSHCI, Table 38, DT field).
///
/// Replaces the raw `u8` values callers previously had to remember: `NoData`
/// for device-management commands, `ToDevice` for writes and `ToHost` for
/// reads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum DataDirection {
    /// No data phase (NOP OUT, QUERY, TEST UNIT READY).
    NoData,
    /// Write: data moves CPU -> device.
    ToDevice,
    /// Read: data moves device -> CPU.
    ToHost,
}

impl DataDirection {
    /// Raw UTRD DW0 DT field value.
    const fn dt_field(self) -> u32 {
        match self {
            DataDirection::NoData => UTP_DATA_DIR_NONE,
            DataDirection::ToDevice => UTP_DATA_DIR_TO_DEVICE,
            DataDirection::ToHost => UTP_DATA_DIR_TO_HOST,
        }
    }

    /// Build the complete UTRD DW0: command type, data direction and the
    /// transfer-request interrupt bit.
    pub(super) const fn dw0(self) -> u32 {
        ((UTP_CMD_TYPE_UFS_STORAGE as u32) << UTP_CMD_TYPE_SHIFT)
            | (self.dt_field() << UTP_TRANSFER_REQ_DATA_DIR_SHIFT)
            | (1u32 << UTP_TRANSFER_REQ_INT_SHIFT)
    }
}

/// UTP Transfer Request Descriptor.
#[repr(C)]
pub(super) struct Utrd {
    pub(super) dw0: u32,
    pub(super) dw1: u32,
    pub(super) dw2: u32,
    pub(super) dw3: u32,
    pub(super) ucdba: u32,
    pub(super) ucdbau: u32,
    pub(super) rul: u16,
    pub(super) ruo: u16,
    pub(super) prdtl: u16,
    pub(super) prdto: u16,
}

/// Physical Region Description Table Entry.
#[repr(C, align(4))]
pub(super) struct Prdt {
    pub(super) dba: u32,
    pub(super) dbau: u32,
    pub(super) reserved: u32,
    pub(super) dbc: u32,
}

/// UFS Command Descriptor (UCD) - matches Linux ALIGNED_UPIU_SIZE = 512.
#[repr(C, align(128))]
pub(super) struct Ucd {
    pub(super) command_upiu: [u8; 512],
    pub(super) response_upiu: [u8; 512],
    pub(super) prdt: [Prdt; UCD_PRDT_ENTRIES],
}

/// Fill one PRDT entry from a DMA address and byte count.
///
/// The data byte count field stores `byte_count - 1` (UFSHCI, Table 42), so
/// the caller must pass a non-zero transfer length of at most
/// [`PRDT_MAX_BYTES`] (the DBC field width). Larger transfers need multiple
/// PRDT entries.
pub(super) fn fill_prdt(prdt: &mut Prdt, dma_addr: u64, byte_count: u32) {
    debug_assert!(byte_count != 0, "zero-length PRDT entry");
    debug_assert!(
        byte_count <= PRDT_MAX_BYTES,
        "PRDT entry exceeds 1 MiB DBC field: {byte_count}"
    );
    prdt.dba = (dma_addr & 0xFFFF_FFFF) as u32;
    prdt.dbau = (dma_addr >> 32) as u32;
    prdt.reserved = 0;
    prdt.dbc = byte_count - 1;
}

/// Read the Overall Command Status (OCS) field from a UTRD.
///
/// # Safety
///
/// - `utrd` must point to a valid, properly aligned `Utrd` within the UTP
///   transfer request list.
/// - The DMA memory must have been made visible to the CPU
///   (`complete_for_cpu`) before this read, and the caller must guarantee
///   that no concurrent aliasing access races with this volatile read.
pub(super) unsafe fn read_utrd_ocs(utrd: *const Utrd) -> u32 {
    // SAFETY: the preconditions are documented in the `# Safety` section.
    unsafe { core::ptr::read_volatile(core::ptr::addr_of!((*utrd).dw2)) & OCS_MASK }
}

/// UPIU (UFS Protocol Information Unit) types (Linux: include/ufs/ufs.h).
pub(super) const UPIU_TRANSACTION_NOP_OUT: u8 = 0x00;
pub(super) const UPIU_TRANSACTION_COMMAND: u8 = 0x01;
pub(super) const UPIU_TRANSACTION_NOP_IN: u8 = 0x20;
pub(super) const UPIU_TRANSACTION_RESPONSE: u8 = 0x21;
pub(super) const UPIU_TRANSACTION_QUERY_REQ: u8 = 0x16;
pub(super) const UPIU_TRANSACTION_QUERY_RSP: u8 = 0x36;

pub(super) const UFS_UPIU_MAX_UNIT_NUM_ID: u8 = 0x7F;
pub(super) const UFS_UPIU_WLUN_ID: u8 = 1 << 7;

/// UPIU command flags.
pub(super) const UPIU_CMD_FLAGS_NONE: u8 = 0x00;
pub(super) const UPIU_CMD_FLAGS_READ: u8 = 0x40;
pub(super) const UPIU_CMD_FLAGS_WRITE: u8 = 0x20;

/// QUERY opcodes (Linux: include/ufs/ufs.h enum query_opcode).
pub(super) const UPIU_QUERY_OPCODE_READ_FLAG: u8 = 0x5;
pub(super) const UPIU_QUERY_OPCODE_SET_FLAG: u8 = 0x6;

/// QUERY function codes (Linux: include/ufs/ufs.h).
pub(super) const UPIU_QUERY_FUNC_STANDARD_READ_REQUEST: u8 = 0x01;
pub(super) const UPIU_QUERY_FUNC_STANDARD_WRITE_REQUEST: u8 = 0x81;

/// QUERY flag idn (Linux: include/ufs/ufs.h enum flag_idn).
pub(super) const QUERY_FLAG_IDN_FDEVICEINIT: u8 = 0x01;

/// Build a NOP OUT UPIU (Linux: ufshcd_prepare_utp_nop_upiu, ufshcd.c:2851).
pub(super) fn build_nop_upiu(upiu: &mut [u8; 512], task_tag: u8) {
    upiu.fill(0);
    upiu[0] = UPIU_TRANSACTION_NOP_OUT;
    upiu[3] = task_tag;
}

/// Build a QUERY REQUEST UPIU (Linux: ufshcd_prepare_utp_query_req_upiu, ufshcd.c:2820).
///
/// Note: the request value is always 0; only the response contains the actual
/// value.
pub(super) fn build_query_upiu(
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

/// Build a SCSI command UPIU (Linux: ufshcd_prepare_utp_scsi_cmd_upiu, ufshcd.c:2780).
pub(super) fn build_scsi_upiu(
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn data_direction_builds_expected_dw0() {
        assert_eq!(DataDirection::NoData.dw0(), 0x1100_0000);
        assert_eq!(DataDirection::ToDevice.dw0(), 0x1300_0000);
        assert_eq!(DataDirection::ToHost.dw0(), 0x1500_0000);
    }

    #[test]
    fn fill_prdt_encodes_address_and_byte_count_minus_one() {
        let mut prdt = Prdt {
            dba: 0,
            dbau: 0,
            reserved: 0,
            dbc: 0,
        };
        fill_prdt(&mut prdt, 0x1_0000_2000, 4096);
        assert_eq!(prdt.dba, 0x0000_2000);
        assert_eq!(prdt.dbau, 1);
        assert_eq!(prdt.reserved, 0);
        assert_eq!(prdt.dbc, 4095);
    }

    #[test]
    fn fill_prdt_accepts_maximum_single_entry_byte_count() {
        let mut prdt = Prdt {
            dba: 0,
            dbau: 0,
            reserved: 0,
            dbc: 0,
        };
        fill_prdt(&mut prdt, 0, PRDT_MAX_BYTES);
        assert_eq!(prdt.dbc, PRDT_MAX_BYTES - 1);
    }

    #[test]
    #[should_panic(expected = "exceeds 1 MiB")]
    fn fill_prdt_rejects_oversized_byte_count() {
        let mut prdt = Prdt {
            dba: 0,
            dbau: 0,
            reserved: 0,
            dbc: 0,
        };
        fill_prdt(&mut prdt, 0, PRDT_MAX_BYTES + 1);
    }

    #[test]
    fn scsi_upiu_places_header_lun_and_cdb() {
        let mut upiu = [0u8; 512];
        let cdb = [0x28, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0];
        build_scsi_upiu(&mut upiu, 3, 0x05, UPIU_CMD_FLAGS_READ, &cdb, 512);
        assert_eq!(upiu[0], UPIU_TRANSACTION_COMMAND);
        assert_eq!(upiu[1], UPIU_CMD_FLAGS_READ);
        assert_eq!(upiu[2], 0x05);
        assert_eq!(upiu[3], 3);
        assert_eq!(&upiu[12..16], &512u32.to_be_bytes());
        assert_eq!(&upiu[16..32], &cdb[..16]);
        assert_eq!(&upiu[32..], &[0u8; 480][..]);
    }

    #[test]
    fn scsi_upiu_truncates_oversized_cdb_to_16_bytes() {
        let mut upiu = [0u8; 512];
        let cdb = [0xaa; 32];
        build_scsi_upiu(&mut upiu, 0, 0, UPIU_CMD_FLAGS_NONE, &cdb, 0);
        assert_eq!(&upiu[16..32], &cdb[..16]);
    }

    #[test]
    fn query_upiu_fills_query_structure() {
        let mut upiu = [0u8; 512];
        build_query_upiu(
            &mut upiu,
            9,
            UPIU_QUERY_FUNC_STANDARD_READ_REQUEST,
            UPIU_QUERY_OPCODE_READ_FLAG,
            0x01,
        );
        assert_eq!(upiu[0], UPIU_TRANSACTION_QUERY_REQ);
        assert_eq!(upiu[3], 9);
        assert_eq!(upiu[5], UPIU_QUERY_FUNC_STANDARD_READ_REQUEST);
        assert_eq!(upiu[12], UPIU_QUERY_OPCODE_READ_FLAG);
        assert_eq!(upiu[13], 0x01);
    }

    #[test]
    fn nop_upiu_sets_transaction_type_and_tag() {
        let mut upiu = [0u8; 512];
        build_nop_upiu(&mut upiu, 4);
        assert_eq!(upiu[0], UPIU_TRANSACTION_NOP_OUT);
        assert_eq!(upiu[3], 4);
    }
}
