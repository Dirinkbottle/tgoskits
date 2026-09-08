use core::{ffi::c_void, ptr::null_mut};

use uefi::{Guid, Status, guid};

const RISCV_EFI_BOOT_PROTOCOL_REVISION: u64 = 0x0001_0000;
const RISCV_EFI_BOOT_PROTOCOL_GUID: Guid = guid!("ccd15fec-6f73-4eec-8395-3e69e4b940bf");

type GetBootHartId =
    unsafe extern "efiapi" fn(this: *mut RiscvEfiBootProtocol, boot_hart_id: *mut usize) -> Status;

#[repr(C)]
struct RiscvEfiBootProtocol {
    revision: u64,
    get_boot_hart_id: GetBootHartId,
}

pub(super) fn boot_hart_id() -> Option<usize> {
    select_boot_hart_id(boot_hart_id_from_protocol(), crate::fdt::boot_hart_id)
}

fn select_boot_hart_id(
    protocol_hart_id: Option<usize>,
    fdt_hart_id: impl FnOnce() -> Option<usize>,
) -> Option<usize> {
    protocol_hart_id.or_else(fdt_hart_id)
}

fn boot_hart_id_from_protocol() -> Option<usize> {
    let system_table = uefi::table::system_table_raw()?;
    // SAFETY: the global system table was installed from the current EFI
    // entry. Boot Services remain active while the architecture handoff asks
    // for the mandatory RISC-V boot protocol.
    let boot_services = unsafe { system_table.as_ref().boot_services.as_ref()? };
    let mut interface = null_mut::<c_void>();
    // SAFETY: LocateProtocol writes either a null pointer or an interface that
    // follows the GUID-defined RISC-V EFI Boot Protocol ABI.
    let status = unsafe {
        (boot_services.locate_protocol)(&RISCV_EFI_BOOT_PROTOCOL_GUID, null_mut(), &mut interface)
    };
    if status != Status::SUCCESS || interface.is_null() {
        return None;
    }

    let protocol = interface.cast::<RiscvEfiBootProtocol>();
    // SAFETY: successful LocateProtocol returned an interface with the
    // protocol's C layout. Read the fixed revision field before invoking its
    // function pointer.
    if unsafe { (*protocol).revision } < RISCV_EFI_BOOT_PROTOCOL_REVISION {
        return None;
    }

    let mut hart_id = 0;
    // SAFETY: `protocol` remains firmware-owned and valid until boot services
    // exit; the output points to a writable machine word for the call.
    let status = unsafe { ((*protocol).get_boot_hart_id)(protocol, &mut hart_id) };
    (status == Status::SUCCESS).then_some(hart_id)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn protocol_revision_matches_riscv_uefi_version_one() {
        assert_eq!(RISCV_EFI_BOOT_PROTOCOL_REVISION, 0x0001_0000);
        assert_eq!(
            RISCV_EFI_BOOT_PROTOCOL_GUID,
            guid!("ccd15fec-6f73-4eec-8395-3e69e4b940bf")
        );
    }

    #[test]
    fn protocol_hart_is_authoritative_and_fdt_is_the_only_fallback() {
        assert_eq!(select_boot_hart_id(Some(7), || Some(3)), Some(7));
        assert_eq!(select_boot_hart_id(None, || Some(3)), Some(3));
        assert_eq!(select_boot_hart_id(None, || None), None);
    }
}
