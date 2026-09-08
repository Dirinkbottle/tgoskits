use core::{arch::naked_asm, mem::offset_of};

#[cfg(efi)]
use crate::efi_stub::{
    __riscv64_efi_pe_entry,
    pe::{
        IMAGE_FILE_DEBUG_STRIPPED, IMAGE_FILE_EXECUTABLE_IMAGE, IMAGE_FILE_LINE_NUMS_STRIPPED,
        IMAGE_FILE_MACHINE_RISCV64, IMAGE_NT_SIGNATURE, IMAGE_SUBSYSTEM_EFI_APPLICATION,
        LINUX_EFISTUB_MAJOR_VERSION, LINUX_EFISTUB_MINOR_VERSION,
    },
};
use crate::{entry::PrimaryCpuInitInfo, smp::PerCpuMeta};

const RISCV_LINUX_IMAGE_TEXT_OFFSET: usize = 0x20_0000;
const RISCV_LINUX_IMAGE_FLAGS: usize = 0;
const RISCV_LINUX_IMAGE_VERSION: usize = 0x0000_0002;
const RISCV_LINUX_IMAGE_MAGIC: usize = 0x0056_4353_4952;
const RISCV_LINUX_IMAGE_MAGIC2: usize = 0x0543_5352;

#[cfg(not(efi))]
#[unsafe(naked)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".head.text.000.header")]
pub unsafe extern "C" fn _head() -> ! {
    naked_asm!(
        ".option push",
        ".option norvc",
        ".option norelax",
        // The RISC-V Image header reserves exactly 8 bytes for code0/code1.
        "2: auipc t0, %pcrel_hi({kernel_entry})",
        "jalr zero, %pcrel_lo(2b)(t0)",
        ".option pop",
        // text_offset
        ".quad {text_offset}",
        // image_size
        ".quad __kernel_load_end - _head",
        // flags
        ".quad {flags}",
        // version + reserved
        ".word {version}",
        ".word 0",
        // reserved
        ".quad 0",
        // magic + magic2 + reserved
        ".quad {magic}",
        ".word {magic2}",
        ".word 0",
        ".global _efi_header_end",
        "_efi_header_end:",
        raw_entry = sym riscv64_raw_entry,
        text_offset = const RISCV_LINUX_IMAGE_TEXT_OFFSET,
        flags = const RISCV_LINUX_IMAGE_FLAGS,
        version = const RISCV_LINUX_IMAGE_VERSION,
        magic = const RISCV_LINUX_IMAGE_MAGIC,
        magic2 = const RISCV_LINUX_IMAGE_MAGIC2,
    )
}

#[cfg(efi)]
#[unsafe(naked)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".head.text.000.header")]
pub unsafe extern "C" fn _head() -> ! {
    naked_asm!(
        ".option push",
        ".option rvc",
        // This instruction encodes to the ASCII DOS signature "MZ" while
        // remaining executable by a direct RISC-V image loader.
        "c.li s4, -13",
        ".option norvc",
        "j {raw_entry}",
        ".balign 8",
        ".quad {text_offset}",
        ".quad __kernel_load_end - _head",
        ".quad {flags}",
        ".word {version}",
        ".word 0",
        ".quad 0",
        ".quad {magic}",
        ".word {magic2}",
        ".word 4f - _head",
        "4:",
        ".long {image_nt_signature}",
        ".short {file_machine}",
        ".short 2",
        ".long 0",
        ".long 0",
        ".long 0",
        ".short 2f - 1f",
        ".short {characteristics}",
        "1:",
        ".short 0x20b",
        ".byte 0x02",
        ".byte 0x14",
        ".long _kernel_code_size",
        ".long _kernel_rsize",
        ".long _kernel_bss_size",
        ".long {efi_entry} - _head",
        ".long _kernel_text_offset",
        ".quad 0",
        ".long PAGE_SIZE",
        ".long PECOFF_FILE_ALIGN",
        ".short 0",
        ".short 0",
        ".short {major_image_version}",
        ".short {minor_image_version}",
        ".short 0",
        ".short 0",
        ".long 0",
        ".long _kernel_image_size",
        ".long _kernel_text_offset",
        ".long 0",
        ".short {image_subsystem}",
        ".short 0",
        ".quad 0",
        ".quad 0",
        ".quad 0",
        ".quad 0",
        ".long 0",
        ".long (2f - .) / 8",
        ".quad 0",
        ".quad 0",
        ".quad 0",
        ".quad 0",
        ".quad 0",
        ".quad 0",
        "2:",
        ".ascii \".text\\0\\0\\0\"",
        ".long _kernel_code_size",
        ".long _kernel_text_offset",
        ".long _kernel_code_size",
        ".long _kernel_text_offset",
        ".long 0",
        ".long 0",
        ".short 0",
        ".short 0",
        ".long 0x60000020",
        ".ascii \".data\\0\\0\\0\"",
        ".long _kernel_vsize",
        ".long _etext - _head",
        ".long _kernel_rsize",
        ".long _etext - _head",
        ".long 0",
        ".long 0",
        ".short 0",
        ".short 0",
        ".long 0xc0000040",
        ".balign 0x1000",
        ".global _efi_header_end",
        "_efi_header_end:",
        ".option pop",
        raw_entry = sym riscv64_raw_entry,
        efi_entry = sym __riscv64_efi_pe_entry,
        text_offset = const RISCV_LINUX_IMAGE_TEXT_OFFSET,
        flags = const RISCV_LINUX_IMAGE_FLAGS,
        version = const RISCV_LINUX_IMAGE_VERSION,
        magic = const RISCV_LINUX_IMAGE_MAGIC,
        magic2 = const RISCV_LINUX_IMAGE_MAGIC2,
        image_nt_signature = const IMAGE_NT_SIGNATURE,
        file_machine = const IMAGE_FILE_MACHINE_RISCV64,
        characteristics = const IMAGE_FILE_DEBUG_STRIPPED
            | IMAGE_FILE_EXECUTABLE_IMAGE
            | IMAGE_FILE_LINE_NUMS_STRIPPED,
        major_image_version = const LINUX_EFISTUB_MAJOR_VERSION,
        minor_image_version = const LINUX_EFISTUB_MINOR_VERSION,
        image_subsystem = const IMAGE_SUBSYSTEM_EFI_APPLICATION,
    )
}

/// Position-independent direct-image trampoline shared by EFI-enabled and
/// ordinary RISC-V Linux Image headers.
#[unsafe(naked)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".head.text.100.raw_entry")]
pub unsafe extern "C" fn riscv64_raw_entry() -> ! {
    naked_asm!(
        ".option push",
        ".option norvc",
        // `lla` avoids the direct JAL range limit while retaining the incoming
        // SBI `a0`/`a1` arguments.
        "lla t0, {kernel_entry}",
        "jr t0",
        ".option pop",
        kernel_entry = sym kernel_entry,
    )
}

#[unsafe(naked)]
pub unsafe extern "C" fn kernel_entry(_hart_id: usize, _fdt_addr: usize) -> ! {
    naked_asm!(
        ".option push",
        ".option norelax",
        "lla gp, __global_pointer$",
        ".option pop",
        "mv t2, a1",
        "lla sp, __cpu0_stack_top",
        "addi sp, sp, -{boot_info_stack_size}",
        "li t0, {abi_magic}",
        "sw t0, {abi_magic_offset}(sp)",
        "li t0, {abi_version}",
        "sh t0, {abi_version_offset}(sp)",
        "li t0, {record_size}",
        "sh t0, {record_size_offset}(sp)",
        "sd a0, {hart_id_offset}(sp)",
        "sd zero, {cpu_meta_paddr_offset}(sp)",
        "sd zero, {early_trap_cause_offset}(sp)",
        "sd zero, {early_trap_pc_offset}(sp)",
        "sd zero, {early_trap_value_offset}(sp)",
        "csrw sscratch, sp",
        "mv tp, zero",
        "mv a0, t2",
        "lla t1, {primary_head_entry}",
        "jr t1",
        primary_head_entry = sym primary_head_entry,
        abi_magic = const super::boot::ABI_MAGIC_VALUE,
        abi_version = const super::boot::ABI_VERSION_VALUE,
        record_size = const super::boot::RECORD_SIZE,
        boot_info_stack_size = const super::boot::STACK_SIZE,
        abi_magic_offset = const super::boot::ABI_MAGIC_OFFSET,
        abi_version_offset = const super::boot::ABI_VERSION_OFFSET,
        record_size_offset = const super::boot::RECORD_SIZE_OFFSET,
        hart_id_offset = const super::boot::HART_ID_OFFSET,
        cpu_meta_paddr_offset = const super::boot::CPU_META_PADDR_OFFSET,
        early_trap_cause_offset = const super::boot::EARLY_TRAP_CAUSE_OFFSET,
        early_trap_pc_offset = const super::boot::EARLY_TRAP_PC_OFFSET,
        early_trap_value_offset = const super::boot::EARLY_TRAP_VALUE_OFFSET,
    )
}

/// Continues a relocated EFI boot without clearing EFI-owned Rust state.
///
/// # Safety
///
/// Relocations must already be applied, `hart_id` must identify the boot hart,
/// and `fdt_addr` must match the validated FDT saved by the EFI setup path.
#[cfg(efi)]
#[unsafe(naked)]
pub(super) unsafe extern "C" fn enter_from_efi(_hart_id: usize, _fdt_addr: usize) -> ! {
    naked_asm!(
        ".option push",
        ".option norelax",
        "lla gp, __global_pointer$",
        ".option pop",
        "lla sp, __cpu0_stack_top",
        "addi sp, sp, -{boot_info_stack_size}",
        "li t0, {abi_magic}",
        "sw t0, {abi_magic_offset}(sp)",
        "li t0, {abi_version}",
        "sh t0, {abi_version_offset}(sp)",
        "li t0, {record_size}",
        "sh t0, {record_size_offset}(sp)",
        "sd a0, {hart_id_offset}(sp)",
        "sd zero, {cpu_meta_paddr_offset}(sp)",
        "sd zero, {early_trap_cause_offset}(sp)",
        "sd zero, {early_trap_pc_offset}(sp)",
        "sd zero, {early_trap_value_offset}(sp)",
        "csrw sscratch, sp",
        "mv tp, zero",
        "lla t1, {primary_entry_from_efi}",
        "jr t1",
        primary_entry_from_efi = sym primary_entry_from_efi,
        abi_magic = const super::boot::ABI_MAGIC_VALUE,
        abi_version = const super::boot::ABI_VERSION_VALUE,
        record_size = const super::boot::RECORD_SIZE,
        boot_info_stack_size = const super::boot::STACK_SIZE,
        abi_magic_offset = const super::boot::ABI_MAGIC_OFFSET,
        abi_version_offset = const super::boot::ABI_VERSION_OFFSET,
        record_size_offset = const super::boot::RECORD_SIZE_OFFSET,
        hart_id_offset = const super::boot::HART_ID_OFFSET,
        cpu_meta_paddr_offset = const super::boot::CPU_META_PADDR_OFFSET,
        early_trap_cause_offset = const super::boot::EARLY_TRAP_CAUSE_OFFSET,
        early_trap_pc_offset = const super::boot::EARLY_TRAP_PC_OFFSET,
        early_trap_value_offset = const super::boot::EARLY_TRAP_VALUE_OFFSET,
    )
}

fn primary_head_entry(fdt_addr: usize) -> ! {
    super::relocate::apply();
    primary_entry(fdt_addr)
}

fn primary_entry(fdt_addr: usize) -> ! {
    clear_bss();
    unsafe {
        crate::fdt::FDT_ADDR = fdt_addr;
    }

    primary_entry_common("RISC-V64 SBI kernel entry.")
}

#[cfg(efi)]
fn primary_entry_from_efi() -> ! {
    primary_entry_common("RISC-V64 UEFI kernel entry.")
}

fn primary_entry_common(entry_message: &str) -> ! {
    early_trap_init();

    <<super::Arch as crate::ArchTrait>::Console as crate::console::ArchConsoleOps>::init();
    println!("{entry_message}");

    let kernel_code_start_lma = ext_sym_addr!(_head);
    let kernel_code_end_lma = ext_sym_addr!(__kernel_code_end);

    crate::entry::primary_init_early(PrimaryCpuInitInfo {
        kernel_start: kernel_code_start_lma.into(),
        kernel_end: kernel_code_end_lma.into(),
        kernel_start_link: crate::consts::VM_LOAD_ADDRESS.into(),
    });
    super::paging::enable_mmu()
}

/// Re-establishes the standard global pointer after entering the high virtual
/// mapping, before any shared Rust code observes the new address space.
///
/// # Safety
///
/// The caller must jump here only after mapping this function and
/// `__global_pointer$` at their linked virtual addresses and installing a
/// valid virtual kernel stack.
#[unsafe(naked)]
pub(crate) unsafe extern "C" fn mmu_entry() -> ! {
    naked_asm!(
        ".option push",
        ".option norelax",
        "lla gp, __global_pointer$",
        ".option pop",
        "lla t0, {mmu_entry_rust}",
        "jr t0",
        mmu_entry_rust = sym mmu_entry_rust,
    )
}

fn mmu_entry_rust() -> ! {
    super::relocate::reset();
    super::trap::setup();
    crate::prime_entry()
}

/// Re-establishes `gp` on the secondary CPU's high virtual mapping and then
/// tail-calls the shared secondary entry in `a1` with its metadata argument
/// recovered from the boot record in `a0`.
///
/// # Safety
///
/// The caller must provide mapped linked addresses, a valid virtual kernel
/// stack, the mapped boot record in `a0`, and the shared entry address in `a1`.
#[unsafe(naked)]
pub(crate) unsafe extern "C" fn secondary_mmu_entry(
    _cpu_boot_info_paddr: usize,
    _entry: usize,
) -> ! {
    naked_asm!(
        ".option push",
        ".option norelax",
        "lla gp, __global_pointer$",
        ".option pop",
        "ld a0, {cpu_meta_offset}(a0)",
        "jr a1",
        cpu_meta_offset = const super::boot::CPU_META_PADDR_OFFSET,
    )
}

unsafe extern "C" {
    fn __kernel_code_end();
    fn __bss_start();
    fn __bss_stop();
    fn __cpu0_stack();
    fn __cpu0_stack_top();
}

#[unsafe(naked)]
unsafe extern "C" fn secondary_early_trap() -> ! {
    naked_asm!(
        "csrr t3, sscratch",
        "csrr t0, scause",
        "csrr t1, sepc",
        "csrr t2, stval",
        "sd t0, {early_trap_cause_offset}(t3)",
        "sd t1, {early_trap_pc_offset}(t3)",
        "sd t2, {early_trap_value_offset}(t3)",
        "1:",
        "j 1b",
        early_trap_cause_offset = const super::boot::EARLY_TRAP_CAUSE_OFFSET,
        early_trap_pc_offset = const super::boot::EARLY_TRAP_PC_OFFSET,
        early_trap_value_offset = const super::boot::EARLY_TRAP_VALUE_OFFSET,
    )
}

#[unsafe(naked)]
pub(crate) unsafe extern "C" fn _secondary_entry(_hartid: usize, _cpu_meta_paddr: usize) -> ! {
    naked_asm!(
        "csrci sstatus, 2",
        "csrw sie, zero",
        ".option push",
        ".option norelax",
        "lla gp, __global_pointer$",
        ".option pop",
        "mv t2, a1",
        "ld sp, {stack_top_offset}(t2)",
        "addi sp, sp, -{boot_info_stack_size}",
        "li t0, {abi_magic}",
        "sw t0, {abi_magic_offset}(sp)",
        "li t0, {abi_version}",
        "sh t0, {abi_version_offset}(sp)",
        "li t0, {record_size}",
        "sh t0, {record_size_offset}(sp)",
        "sd a0, {hart_id_offset}(sp)",
        "sd t2, {cpu_meta_paddr_offset}(sp)",
        "sd zero, {early_trap_cause_offset}(sp)",
        "sd zero, {early_trap_pc_offset}(sp)",
        "sd zero, {early_trap_value_offset}(sp)",
        "csrw sscratch, sp",
        "lla t0, {secondary_early_trap}",
        "csrw stvec, t0",
        "mv tp, zero",
        "mv a0, sp",
        "lla t1, {secondary_start}",
        "jr t1",
        secondary_start = sym secondary_start,
        secondary_early_trap = sym secondary_early_trap,
        stack_top_offset = const offset_of!(PerCpuMeta, stack_top),
        abi_magic = const super::boot::ABI_MAGIC_VALUE,
        abi_version = const super::boot::ABI_VERSION_VALUE,
        record_size = const super::boot::RECORD_SIZE,
        boot_info_stack_size = const super::boot::STACK_SIZE,
        abi_magic_offset = const super::boot::ABI_MAGIC_OFFSET,
        abi_version_offset = const super::boot::ABI_VERSION_OFFSET,
        record_size_offset = const super::boot::RECORD_SIZE_OFFSET,
        hart_id_offset = const super::boot::HART_ID_OFFSET,
        cpu_meta_paddr_offset = const super::boot::CPU_META_PADDR_OFFSET,
        early_trap_cause_offset = const super::boot::EARLY_TRAP_CAUSE_OFFSET,
        early_trap_pc_offset = const super::boot::EARLY_TRAP_PC_OFFSET,
        early_trap_value_offset = const super::boot::EARLY_TRAP_VALUE_OFFSET,
    )
}

fn secondary_start(cpu_boot_info_paddr: usize) -> ! {
    super::paging::enable_mmu_secondary(cpu_boot_info_paddr)
}

fn clear_bss() {
    let start = __bss_start as *const () as usize;
    let end = __bss_stop as *const () as usize;
    let stack_start = __cpu0_stack as *const () as usize;
    let stack_end = __cpu0_stack_top as *const () as usize;

    clear_bss_range(start, stack_start.min(end));
    clear_bss_range(stack_end.max(start), end);
}

fn clear_bss_range(start: usize, end: usize) {
    if end <= start {
        return;
    }

    unsafe {
        core::ptr::write_bytes(start as *mut u8, 0, end - start);
    }
}

fn early_trap_init() {
    super::disable_local_irqs();
    let _ = super::sbi::set_timer(u64::MAX);

    if cfg!(efi) || crate::consts::VM_LOAD_ADDRESS == crate::consts::KERNEL_LOAD_ADDRESS {
        super::trap::setup();
    }
}
