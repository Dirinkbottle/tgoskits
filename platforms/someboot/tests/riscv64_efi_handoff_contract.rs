#![cfg(all(
    target_os = "linux",
    target_pointer_width = "64",
    target_endian = "little"
))]

use std::{
    ffi::OsString,
    fs,
    path::{Path, PathBuf},
    process::Command,
    time::{SystemTime, UNIX_EPOCH},
};

const TARGET: &str = "riscv64gc-unknown-none-elf";
const EFI_ENTRY: &str = "<someboot::arch::Arch as someboot::ArchTrait>::efi_enter_kernel";
const EFI_WRAPPER: &str = "__riscv64_efi_pe_entry";
const EFI_CONTINUATION: &str = "someboot::arch::entry::enter_from_efi";
const RELOCATE: &str = "someboot::arch::relocate::apply";
const DIRECT_ENTRY: &str = "someboot::arch::entry::kernel_entry";
const DIRECT_RELOCATION: &str = "someboot::arch::entry::primary_head_entry";

#[test]
fn riscv64_efi_handoff_preserves_direct_boot_contract() {
    let temporary_directory = TemporaryDirectory::new();
    let archive = build_riscv64_efi_archive(temporary_directory.path());
    let disassembly = run_output(
        Command::new("rust-objdump")
            .args(["-dr", "-C"])
            .arg(&archive),
        "disassemble the RISC-V EFI someboot archive",
    );
    assert!(
        disassembly.contains("file format elf64-littleriscv"),
        "the contract must inspect a RISC-V target artifact"
    );

    let wrapper = function_disassembly(&disassembly, EFI_WRAPPER);
    for required in [
        "__global_pointer$",
        RELOCATE,
        "someboot::efi_stub::efi_pe_entry_main",
    ] {
        assert!(
            wrapper.contains(required),
            "the EFI wrapper must reference {required}:\n{wrapper}"
        );
    }
    assert_eq!(
        wrapper.matches(RELOCATE).count(),
        1,
        "the EFI wrapper must apply relocation exactly once:\n{wrapper}"
    );

    let efi_entry = function_disassembly(&disassembly, EFI_ENTRY);
    assert_symbol_order(
        efi_entry,
        "someboot::efi_stub::setup_service",
        "someboot::arch::efi::boot_hart_id",
    );
    assert_symbol_order(
        efi_entry,
        "someboot::arch::efi::boot_hart_id",
        EFI_CONTINUATION,
    );
    for forbidden in ["kernel_entry", "__bss_start", "__bss_stop"] {
        assert!(
            !efi_entry.contains(forbidden),
            "the EFI handoff must not reference {forbidden}:\n{efi_entry}"
        );
    }

    let continuation = function_disassembly(&disassembly, EFI_CONTINUATION);
    for required in ["__cpu0_stack_top", "primary_entry_from_efi"] {
        assert!(
            continuation.contains(required),
            "the EFI continuation must reference {required}:\n{continuation}"
        );
    }
    for forbidden in ["relocate::apply", "__bss_start", "__bss_stop"] {
        assert!(
            !continuation.contains(forbidden),
            "the EFI continuation must preserve populated state instead of referencing \
             {forbidden}:\n{continuation}"
        );
    }

    let direct_entry = function_disassembly(&disassembly, DIRECT_ENTRY);
    for required in [
        "__global_pointer$",
        "primary_head_entry",
        "__cpu0_stack_top",
    ] {
        assert!(
            direct_entry.contains(required),
            "direct SBI entry must retain {required}:\n{direct_entry}"
        );
    }
    let direct_relocation = function_disassembly(&disassembly, DIRECT_RELOCATION);
    assert_symbol_order(direct_relocation, RELOCATE, "primary_entry");
}

fn build_riscv64_efi_archive(target_directory: &Path) -> PathBuf {
    let cargo = std::env::var_os("CARGO").unwrap_or_else(|| OsString::from("cargo"));
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let workspace_manifest = manifest_dir
        .join("../..")
        .join("Cargo.toml")
        .canonicalize()
        .expect("workspace manifest must be available");
    run_checked(
        Command::new(cargo)
            .args([
                "build",
                "--quiet",
                "--package",
                "someboot",
                "--lib",
                "--target",
                TARGET,
                "--no-default-features",
                "--features",
                "efi",
                "--target-dir",
            ])
            .arg(target_directory)
            .arg("--manifest-path")
            .arg(workspace_manifest),
        "compile someboot for RISC-V EFI",
    );

    let dependency_directory = target_directory.join(TARGET).join("debug").join("deps");
    let mut archives = fs::read_dir(&dependency_directory)
        .expect("RISC-V dependency directory must be readable")
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.file_name()
                .and_then(|name| name.to_str())
                .is_some_and(|name| name.starts_with("libsomeboot-") && name.ends_with(".rlib"))
        })
        .collect::<Vec<_>>();
    assert_eq!(
        archives.len(),
        1,
        "the isolated target directory must contain one someboot archive"
    );
    archives.pop().expect("the someboot archive must exist")
}

fn assert_symbol_order(disassembly: &str, first: &str, second: &str) {
    let first_offset = disassembly
        .find(first)
        .unwrap_or_else(|| panic!("compiled function must reference {first}:\n{disassembly}"));
    let second_offset = disassembly
        .find(second)
        .unwrap_or_else(|| panic!("compiled function must reference {second}:\n{disassembly}"));
    assert!(
        first_offset < second_offset,
        "{first} must execute before {second}:\n{disassembly}"
    );
}

fn function_disassembly<'output>(output: &'output str, symbol: &str) -> &'output str {
    let label = format!("<{symbol}>:");
    let start = output
        .find(&label)
        .unwrap_or_else(|| panic!("compiled archive must define {symbol}"));
    let function = &output[start..];
    let remainder = &function[label.len()..];
    let end = remainder
        .find("\n0000000000000000 <")
        .unwrap_or(remainder.len());
    &function[..label.len() + end]
}

fn run_checked(command: &mut Command, operation: &str) {
    let output = command
        .output()
        .unwrap_or_else(|error| panic!("failed to {operation}: {error}"));
    assert!(
        output.status.success(),
        "failed to {operation}:\nstdout:\n{}\nstderr:\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

fn run_output(command: &mut Command, operation: &str) -> String {
    let output = command
        .output()
        .unwrap_or_else(|error| panic!("failed to {operation}: {error}"));
    assert!(
        output.status.success(),
        "failed to {operation}:\nstdout:\n{}\nstderr:\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    String::from_utf8(output.stdout).expect("tool output must be UTF-8")
}

struct TemporaryDirectory(PathBuf);

impl TemporaryDirectory {
    fn new() -> Self {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system clock must follow the Unix epoch")
            .as_nanos();
        let path = std::env::temp_dir().join(format!(
            "someboot-riscv64-efi-handoff-{}-{nonce}",
            std::process::id()
        ));
        fs::create_dir_all(&path).expect("temporary RISC-V target directory must be created");
        Self(path)
    }

    fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for TemporaryDirectory {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}
