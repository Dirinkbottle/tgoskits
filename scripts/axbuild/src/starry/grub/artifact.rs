use std::{fs, path::Path, process::Command};

use anyhow::{Context, bail};
use object::{Object, ObjectKind};

use super::config::GrubPackageConfig;
use crate::support::process::ProcessExt;

const ELF_CLASS_64: u8 = 2;
const ELF_DATA_LITTLE_ENDIAN: u8 = 1;
const ELF_TYPE_DYNAMIC: u16 = 3;
const ELF_MACHINE_RISCV: u16 = 243;
const ELF_PROGRAM_HEADER_TLS: u32 = 7;
const PE_MACHINE_RISCV64: u16 = 0x5064;
const PE_OPTIONAL_HEADER_64: u16 = 0x20b;
const PE_SUBSYSTEM_EFI_APPLICATION: u16 = 10;
const PE_SECTION_EXECUTABLE: u32 = 0x2000_0000;
const PE_SECTION_WRITABLE: u32 = 0x8000_0000;

pub(super) fn validate_kernel_elf(path: &Path) -> anyhow::Result<()> {
    let bytes = fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    let file = object::File::parse(&*bytes)
        .with_context(|| format!("failed to parse kernel ELF {}", path.display()))?;
    if file.kind() != ObjectKind::Dynamic {
        bail!("Starry GRUB kernel ELF must be ET_DYN: {}", path.display());
    }
    for forbidden in [".tdata", ".tbss"] {
        if file.section_by_name(forbidden).is_some() {
            bail!(
                "Starry GRUB kernel ELF contains forbidden TLS section {forbidden}: {}",
                path.display()
            );
        }
    }
    validate_elf64_header(&bytes)
        .with_context(|| format!("invalid Starry GRUB ELF {}", path.display()))
}

pub(super) fn stage_kernel_and_dtb(
    kernel_elf: &Path,
    build_config_path: &Path,
    config: &GrubPackageConfig,
    staging_dir: &Path,
) -> anyhow::Result<()> {
    let kernel_path = staging_dir.join("starryos.efi");
    Command::new("rust-objcopy")
        .arg("--strip-all")
        .arg("-O")
        .arg("binary")
        .arg(kernel_elf)
        .arg(&kernel_path)
        .exec()
        .with_context(|| format!("failed to create {}", kernel_path.display()))?;
    let kernel = fs::read(&kernel_path)
        .with_context(|| format!("failed to read {}", kernel_path.display()))?;
    validate_pe_image(&kernel)
        .with_context(|| format!("invalid RISC-V EFI image {}", kernel_path.display()))?;

    let dts_path = build_config_path
        .parent()
        .context("Starry build config has no parent directory")?
        .join(&config.dts);
    if !dts_path.is_file() {
        bail!("GRUB DTS does not exist: {}", dts_path.display());
    }
    let dtb_path = staging_dir.join("spacemit-k3-com260-ifx.dtb");
    Command::new("dtc")
        .args(["-q", "-I", "dts", "-O", "dtb", "-o"])
        .arg(&dtb_path)
        .arg(&dts_path)
        .exec()
        .with_context(|| format!("failed to compile {}", dts_path.display()))?;
    if fs::metadata(&dtb_path)?.len() == 0 {
        bail!("dtc produced an empty DTB: {}", dtb_path.display());
    }
    Ok(())
}

fn validate_elf64_header(bytes: &[u8]) -> anyhow::Result<()> {
    if bytes.len() < 64 || &bytes[..4] != b"\x7fELF" {
        bail!("missing ELF magic or truncated ELF64 header");
    }
    if bytes[4] != ELF_CLASS_64 || bytes[5] != ELF_DATA_LITTLE_ENDIAN {
        bail!("kernel must be a little-endian ELF64 image");
    }
    if read_u16(bytes, 16)? != ELF_TYPE_DYNAMIC {
        bail!("kernel ELF type is not ET_DYN");
    }
    if read_u16(bytes, 18)? != ELF_MACHINE_RISCV {
        bail!("kernel ELF machine is not RISC-V");
    }

    let program_offset = usize::try_from(read_u64(bytes, 32)?)
        .context("ELF program-header offset does not fit this host")?;
    let entry_size = read_u16(bytes, 54)? as usize;
    let entry_count = read_u16(bytes, 56)? as usize;
    if entry_count != 0 && entry_size < 56 {
        bail!("ELF64 program-header entry is too small");
    }
    for index in 0..entry_count {
        let offset = program_offset
            .checked_add(
                index
                    .checked_mul(entry_size)
                    .context("ELF program-header overflow")?,
            )
            .context("ELF program-header overflow")?;
        if read_u32(bytes, offset)? == ELF_PROGRAM_HEADER_TLS {
            bail!("kernel ELF contains a PT_TLS program header");
        }
    }
    Ok(())
}

fn validate_pe_image(bytes: &[u8]) -> anyhow::Result<()> {
    if bytes.get(..2) != Some(b"MZ") {
        bail!("missing DOS MZ signature");
    }
    let pe_offset = read_u32(bytes, 0x3c)? as usize;
    if image_field(bytes, pe_offset, 4)? != b"PE\0\0" {
        bail!("invalid PE signature offset or signature");
    }

    let coff = pe_offset.checked_add(4).context("PE header overflow")?;
    if read_u16(bytes, coff)? != PE_MACHINE_RISCV64 {
        bail!("PE machine is not RISC-V64 (0x5064)");
    }
    let section_count = read_u16(bytes, coff + 2)? as usize;
    if section_count != 2 {
        bail!("RISC-V EFI image must contain exactly two PE sections");
    }
    let optional_size = read_u16(bytes, coff + 16)? as usize;
    let optional = coff
        .checked_add(20)
        .context("PE optional-header offset overflow")?;
    if optional_size < 70 || read_u16(bytes, optional)? != PE_OPTIONAL_HEADER_64 {
        bail!("PE optional header is not PE32+");
    }
    let optional_end = optional
        .checked_add(optional_size)
        .context("PE optional-header overflow")?;
    if optional_end > bytes.len() {
        bail!("PE optional header extends past the image");
    }

    let entry = read_u32(bytes, optional + 16)?;
    let image_base = read_u64(bytes, optional + 24)?;
    let section_alignment = read_u32(bytes, optional + 32)?;
    let file_alignment = read_u32(bytes, optional + 36)?;
    let image_size = read_u32(bytes, optional + 56)?;
    let header_size = read_u32(bytes, optional + 60)?;
    let subsystem = read_u16(bytes, optional + 68)?;
    if entry == 0 || entry >= image_size {
        bail!("PE entry point is outside the image");
    }
    if image_base != 0 {
        bail!("RISC-V EFI image base must be zero");
    }
    if section_alignment != 0x1000 || file_alignment != 0x200 {
        bail!("RISC-V EFI image must use 4 KiB section and 512-byte file alignment");
    }
    if subsystem != PE_SUBSYSTEM_EFI_APPLICATION {
        bail!("PE subsystem is not EFI Application");
    }
    if header_size == 0 || header_size as usize > bytes.len() {
        bail!("PE header size is outside the file");
    }

    let section_table = optional_end;
    let mut entry_is_executable = false;
    for index in 0..section_count {
        let offset = section_table
            .checked_add(index.checked_mul(40).context("PE section-table overflow")?)
            .context("PE section-table overflow")?;
        let name = image_field(bytes, offset, 8).context("truncated PE section name")?;
        let virtual_size = read_u32(bytes, offset + 8)?;
        let virtual_address = read_u32(bytes, offset + 12)?;
        let raw_size = read_u32(bytes, offset + 16)?;
        let raw_offset = read_u32(bytes, offset + 20)?;
        let characteristics = read_u32(bytes, offset + 36)?;
        let raw_end = raw_offset
            .checked_add(raw_size)
            .context("PE section raw range overflow")? as usize;
        if raw_end > bytes.len() {
            bail!("PE section raw range extends past the file");
        }
        let virtual_end = virtual_address
            .checked_add(virtual_size.max(raw_size))
            .context("PE section virtual range overflow")?;
        if (virtual_address..virtual_end).contains(&entry)
            && characteristics & PE_SECTION_EXECUTABLE != 0
        {
            entry_is_executable = true;
        }

        match trim_section_name(name) {
            b".text" if characteristics & PE_SECTION_EXECUTABLE != 0 => {}
            b".data" if characteristics & PE_SECTION_WRITABLE != 0 => {}
            b".text" => bail!("PE .text section is not executable"),
            b".data" => bail!("PE .data section is not writable"),
            other => bail!(
                "unexpected PE section name {:?}",
                String::from_utf8_lossy(other)
            ),
        }
    }
    if !entry_is_executable {
        bail!("PE entry point is not inside an executable section");
    }
    Ok(())
}

fn trim_section_name(name: &[u8]) -> &[u8] {
    let end = name
        .iter()
        .position(|byte| *byte == 0)
        .unwrap_or(name.len());
    &name[..end]
}

fn read_u16(bytes: &[u8], offset: usize) -> anyhow::Result<u16> {
    let value = image_field(bytes, offset, 2).context("truncated 16-bit image field")?;
    Ok(u16::from_le_bytes([value[0], value[1]]))
}

fn read_u32(bytes: &[u8], offset: usize) -> anyhow::Result<u32> {
    let value = image_field(bytes, offset, 4).context("truncated 32-bit image field")?;
    Ok(u32::from_le_bytes([value[0], value[1], value[2], value[3]]))
}

fn read_u64(bytes: &[u8], offset: usize) -> anyhow::Result<u64> {
    let value = image_field(bytes, offset, 8).context("truncated 64-bit image field")?;
    Ok(u64::from_le_bytes([
        value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7],
    ]))
}

fn image_field(bytes: &[u8], offset: usize, size: usize) -> anyhow::Result<&[u8]> {
    let end = offset
        .checked_add(size)
        .context("image field offset overflow")?;
    bytes.get(offset..end).context("image field is truncated")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accepts_riscv64_pe32_plus_application_with_two_sections() {
        validate_pe_image(&valid_pe_image()).unwrap();
    }

    #[test]
    fn rejects_wrong_machine_subsystem_and_non_executable_entry() {
        let mut wrong_machine = valid_pe_image();
        write_u16(&mut wrong_machine, 0x44, 0x8664);
        assert!(validate_pe_image(&wrong_machine).is_err());

        let mut wrong_subsystem = valid_pe_image();
        write_u16(&mut wrong_subsystem, 0x40 + 24 + 68, 3);
        assert!(validate_pe_image(&wrong_subsystem).is_err());

        let mut non_executable = valid_pe_image();
        let text_characteristics = 0x40 + 24 + 0xa0 + 36;
        write_u32(&mut non_executable, text_characteristics, 0x4000_0040);
        assert!(validate_pe_image(&non_executable).is_err());
    }

    fn valid_pe_image() -> Vec<u8> {
        let mut image = vec![0; 0x600];
        image[..2].copy_from_slice(b"MZ");
        write_u32(&mut image, 0x3c, 0x40);
        image[0x40..0x44].copy_from_slice(b"PE\0\0");
        let coff = 0x44;
        write_u16(&mut image, coff, PE_MACHINE_RISCV64);
        write_u16(&mut image, coff + 2, 2);
        write_u16(&mut image, coff + 16, 0xa0);
        write_u16(&mut image, coff + 18, 0x0206);
        let optional = coff + 20;
        write_u16(&mut image, optional, PE_OPTIONAL_HEADER_64);
        write_u32(&mut image, optional + 16, 0x1000);
        write_u32(&mut image, optional + 20, 0x1000);
        write_u64(&mut image, optional + 24, 0);
        write_u32(&mut image, optional + 32, 0x1000);
        write_u32(&mut image, optional + 36, 0x200);
        write_u32(&mut image, optional + 56, 0x3000);
        write_u32(&mut image, optional + 60, 0x200);
        write_u16(&mut image, optional + 68, PE_SUBSYSTEM_EFI_APPLICATION);

        let text = optional + 0xa0;
        image[text..text + 8].copy_from_slice(b".text\0\0\0");
        write_u32(&mut image, text + 8, 0x200);
        write_u32(&mut image, text + 12, 0x1000);
        write_u32(&mut image, text + 16, 0x200);
        write_u32(&mut image, text + 20, 0x200);
        write_u32(&mut image, text + 36, 0x6000_0020);

        let data = text + 40;
        image[data..data + 8].copy_from_slice(b".data\0\0\0");
        write_u32(&mut image, data + 8, 0x1000);
        write_u32(&mut image, data + 12, 0x2000);
        write_u32(&mut image, data + 16, 0x200);
        write_u32(&mut image, data + 20, 0x400);
        write_u32(&mut image, data + 36, 0xc000_0040);
        image
    }

    fn write_u16(bytes: &mut [u8], offset: usize, value: u16) {
        bytes[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
    }

    fn write_u32(bytes: &mut [u8], offset: usize, value: u32) {
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn write_u64(bytes: &mut [u8], offset: usize, value: u64) {
        bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }
}
