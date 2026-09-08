# RISC-V StarryOS UEFI/GRUB Boot Contract

## Status and scope

This document defines the boot and packaging contract used to start the
SpacemiT K3 COM260 IFX StarryOS image from the board's existing GRUB2 EFI
installation. It is a high-risk boot-interface change because the same final
image must remain usable by both EFI firmware and direct SBI/Linux-image
loaders.

The immediate user is the K3 board workflow. The reusable part belongs in
`someboot`: RISC-V EFI entry, boot-hart discovery, relocation, and the hybrid
image header. Board paths, partition identifiers, and GRUB menu generation
remain StarryOS/`axbuild` configuration concerns.

This change does not add initrd support, Secure Boot signing, a shim, generic
EFI load-option parsing, or an alternative EFI boot manager.

## Problem and success criteria

The current K3 StarryOS binary has a RISC-V Linux Image header and can be
entered with the SBI convention (`a0 = hartid`, `a1 = FDT`). GRUB's EFI Linux
loader does not execute that plain image on RISC-V: it calls UEFI `LoadImage`,
which requires a valid RISC-V PE32+ EFI application.

The feature is complete when all of the following are true:

- one EFI-enabled binary is accepted as PE32+ RISC-V (`Machine = 0x5064`,
  `Subsystem = EFI Application`);
- the same binary still enters through the Linux Image/SBI convention;
- EFI entry applies dynamic relocations exactly once and establishes `gp`
  before Rust code;
- EFI-populated image handle, system table, and FDT state survive the handoff;
- the boot hart comes from the RISC-V EFI Boot Protocol, with an exact FDT
  `/chosen/boot-hartid` fallback and no guessed default;
- the K3 package command produces a validated EFI image, compiled DTB, GRUB
  menu fragment, and GRUB defaults fragment;
- QEMU covers both OVMF and direct `-kernel` entry, and the physical board can
  perform a one-shot GRUB boot before returning to Ubuntu.

## Prior art and normative inputs

The design follows these upstream interfaces, inspected on 2026-08-23:

- Linux RISC-V image entry and hybrid `MZ` instruction:
  <https://github.com/torvalds/linux/blob/master/arch/riscv/kernel/head.S>
- Linux RISC-V PE/COFF layout:
  <https://github.com/torvalds/linux/blob/master/arch/riscv/kernel/efi-header.S>
- RISC-V EFI Boot Protocol revision `0x00010000`:
  <https://github.com/riscv-non-isa/riscv-uefi/blob/main/boot_protocol.adoc>
- EDK2 protocol ABI (`EFI_STATUS GetBootHartId(This, UINTN *)`):
  <https://github.com/tianocore/edk2/blob/master/UefiCpuPkg/Include/Protocol/RiscVBootProtocol.h>
- Linux protocol-first, FDT-second boot-hart selection:
  <https://github.com/torvalds/linux/blob/master/drivers/firmware/efi/libstub/riscv.c>
- GRUB's EFI Linux loader, which delegates loading to UEFI:
  <https://github.com/rhboot/grub2/blob/master/grub-core/loader/efi/linux.c>

The implementation borrows wire-format and handoff semantics, not Linux code.
It keeps the existing TGOSKits `someboot` relocation, memory-map, FDT, paging,
and platform boundaries.

## Alternatives

| Alternative | Result | Decision |
| --- | --- | --- |
| Keep the plain Linux Image | GRUB EFI rejects it before StarryOS entry | Rejected |
| Install a separate chainloader | Adds a second boot component and update contract | Rejected |
| Produce separate direct and EFI kernels | Easier headers, but allows the two boot paths to drift | Rejected |
| Replace Ubuntu GRUB or EFI BootOrder | Unnecessary persistent board risk | Rejected |
| Add an EFI stub to the existing image | One artifact covers firmware and direct loaders | Selected |

The selected hybrid header costs one page of header space and an
architecture-specific entry wrapper. That cost is bounded and independently
validated by artifact checks.

## Image layout

When `someboot/efi` is enabled for RISC-V, offset zero contains the standard
64-byte RISC-V Linux Image header and also acts as a DOS header:

- bytes `0..2` are the compressed instruction `c.li s4, -13`, whose encoding
  is ASCII `MZ`;
- the following non-compressed jump enters the direct-image trampoline;
- the ordinary Linux Image fields stay at offsets `0x08..0x3b`;
- the word at `0x3c` points to the PE signature at `0x40`;
- the PE32+ optional header uses image base zero, 4 KiB section alignment, two
  sections, RISC-V64 machine `0x5064`, and EFI Application subsystem `10`;
- the PE header is padded to 4 KiB; executable raw-entry and Rust text follow;
- `.data` describes initialized data through `_edata` and virtual data through
  `_end`, so BSS is allocated but not copied from the file.

Without `someboot/efi`, the existing Linux Image header remains the externally
visible contract. Both variants enter the same direct trampoline and preserve
the SBI argument convention.

## Entry and state ownership

### Direct SBI path

1. The image-header jump reaches a naked, position-independent trampoline.
2. `kernel_entry` establishes `gp`, stack, boot record, `sscratch`, and `tp`.
3. The direct path applies RISC-V relative relocations.
4. It clears BSS once, records the incoming FDT, initializes early traps and
   console, then enters the existing memory/paging path.

### EFI path

1. Firmware enters the PE entry with image handle and system table.
2. A naked RISC-V wrapper establishes runtime `gp`, saves both EFI arguments
   on the firmware stack, and calls the existing relocation walker once.
3. The generic EFI setup saves the image handle/system table and discovers the
   firmware FDT before architecture handoff.
4. RISC-V obtains the boot hart. The standard protocol is authoritative when
   it exists and succeeds. Otherwise a structurally valid FDT property of
   exactly one 32-bit or 64-bit cell is accepted. Missing, malformed, or
   out-of-range data returns `EFI_UNSUPPORTED`.
5. The EFI-only continuation creates the same boot record and stack state as
   direct entry, but does not relocate, clear BSS, or overwrite the FDT.
6. The common early-memory path exits boot services with the existing
   retry/key discipline, treats the resulting EFI memory map as authoritative,
   reserves and maps only the page-rounded PE `SizeOfImage` extent, and copies
   the validated firmware FDT before ordinary kernel allocation can reuse its
   backing pages. GRUB may allocate that FDT exactly at the first byte after
   the image, so huge-page rounding must not claim it. A direct SBI entry
   instead builds its memory map from FDT.

No code calls UEFI Boot Services after a successful `ExitBootServices`.

## Package and GRUB contract

The public command is:

```bash
cargo xtask starry grub package --config spacemitk3-com260kit
```

The board TOML owns a `[grub]` table containing the DTS, installation path,
root label, menu ID, and title. The command first performs the ordinary Starry
build, then rejects an ELF that is not ET_DYN or contains TLS, rejects an
invalid PE image, compiles the DTS with `dtc`, and atomically stages:

```text
target/starry-grub/spacemitk3-com260kit/
  starryos.efi
  spacemit-k3-com260-ifx.dtb
  grub.d/42_starryos
  default-grub.d/60-starryos.cfg
```

The GRUB entry searches the Linux root filesystem by label `writable`, loads
the EFI application with `linux`, and supplies the repository DTB with
`devicetree`. The DTB is the authoritative source of StarryOS boot arguments;
no initrd or GRUB kernel command line is used.

The board installation never replaces `grubriscv64.efi` and never changes EFI
BootOrder. Ubuntu remains GRUB default entry zero; `grub-reboot` selects the
StarryOS menu ID for one boot only.

## Validation and rollback

The lowest-layer tests inspect a compiled RISC-V artifact, not only Rust source
text. They verify the EFI wrapper, relocation ordering, separate continuation,
and direct-entry preservation. Host packaging tests cover configuration,
wire-format validation, menu generation, and failure paths. Final artifacts
are also checked with `grub-file`, `file`, `rust-readobj`, `dtc`, and `fdtget`.

Two QEMU cases share one EFI-enabled build configuration: OVMF loading and the
direct `-kernel` path. Both must reach unique shell markers and fail on panic.

For a physical-board rollout, existing files are backed up before atomic
installation. A serial capture must be active before setting the one-shot GRUB
entry. Reset consumes or clears that one-shot selection and returns to Ubuntu.
Removing/restoring the three installed paths and rerunning `update-grub`
fully rolls back the menu without touching firmware state.
