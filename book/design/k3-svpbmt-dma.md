# SpacemiT K3 Svpbmt DMA memory attributes

## Problem and success criteria

The K3 USB xHCI command ring is allocated through `CoherentArray`, whose platform contract maps
the pages uncached. The K3 build enabled Zicbom cache-block operations but did not encode an
uncached memory type in either the boot or runtime RISC-V page-table entry. Consequently, the CPU
could retain a submitted command TRB in cache after the command doorbell was rung, leaving xHCI
waiting indefinitely for a command it could not observe.

The board DT advertises both `svpbmt` and `zicbom`. Completion requires the K3 build to preserve
those as separate capabilities:

- `svpbmt` encodes `Uncached` as PBMT `NC` and `Device` as PBMT `IO` in leaf PTEs;
- `zicbom` continues to provide explicit cache maintenance for streaming DMA buffers;
- the T-Head MAE page attributes and cache instructions remain disabled on K3;
- a regression test must fail when runtime RISC-V PTEs discard either memory type.

Non-goals are changing DMA ownership APIs, inferring CPU extensions from unrelated features, or
declaring every RISC-V platform to support Svpbmt.

## Architecture and alternatives

Svpbmt version 1.0 assigns leaf-PTE bits 62-61 to PBMT: `0` uses the physical memory attributes,
`1` requests non-cacheable main memory, and `2` requests strongly ordered I/O. K3 firmware
explicitly advertises the extension, so a dedicated feature carries that hardware contract through
`axplat-dyn` to both page-table owners: `someboot` for boot mappings and `ax-cpu` for runtime
stage-1 mappings. The firmware contract also includes enabling S-mode use through
`menvcfg.PBMTE`; S-mode cannot enable that M-mode control itself.

The rejected alternatives were:

- treating Zicbom as proof of Svpbmt, which conflates independent ISA extensions;
- enabling `thead-mae`, whose PTE and cache-instruction encodings are not the standard K3 ISA;
- flushing only the xHCI command TRB, which leaves transfer rings, event rings, contexts, and
  other coherent allocations with the same broken contract;
- converting xHCI coherent allocations to streaming buffers, which duplicates platform policy
  inside one driver and changes their ownership semantics.

## Validation and rollback

Host-side PTE tests compile the RISC-V implementation directly and verify round trips for PBMT
`NC` and `IO`. Cross-target checks cover feature propagation through the K3 build. Physical-board
validation must observe `EnableSlot`, `AddressDevice`, and the device-descriptor control transfer
completing after their doorbells.

Rollback is disabling `axplat-dyn/svpbmt` in the K3 OS feature together with this encoding change;
doing so restores the previous behavior but also restores the non-coherent DMA failure.

## References

- [RISC-V Privileged Architecture, Svpbmt version 1.0, Chapter 14](https://docs.riscv.org/reference/isa/_attachments/riscv-privileged.pdf).
- `os/StarryOS/configs/board/spacemit-k3-com260-ifx.dtb`, whose CPU nodes advertise `svpbmt` and
  `zicbom` and a 64-byte cache-block size.
