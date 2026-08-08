//! DWMAC4 descriptor helpers.

pub use super::generated::bindings::dma_desc as DmaDesc;
use super::{generated::bindings::dma_desc, regs};

pub const TDES3_OWN: u32 = super::generated::bindings::k3gmac_generated_values_K3GMAC_TDES3_OWN;
pub const RDES3_OWN: u32 = super::generated::bindings::k3gmac_generated_values_K3GMAC_RDES3_OWN;

const TDES2_BUFFER1_SIZE_MASK: u32 =
    super::generated::bindings::k3gmac_generated_values_K3GMAC_TDES2_BUFFER1_SIZE_MASK;
const TDES3_PACKET_SIZE_MASK: u32 =
    super::generated::bindings::k3gmac_generated_values_K3GMAC_TDES3_PACKET_SIZE_MASK;
const TDES3_CHECKSUM_INSERTION_MASK: u32 =
    super::generated::bindings::k3gmac_generated_values_K3GMAC_TDES3_CHECKSUM_INSERTION_MASK;
const TDES3_CHECKSUM_INSERTION_SHIFT: u32 =
    super::generated::bindings::TDES3_CHECKSUM_INSERTION_SHIFT;
const TDES3_FIRST_DESCRIPTOR: u32 =
    super::generated::bindings::k3gmac_generated_values_K3GMAC_TDES3_FIRST_DESCRIPTOR;
const TDES3_LAST_DESCRIPTOR: u32 =
    super::generated::bindings::k3gmac_generated_values_K3GMAC_TDES3_LAST_DESCRIPTOR;
const TDES2_INTERRUPT_ON_COMPLETION: u32 =
    super::generated::bindings::k3gmac_generated_values_K3GMAC_TDES2_INTERRUPT_ON_COMPLETION;
const TDES3_ERROR_SUMMARY: u32 =
    super::generated::bindings::k3gmac_generated_values_K3GMAC_TDES3_ERROR_SUMMARY;

const RDES3_PACKET_SIZE_MASK: u32 =
    super::generated::bindings::k3gmac_generated_values_K3GMAC_RDES3_PACKET_SIZE_MASK;
const RDES3_ERROR_SUMMARY: u32 =
    super::generated::bindings::k3gmac_generated_values_K3GMAC_RDES3_ERROR_SUMMARY;
const RDES3_LAST_DESCRIPTOR: u32 =
    super::generated::bindings::k3gmac_generated_values_K3GMAC_RDES3_LAST_DESCRIPTOR;
const RDES3_FIRST_DESCRIPTOR: u32 =
    super::generated::bindings::k3gmac_generated_values_K3GMAC_RDES3_FIRST_DESCRIPTOR;
const RDES3_BUFFER1_VALID_ADDR: u32 =
    super::generated::bindings::k3gmac_generated_values_K3GMAC_RDES3_BUFFER1_VALID_ADDR;
const RDES3_INT_ON_COMPLETION_EN: u32 =
    super::generated::bindings::k3gmac_generated_values_K3GMAC_RDES3_INT_ON_COMPLETION_EN;

const TX_CIC_FULL: u32 = super::generated::bindings::TX_CIC_FULL;

pub fn clear(desc: &mut dma_desc) {
    desc.des0 = 0;
    desc.des1 = 0;
    desc.des2 = 0;
    desc.des3 = 0;
}

pub fn set_addr(desc: &mut dma_desc, bus_addr: u64) {
    desc.des0 = bus_addr as u32;
    desc.des1 = (bus_addr >> 32) as u32;
}

pub fn prepare_rx(desc: &mut dma_desc, bus_addr: u64) {
    set_addr(desc, bus_addr);
    desc.des2 = 0;
    dma_wmb();
    desc.des3 = RDES3_OWN | RDES3_BUFFER1_VALID_ADDR | RDES3_INT_ON_COMPLETION_EN;
}

pub fn prepare_tx(desc: &mut dma_desc, bus_addr: u64, len: usize, checksum: bool) {
    let len = len.min(TDES2_BUFFER1_SIZE_MASK as usize) as u32;
    set_addr(desc, bus_addr);
    desc.des2 = len | TDES2_INTERRUPT_ON_COMPLETION;

    let mut des3 = len & TDES3_PACKET_SIZE_MASK;
    des3 |= TDES3_FIRST_DESCRIPTOR | TDES3_LAST_DESCRIPTOR;
    if checksum {
        des3 |= (TX_CIC_FULL << TDES3_CHECKSUM_INSERTION_SHIFT) & TDES3_CHECKSUM_INSERTION_MASK;
    }
    dma_wmb();
    desc.des3 = des3 | TDES3_OWN;
}

pub fn dma_wmb() {
    // Linux RISC-V: dma_wmb() = wmb() = fence ow,ow (arch/riscv/include/asm/barrier.h),
    // same helper as k3-ufs before ringing the UFS doorbell.
    unsafe {
        core::arch::asm!("fence ow, ow", options(nostack));
    }
}

pub fn dma_rmb() {
    // Linux RISC-V: dma_rmb() = rmb() = fence ir,ir; orders CPU reads of
    // device-owned descriptors/data after the ownership check.
    unsafe {
        core::arch::asm!("fence ir, ir", options(nostack));
    }
}

pub fn tx_owned(desc: &dma_desc) -> bool {
    (desc.des3 & TDES3_OWN) != 0
}

pub fn tx_has_error(desc: &dma_desc) -> bool {
    (desc.des3 & TDES3_ERROR_SUMMARY) != 0
}

pub fn rx_owned(desc: &dma_desc) -> bool {
    (desc.des3 & RDES3_OWN) != 0
}

pub fn rx_ready(desc: &dma_desc) -> bool {
    !rx_owned(desc)
        && (desc.des3 & RDES3_LAST_DESCRIPTOR) != 0
        && (desc.des3 & RDES3_FIRST_DESCRIPTOR) != 0
}

pub fn rx_len(desc: &dma_desc) -> usize {
    (desc.des3 & RDES3_PACKET_SIZE_MASK) as usize
}

pub fn rx_has_error(desc: &dma_desc) -> bool {
    (desc.des3 & RDES3_ERROR_SUMMARY) != 0
}

pub fn ring_offset(index: usize) -> u64 {
    (index * core::mem::size_of::<dma_desc>()) as u64
}

pub fn split_addr(addr: u64) -> (u32, u32) {
    (addr as u32, (addr >> 32) as u32)
}

pub fn mtl_fifo_words(bytes: u32) -> u32 {
    bytes.saturating_div(256).saturating_sub(1)
}

pub fn rx_buf_size_bits(size: usize) -> u32 {
    ((size as u32) << regs::DMA_RBSZ_SHIFT) & regs::DMA_RBSZ_MASK
}
