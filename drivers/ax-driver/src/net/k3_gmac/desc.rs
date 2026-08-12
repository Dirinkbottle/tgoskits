//! DWMAC4 描述符格式与 TX/RX 辅助函数。
//!
//! DWMAC4/5 使用 4×u32（16 字节）描述符。CPU 写"读格式"并置 OWN 位交给 DMA，
//! DMA 完成后清 OWN 并回写状态。详见 Linux `dwmac4_descs.h`。

use super::regs;

/// DWMAC4 DMA 描述符（4×u32，小端，`#[repr(C)]` 保证内存布局）。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct DmaDesc {
    pub des0: u32, // buffer1 物理地址低 32 位（或回写状态）
    pub des1: u32, // buffer2 物理地址低 32 位
    pub des2: u32, // buffer1/2 size + 控制
    pub des3: u32, // OWN + FIRST/LAST/CIC/PACKET_SIZE（TX）或 OWN+BUF1V+IOC（RX）
}

// ---------------------------------------------------------------------------
// TX 描述符位域（dwmac4_descs.h TDES2/TDES3）
// ---------------------------------------------------------------------------
const TDES2_BUFFER1_SIZE_MASK: u32 = 0x3fff; // bit13:0
const TDES2_INTERRUPT_ON_COMPLETION: u32 = 1 << 31; // bit31 IOC

const TDES3_OWN: u32 = 1 << 31; // DMA 拥有
const TDES3_ERROR_SUMMARY: u32 = 1 << 15; // 回写错误位
const TDES3_LAST_DESCRIPTOR: u32 = 1 << 28; // 包末尾描述符
const TDES3_FIRST_DESCRIPTOR: u32 = 1 << 29; // 包起始描述符
const TDES3_PACKET_SIZE_MASK: u32 = 0x7fff; // bit14:0
const TDES3_CHECKSUM_INSERTION_SHIFT: u32 = 16;
const TDES3_CHECKSUM_INSERTION_MASK: u32 = 0x3 << 16; // CIC 全 IP+pseudo header = 3
const TX_CIC_FULL: u32 = 3;

// ---------------------------------------------------------------------------
// RX 描述符位域（dwmac4_descs.h RDES3）
// ---------------------------------------------------------------------------
const RDES3_OWN: u32 = 1 << 31;
const RDES3_ERROR_SUMMARY: u32 = 1 << 15;
const RDES3_LAST_DESCRIPTOR: u32 = 1 << 28;
const RDES3_FIRST_DESCRIPTOR: u32 = 1 << 29;
const RDES3_BUFFER1_VALID_ADDR: u32 = 1 << 24; // bit24 BUF1V
const RDES3_INT_ON_COMPLETION_EN: u32 = 1 << 30; // bit30 IOC
const RDES3_PACKET_SIZE_MASK: u32 = 0x7fff; // bit14:0（回写时包长度）

/// 清零描述符（所有 4 字段归零）。
pub fn clear(desc: &mut DmaDesc) {
    desc.des0 = 0;
    desc.des1 = 0;
    desc.des2 = 0;
    desc.des3 = 0;
}

/// 写入 buffer1 物理地址（64 位拆分为低/高 32 位）。
pub fn set_addr(desc: &mut DmaDesc, bus_addr: u64) {
    desc.des0 = bus_addr as u32;
    desc.des1 = (bus_addr >> 32) as u32;
}

/// 准备 RX 描述符：写地址 + 置 OWN|BUF1V|IOC，交 DMA 接收。
pub fn prepare_rx(desc: &mut DmaDesc, bus_addr: u64) {
    set_addr(desc, bus_addr);
    desc.des2 = 0;
    dma_wmb();
    desc.des3 = RDES3_OWN | RDES3_BUFFER1_VALID_ADDR | RDES3_INT_ON_COMPLETION_EN;
}

/// 准备 TX 描述符：写地址 + 长度 + FIRST|LAST|OWN（可选 CIC 校验和插入）。
pub fn prepare_tx(desc: &mut DmaDesc, bus_addr: u64, len: usize, checksum: bool) {
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

/// DMA 写屏障：保证描述符字段在置 OWN 位前已对设备可见。
/// DWMAC4 的缓存一致性问题由 dma-api 后端（SVPBMT uncached + zicbom）处理，
/// 这里用 Release fence 保证 CPU 侧写顺序。
pub fn dma_wmb() {
    core::sync::atomic::fence(core::sync::atomic::Ordering::Release);
}

// --- TX 状态查询 ---
pub fn tx_owned(desc: &DmaDesc) -> bool {
    (desc.des3 & TDES3_OWN) != 0
}
pub fn tx_has_error(desc: &DmaDesc) -> bool {
    (desc.des3 & TDES3_ERROR_SUMMARY) != 0
}

// --- RX 状态查询 ---
pub fn rx_owned(desc: &DmaDesc) -> bool {
    (desc.des3 & RDES3_OWN) != 0
}
/// RX 描述符就绪：非 DMA 拥有且 FIRST|LAST 均置位（完整单包）。
pub fn rx_ready(desc: &DmaDesc) -> bool {
    !rx_owned(desc)
        && (desc.des3 & RDES3_LAST_DESCRIPTOR) != 0
        && (desc.des3 & RDES3_FIRST_DESCRIPTOR) != 0
}
pub fn rx_len(desc: &DmaDesc) -> usize {
    (desc.des3 & RDES3_PACKET_SIZE_MASK) as usize
}
pub fn rx_has_error(desc: &DmaDesc) -> bool {
    (desc.des3 & RDES3_ERROR_SUMMARY) != 0
}

// --- 地址与 FIFO 辅助 ---
pub fn ring_offset(index: usize) -> u64 {
    (index * core::mem::size_of::<DmaDesc>()) as u64
}
pub fn split_addr(addr: u64) -> (u32, u32) {
    (addr as u32, (addr >> 32) as u32)
}

/// 字节数转 MTL FIFO 队列大小编码（每 256 字节一档，编码 = bytes/256 - 1）。
pub fn mtl_fifo_words(bytes: u32) -> u32 {
    bytes.saturating_div(256).saturating_sub(1)
}

/// RX buffer 大小编码进 DMA_CHAN_RX_CONTROL bit14:1。
pub fn rx_buf_size_bits(size: usize) -> u32 {
    ((size as u32) << regs::DMA_RBSZ_SHIFT) & regs::DMA_RBSZ_MASK
}
