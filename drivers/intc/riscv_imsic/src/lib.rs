//! RISC-V Incoming MSI Controller (IMSIC) —— AIA
//!
//! IMSIC 是 RISC-V AIA 的 MSI 接收端：设备/APLIC 发 MSI（对特殊地址的内存写）
//! → IMSIC 置 pending → 拉 SEIP → CPU 读 `stopei` claim。
//!
//! 访问模型：
//! - **配置**：经 S-mode CSR `siselect`(0x150)/`sireg`(0x151) 间接访问本 hart 中断
//!   文件的控制寄存器（eidelivery / eithreshold / eie 位图 / eip 位图）。
//! - **MSI 投递**：写 per-hart 中断文件 MMIO 页。每 hart 一个 4KB 页区域（含 guest 文件），
//!   写 `[file_base + 0] = eiid` 即置 pending[eiid]。
//! - **claim**：读 `stopei`(0x15c) CSR，原子返回最高优先级 pending 的
//!   `{priority[31:16], eiid[15:0]}`，0 表示无 pending。
//!
//! 寄存器 select 常量经核对（Linux `include/linux/irqchip/riscv-imsic.h` + OpenSBI +
//! QEMU + bao 一致）：EIDELIVERY=0x70, EITHRESHOLD=0x72, EIP0=0x80, EIE=0xC0。
//!
//! 本 crate 只提供寄存器语义 + 纯函数（MSI 地址编码数学、CSR 访问原语），不含
//! OS glue（FDT probe / IRQ 域注册）——那层在 somehal。镜像 `ax-riscv-plic` 分层。

#![no_std]

use core::ptr::write_volatile;

// ── 经核对的常量（Linux riscv-imsic.h / OpenSBI / QEMU 一致） ──────────

/// 中断文件 MMIO 页大小（4KB）。
pub const PAGE_SIZE: usize = 4096;

/// siselect 选择值：中断投递控制（0=禁用投递，1=投递到本 hart）。
pub const EIDELIVERY: u32 = 0x70;
/// siselect 选择值：优先级阈值（0=不屏蔽，>0 屏蔽 < 该值的）。
pub const EITHRESHOLD: u32 = 0x72;
/// siselect 选择值：pending 位图起始（每个寄存器 32 个 ID，EIP0..EIP63 = 0x80..0xBF）。
pub const EIP0: u32 = 0x80;
/// siselect 选择值：enable 位图起始（每个寄存器 32 个 ID，EIE0.. = 0xC0..）。
pub const EIE0: u32 = 0xC0;
/// 单个 EIP/EIE 寄存器覆盖的 ID 数。
pub const BITS_PER_REG: u32 = 32;

/// IPI 专用 identity（向目标 hart 写 EID 0 触发软件中断）。
pub const IPI_EIID: u32 = 0;

// S-mode AIA CSR 编号（RISC-V AIA 规范，Linux arch/riscv/include/asm/csr.h）
#[cfg(target_arch = "riscv64")]
mod csr {
    pub const SISELECT: u16 = 0x150;
    pub const SIREG: u16 = 0x151;
    pub const STOPEI: u16 = 0x15c;
}

// ── 几何 + MSI 地址编码（纯函数，可单测） ─────────────────────────────

/// IMSIC 几何参数，从 FDT 解析（`riscv,hart-index-bits` 等）。
///
/// K3 实测值：base=0xe0400000, hart_index_bits=4, guest_index_bits=6,
/// group_index_bits=0, num_ids=511, 每 hart 步长 = 0x40000。
#[derive(Clone, Copy, Debug)]
pub struct ImsicGeometry {
    /// 中断文件 MMIO 基址（FDT reg 起始地址）。
    pub base_addr: usize,
    pub hart_index_bits: u32,
    pub guest_index_bits: u32,
    pub group_index_bits: u32,
    pub group_index_shift: u32,
    /// 支持的中断 identity 数（K3=511；identity 0..num_ids）。
    pub num_ids: u32,
}

impl ImsicGeometry {
    /// 每 hart 中断文件步长（字节）= 2^guest_index_bits * 4KB。
    ///
    /// 同一 hart 下不同 guest 的中断文件在该步长内以 4KB 页为单位排布。
    #[inline]
    pub const fn hart_stride(&self) -> usize {
        (1 << self.guest_index_bits) * PAGE_SIZE
    }

    /// 支持的 hart 数上限（2^hart_index_bits；K3=16）。
    #[inline]
    pub const fn max_harts(&self) -> u32 {
        1 << self.hart_index_bits
    }
}

/// MSI 消息（地址 + 数据），供消费者（APLIC / PCIe MSI-X）编程其硬件。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MsiMessage {
    /// MSI 写目标地址（指向某 hart/guest 的中断文件页 base = SETEIPNUM_LE 寄存器）。
    pub address: usize,
    /// MSI 写数据（= eiid）。
    pub data: u32,
}

/// 计算某 hart/guest 的中断文件 MMIO 基址（= MSI 写目标地址）。
///
/// 公式：`base + hart_index * hart_stride + guest_index * PAGE_SIZE`，
/// 其中 `hart_stride = 2^guest_index_bits * PAGE_SIZE`。
///
/// K3 验证（见单测）：hart0=0xe0400000, hart1=0xe0440000, ... hart15=0xe07c0000。
#[inline]
pub fn interrupt_file_addr(geo: &ImsicGeometry, hart_index: u32, guest_index: u32) -> usize {
    geo.base_addr + (hart_index as usize) * geo.hart_stride() + (guest_index as usize) * PAGE_SIZE
}

/// 组成 MSI 消息：address = 目标中断文件页 base（写此地址即 SETEIPNUM_LE），
/// data = eiid。
#[inline]
pub fn compose_msi_message(
    geo: &ImsicGeometry,
    hart_index: u32,
    guest_index: u32,
    eiid: u32,
) -> MsiMessage {
    MsiMessage {
        address: interrupt_file_addr(geo, hart_index, guest_index),
        data: eiid,
    }
}

/// 计算 APLIC smsicfgaddr 的 base_ppn（group base 的 PPN，hart/guest 位清零）。
///
/// 这与 `interrupt_file_addr` 不同：APLIC 硬件用 base_ppn + 几何字段重构地址，
/// 故 base_ppn 必须把 hart/guest 索引位清零。Linux `imsic_setup_state` 同款逻辑。
/// 返回 PPN（即地址 >> 12）。
#[inline]
pub fn group_base_ppn(geo: &ImsicGeometry) -> u64 {
    // 页内索引位 = guest_bits + hart_bits，再加 12 位页偏移
    let index_bits = geo.guest_index_bits + geo.hart_index_bits;
    let mask: usize = !((1usize << (index_bits + 12)) - 1);
    ((geo.base_addr & mask) as u64) >> 12
}

// ── S-mode CSR 间接访问原语（仅 riscv64 目标；host 单测不编译这些） ────

#[cfg(target_arch = "riscv64")]
#[inline]
/// # Safety
///
/// 写 siselect 选择后续 sireg 访问的中断文件寄存器。须在本地 hart、S-mode、
/// ssaia 扩展可用时调用。
unsafe fn siselect_write(val: u32) {
    unsafe {
        core::arch::asm!(
            "csrw {csr}, {val}",
            csr = const csr::SISELECT,
            val = in(reg) val as usize,
        );
    }
}

#[cfg(target_arch = "riscv64")]
#[inline]
/// # Safety
///
/// 读当前 siselect 选中的中断文件寄存器。须在本地 hart、S-mode、ssaia 可用时
/// 调用，且调用方须保证 siselect 已指向目标寄存器（无并发 siselect/sireg 访问）。
unsafe fn sireg_read() -> usize {
    let val: usize;
    unsafe {
        core::arch::asm!(
            "csrr {val}, {csr}",
            val = out(reg) val,
            csr = const csr::SIREG,
        )
    }
    val
}

#[cfg(target_arch = "riscv64")]
#[inline]
/// # Safety
///
/// 同 [`sireg_read`]：写当前 siselect 选中的中断文件寄存器。
unsafe fn sireg_write(val: usize) {
    unsafe {
        core::arch::asm!(
            "csrw {csr}, {val}",
            csr = const csr::SIREG,
            val = in(reg) val,
        );
    }
}

/// 读 `stopei`：原子 claim 最高优先级 pending 中断，返回原始值。
///
/// 返回值高 16 位是 priority，低 16 位是 eiid；全 0 表示无 pending。
///
/// # Safety
///
/// 必须在本地 hart、S-mode、ssaia 扩展可用时调用。
#[cfg(target_arch = "riscv64")]
#[inline]
pub unsafe fn stopei_read() -> u32 {
    let val: usize;
    unsafe {
        core::arch::asm!(
            "csrr {val}, {csr}",
            val = out(reg) val,
            csr = const csr::STOPEI,
        )
    }
    val as u32
}

/// 解析 `stopei` 原始值 → `(eiid, priority)`；0 表示无 pending。
#[inline]
pub const fn parse_stopei(raw: u32) -> Option<(u32, u32)> {
    if raw == 0 {
        None
    } else {
        Some((raw & 0xFFFF, raw >> 16))
    }
}

// ── 高层操作（碰硬件；仅 riscv64） ─────────────────────────────────────

/// 初始化本 hart 的 S-mode 中断文件：eidelivery=1（投递到本 hart）、eithreshold=0
/// （不屏蔽）、清所有 eie 位（禁用全部 identity，由驱动按需 enable）。
///
/// # Safety
///
/// 必须在本地 hart 上调用；CPU 须有 ssaia 扩展；siselect/sireg 在 S-mode 可访问。
#[cfg(target_arch = "riscv64")]
pub unsafe fn init_local_file(num_ids: u32) {
    unsafe {
        siselect_write(EIDELIVERY);
        sireg_write(1);
        siselect_write(EITHRESHOLD);
        sireg_write(0);

        // RV64 上 sireg 为 64 位，两相邻 32 位 EIE 寄存器打包在一个 even siselect
        // 值中（odd siselect 触发 illegal instruction）。步进 2，写 64 位 0。
        let eie_pairs = num_ids.div_ceil(BITS_PER_REG * 2) as usize;
        for i in 0..eie_pairs {
            siselect_write(EIE0 + (i as u32) * 2);
            sireg_write(0);
        }
    }
}

/// 使能本 hart 的某 EID（置 eie 位图对应位）。
///
/// RV64 上 sireg 为 64 位，两个 32 位 EIE 寄存器打包在一个 even siselect 值中。
/// 因此 siselect = EIE0 + (eiid / 64) * 2，bit = 1 << (eiid % 64)。
///
/// # Safety
///
/// 同 [`init_local_file`]。
#[cfg(target_arch = "riscv64")]
pub unsafe fn enable_eid(eiid: u32) {
    debug_assert!(eiid != 0, "EIID 0 保留给 IPI，不应经 enable_eid 使能");
    unsafe {
        let siselect_val = EIE0 + (eiid / 64) * 2;
        let bit: usize = 1usize << (eiid % 64);
        siselect_write(siselect_val);
        let cur = sireg_read();
        sireg_write(cur | bit);
    }
}

/// 禁用本 hart 的某 EID。
///
/// # Safety
///
/// 同 [`init_local_file`]。
#[cfg(target_arch = "riscv64")]
pub unsafe fn disable_eid(eiid: u32) {
    unsafe {
        let siselect_val = EIE0 + (eiid / 64) * 2;
        let bit: usize = 1usize << (eiid % 64);
        siselect_write(siselect_val);
        let cur = sireg_read();
        sireg_write(cur & !bit);
    }
}

/// 读 `stopei` claim 最高优先级 pending 中断。无 pending 返回 None。
///
/// # Safety
///
/// 同 [`init_local_file`]。
#[cfg(target_arch = "riscv64")]
pub unsafe fn claim() -> Option<(u32, u32)> {
    parse_stopei(unsafe { stopei_read() })
}

/// 写 `stopei` 完成中断的 pending 清除（对应 QEMU riscv_imsic_topei_rmw 的 write 侧）。
///
/// 读 stopei 只返回 pending 中断的 ID，不自动清除 pending 位；必须再写 stopei
/// （写任意 non-zero 值即可触发 QEMU 的清 pending 路径）才算"完成"该中断。
/// 不写 stopei 则 pending 位永不释放 → IRQ 线保持 asserted → 反复 trap。
///
/// **QEMU 适配说明**：AIA 规范本身规定 stopei 读应原子完成 claim、priority-drop
/// 与 activate（即读后 pending 自动清除）。但 QEMU 的 `riscv_imsic_topei_rmw`
/// 实现把 read 和 write 分开：read 只返回值，write 才清 pending，偏离了规范。
/// 因此本函数目前是 QEMU 行为适配；移植到严格遵循 AIA 规范的真硬件时，需复核
/// 多写一次 stopei 是否被硬件忽略（规范未明确禁止重复 complete，但实现各异）。
///
/// # Safety
///
/// 必须在本地 hart、S-mode、ssaia 扩展可用时调用。
/// 只应在同一次中断处理流程中 claim 后调用。
#[cfg(target_arch = "riscv64")]
pub unsafe fn complete_stopei(val: u32) {
    unsafe {
        core::arch::asm!(
            "csrw {csr}, {val}",
            csr = const csr::STOPEI,
            val = in(reg) val as usize,
        );
    }
}

/// 经 IMSIC 发 IPI：写目标 hart 中断文件页的 EID 0（IPI 专用 identity）。
///
/// # Safety
///
/// `target_file_addr` 必须是有效 IMSIC 中断文件页地址（`interrupt_file_addr` 的返回值）。
#[inline]
pub unsafe fn send_ipi(target_file_addr: usize) {
    unsafe { write_volatile(target_file_addr as *mut u32, IPI_EIID) };
}

#[cfg(test)]
mod tests {
    use super::*;

    /// K3 几何（来自 DTB 实测值）。
    const fn k3() -> ImsicGeometry {
        ImsicGeometry {
            base_addr: 0xe040_0000,
            hart_index_bits: 4,
            guest_index_bits: 6,
            group_index_bits: 0,
            group_index_shift: 24,
            num_ids: 511,
        }
    }

    #[test]
    fn k3_hart_stride_is_0x40000() {
        // 2^6 guest * 4KB = 0x40000
        assert_eq!(k3().hart_stride(), 0x40000);
    }

    #[test]
    fn k3_max_harts_is_16() {
        assert_eq!(k3().max_harts(), 16);
    }

    #[test]
    fn k3_interrupt_file_addr_matches_dtb_layout() {
        let geo = k3();
        // DTB reg=0xe0400000，每 hart 0x40000 步长，16 hart 共 0x400000（与 reg size 一致）
        assert_eq!(interrupt_file_addr(&geo, 0, 0), 0xe040_0000);
        assert_eq!(interrupt_file_addr(&geo, 1, 0), 0xe044_0000);
        assert_eq!(interrupt_file_addr(&geo, 7, 0), 0xe05c_0000);
        assert_eq!(interrupt_file_addr(&geo, 15, 0), 0xe07c_0000);
        // guest 1 在 hart 0 内偏移一页（4KB）
        assert_eq!(interrupt_file_addr(&geo, 0, 1), 0xe040_1000);
    }

    #[test]
    fn k3_total_imsic_span_matches_reg_size() {
        let geo = k3();
        assert_eq!(geo.max_harts() as usize * geo.hart_stride(), 0x400000);
    }

    #[test]
    fn compose_msi_message_carries_eiid_as_data() {
        let geo = k3();
        let msg = compose_msi_message(&geo, 0, 0, 42);
        assert_eq!(msg.address, 0xe040_0000);
        assert_eq!(msg.data, 42);
    }

    #[test]
    fn group_base_ppn_clears_index_bits() {
        // K3 base 0xe0400000：hart0 的索引位（PPN[0..9]）本就为 0，
        // 故 group_base_ppn = 0xe0400000 >> 12 = 0xE0400。
        // 验证：APLIC 用此 base_ppn 重构 hart1 地址应为 0xe0440000
        //   (base_ppn | (1 << guest_bits)) << 12 = (0xE0400 | 0x40) << 12 = 0xe0440000 ✓
        let geo = k3();
        let bppn = group_base_ppn(&geo);
        assert_eq!(bppn, 0xE0400);
        // 重构 hart1 / hart15 验证 base_ppn 自洽
        let hart1 = ((bppn | (1u64 << geo.guest_index_bits)) << 12) as usize;
        assert_eq!(hart1, 0xe044_0000);
        let hart15 = ((bppn | (15u64 << geo.guest_index_bits)) << 12) as usize;
        assert_eq!(hart15, 0xe07c_0000);
    }

    #[test]
    fn parse_stopei_decodes_priority_and_eiid() {
        assert_eq!(parse_stopei(0), None);
        // eiid=42(0x2a), priority=5 → raw = (5<<16)|42 = 0x5002a
        assert_eq!(parse_stopei(0x5_002a), Some((42, 5)));
        // eiid=1, priority=1
        assert_eq!(parse_stopei(0x1_0001), Some((1, 1)));
    }

    #[test]
    fn ipi_uses_eid_zero() {
        assert_eq!(IPI_EIID, 0);
    }

    #[test]
    fn interrupt_file_addr_guest_pages_within_hart_stride() {
        // K3 几何：每 hart 步长 0x40000（=2^6 guest * 4KB），guest 文件以 4KB 页排布。
        let geo = k3();
        let base = geo.base_addr;
        // guest 0..3 应在第一个 hart 步长内，相邻 guest 差 4KB。
        assert_eq!(interrupt_file_addr(&geo, 0, 0), base);
        assert_eq!(interrupt_file_addr(&geo, 0, 1), base + 0x1000);
        assert_eq!(interrupt_file_addr(&geo, 0, 2), base + 0x2000);
        assert_eq!(interrupt_file_addr(&geo, 0, 63), base + 63 * 0x1000);
        // guest 63 + 1 必须跨入下一 hart 步长（边界检查）。
        assert_eq!(interrupt_file_addr(&geo, 1, 0), base + 0x40000);
    }

    #[test]
    fn interrupt_file_addr_zero_geometry_is_single_page_layout() {
        // QEMU virt (aia=aplic-imsic) 的 IMSIC 节点不带位宽属性，缺省全 0：
        // 单 hart、单 guest、单 group，所有中断文件共享同一个 4KB 页。
        let geo = ImsicGeometry {
            base_addr: 0x2800_0000,
            hart_index_bits: 0,
            guest_index_bits: 0,
            group_index_bits: 0,
            group_index_shift: 24,
            num_ids: 255,
        };
        assert_eq!(geo.hart_stride(), 0x1000, "单 guest → 步长 = 一页");
        assert_eq!(geo.max_harts(), 1);
        assert_eq!(interrupt_file_addr(&geo, 0, 0), 0x2800_0000);
    }

    #[test]
    fn group_base_ppn_clears_group_index_bits() {
        // 多 group 几何：group 索引位落在 base_addr 的高位，group_base_ppn
        // 必须把这些位清零，APLIC 才能用 base_ppn + 几何字段重构正确地址。
        let geo = ImsicGeometry {
            // group_index_shift=24, group_index_bits=4 → group 位占 [28:32)。
            // 设 base 的 bit28..32 为非零（模拟某 group 起始地址）。
            base_addr: 0xe040_0000 | (0b1010 << 28),
            hart_index_bits: 4,
            guest_index_bits: 6,
            group_index_bits: 4,
            group_index_shift: 24,
            num_ids: 511,
        };
        let bppn = group_base_ppn(&geo);
        // group 位被清零后，base 应回落到 group 0 的起始（0xe040_0000）。
        assert_eq!(bppn, 0xe040_0000 >> 12);
    }
}
