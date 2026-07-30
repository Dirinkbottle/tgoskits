//! RISC-V Advanced Platform-Level Interrupt Controller (APLIC) —— AIA，MSI 模式
//!
//! APLIC 把**有线中断**（UART / mailbox / SD 等的物理 IRQ 线）转成 **MSI** 发给
//! IMSIC。K3 的 APLIC 仅支持 MSI 模式（手册 §7 "MSI output mode only"），不能
//! 直送 CPU。
//!
//! 配置流程：`init_msi_mode`（domaincfg.DM=1 + smsicfgaddr/h 用 IMSIC base PPN）
//! → 对每个 wired 源 `configure_source`（sourcecfg 触发类型 + target 的 hart/guest/eiid）
//! → `enable_source`。
//!
//! 本 crate 只提供寄存器语义（MMIO 操作），不含 OS glue——那层在 somehal。
//! 寄存器偏移经 Linux `include/linux/irqchip/riscv-aplic.h` 核对。

#![no_std]

use core::{
    num::NonZeroU32,
    ptr::{read_volatile, write_volatile},
};

// ── 寄存器偏移（Linux riscv-aplic.h，多源核对一致） ───────────────────

const DOMAINCFG: usize = 0x0000;
const SOURCECFG_BASE: usize = 0x0004;
const SMSICFGADDR: usize = 0x1bc8;
const SMSICFGADDRH: usize = 0x1bcc;
const SETIP_BASE: usize = 0x1c00;
const CLRIP_BASE: usize = 0x1d00;
const SETIE_BASE: usize = 0x1e00;
const CLRIE_BASE: usize = 0x1f00;
const SETIPNUM_LE: usize = 0x2000;
const SETIENUM: usize = 0x1edc;
const CLRIENUM: usize = 0x1fdc;
const TARGET_BASE: usize = 0x3004;
const GENMSI: usize = 0x3000;

// ── domaincfg 位 ──────────────────────────────────────────────────────

const DOMAINCFG_IE: u32 = 1 << 8;
const DOMAINCFG_DM: u32 = 1 << 2;

// ── sourcecfg SM 触发类型编码（Linux APLIC_SOURCECFG_SM_*） ───────────

const SM_INACTIVE: u32 = 0x0;
const SM_DETACH: u32 = 0x1;
const SM_EDGE_RISE: u32 = 0x4;
const SM_EDGE_FALL: u32 = 0x5;
const SM_LEVEL_HIGH: u32 = 0x6;
const SM_LEVEL_LOW: u32 = 0x7;

// ── target 位（MSI 模式：HART_IDX | GUEST_IDX | EIID） ────────────────

const TARGET_HART_IDX_SHIFT: u32 = 18;
const TARGET_HART_IDX_MASK: u32 = 0x3fff;
const TARGET_GUEST_IDX_SHIFT: u32 = 12;
const TARGET_GUEST_IDX_MASK: u32 = 0x3f;
const TARGET_EIID_MASK: u32 = 0x7ff;

// ── smsicfgaddrh 位（取值同 Linux APLIC_xMSICFGADDRH_* 共享宏） ─────────
//
// 注意 AIA 规范的 S/M 区分：S-mode 的 smsiaddrcfgh（SMSICFGADDRH=0x1bcc）
// 官方仅定义 LHXS + BAPPN 两段；L / HHXS / HHXW / LHXW 是 M-mode
// mmsiaddrcfgh（MMSICFGADDRH=0x1bc4）的字段。Linux 仅在 CONFIG_RISCV_M_MODE
// 下把这些几何字段写进 M-mode 寄存器，S-mode Linux 不写、依赖固件预先配置。
// 本 crate 沿用 Linux 的 xMSICFGADDRH_* 取值统一表达两套寄存器位定义；
// 实际写入哪个寄存器由 somehal 调用方决定（见 init_msi_mode 说明）。

const CFGADDRH_L: u32 = 1 << 31;
const CFGADDRH_HHXS_SHIFT: u32 = 24;
const CFGADDRH_HHXS_MASK: u32 = 0x1f;
const CFGADDRH_LHXS_SHIFT: u32 = 20;
const CFGADDRH_LHXS_MASK: u32 = 0x7;
const CFGADDRH_HHXW_SHIFT: u32 = 16;
const CFGADDRH_HHXW_MASK: u32 = 0x7;
const CFGADDRH_LHXW_SHIFT: u32 = 12;
const CFGADDRH_LHXW_MASK: u32 = 0xf;
const CFGADDRH_BAPPN_MASK: u32 = 0xfff;

/// sourcecfg.SM 触发类型。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SourceTrigger {
    Inactive,
    Detach,
    EdgeRise,
    EdgeFall,
    LevelHigh,
    LevelLow,
}

impl SourceTrigger {
    const fn to_sm(self) -> u32 {
        match self {
            SourceTrigger::Inactive => SM_INACTIVE,
            SourceTrigger::Detach => SM_DETACH,
            SourceTrigger::EdgeRise => SM_EDGE_RISE,
            SourceTrigger::EdgeFall => SM_EDGE_FALL,
            SourceTrigger::LevelHigh => SM_LEVEL_HIGH,
            SourceTrigger::LevelLow => SM_LEVEL_LOW,
        }
    }

    /// 从 RISC-V 设备树 interrupt specifier 的触发标志 cell（第二 cell）
    /// 解析 APLIC 触发类型。
    ///
    /// RISC-V DT binding 沿用 POSIX `interrupts extended` 标志位编码：
    /// - `0x01` → edge rising（低→高边沿）
    /// - `0x02` → edge falling（高→低边沿）
    /// - `0x04` → level high（高电平有效）
    /// - `0x08` → level low（低电平有效）
    ///
    /// 其它值（含 `0`）返回 `None`，由调用方决定缺省策略（多数平台 fallback
    /// 为 `LevelHigh`，见 somehal 集成层）。
    ///
    /// 历史背景：本映射曾因错误推论"PCI INTx 必为 level-low"被绕过，导致
    /// QEMU virt 下所有 wired 源被硬编码为 `LevelLow`，rectified 信号反极性，
    /// virtio-blk INTx 永不触发。把映射集中到 driver core 层并锁定回归测试，
    /// 避免再次回退到平台级硬编码。
    #[inline]
    pub const fn from_fdt_interrupt_spec(flag: u32) -> Option<Self> {
        match flag {
            0x01 => Some(SourceTrigger::EdgeRise),
            0x02 => Some(SourceTrigger::EdgeFall),
            0x04 => Some(SourceTrigger::LevelHigh),
            0x08 => Some(SourceTrigger::LevelLow),
            _ => None,
        }
    }
}

/// APLIC MSI 目标（写进 target[i] 寄存器）：hart_index + guest_index + eiid。
#[derive(Clone, Copy, Debug)]
pub struct MsiTarget {
    pub hart_index: u32,
    pub guest_index: u32,
    pub eiid: u32,
}

impl MsiTarget {
    const fn pack(self) -> u32 {
        ((self.hart_index & TARGET_HART_IDX_MASK) << TARGET_HART_IDX_SHIFT)
            | ((self.guest_index & TARGET_GUEST_IDX_MASK) << TARGET_GUEST_IDX_SHIFT)
            | (self.eiid & TARGET_EIID_MASK)
    }
}

/// IMSIC base PPN + 地址几何，写进 smsicfgaddr / smsicfgaddrh。
///
/// 字段对应 Linux `APLIC_xMSICFGADDRH_{BAPPN,LHXW,HHXW,LHXS,HHXS}`。
#[derive(Clone, Copy, Debug)]
pub struct MsiConfig {
    pub base_ppn: u64,
    pub lhxw: u32,
    pub lhxs: u32,
    pub hhxw: u32,
    pub hhxs: u32,
}

/// 把 [`MsiConfig`] 编码成 `smsicfgaddrh`（偏移 0x1bcc）寄存器值。
///
/// 纯函数，供 [`Aplic::init_msi_mode`] 与单测共享同一段位打包逻辑。
/// 位置约束见上方 `CFGADDRH_*` 常量块：S-mode 下仅 LHXS+BAPPN 段被硬件采纳。
#[inline]
pub const fn pack_smsicfg_addrh(cfg: &MsiConfig) -> u32 {
    CFGADDRH_L
        | ((cfg.hhxs & CFGADDRH_HHXS_MASK) << CFGADDRH_HHXS_SHIFT)
        | ((cfg.lhxs & CFGADDRH_LHXS_MASK) << CFGADDRH_LHXS_SHIFT)
        | ((cfg.hhxw & CFGADDRH_HHXW_MASK) << CFGADDRH_HHXW_SHIFT)
        | ((cfg.lhxw & CFGADDRH_LHXW_MASK) << CFGADDRH_LHXW_SHIFT)
        | ((cfg.base_ppn >> 32) as u32 & CFGADDRH_BAPPN_MASK)
}

/// Advanced Platform-Level Interrupt Controller（MSI 模式）。
pub struct Aplic {
    base: *mut u8,
    num_sources: u32,
}

unsafe impl Send for Aplic {}
unsafe impl Sync for Aplic {}

impl Aplic {
    /// # Safety
    ///
    /// `base` 必须指向有效、独占的 APLIC MMIO 寄存器区。
    #[inline]
    pub const unsafe fn new(base: *mut u8, num_sources: u32) -> Self {
        Self { base, num_sources }
    }

    #[inline]
    pub fn num_sources(&self) -> u32 {
        self.num_sources
    }

    #[inline]
    fn reg_ptr(&self, offset: usize) -> *mut u32 {
        // `offset` 是字节偏移（如 SOURCECFG_BASE=0x0004），base 为 *mut u8，
        // 故 `.add(offset)` 按字节步进，再转 *mut u32 读 4 字节寄存器。
        // 安全性由 `Aplic::new` 的契约承载：base 指向有效且独占的 MMIO 区，
        // 且各方法传入的 offset 均在 [0, 寄存器区大小) 内。
        unsafe { self.base.add(offset) as *mut u32 }
    }

    #[inline]
    fn read(&self, offset: usize) -> u32 {
        unsafe { read_volatile(self.reg_ptr(offset)) }
    }

    pub fn read_raw(&self, offset: usize) -> u32 {
        self.read(offset)
    }

    #[inline]
    fn write(&self, offset: usize, val: u32) {
        unsafe { write_volatile(self.reg_ptr(offset), val) }
    }

    /// sourcecfg/target 的下标按 source id（1-based）算偏移。
    #[inline]
    fn source_offset(source: NonZeroU32, base: usize) -> usize {
        base + (source.get() as usize - 1) * 4
    }

    /// 禁用所有 source：清全部 setie 位图 + 设全部 sourcecfg 为 inactive。
    pub fn disable_all_sources(&mut self) {
        let ie_regs = self.num_sources.div_ceil(32) as usize;
        for i in 0..ie_regs {
            self.write(CLRIE_BASE + i * 4, !0);
        }
        for s in 1..=self.num_sources {
            let off = (s as usize - 1) * 4;
            self.write(SOURCECFG_BASE + off, SM_INACTIVE);
        }
    }

    /// 清所有 pending 位（初始化时清残留 pending）。
    pub fn clear_all_pending(&mut self) {
        let ip_regs = self.num_sources.div_ceil(32) as usize;
        for i in 0..ip_regs {
            self.write(CLRIP_BASE + i * 4, !0);
        }
    }

    /// 读 pending 位图。`reg_index` 以 32 个 source 为一组，从 0 开始。
    pub fn pending_bitmap(&self, reg_index: usize) -> u32 {
        self.read(SETIP_BASE + reg_index * 4)
    }

    /// 读 enable 位图。`reg_index` 以 32 个 source 为一组，从 0 开始。
    pub fn enable_bitmap(&self, reg_index: usize) -> u32 {
        self.read(SETIE_BASE + reg_index * 4)
    }

    pub fn is_pending(&self, source: NonZeroU32) -> bool {
        let s = source.get() as usize;
        self.pending_bitmap(s / 32) & (1 << (s % 32)) != 0
    }

    /// 进入 MSI 模式：写 smsicfgaddr/h（IMSIC base PPN + 几何）+ domaincfg（IE=1, DM=1）。
    ///
    /// APLIC 硬件据此把每个 wired 源的 pending 转成 MSI 写：地址 = base_ppn +
    /// hart/guest 索引字段，数据 = target[i].EIID。
    ///
    /// **S-mode 注意**（见上方 smsicfgaddrh 常量块）：本方法把 L/HHXS/HHXW/LHXW
    /// 一并写进 S-mode 的 SMSICFGADDRH(0x1bcc)，但 AIA 规范下该寄存器仅认 LHXS+BAPPN，
    /// 其余位被硬件忽略（QEMU `SMSICFGADDRH_VALID_MASK` 印证）。多 hart 场景的几何
    /// （尤其 LHXW = hart 索引宽度）须由 M-mode 固件写 mmsiaddrcfgh(0x1bc4) 预先配好；
    /// S-mode 直写这些位无害但无效。调用方需确认目标平台的固件已配置 MSI 几何。
    pub fn init_msi_mode(&mut self, cfg: &MsiConfig) {
        let addr_lo = cfg.base_ppn as u32;
        let addr_hi = pack_smsicfg_addrh(cfg);
        self.write(SMSICFGADDR, addr_lo);
        self.write(SMSICFGADDRH, addr_hi);
        let dom = self.read(DOMAINCFG) | DOMAINCFG_IE | DOMAINCFG_DM;
        self.write(DOMAINCFG, dom);
    }

    /// 配置 source 的触发类型 + MSI 目标（sourcecfg[i] + target[i]）。
    pub fn configure_source(
        &mut self,
        source: NonZeroU32,
        trigger: SourceTrigger,
        target: MsiTarget,
    ) {
        let scfg_off = Self::source_offset(source, SOURCECFG_BASE);
        self.write(scfg_off, trigger.to_sm());
        let tgt_off = Self::source_offset(source, TARGET_BASE);
        self.write(tgt_off, target.pack());
    }

    pub fn enable_source(&mut self, source: NonZeroU32) {
        self.write(SETIENUM, source.get());
    }

    pub fn disable_source(&mut self, source: NonZeroU32) {
        self.write(CLRIENUM, source.get());
    }

    /// 清除指定 source 的 pending 位（写 CLRIP 位图）。
    /// 用于 enable 前清掉 sourcecfg 写入时因当前电平 rectified 值意外置上的
    /// pending，避免 enable 立即触发 spurious MSI。
    pub fn clear_source_pending(&mut self, source: NonZeroU32) {
        let s = source.get();
        let word = (s / 32) as usize;
        self.write(CLRIP_BASE + word * 4, 1 << (s % 32));
    }

    /// 软件触发某 source（自测用）：写 SETIPNUM_LE = source id。
    pub fn trigger_source(&mut self, source: NonZeroU32) {
        self.write(SETIPNUM_LE, source.get());
    }

    /// 生成 MSI（自测用，不经 wired 源）：写 GENMSI，直接让 APLIC 发一次 MSI。
    pub fn generate_msi(&mut self, target: MsiTarget) {
        self.write(GENMSI, target.pack());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn msi_target_packs_fields() {
        let t = MsiTarget {
            hart_index: 3,
            guest_index: 0,
            eiid: 42,
        };
        let packed = t.pack();
        assert_eq!(packed >> TARGET_HART_IDX_SHIFT & TARGET_HART_IDX_MASK, 3);
        assert_eq!(packed >> TARGET_GUEST_IDX_SHIFT & TARGET_GUEST_IDX_MASK, 0);
        assert_eq!(packed & TARGET_EIID_MASK, 42);
    }

    #[test]
    fn source_trigger_encoding_matches_linux() {
        assert_eq!(SourceTrigger::Inactive.to_sm(), 0x0);
        assert_eq!(SourceTrigger::EdgeRise.to_sm(), 0x4);
        assert_eq!(SourceTrigger::LevelHigh.to_sm(), 0x6);
    }

    #[test]
    fn source_offset_is_one_indexed() {
        let s1 = NonZeroU32::new(1).unwrap();
        let s2 = NonZeroU32::new(2).unwrap();
        assert_eq!(Aplic::source_offset(s1, SOURCECFG_BASE), 0x0004);
        assert_eq!(Aplic::source_offset(s2, SOURCECFG_BASE), 0x0008);
        assert_eq!(Aplic::source_offset(s1, TARGET_BASE), 0x3004);
    }

    #[test]
    fn pack_smsicfg_addrh_encodes_geometry_and_base_ppn() {
        // L 位（bit31）必须置位；各几何字段按 CFGADDRH_* 段打包。
        let cfg = MsiConfig {
            base_ppn: 0x0000_0001_2345_6789, // 高 32 位 = 0x1 → BAPPN 段
            lhxw: 2,
            lhxs: 3,
            hhxw: 4,
            hhxs: 5,
        };
        let v = pack_smsicfg_addrh(&cfg);
        assert_eq!(v & CFGADDRH_L, CFGADDRH_L, "L 位必须置位");
        assert_eq!(
            (v >> CFGADDRH_HHXS_SHIFT) & CFGADDRH_HHXS_MASK,
            5,
            "HHXS 段"
        );
        assert_eq!(
            (v >> CFGADDRH_LHXS_SHIFT) & CFGADDRH_LHXS_MASK,
            3,
            "LHXS 段"
        );
        assert_eq!(
            (v >> CFGADDRH_HHXW_SHIFT) & CFGADDRH_HHXW_MASK,
            4,
            "HHXW 段"
        );
        assert_eq!(
            (v >> CFGADDRH_LHXW_SHIFT) & CFGADDRH_LHXW_MASK,
            2,
            "LHXW 段"
        );
        assert_eq!(v & CFGADDRH_BAPPN_MASK, 1, "BAPPN = base_ppn 高 32 位");
    }

    #[test]
    fn pack_smsicfg_addrh_masks_overflowing_fields() {
        // 字段超出 mask 的高位必须被丢弃，不得污染相邻段。
        let cfg = MsiConfig {
            base_ppn: 0xffff_ffff_ffff_ffff,
            lhxw: 0xff,
            lhxs: 0xff,
            hhxw: 0xff,
            hhxs: 0xff,
        };
        let v = pack_smsicfg_addrh(&cfg);
        assert_eq!(
            (v >> CFGADDRH_HHXS_SHIFT) & CFGADDRH_HHXS_MASK,
            CFGADDRH_HHXS_MASK
        );
        assert_eq!(
            (v >> CFGADDRH_LHXS_SHIFT) & CFGADDRH_LHXS_MASK,
            CFGADDRH_LHXS_MASK
        );
        assert_eq!(
            (v >> CFGADDRH_HHXW_SHIFT) & CFGADDRH_HHXW_MASK,
            CFGADDRH_HHXW_MASK
        );
        assert_eq!(
            (v >> CFGADDRH_LHXW_SHIFT) & CFGADDRH_LHXW_MASK,
            CFGADDRH_LHXW_MASK
        );
        assert_eq!(v & CFGADDRH_BAPPN_MASK, CFGADDRH_BAPPN_MASK);
    }

    #[test]
    fn from_fdt_interrupt_spec_maps_posix_flags() {
        // 回归锁定：0x04 → LevelHigh。此前曾因"PCI INTx 必为 level-low"的
        // 错误推论被绕过、硬编码为 LevelLow，导致 QEMU virt 下 rectified 信号
        // 反极性、virtio-blk INTx 永不触发（见 commit 15d166ba8）。
        assert_eq!(
            SourceTrigger::from_fdt_interrupt_spec(0x01),
            Some(SourceTrigger::EdgeRise)
        );
        assert_eq!(
            SourceTrigger::from_fdt_interrupt_spec(0x02),
            Some(SourceTrigger::EdgeFall)
        );
        assert_eq!(
            SourceTrigger::from_fdt_interrupt_spec(0x04),
            Some(SourceTrigger::LevelHigh)
        );
        assert_eq!(
            SourceTrigger::from_fdt_interrupt_spec(0x08),
            Some(SourceTrigger::LevelLow)
        );
    }

    #[test]
    fn from_fdt_interrupt_spec_rejects_unknown_and_zero() {
        // 未知 flag / 缺省（0）必须返回 None，由调用方决定 fallback
        // （somehal 集成层 fallback 为 LevelHigh）。禁止猜测成具体触发类型。
        assert_eq!(SourceTrigger::from_fdt_interrupt_spec(0), None);
        assert_eq!(SourceTrigger::from_fdt_interrupt_spec(0x10), None);
        assert_eq!(SourceTrigger::from_fdt_interrupt_spec(u32::MAX), None);
    }
}
