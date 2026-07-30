//! RISC-V AIA (IMSIC + APLIC) OS glue for somehal.
//!
//! 镜像 `plic.rs` 的分层：本模块做 FDT probe / IRQ 域注册 / trap 分发接线，
//! 寄存器语义在 `ax-riscv-imsic` / `ax-riscv-aplic` crate。
//!
//! 域层级：IMSIC = 根域（claim 经 stopei），APLIC = IMSIC 子域（wired 源转 MSI）。

use alloc::{format, vec::Vec};
use core::num::NonZeroU32;

use ax_riscv_aplic::{Aplic, MsiConfig, MsiTarget, SourceTrigger};
use ax_riscv_imsic::{self, ImsicGeometry};
use kernutil::StaticCell;
use rdif_intc::Interface;
use rdif_msi::{
    Interface as MsiInterface, IrqAffinity as MsiIrqAffinity, Msi, MsiAllocation, MsiEventId,
    MsiMessage, MsiProviderId, MsiRequest, MsiVector, MsiVectorIndex,
};
use rdrive::{
    module_driver,
    probe::OnProbeError,
    register::ProbeFdt,
    DriverGeneric,
};

use crate::{
    common::ioremap,
    irq::{
        alloc_child_irq_domain, alloc_irq_domain, domain_by_kind_fast, map_irq_route, HwIrq,
        IrqDomainKind, IrqId,
    },
};

use ax_kspin::SpinNoIrq;

use crate::irq::IrqDomainId;

static IMSIC_STATE: StaticCell<ImsicState> = StaticCell::uninit();

/// Unified EID allocator shared by APLIC wired sources and PCI MSI-X vectors.
///
/// Both consumers draw from the same pool so EIDs can never collide. EID 0 is
/// reserved for IPI; allocation starts at 1. Backed by a bitmap with O(1)
/// free and a next-hint fast path.
struct EidAllocator {
    bitmap: SpinNoIrq<EidBitmap>,
    max: u32,
}

struct EidBitmap {
    bits: Vec<u64>,
    next_hint: u32,
}

impl EidAllocator {
    fn new(max: u32) -> Self {
        let words = (max as usize).div_ceil(64).max(1);
        Self {
            bitmap: SpinNoIrq::new(EidBitmap {
                bits: vec![0u64; words],
                next_hint: 1,
            }),
            max,
        }
    }

    fn allocate(&self) -> Option<u32> {
        let mut bmp = self.bitmap.lock();
        // Fast path: scan from next_hint to max.
        for eid in bmp.next_hint..self.max {
            let (word, bit) = (eid as usize / 64, 1u64 << (eid % 64));
            if bmp.bits[word] & bit == 0 {
                bmp.bits[word] |= bit;
                bmp.next_hint = eid + 1;
                return Some(eid);
            }
        }
        // Slow path: wrap around from 1 (skip EID 0 = IPI).
        for eid in 1..bmp.next_hint {
            let (word, bit) = (eid as usize / 64, 1u64 << (eid % 64));
            if bmp.bits[word] & bit == 0 {
                bmp.bits[word] |= bit;
                bmp.next_hint = eid + 1;
                return Some(eid);
            }
        }
        None
    }

    fn free(&self, eid: u32) {
        if eid == 0 || eid >= self.max {
            return;
        }
        let mut bmp = self.bitmap.lock();
        let (word, bit) = (eid as usize / 64, 1u64 << (eid % 64));
        bmp.bits[word] &= !bit;
        if eid < bmp.next_hint {
            bmp.next_hint = eid;
        }
    }
}

struct ImsicState {
    geometry: ImsicGeometry,
    eid_allocator: EidAllocator,
    // 注：IMSIC / MSI-X 域 id 不在此缓存——probe 后用 domain_by_kind_fast(RiscvImsic)
    // 动态查找，避免与 somehal::irq 域注册表产生冗余真相源（曾因此触发 dead_code）。
}

fn get_imsic_state() -> Option<&'static ImsicState> {
    if IMSIC_STATE.is_init() {
        Some(&IMSIC_STATE)
    } else {
        None
    }
}

module_driver!(
    name: "RISC-V IMSIC",
    level: ProbeLevel::PreKernel,
    priority: ProbePriority::INTC,
    probe_kinds: &[ProbeKind::Fdt {
        compatibles: &["riscv,imsics"],
        on_probe: probe_imsic,
    }],
);

module_driver!(
    name: "RISC-V APLIC",
    level: ProbeLevel::PreKernel,
    priority: ProbePriority::MSI,
    probe_kinds: &[ProbeKind::Fdt {
        compatibles: &["riscv,aplic"],
        on_probe: probe_aplic,
    }],
);

fn probe_imsic(probe: ProbeFdt<'_>) -> Result<(), OnProbeError> {
    let (info, dev) = probe.into_parts();
    let reg = info
        .node
        .regs()
        .into_iter()
        .next()
        .ok_or_else(|| OnProbeError::other(format!("[{}] has no reg", info.node.name())))?;

    let geometry = ImsicGeometry {
        base_addr: reg.address as usize,
        // 位宽属性按 AIA IMSIC DT binding 可缺省：缺省即表示单 hart / 单 guest /
        // 单 group（几何相应为 0）。QEMU virt (aia=aplic-imsic) 的 IMSIC 节点就
        // 不带这几个属性，缺省值对它的单核单页布局正好正确。故缺省 0 而非报错。
        hart_index_bits: fdt_u32(&info, "riscv,hart-index-bits").unwrap_or(0),
        guest_index_bits: fdt_u32(&info, "riscv,guest-index-bits").unwrap_or(0),
        group_index_bits: fdt_u32(&info, "riscv,group-index-bits").unwrap_or(0),
        // group-index-shift 在 binding 中缺省 24。
        group_index_shift: fdt_u32(&info, "riscv,group-index-shift").unwrap_or(24),
        // num-ids 是 binding 必填且无合理缺省（identity 数因实现而异），缺失即失败。
        num_ids: fdt_u32_required(&info, "riscv,num-ids")?,
    };

    let imsic_domain = alloc_irq_domain(dev.descriptor.device_id(), IrqDomainKind::RiscvImsic)
        .map_err(|e| OnProbeError::other(format!("failed to register IMSIC domain: {e:?}")))?;

    // PCI MSI-X 子域：IMSIC 是根域，PCI MSI-X 是子域。
    // msix_domain 只在 probe 作用域内用于注册 MSI provider，不入 ImsicState。
    let msix_domain = alloc_child_irq_domain(
        dev.descriptor.device_id(),
        imsic_domain,
        IrqDomainKind::PciMsix,
    )
    .map_err(|e| OnProbeError::other(format!("failed to register MSI-X domain: {e:?}")))?;

    IMSIC_STATE.init(ImsicState {
        geometry,
        eid_allocator: EidAllocator::new(geometry.num_ids),
    });
    info!(
        "IMSIC probed: base={:#x} hart_bits={} guest_bits={} num_ids={}",
        geometry.base_addr, geometry.hart_index_bits, geometry.guest_index_bits, geometry.num_ids
    );

    // Safety: init_local_file touches per-hart IMSIC CSRs (siselect/sireg).
    // Called during PreKernel probe with interrupts disabled.
    unsafe { ax_riscv_imsic::init_local_file(geometry.num_ids) };
    // Safety: set_sext enables the S-mode external interrupt enable bit.
    unsafe { riscv::register::sie::set_sext() };

    dev.register(rdif_intc::Intc::new(imsic_domain, RiscvImsic));

    let msi_provider = ImsicMsiProvider {
        geometry,
        imsic_domain,
        msix_domain,
        eid_allocator: &IMSIC_STATE.eid_allocator,
    };
    dev.register(Msi::new(
        MsiProviderId(u64::from(dev.descriptor.device_id())),
        msi_provider,
    ));
    info!("IMSIC MSI provider registered");
    Ok(())
}

fn probe_aplic(probe: ProbeFdt<'_>) -> Result<(), OnProbeError> {
    let (info, dev) = probe.into_parts();
    let reg = info
        .node
        .regs()
        .into_iter()
        .next()
        .ok_or_else(|| OnProbeError::other(format!("[{}] has no reg", info.node.name())))?;

    let num_sources = fdt_u32(&info, "riscv,num-sources").unwrap_or(1024);
    let mmio = ioremap(reg.address, reg.size.unwrap_or(0x4000) as usize)
        .map_err(|e| OnProbeError::other(format!("failed to map APLIC: {e:?}")))?;
    // Safety: `mmio` 刚由 ioremap 映射并独占持有，`num_sources` 来自 DT（合法 source
    // 数）；满足 `Aplic::new` 的 MMIO 基址有效且独占的契约。
    let mut aplic = unsafe { Aplic::new(mmio.as_ptr(), num_sources) };

    aplic.disable_all_sources();
    aplic.clear_all_pending();

    let imsic_domain = domain_by_kind_fast(IrqDomainKind::RiscvImsic).ok_or_else(|| {
        OnProbeError::other("IMSIC domain not found; APLIC needs IMSIC to probe first")
    })?;
    let state = get_imsic_state()
        .ok_or_else(|| OnProbeError::other("IMSIC state not initialized"))?;
    let geo = &state.geometry;

    let msi_cfg = MsiConfig {
        base_ppn: ax_riscv_imsic::group_base_ppn(geo),
        lhxw: geo.hart_index_bits,
        lhxs: geo.guest_index_bits,
        hhxw: geo.group_index_bits,
        hhxs: geo
            .group_index_shift
            .saturating_sub(12 + geo.guest_index_bits + geo.hart_index_bits),
    };
    aplic.init_msi_mode(&msi_cfg);

    let domain = alloc_child_irq_domain(
        dev.descriptor.device_id(),
        imsic_domain,
        IrqDomainKind::RiscvAplic,
    )
    .map_err(|e| OnProbeError::other(format!("failed to register APLIC domain: {e:?}")))?;
    dev.register(rdif_intc::Intc::new(
        domain,
        RiscvAplicDriver {
            inner: aplic,
            num_sources,
            imsic_domain,
            aplic_domain: domain,
            eid_allocator: &state.eid_allocator,
            source_eid: SpinNoIrq::new(vec![None; num_sources as usize + 1]),
            source_trigger: SpinNoIrq::new(vec![None; num_sources as usize + 1]),
        },
    ));
    info!("APLIC probed: num_sources={num_sources}, MSI mode configured");
    Ok(())
}

fn fdt_u32(info: &rdrive::register::FdtInfo<'_>, name: &str) -> Option<u32> {
    info.node
        .as_node()
        .get_property(name)
        .and_then(|p| p.get_u32())
}

/// 读取 AIA DT binding 的必填 u32 属性，缺失则 probe 失败。
fn fdt_u32_required(
    info: &rdrive::register::FdtInfo<'_>,
    name: &str,
) -> Result<u32, OnProbeError> {
    fdt_u32(info, name).ok_or_else(|| {
        OnProbeError::other(format!(
            "[{}] 缺少必填属性 `{}`",
            info.node.name(),
            name
        ))
    })
}

pub fn is_aia_active() -> bool {
    domain_by_kind_fast(IrqDomainKind::RiscvImsic).is_some()
}

pub fn begin_external_irq() -> Option<super::plic::ActiveIrq> {
    // Safety: 在 S-mode 外部中断 trap 上下文中调用，本 hart 的 ssaia 已由 probe 期间
    // 的 init_local_file 初始化，满足 `claim`(stopei) 的本地 hart + S-mode 契约。
    let (eiid, _prio) = unsafe { ax_riscv_imsic::claim() }?;
    Some(super::plic::ActiveIrq::new_no_completion(
        (eiid as usize).into(),
    ))
}

pub fn secondary_init_intc() {
    if let Some(state) = get_imsic_state() {
        // Safety: 在副核引导早期、中断关闭上下文调用，针对本 hart 初始化中断文件。
        unsafe { ax_riscv_imsic::init_local_file(state.geometry.num_ids) };
    }
    // Safety: 仅写本 hart 的 sie CSR 使能位，无别名或并发问题。
    unsafe {
        riscv::register::sie::set_ssoft();
        riscv::register::sie::set_stimer();
        riscv::register::sie::set_sext();
    }
}

struct RiscvImsic;

impl DriverGeneric for RiscvImsic {
    fn name(&self) -> &str {
        "RISC-V IMSIC"
    }
}

impl Interface for RiscvImsic {
    fn translate_fdt(
        &self,
        _irq_prop: &[u32],
    ) -> Result<rdif_intc::ControllerIrqTranslation, rdif_intc::IrqError> {
        Err(rdif_intc::IrqError::Unsupported)
    }

    fn set_enabled(
        &mut self,
        hwirq: HwIrq,
        enabled: bool,
    ) -> Result<(), rdif_intc::IrqError> {
        // hwirq 即 IMSIC 的 EID（MSI 向量分配时由 EidAllocator 派发，APPLIC 有线源
        // 的 set_enabled 也以 EID 形式回灌到此）。此处被 IRQ 框架在注册/使能 MSI-X
        // 叶子 handler 时，经 leaf→parent 解析后调用。
        // Safety: enable_eid/disable_eid 经 siselect/sireg 间接访问本 hart 的 IMSIC
        // CSR。调用方为控制面路径（set_controller_irq_enabled 持有 Intc 锁；IRQ 框架
        // 的 request/enable 关中断运行），无并发 siselect/sireg 访问。
        unsafe {
            if enabled {
                ax_riscv_imsic::enable_eid(hwirq.0);
            } else {
                ax_riscv_imsic::disable_eid(hwirq.0);
            }
        }
        Ok(())
    }
}

struct RiscvAplicDriver {
    inner: Aplic,
    num_sources: u32,
    imsic_domain: IrqDomainId,
    aplic_domain: IrqDomainId,
    eid_allocator: &'static EidAllocator,
    source_eid: SpinNoIrq<Vec<Option<u32>>>,
    source_trigger: SpinNoIrq<Vec<Option<SourceTrigger>>>,
}

impl DriverGeneric for RiscvAplicDriver {
    fn name(&self) -> &str {
        "RISC-V APLIC"
    }
}

impl Interface for RiscvAplicDriver {
    fn translate_fdt(
        &self,
        irq_prop: &[u32],
    ) -> Result<rdif_intc::ControllerIrqTranslation, rdif_intc::IrqError> {
        let source = irq_prop
            .first()
            .copied()
            .ok_or(rdif_intc::IrqError::InvalidIrq)?;
        if source == 0 || source > self.num_sources {
            return Err(rdif_intc::IrqError::InvalidIrq);
        }
        let trigger = irq_prop.get(1).copied().and_then(fdt_flag_to_aplic_trigger);
        if let Some(trigger) = trigger {
            self.source_trigger.lock()[source as usize] = Some(trigger);
        }
        Ok(rdif_intc::ControllerIrqTranslation::new(HwIrq(source)))
    }

    fn configure(
        &mut self,
        _translation: &rdif_intc::IrqTranslation,
    ) -> Result<(), rdif_intc::IrqError> {
        Ok(())
    }

    fn supports_acpi_gsi(&self, _route: &rdif_intc::AcpiGsiRoute) -> bool {
        false
    }

    fn set_enabled(
        &mut self,
        hwirq: HwIrq,
        enabled: bool,
    ) -> Result<(), rdif_intc::IrqError> {
        let source = NonZeroU32::new(hwirq.0).ok_or(rdif_intc::IrqError::InvalidIrq)?;
        if source.get() > self.num_sources {
            return Err(rdif_intc::IrqError::InvalidIrq);
        }
        if enabled {
            let idx = source.get() as usize;
            let hart_idx = crate::cpu::current_cpu_idx().unwrap_or(0) as u32;

            // Allocate an EID from the unified IMSIC pool on first enable.
            let eid = {
                let mut guard = self.source_eid.lock();
                match guard[idx] {
                    Some(existing) => existing,
                    None => {
                        let eid = self
                            .eid_allocator
                            .allocate()
                            .ok_or(rdif_intc::IrqError::NoMemory)?;
                        guard[idx] = Some(eid);
                        eid
                    }
                }
            };

            // Safety: enable_eid touches siselect/sireg CSRs on this hart.
            // Callers must ensure no concurrent siselect/sireg access (control
            // plane path runs with IRQs disabled).
            unsafe { ax_riscv_imsic::enable_eid(eid) };

            // 触发类型优先从 FDT interrupt specifier 提取，fallback 为 LevelHigh。
            // QEMU virt GPIO 模型下，所有中断源 assert 时 level=1（raise）、deassert
            // 时 level=0（lower），因此 LevelHigh 是正确的缺省值。
            let trigger = {
                let guard = self.source_trigger.lock();
                guard[source.get() as usize]
            };
            let trigger = trigger.unwrap_or(SourceTrigger::LevelHigh);
            // 配置前先清掉之前可能残留的 pending。
            // configure_source 写 sourcecfg 时 QEMU 会基于当前 rectified 电平
            // 重新评估 pending：若设备线已 assert → pending=1，随后 enable_source
            // 立即投递 MSI，正确处理"使能时设备已就绪"的场景。
            self.inner.clear_source_pending(source);
            self.inner.configure_source(
                source,
                trigger,
                MsiTarget {
                    hart_index: hart_idx,
                    guest_index: 0,
                    eiid: eid,
                },
            );

            // Wire the IRQ route so resolve_irq_route maps stopei's EID back
            // to this APLIC source.
            map_irq_route(
                IrqId::new(self.imsic_domain, HwIrq(eid)),
                IrqId::new(self.aplic_domain, hwirq),
            )?;

            self.inner.enable_source(source);
        } else {
            self.inner.disable_source(source);
        }
        Ok(())
    }
}

struct ImsicMsiProvider {
    geometry: ImsicGeometry,
    imsic_domain: IrqDomainId,
    msix_domain: IrqDomainId,
    eid_allocator: &'static EidAllocator,
}

impl DriverGeneric for ImsicMsiProvider {
    fn name(&self) -> &str {
        "riscv-imsic-msi"
    }
}

impl MsiInterface for ImsicMsiProvider {
    fn allocate_vectors(
        &mut self,
        request: &MsiRequest,
    ) -> Result<Vec<MsiVector>, rdif_intc::IrqError> {
        if request.vector_count == 0 {
            return Err(rdif_intc::IrqError::InvalidIrq);
        }
        let mut vectors = Vec::with_capacity(usize::from(request.vector_count));
        for index in 0..request.vector_count {
            let eid = self
                .eid_allocator
                .allocate()
                .ok_or(rdif_intc::IrqError::NoMemory)?;

            let parent_irq = IrqId::new(self.imsic_domain, HwIrq(eid));
            let leaf_irq = IrqId::new(self.msix_domain, HwIrq(eid));
            crate::irq::map_irq_route(parent_irq, leaf_irq)?;

            vectors.push(MsiVector::with_parent(
                MsiVectorIndex(index),
                MsiEventId(eid),
                leaf_irq,
                parent_irq,
            ));
        }
        Ok(vectors)
    }

    fn compose_message(
        &self,
        vector: &MsiVector,
    ) -> Result<MsiMessage, rdif_intc::IrqError> {
        let hart_index = crate::cpu::current_cpu_idx().unwrap_or(0) as u32;
        let msg = ax_riscv_imsic::compose_msi_message(
            &self.geometry,
            hart_index,
            0,
            vector.event.0,
        );
        Ok(MsiMessage::new(msg.address as u64, msg.data))
    }

    fn set_vector_enabled(
        &mut self,
        vector: &MsiVector,
        enabled: bool,
    ) -> Result<(), rdif_intc::IrqError> {
        // Safety: siselect/sireg access; control-plane caller holds no
        // concurrent CSR indirect access on this hart.
        unsafe {
            if enabled {
                ax_riscv_imsic::enable_eid(vector.event.0);
            } else {
                ax_riscv_imsic::disable_eid(vector.event.0);
            }
        }
        Ok(())
    }

    fn set_vector_affinity(
        &mut self,
        _vector: &MsiVector,
        affinity: MsiIrqAffinity,
    ) -> Result<(), rdif_intc::IrqError> {
        match affinity {
            MsiIrqAffinity::Any => Ok(()),
            MsiIrqAffinity::Fixed { .. } => Err(rdif_intc::IrqError::Unsupported),
        }
    }

    fn free_vectors(
        &mut self,
        allocation: MsiAllocation,
    ) -> Result<(), rdif_intc::IrqError> {
        for vector in allocation.vectors() {
            let _ = crate::irq::unmap_irq_route(vector.parent_irq, vector.irq);
            self.eid_allocator.free(vector.event.0);
        }
        Ok(())
    }
}

fn fdt_flag_to_aplic_trigger(cell: u32) -> Option<SourceTrigger> {
    Some(match cell {
        0x01 => SourceTrigger::EdgeRise,
        0x02 => SourceTrigger::EdgeFall,
        0x04 => SourceTrigger::LevelHigh,
        0x08 => SourceTrigger::LevelLow,
        _ => return None,
    })
}
