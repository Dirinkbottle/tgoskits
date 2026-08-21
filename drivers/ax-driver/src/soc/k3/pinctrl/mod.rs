//! SpacemiT K3 pinctrl 驱动（rdrive 注册的 `PinctrlDevice`）。
//!
//! 对照 Linux `drivers/pinctrl/spacemit/pinctrl-k1.c`（K3 路径）移植，支持：
//! - **mux**：每个 pin 的 MFPR `PAD_MUX`（bit0:2），写法 `(old & !MUX) | new_mux`
//! - **bias**：bias-disable / bias-pull-up / bias-pull-down（PULL_EN | PULLUP/PULLDN）
//! - **drive-strength**：按 K3 1V8/3V3 16 项查表（`spacemit_get_ds_value` 语义）
//! - **power-source**：IO 电源域选择（APBC ASAR 一次性解锁 + io_pd_reg 写 1.8V/3.3V）
//! - **input-schmitt**：`PAD_SCHMITT_K3`（BIT(8)）
//!
//! 注册成 `PinctrlDevice` 后，rdrive 框架在每个 consumer probe 前自动调
//! `apply_default_pinctrl`（`rdrive/src/probe/fdt/mod.rs:1065`），按 consumer 的
//! `pinctrl-0` 引用解析 `*-cfg` 配置节点、生成 `PinState`、调本驱动 `apply_*`。
//! 故 consumer（GMAC/UART/I2C…）无需自己碰 pinctrl 寄存器。

extern crate alloc;

use alloc::format;
use core::ptr::NonNull;

use fdt_edit::{Fdt, Phandle, RegFixed};
use log::info;
use rdif_pinctrl::{
    Bias, ConfigSetting, ConfigTarget, FunctionId, Interface as RdifPinctrl, MuxSetting, PinConfig,
    PinState, PinctrlDevice, PinctrlError,
};
use rdrive::{
    DriverGeneric,
    probe::OnProbeError,
    register::{ProbeFdt, ProbeKind, ProbeLevel, ProbePriority},
};

use crate::mmio::iomap;

mod fdt_parser;
mod regs;

pub use fdt_parser::K3FdtPinctrlParser;
use regs::{
    APBC_ASFAR, APBC_ASFAR_AKEY, APBC_ASSAR, APBC_ASSAR_AKEY, IO_PWR_DOMAIN_V18EN, K3_DS_1V8,
    K3_DS_3V3, PAD_DRIVE_K3, PAD_DRIVE_K3_SHIFT, PAD_MUX, PAD_PULL_EN, PAD_PULLDOWN, PAD_PULLUP,
    PAD_SCHMITT_K3, PAD_STRONG_PULL, ds_to_val, pin_to_io_pd_offset, pin_to_offset,
};

/// Vendor config param：纯 IO 电源域设置（value = mV，1800/3300）。
/// 用于有 power-source 但无 drive-strength 的场景。
pub(super) const K3_PIN_CONFIG_POWER_SOURCE: u32 = 1;

/// Vendor config param：合并 drive-strength + power-source（value = `(mA << 16) | mV`）。
///
/// Linux 在 `spacemit_pinconf_generate_config` 里一次性收齐该 pin 的所有 configs 再
/// 算 drive-strength（需要 voltage 选 1V8/3V3 表），但 rdif 的 `apply_config` 是
/// 逐个 config 调用。为精确复刻 Linux 语义（drive-strength 应用时同步配 IO 电源域），
/// parser 把 drive-strength + power-source 合并成此单个 Vendor config。
pub(super) const K3_PIN_CONFIG_DRIVE_WITH_VOLTAGE: u32 = 2;

/// Vendor config param：施密特触发器（value = 0/1）。
pub(super) const K3_PIN_CONFIG_INPUT_SCHMITT: u32 = 3;

/// Vendor config param：强上拉（value 未使用）。
///
/// 对应 DTS `bias-pull-up = <0x01>`。rdif 的 `Bias::PullUp` 无字段携带 arg，无法
/// 区分普通/强上拉，故 parser 把带 arg==1 的 `bias-pull-up` 翻译成此 vendor config
/// （对照上游 pinctrl-k1.c `PIN_CONFIG_BIAS_PULL_UP` 的 `if (arg == 1)` 分支）。
/// apply 时设 `PULL_EN | PULLUP | STRONG_PULL`（强上拉在硬件上是普通上拉的超集）。
pub(super) const K3_PIN_CONFIG_STRONG_PULL: u32 = 4;

// ============================================================================
// model_register!
// ============================================================================

crate::model_register!(
    name: "SpacemiT K3 PinCtrl",
    level: ProbeLevel::PostKernel,
    priority: ProbePriority::CLK,
    probe_kinds: &[
        ProbeKind::Fdt {
            compatibles: &["spacemit,k3-pinctrl"],
            on_probe: probe,
        }
    ],
);

// ============================================================================
// Mmio：轻量 MMIO 封装（仿 net/k3_gmac/regs.rs::Mmio）
// ============================================================================

/// 持有一个已 ioremap 的设备 MMIO 基址，提供 read/write。
struct Mmio {
    base: NonNull<u8>,
}

// SAFETY: Mmio 仅持有设备 MMIO 基址指针，通过 volatile 访问设备内存；无可变状态，
// 可安全跨线程共享（Sync/Send 的实际安全性由调用方保证同一时刻单一可变访问）。
unsafe impl Send for Mmio {}
unsafe impl Sync for Mmio {}

impl Mmio {
    /// # Safety
    /// `base` 必须指向一段已 ioremap 的有效设备 MMIO 区域，且其生命周期不短于本
    /// `Mmio` 的所有使用。
    const unsafe fn new(base: NonNull<u8>) -> Self {
        Self { base }
    }

    fn read(&self, offset: u32) -> u32 {
        // SAFETY: 调用方在构造时保证 base 指向有效 MMIO；read_volatile 保证不被优化消除。
        unsafe {
            self.base
                .as_ptr()
                .add(offset as usize)
                .cast::<u32>()
                .read_volatile()
        }
    }

    fn write(&self, offset: u32, value: u32) {
        // SAFETY: 同 read。
        unsafe {
            self.base
                .as_ptr()
                .add(offset as usize)
                .cast::<u32>()
                .write_volatile(value);
        }
    }
}

// ============================================================================
// K3Pinctrl：驱动主结构
// ============================================================================

/// K3 pinctrl 驱动实例。probe 时持三段 MMIO：
/// - `pad`：MFPR 寄存器组（regs[0]，0xd401e000）—— pin N 在 `pad + N*4`
/// - `io_pd`：IO 电源域寄存器组（regs[1]，0xd401e800）
/// - `apbc`：APBC syscon（spacemit,apbc 指向的 syscon_apbc，0xd4015000）—— ASAR 解锁用
pub struct K3Pinctrl {
    pad: Mmio,
    io_pd: Mmio,
    apbc: Mmio,
    /// `spacemit,apbc = <phandle offset>` 的 offset（= 0x50）：ASAR 解锁寄存器在
    /// `apbc + asar_offset + ASFAR/ASSAR`。
    asar_offset: u32,
    driver_name: &'static str,
}

impl K3Pinctrl {
    /// mux 写入（pinctrl-k1.c:656-658）：清 `PAD_MUX`，保留其余位，写新 mux。
    fn set_mux(&self, pin: u32, mux: u32) {
        let off = pin_to_offset(pin);
        let old = self.pad.read(off);
        self.pad.write(off, (old & !PAD_MUX) | (mux & PAD_MUX));
    }

    /// config 写入（pinctrl-k1.c:793-794）：保留 `PAD_MUX`，替换其余所有位。
    ///
    /// 注意：这是"构建新 config 字"模型——`value` 仅含本次要设置的位（从 0 起步
    /// 累加），非 mux 位被整体替换。这与 Linux `spacemit_pin_set_config` 一致。
    fn set_config_value(&self, pin: u32, value: u32) {
        let off = pin_to_offset(pin);
        let old = self.pad.read(off);
        self.pad.write(off, (old & PAD_MUX) | value);
    }

    /// IO 电源域配置（pinctrl-k1.c:479-509）：
    /// 1. 写 ASFAR=0xbaba + ASSAR=0xeb10（AIB Secure Access 一次性解锁）
    /// 2. 立即写 io_pd 寄存器：1V8 → BIT(2)，3V3 → 0
    ///
    /// ASAR 解锁只允许**恰好一次**紧随其后的 io_pd 访问；访问完成后 keys 自动清零、
    /// 寄存器重新上锁。故每次写 io_pd 都必须重发解锁序列。
    fn set_io_pwr_domain(&self, pin: u32, is_1v8: bool) {
        let off = pin_to_io_pd_offset(pin);
        if off == 0 {
            // 该 pin 无独立 IO 电源域寄存器（fixed-voltage 或 reserved pin）。
            return;
        }
        self.apbc
            .write(self.asar_offset + APBC_ASFAR, APBC_ASFAR_AKEY);
        self.apbc
            .write(self.asar_offset + APBC_ASSAR, APBC_ASSAR_AKEY);
        let val = if is_1v8 { IO_PWR_DOMAIN_V18EN } else { 0 };
        self.io_pd.write(off, val);
    }

    /// bias → config 位（pinctrl-k1.c:720-734）。
    fn bias_bits(bias: Bias) -> u32 {
        match bias {
            // bias-disable：清 PULL_EN | PULLDN | PULLUP | STRONG_PULL（生成 0）。
            Bias::Disabled => 0,
            Bias::PullDown => PAD_PULL_EN | PAD_PULLDOWN,
            Bias::PullUp => PAD_PULL_EN | PAD_PULLUP,
            // BusHold / PullPinDefault：K3 硬件不支持，按 disable 处理。
            _ => 0,
        }
    }

    /// drive-strength（mA）+ voltage（mV）→ drive 位（pinctrl-k1.c:751-773）。
    ///
    /// 返回写入 MFPR 的 drive 位（已左移到 `PAD_DRIVE_K3` 字段）。
    fn drive_bits(ma: u32, mv: u32) -> u32 {
        let table = if mv == 1800 {
            &K3_DS_1V8[..]
        } else {
            &K3_DS_3V3[..]
        };
        let val = ds_to_val(table, ma);
        (val << PAD_DRIVE_K3_SHIFT) & PAD_DRIVE_K3
    }

    /// input-schmitt（0/1）→ config 位（pinctrl-k1.c:739-741）。
    fn schmitt_bits(arg: u32) -> u32 {
        if arg != 0 { PAD_SCHMITT_K3 } else { 0 }
    }
}

/// 检查 pin id 是否在 K3 有效范围内（0..=144，pinctrl-k1.c 的 K3 pin 数据库）。
///
/// 用于 `validate_state`/`can_mux` 的范围检查，替代完整的 pin name 数据库
/// （省 ~300 行，对功能无影响）。pin 145-152 是 gap，153+ 是 eMMC pin（走 APMU 路径，
/// 本驱动不处理）。
fn k3_pin_valid(pin: u32) -> bool {
    pin <= 144
}

// SAFETY: 内部仅持 MMIO 指针，通过 volatile 访问；无可变共享状态。
unsafe impl Send for K3Pinctrl {}

impl DriverGeneric for K3Pinctrl {
    fn name(&self) -> &str {
        self.driver_name
    }

    fn raw_any(&self) -> Option<&dyn core::any::Any> {
        Some(self)
    }

    fn raw_any_mut(&mut self) -> Option<&mut dyn core::any::Any> {
        Some(self)
    }
}

impl RdifPinctrl for K3Pinctrl {
    /// K3 是 per-pin mux：group == pin id，function == mux value（0-7，3 bits）。
    /// 不查静态 function/group 表（Linux 也是运行时从 DTS 构建），用 pin id 范围检查。
    fn can_mux(&self, group: rdif_pinctrl::GroupId, function: FunctionId) -> bool {
        k3_pin_valid(group.raw()) && function.raw() <= 0x7
    }

    /// 用 pin id 范围检查替代默认的 groups()/functions() 查表
    /// （默认实现会因 groups() 返回空 slice 而拒绝所有 group，见
    /// rdif-pinctrl/src/interface.rs:210-228）。
    fn validate_state(&self, state: &PinState) -> Result<(), PinctrlError> {
        use rdif_pinctrl::GroupId;
        for mux in state.muxes() {
            if !k3_pin_valid(mux.group.raw()) {
                return Err(PinctrlError::InvalidGroup(GroupId::new(mux.group.raw())));
            }
            if !self.can_mux(mux.group, mux.function) {
                return Err(PinctrlError::InvalidMux {
                    group: mux.group,
                    function: mux.function,
                });
            }
        }
        for config in state.configs() {
            match config.target {
                ConfigTarget::Pin(pin) => {
                    if !k3_pin_valid(pin.raw()) {
                        return Err(PinctrlError::InvalidPin(pin));
                    }
                }
                ConfigTarget::Group(group) => {
                    if !k3_pin_valid(group.raw()) {
                        return Err(PinctrlError::InvalidGroup(group));
                    }
                }
            }
        }
        Ok(())
    }

    fn apply_mux(&mut self, setting: &MuxSetting) -> Result<(), PinctrlError> {
        self.set_mux(setting.group.raw(), setting.value.raw());
        Ok(())
    }

    fn apply_config(&mut self, setting: &ConfigSetting) -> Result<(), PinctrlError> {
        let pin = match setting.target {
            ConfigTarget::Pin(p) => p.raw(),
            ConfigTarget::Group(g) => g.raw(),
        };
        match setting.config {
            PinConfig::Bias(bias) => {
                self.set_config_value(pin, Self::bias_bits(bias));
                Ok(())
            }
            // 纯 power-source（无 drive-strength）：仅配 IO 电源域。
            PinConfig::Vendor { param, value } if param == K3_PIN_CONFIG_POWER_SOURCE => {
                if value != 1800 && value != 3300 {
                    return Err(PinctrlError::other(format!(
                        "invalid power-source {value} (expect 1800/3300)"
                    )));
                }
                self.set_io_pwr_domain(pin, value == 1800);
                Ok(())
            }
            // drive-strength + power-source 合并（复刻 Linux generate+finalize）。
            // value = (mA << 16) | mV
            PinConfig::Vendor { param, value } if param == K3_PIN_CONFIG_DRIVE_WITH_VOLTAGE => {
                let ma = value >> 16;
                let mv = value & 0xffff;
                if mv != 1800 && mv != 3300 {
                    return Err(PinctrlError::other(format!(
                        "invalid power-source {mv} (expect 1800/3300)"
                    )));
                }
                // 先选 IO 电源域（EXTERNAL pin 必需），再写 drive 位。
                self.set_io_pwr_domain(pin, mv == 1800);
                let drive = Self::drive_bits(ma, mv);
                self.set_config_value(pin, drive);
                Ok(())
            }
            PinConfig::Vendor { param, value } if param == K3_PIN_CONFIG_INPUT_SCHMITT => {
                self.set_config_value(pin, Self::schmitt_bits(value));
                Ok(())
            }
            // 强上拉（bias-pull-up = <1>）：设 PULL_EN | PULLUP | STRONG_PULL。
            // value 未使用（上游仅按 arg==1 触发，parser 已负责判定）。
            PinConfig::Vendor { param, value: _ } if param == K3_PIN_CONFIG_STRONG_PULL => {
                self.set_config_value(pin, PAD_PULL_EN | PAD_PULLUP | PAD_STRONG_PULL);
                Ok(())
            }
            _ => Err(PinctrlError::NotSupported),
        }
    }
}

// ============================================================================
// probe
// ============================================================================

fn probe(probe: ProbeFdt<'_>) -> Result<(), OnProbeError> {
    let (info, plat_dev) = probe.into_parts();
    let node = info.node;

    // 两个 reg：regs[0]=MFPR(0xd401e000/0x400)，regs[1]=io_pd(0xd401e800/0x34)。
    let mut regs = node.regs().into_iter();
    let pad_reg = regs
        .next()
        .ok_or_else(|| OnProbeError::other("k3-pinctrl: missing regs[0] (MFPR)"))?;
    let io_pd_reg = regs
        .next()
        .ok_or_else(|| OnProbeError::other("k3-pinctrl: missing regs[1] (io_pd)"))?;
    let pad = unsafe { Mmio::new(map_reg(pad_reg, "MFPR")?) };
    let io_pd = unsafe { Mmio::new(map_reg(io_pd_reg, "io_pd")?) };

    // spacemit,apbc = <phandle offset>：解析 syscon_apbc 节点 reg + asar offset。
    let (apbc_phandle, asar_offset) = parse_apbc_phandle(node.as_node())?;
    let fdt = live_fdt()?;
    let apbc_node = fdt.get_by_phandle(apbc_phandle).ok_or_else(|| {
        OnProbeError::other(format!(
            "k3-pinctrl: spacemit,apbc phandle {apbc_phandle:?} not found"
        ))
    })?;
    let apbc_reg = apbc_node
        .regs()
        .into_iter()
        .next()
        .ok_or_else(|| OnProbeError::other("k3-pinctrl: apbc syscon node has no reg"))?;
    let apbc = unsafe { Mmio::new(map_reg(apbc_reg, "apbc")?) };

    plat_dev.register(PinctrlDevice::with_fdt_parser(
        K3Pinctrl {
            pad,
            io_pd,
            apbc,
            asar_offset,
            driver_name: "SpacemiT K3 PinCtrl",
        },
        K3FdtPinctrlParser,
    ));
    info!(
        "k3-pinctrl: registered (pad={:#x}, io_pd={:#x}, apbc={:#x}+{:#x})",
        pad_reg.address, io_pd_reg.address, apbc_reg.address, asar_offset
    );
    Ok(())
}

/// 解析 `spacemit,apbc = <phandle offset>`，返回 (phandle, asar_offset)。
/// `offset` 缺失时默认 0x50（k3.dtsi 的值）。
fn parse_apbc_phandle(node: &fdt_edit::Node) -> Result<(Phandle, u32), OnProbeError> {
    let prop = node
        .get_property("spacemit,apbc")
        .ok_or_else(|| OnProbeError::other("k3-pinctrl: missing spacemit,apbc property"))?;
    let mut cells = prop.get_u32_iter();
    let raw = cells
        .next()
        .ok_or_else(|| OnProbeError::other("k3-pinctrl: malformed spacemit,apbc"))?;
    let offset = cells.next().unwrap_or(0x50);
    Ok((Phandle::from(raw), offset))
}

/// 取 rdrive 当前活跃的 FDT（用于跨节点 phandle 解析）。
fn live_fdt() -> Result<Fdt, OnProbeError> {
    rdrive::with_fdt(Clone::clone)
        .ok_or_else(|| OnProbeError::other("k3-pinctrl: live FDT not found"))
}

/// iomap 一个 reg，size 缺失时用默认值。
fn map_reg(reg: RegFixed, context: &str) -> Result<NonNull<u8>, OnProbeError> {
    let size = reg.size.unwrap_or(0x400) as usize;
    iomap(reg.address as usize, size.max(1))
        .map_err(|e| OnProbeError::other(format!("k3-pinctrl: iomap {context} failed: {e:?}")))
}
