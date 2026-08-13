//! K3 pinctrl FDT 解析器：把 DTS 的 `*-cfg` 容器节点（含若干 `*-pins` 子节点）
//! 翻译成 rdif 中性的 `PinState`（`MuxSetting` + `ConfigSetting`）。
//!
//! ## DTS 格式（com260-ifx.dts 的 gmac1-cfg 为例）
//!
//! ```dts
//! gmac1-cfg {                      // ← 框架传给 parse_pinctrl_node 的 node
//!     phandle = <0xce>;
//!     gmac1-0-pins {               // ← 子节点（一组同配置 pins）
//!         pinmux = <0x150001 0x160001 ...>;   // 每个 u32 = (pin_id<<16)|mux
//!         drive-strength = <0x19>;            // mA = 25
//!         bias-disable;
//!         power-source = <0x708>;             // 1800mV = 1.8V
//!     };
//!     gmac1-1-pins { ... };
//! };
//! ```
//!
//! ## pinmux 编码（pinctrl-k1.c:266-274）
//!
//! 每个 u32 cell：`pin_id = raw >> 16`，`mux = raw & 0xffff`（写入 MFPR 时仅低 3 位有效）。

extern crate alloc;

use alloc::vec::Vec;

use fdt_edit::{Fdt, NodeType};
use log::warn;
use rdif_pinctrl::{
    Bias, ConfigSetting, FdtPinctrlParser, FunctionId, GpioLineId, GroupId, MuxSetting, MuxValue,
    PinConfig, PinId, PinState, PinctrlError,
};
use rdrive::probe::fdt::child_nodes;

use super::{
    K3_PIN_CONFIG_DRIVE_WITH_VOLTAGE, K3_PIN_CONFIG_INPUT_SCHMITT, K3_PIN_CONFIG_POWER_SOURCE,
    K3_PIN_CONFIG_STRONG_PULL,
};

/// K3 pinctrl FDT 解析器。
pub struct K3FdtPinctrlParser;

impl FdtPinctrlParser for K3FdtPinctrlParser {
    fn parse_pinctrl_node(
        &self,
        _fdt: &Fdt,
        node: NodeType<'_>,
        state: &mut PinState,
    ) -> Result<(), PinctrlError> {
        // 框架传入的 node = pinctrl-0 指向的 config 容器（如 gmac1-cfg）。
        // 遍历其子节点（gmac1-0-pins 等），每个子节点是一组同配置 pins。
        for pins_node in child_nodes(node) {
            append_k3_pins(pins_node, state)?;
        }
        Ok(())
    }

    fn parse_gpio_line(
        &self,
        _fdt: &Fdt,
        _consumer: &fdt_edit::Node,
        _prop_name: &str,
    ) -> Option<Result<GpioLineId, PinctrlError>> {
        None
    }

    fn gpio_lines_from_state(&self, _state: &PinState) -> Result<Vec<GpioLineId>, PinctrlError> {
        Ok(Vec::new())
    }
}

/// 把一个 `*-pins` 子节点的内容追加到 `PinState`。
fn append_k3_pins(node: NodeType<'_>, state: &mut PinState) -> Result<(), PinctrlError> {
    let inner = node.as_node();

    // pinmux 属性：每个 u32 = (pin_id << 16) | mux。
    let Some(pinmux_prop) = inner.get_property("pinmux") else {
        // 无 pinmux 的子节点（纯配置容器）跳过。
        return Ok(());
    };
    let pinmux: Vec<u32> = pinmux_prop.get_u32_iter().collect();
    if pinmux.is_empty() {
        return Ok(());
    }

    // 读配置属性（这些属性对组内所有 pin 共享——见 pinctrl-k1.c:814-838 group_set）。
    let bias = parse_bias(inner);
    let drive_ma = inner
        .get_property("drive-strength")
        .and_then(|p| p.get_u32());
    let power_mv = inner.get_property("power-source").and_then(|p| p.get_u32());
    let schmitt = inner
        .get_property("input-schmitt")
        .and_then(|p| p.get_u32());

    for raw in pinmux {
        // pinctrl-k1.c:266-274：pin_id = raw >> 16，mux = raw & 0xffff。
        let pin_id = raw >> 16;
        let mux = raw & 0xffff;
        let pin = PinId::new(pin_id);
        let group = GroupId::new(pin_id);

        state.push_mux(MuxSetting::new(
            group,
            FunctionId::new(mux),
            MuxValue::new(mux),
        ));

        match bias {
            Some(K3Bias::Normal(b)) => {
                state.push_config(ConfigSetting::pin(pin, PinConfig::Bias(b)));
            }
            // bias-pull-up = <1>：强上拉。走 vendor config（带 STRONG_PULL 位）。
            // rdif 的 Bias::PullUp 无字段携带 arg，故单独成路径（对照上游
            // pinctrl-k1.c PIN_CONFIG_BIAS_PULL_UP 的 if (arg==1) 分支）。
            Some(K3Bias::StrongPullUp) => {
                state.push_config(ConfigSetting::pin(
                    pin,
                    PinConfig::Vendor {
                        param: K3_PIN_CONFIG_STRONG_PULL,
                        value: 0,
                    },
                ));
            }
            None => {}
        }

        // drive-strength + power-source 合并：精确复刻 Linux generate+finalize 语义
        // （Linux 在一次 pinconf_generate_config 里收齐所有 config 后才算 drive，
        //  且对 IO_TYPE_EXTERNAL pin 同步调 set_io_pwr_domain）。
        if let (Some(ma), Some(mv)) = (drive_ma, power_mv) {
            state.push_config(ConfigSetting::pin(
                pin,
                PinConfig::Vendor {
                    param: K3_PIN_CONFIG_DRIVE_WITH_VOLTAGE,
                    value: (ma << 16) | (mv & 0xffff),
                },
            ));
        } else if let Some(mv) = power_mv {
            // 有 power-source 无 drive-strength：单独配 IO 电源域。
            state.push_config(ConfigSetting::pin(
                pin,
                PinConfig::Vendor {
                    param: K3_PIN_CONFIG_POWER_SOURCE,
                    value: mv,
                },
            ));
        } else if drive_ma.is_some() {
            // 有 drive-strength 无 power-source：EXTERNAL pin 在 Linux 会 panic
            // （spacemit_pctrl_check_power）。这里降级为 warn，按 3V3 默认处理。
            warn!(
                "k3-pinctrl: [{}] has drive-strength but no power-source; assuming 3V3",
                inner.name()
            );
        }

        if let Some(s) = schmitt {
            state.push_config(ConfigSetting::pin(
                pin,
                PinConfig::Vendor {
                    param: K3_PIN_CONFIG_INPUT_SCHMITT,
                    value: s,
                },
            ));
        }
    }
    Ok(())
}

/// bias 解析结果。
///
/// `bias-pull-up = <1>`（带 arg==1）走 `StrongPullUp`（设 PAD_STRONG_PULL），
/// 其它 `bias-pull-up`（无值或 arg!=1）走普通 `Bias::PullUp`。对照上游
/// pinctrl-k1.c `PIN_CONFIG_BIAS_PULL_UP` 的 `if (arg == 1) v |= PAD_STRONG_PULL`。
enum K3Bias {
    Normal(Bias),
    StrongPullUp,
}

/// 解析 bias-* 属性。
///
/// DTS 约定（pinctrl-k1.yaml + generic pinconf）：`bias-pull-up` 可带一个 u32 arg，
/// arg==1 表示强上拉（硬件置 `PAD_STRONG_PULL`）。无值或 arg!=1 为普通上拉。
fn parse_bias(node: &fdt_edit::Node) -> Option<K3Bias> {
    if node.get_property("bias-disable").is_some() {
        Some(K3Bias::Normal(Bias::Disabled))
    } else if let Some(prop) = node.get_property("bias-pull-up") {
        // 带 arg==1 的 bias-pull-up = <1> → 强上拉；其它（无值 / arg!=1）→ 普通上拉。
        let arg = prop.get_u32();
        if arg == Some(1) {
            Some(K3Bias::StrongPullUp)
        } else {
            Some(K3Bias::Normal(Bias::PullUp))
        }
    } else if node.get_property("bias-pull-down").is_some() {
        Some(K3Bias::Normal(Bias::PullDown))
    } else {
        None
    }
}
