//! SpacemiT K3 (k1-dwc3) USB3 xHCI 主机探测驱动
//!
//! 目标硬件: com260kit 板卡 usb3@81400000 (USB3 Port B, dr_mode = "host")。
//! k1-dwc3 是 DesignWare DWC3 控制器: host 模式下偏移 [0x0, 0x7fff] 为标准 xHCI
//! 寄存器接口 (交给 crab-usb xHCI 后端), 偏移 [0xc100, 0xc6ff] 为 DWC3 全局寄存器。
//!
//! 驱动流程 (镜像 Linux dwc3-generic-plat.c + dwc3_core_init + phy 驱动):
//!   1. APMU 时钟门控 + 复位 (开时钟门 -> assert -> 2us -> deassert)
//!   2. GUSB2PHYCFG0 / GUSB3PIPECTL0 预配置 (phy init 前必须清 SUSPHY)
//!   3. USB2 PHY 初始化 (phy@81500000, 参考 phy-k1-usb2.c)
//!   4. USB3 PHY 初始化 (phy@81f00000, 参考 phy-k3-usb3.c, 含 combo 复用)
//!   5. DWC3 全局寄存器配置 (GCTL power-opt / PRTCAPDIR=HOST / GUCTL1 quirks)
//!   6. 移交 crab-usb xHCI 后端并注册主机
//!
//! 说明: dr_mode 非 "host" 的 k1-dwc3 节点 (如 Port A OTG) 直接跳过。

extern crate alloc;

use alloc::{format, vec::Vec};
use core::{ptr::NonNull, time::Duration};

use crab_usb::usb_if::Speed;
use fdt_edit::{Node, NodeType, Phandle, RegFixed};
use log::{error, warn};
use rdrive::{
    probe::OnProbeError,
    register::{FdtInfo, ProbeFdt},
};

use super::{ProbeFdtUsbHost, usb_kernel};
use crate::mmio::iomap;

mod regs;

use regs::*;

/// 驱动名 (用于注册设备)
const DRIVER_NAME: &str = "usb-k3com260-dwc3";

/// DWC3 控制器默认映射大小 (DT reg 0x10000)
const DWC3_MMIO_DEFAULT_SIZE: usize = 0x10000;
/// PHY 默认映射大小 (DT reg 0x200, iomap 自动对齐 4K)
const PHY_MMIO_DEFAULT_SIZE: usize = 0x200;

/// PHY PLL / RCAL 轮询时间参数
/// 参考: drivers/phy/spacemit/phy-k3-usb3.c:16-18, phy-k1-usb2.c:58
const POLL_DELAY: Duration = Duration::from_micros(500);
const PU_CAL_TIMEOUT: Duration = Duration::from_millis(2000);
const PLL_TIMEOUT: Duration = Duration::from_millis(500);
const USB2_PLL_TIMEOUT: Duration = Duration::from_millis(50);

/// USB2 PHY 初始化前等待 (保证控制器已退出复位)
/// 参考: drivers/phy/spacemit/phy-k1-usb2.c:105 (usleep_range(150, 200))
const USB2PHY_POST_RESET_DELAY: Duration = Duration::from_micros(200);

/// 复位 assert 与 deassert 之间的安全间隔
/// 参考: drivers/usb/dwc3/dwc3-generic-plat.c:206 (udelay(2))
const RESET_ASSERT_DELAY: Duration = Duration::from_micros(2);

crate::model_register!(
    name: "SpacemiT K3 (k1-dwc3) USB3 xHCI",
    level: ProbeLevel::PostKernel,
    priority: ProbePriority::DEFAULT,
    probe_kinds: &[
        ProbeKind::Fdt {
            compatibles: &["spacemit,k1-dwc3"],
            on_probe: probe
        }
    ],
);

/// DWC3 DT 节点解析出的 quirk 位 (与 Linux dwc3_core.c 解析的 DT 属性一一对应)
#[derive(Debug, Clone, Copy, Default)]
struct K3Dwc3Quirks {
    /// snps,dis_enblslpm_quirk
    dis_enblslpm: bool,
    /// snps,dis-tx-ipgap-linecheck-quirk
    dis_tx_ipgap_linecheck: bool,
    /// snps,parkmode-disable-ss-quirk
    parkmode_disable_ss: bool,
}

/// 从 FDT 收集到的全部寄存器资源
struct K3UsbResources {
    /// DWC3 控制器 MMIO 资源 (usb3@81400000)
    ctrl: RegFixed,
    /// spacemit,k3-syscon-apmu (时钟/复位/组合 PHY 矩阵)
    apmu: RegFixed,
    /// u3phy 节点引用的 spacemit,k3-pll (APB_SPARE, RCAL 校准)
    apb_spare: RegFixed,
    /// spacemit,k3-usb2-phy (UTMI PHY)
    usb2_phy: RegFixed,
    /// spacemit,k3-usb3-phy (PIPE3 组合 PHY)
    usb3_phy: RegFixed,
    /// u3phy 的 combo-usb-bit (None = 非组合 PHY)
    combo_usb_bit: Option<u32>,
    quirks: K3Dwc3Quirks,
}

fn probe(probe: ProbeFdt<'_>) -> Result<(), OnProbeError> {
    let info = probe.info();
    let node_name = info.node.name();

    // DWC3 探测入口：这里只接管 host 模式的节点。Port A 的 OTG 节点仍交给
    // 后续专用路径，Port B 的 usb3@81400000 才会继续走下面的 xHCI 初始化。
    match prop_str(info.node.as_node(), "dr_mode") {
        Some("host") => {}
        Some(_) => {
            return Err(OnProbeError::NotMatch);
        }
        None => {
            return Err(OnProbeError::NotMatch);
        }
    }

    let resources = collect_resources(info)?;

    // DT 资源已经解析完成，说明找到了这一实例的 DWC3、APMU 和两组 PHY。
    // 后续所有硬件状态验证都用 error! 打印，方便在没有更完整调试串口时定位
    // 是 DWC3 探测、时钟/复位，还是 PHY 初始化阶段停住。
    error!("[k3-usb] DWC3 probe: host node={node_name}");

    let apmu = map_reg(resources.apmu, APMU_SIZE)?;
    let usb_clk_res_ctrl = APMU_USB_CLK_RES_CTRL;

    // 第一步：打开 DWC3 Port B 的总线时钟。APMU 的 bit8 是 clock gate；
    // 没有这个时钟，DWC3/xHCI 的寄存器虽然存在，内部状态机却不会运行。
    update32(apmu, usb_clk_res_ctrl, |value| {
        value | USB3_PORTB_BUS_CLK_GATE
    });
    io_write_fence();
    let clock_status = read32(apmu, usb_clk_res_ctrl);
    error!(
        "[k3-usb] DWC3 clock configured: reg={clock_status:#010x}, enabled={}",
        clock_status & USB3_PORTB_BUS_CLK_GATE != 0,
    );

    // 第二步：从确定状态开始做一次复位。bits[11:9] 清零表示 assert reset，
    // 保持 2us 后再置 1 释放 reset；这样 DWC3 数字核心从同一个初始状态启动。
    update32(apmu, usb_clk_res_ctrl, |value| {
        value & !USB3_PORTB_RESET_DEASSERT
    });
    io_write_fence();
    axklib::time::busy_wait(RESET_ASSERT_DELAY);
    update32(apmu, usb_clk_res_ctrl, |value| {
        value | USB3_PORTB_RESET_DEASSERT
    });
    io_write_fence();
    let apmu_status = read32(apmu, usb_clk_res_ctrl);
    error!(
        "[k3-usb] DWC3 reset released: reg={apmu_status:#010x}, released={}",
        apmu_status & USB3_PORTB_RESET_DEASSERT == USB3_PORTB_RESET_DEASSERT,
    );

    // 第三步：DWC3 host 模式把同一块 MMIO 的低地址区复用为标准 xHCI。
    // 读取 CAPLENGTH、HCSPARAMS1 和 USBSTS，是直接向控制器确认“时钟已通、
    // 复位已释放、xHCI/DWC3 数字核心已经活着”，而不是只检查 DT 的 status。
    let ctrl = map_reg(resources.ctrl, DWC3_MMIO_DEFAULT_SIZE)?;
    let capbase = read32(ctrl, XHCI_CAPLENGTH);
    let hcsparams1 = read32(ctrl, XHCI_HCSPARAMS1);
    let cap_length = (capbase & XHCI_CAPLENGTH_OPREGS_MASK) as usize;
    let max_slots = hcsparams1 & 0xff;
    let usbsts = read32(ctrl, cap_length + XHCI_USBSTS);
    error!(
        "[k3-usb] xHCI registers: CAPLENGTH={cap_length:#x}, MAX_SLOTS={max_slots}, \
         USBSTS={usbsts:#010x}"
    );

    // 第四步：先让 DWC3 的 USB2/USB3 PHY 接口退出 suspend，进入 PHY 可配置的
    // P0 状态；随后分别配置 USB2 UTMI PHY 和 USB3 PIPE3 PHY 的 PLL/复位。
    dwc3_phy_iface_setup(ctrl, &resources.quirks);
    k3_usb2_phy_init(&resources)?;
    k3_usb3_phy_init(&resources)?;

    // TODO: 临时停在这里，不进入后续 xHCI 注册流程。
    // DWC3 host 模式下的 PORTSC 位于 xHCI operational registers：
    //   DWC3 base + CAPLENGTH + 0x400 + port * 0x10
    // 每秒读取所有 root port，插拔设备时观察 PORTSC 是否发生变化。
    let port_count = ((hcsparams1 & XHCI_HCSPARAMS1_MAX_PORTS_MASK) >> 24) as usize;
    error!("[k3-usb] temporary PORTSC polling: op_base={cap_length:#x}, ports={port_count}");
    if port_count == 0 {
        error!("[k3-usb] temporary PORTSC polling skipped: no root ports");
    } else {
        for port in 0..port_count {
            let portsc_offset = cap_length + XHCI_PORTSC_BASE + XHCI_PORTSC_STRIDE * port;
            let portsc = read32(ctrl, portsc_offset);
            error!(
                "[k3-usb] PORTSC{} @ {portsc_offset:#x} for bit 0 = {} ",
                port + 1,
                portsc & 0x1
            );
        }
        axklib::time::busy_wait(Duration::from_secs(1));
    }

    // 第五步：完成 DWC3 全局 host 配置，再由 xHCI 后端执行控制器级初始化并注册
    // 主机。到这里，前面的寄存器读取只是“核心活着”，这里才把它交给 USB 栈使用。
    dwc3_core_host_setup(ctrl, &resources.quirks);

    // HCRST 前关闭根端口电源, 防止复位期间 VBUS 抖动 (K3 板级要求)
    dwc3_power_off_roothub_ports(ctrl);

    let host = crab_usb::USBHost::new_xhci(ctrl, usb_kernel()).map_err(|err| {
        OnProbeError::other(format!(
            "failed to create xHCI host for [{node_name}]: {err}"
        ))
    })?;
    probe.register_usb_host_with_root_hub_speed(DRIVER_NAME, host, Speed::SuperSpeed)?;
    Ok(())
}

/// 从 FDT 收集驱动所需的全部 MMIO 资源与 quirk
fn collect_resources(info: &FdtInfo<'_>) -> Result<K3UsbResources, OnProbeError> {
    let ctrl = first_reg(info.node, "dwc3")?;
    let node = info.node.as_node();
    let apmu = phandle_reg(info, node, "spacemit,syscon-apmu", "apmu")?;
    let (usb2_phy, usb3_phy) = parse_phys(info)?;

    let usb3_node = info.get_by_phandle(usb3_phy).ok_or_else(|| {
        OnProbeError::other(format!(
            "usb3-phy phandle {usb3_phy:?} not found for [{}]",
            info.node.name()
        ))
    })?;
    let apb_spare = phandle_reg(
        info,
        usb3_node.as_node(),
        "spacemit,syscon-apb-spare",
        "apb-spare",
    )?;
    let combo_usb_bit = prop_u32(usb3_node.as_node(), "combo-usb-bit");

    let usb2_phy_reg = phandle_target_reg(info, usb2_phy, "usb2-phy")?;
    let usb3_phy_reg = phandle_target_reg(info, usb3_phy, "usb3-phy")?;
    Ok(K3UsbResources {
        ctrl,
        apmu,
        apb_spare,
        usb2_phy: usb2_phy_reg,
        usb3_phy: usb3_phy_reg,
        combo_usb_bit,
        quirks: K3Dwc3Quirks {
            dis_enblslpm: has_prop(info.node.as_node(), "snps,dis_enblslpm_quirk"),
            dis_tx_ipgap_linecheck: has_prop(
                info.node.as_node(),
                "snps,dis-tx-ipgap-linecheck-quirk",
            ),
            parkmode_disable_ss: has_prop(info.node.as_node(), "snps,parkmode-disable-ss-quirk"),
        },
    })
}

/// 通过 phy-names 解析 usb2-phy / usb3-phy 的 phandle (Port A/B 顺序不同, 不能按索引)
fn parse_phys(info: &FdtInfo<'_>) -> Result<(Phandle, Phandle), OnProbeError> {
    let node = info.node.as_node();
    let phys = node
        .get_property("phys")
        .ok_or_else(|| OnProbeError::other(format!("[{}] has no phys", node.name())))?
        .get_u32_iter()
        .map(Phandle::from)
        .collect::<Vec<_>>();
    let names = node
        .get_property("phy-names")
        .ok_or_else(|| OnProbeError::other(format!("[{}] has no phy-names", node.name())))?
        .as_str_iter()
        .collect::<Vec<_>>();

    let pick = |name: &str| -> Result<Phandle, OnProbeError> {
        names
            .iter()
            .position(|n| *n == name)
            .and_then(|i| phys.get(i).copied())
            .ok_or_else(|| {
                OnProbeError::other(format!(
                    "[{}] has no phy-names entry for {name}",
                    node.name()
                ))
            })
    };
    Ok((pick("usb2-phy")?, pick("usb3-phy")?))
}

/// 读取某节点 phandle 属性指向节点的第一个 reg
fn phandle_reg(
    info: &FdtInfo<'_>,
    node: &Node,
    prop_name: &str,
    context: &str,
) -> Result<RegFixed, OnProbeError> {
    let phandle = prop_phandle(node, prop_name)
        .ok_or_else(|| OnProbeError::other(format!("[{context}] node has no {prop_name}")))?;
    phandle_target_reg(info, phandle, context)
}

/// 读取 phandle 指向节点的第一个 reg
fn phandle_target_reg(
    info: &FdtInfo<'_>,
    phandle: Phandle,
    context: &str,
) -> Result<RegFixed, OnProbeError> {
    let node = info
        .get_by_phandle(phandle)
        .ok_or_else(|| OnProbeError::other(format!("{context} phandle {phandle:?} not found")))?;
    first_reg(node, context)
}

/// 读取节点第一个 reg
fn first_reg(node: NodeType<'_>, context: &str) -> Result<RegFixed, OnProbeError> {
    node.regs()
        .into_iter()
        .next()
        .ok_or_else(|| OnProbeError::other(format!("[{context}] has no reg")))
}

/// USB2 UTMI PHY 初始化
///
/// 参考: drivers/phy/spacemit/phy-k1-usb2.c:93-124 (spacemit_usb2phy_init)
fn k3_usb2_phy_init(resources: &K3UsbResources) -> Result<(), OnProbeError> {
    let phy = map_reg(resources.usb2_phy, PHY_MMIO_DEFAULT_SIZE)?;

    // 等待控制器退出复位后再配置 (usleep_range(150, 200))
    axklib::time::busy_wait(USB2PHY_POST_RESET_DELAY);

    // USB2 PHY +0x98：打开本地分频器，并把 24MHz 参考时钟送给 PHY PLL。
    //   fdiv_reg<21:0> = 0x1ec4 | freq_sel(24MHz)=0x1<<13 | DIV_LOCAL_EN=1<<15
    let pll_divider = U2PHY_FDIV_REG_VAL | U2PHY_SEL_FREQ_24MHZ | U2PHY_DIV_LOCAL_EN;
    write32(phy, U2PHY_PLL_DIV_CFG, pll_divider);
    error!("[k3-usb] USB2 PHY PLL configured: DIV_CFG={pll_divider:#010x}");

    // PLL 分频配置写入后，等待 RST_MODE_CTRL 的 PLL_RDY。只有 PLL lock，
    // 后面的 UTMI 时钟和 PHY 内部复位释放才有可靠的参考。
    if !poll_until(
        phy,
        U2PHY_RST_MODE_CTRL,
        U2PHY_PLL_RDY,
        POLL_DELAY,
        USB2_PLL_TIMEOUT,
    ) {
        return Err(OnProbeError::other("k3 usb2-phy: PLL ready timeout"));
    }
    error!(
        "[k3-usb] USB2 PHY PLL lock: RST_MODE_CTRL={:#010x}",
        read32(phy, U2PHY_RST_MODE_CTRL)
    );

    // USB2 PHY +0x04：释放 PLL/CDR/MAC 的 resetn，同时打开 CDR/PLL/MAC 时钟，
    // 让 PHY 内部逻辑真正开始工作。
    let reset_released = U2PHY_HS_LINE_TX_MODE
        | U2PHY_FS_LINE_TX_MODE
        | U2PHY_CLK_CDR_EN
        | U2PHY_CLK_PLL_EN
        | U2PHY_CLK_MAC_EN
        | U2PHY_PLL_RSTN
        | U2PHY_CDR_RSTN
        | U2PHY_MAC_RSTN;
    write32(phy, U2PHY_RST_MODE_CTRL, reset_released);
    error!(
        "[k3-usb] USB2 PHY reset released: RST_MODE_CTRL={:#010x}",
        read32(phy, U2PHY_RST_MODE_CTRL)
    );

    // USB2 PHY +0x34 HSTXP：打开内部 host 发射侧时钟，并置 HSTXP_RSTN
    // 释放发射侧复位；HSTXP_MODE 保持 host 发射路径模式。
    let tx_path_enabled = U2PHY_HSTXP_RSTN | U2PHY_CLK_HSTXP_EN | U2PHY_HSTXP_MODE;
    write32(phy, U2PHY_HSTXP_HW_CTRL, tx_path_enabled);
    error!(
        "[k3-usb] USB2 PHY TX path enabled: HSTXP_HW_CTRL={:#010x}",
        read32(phy, U2PHY_HSTXP_HW_CTRL)
    );

    // USB2 PHY +0x10 bit2：设置 disconnect auto-clear。设备拔出再插回时，
    // 自动清掉第一次 disconnect 状态，避免旧状态一直阻止后续重连。
    update32(phy, U2PHY_TX_HOST_CTRL, |value| {
        value | U2PHY_HST_DISC_AUTO_CLR
    });
    let tx_host_ctrl = read32(phy, U2PHY_TX_HOST_CTRL);
    error!(
        "[k3-usb] USB2 PHY disconnect auto-clear configured: TX_HOST_CTRL={tx_host_ctrl:#010x}, \
         enabled={}",
        tx_host_ctrl & U2PHY_HST_DISC_AUTO_CLR != 0,
    );
    Ok(())
}

/// USB3 PIPE3 PHY 初始化 (含组合 PHY 通道复用与 RCAL 校准)
///
/// 参考: drivers/phy/spacemit/phy-k3-usb3.c:197-213 (k3_usb3phy_init)
///       drivers/phy/spacemit/phy-k3-usb3.c:91-107 (k3_usb3phy_combo_set_usb)
///       drivers/phy/spacemit/phy-k3-usb3.c:109-207 (k3_usb3phy_init_single)
///       drivers/phy/spacemit/phy-k3-usb3.c:224-228 (k3_usb3phy_power_on)
fn k3_usb3_phy_init(resources: &K3UsbResources) -> Result<(), OnProbeError> {
    let apmu = map_reg(resources.apmu, APMU_SIZE)?;
    let apb_spare = map_reg(resources.apb_spare, APB_SPARE_SIZE)?;
    let phy = map_reg(resources.usb3_phy, PHY_MMIO_DEFAULT_SIZE)?;

    // 组合 PHY (combo-usb-bit 存在): 关闭 PCIe A X8, 通道切给 USB
    if let Some(combo_bit) = resources.combo_usb_bit {
        if combo_bit > 2 {
            return Err(OnProbeError::other(format!(
                "k3 usb3-phy: invalid combo-usb-bit {combo_bit}"
            )));
        }
        update32(apmu, PMUA_PCIE_SUBSYS_MGMT, |value| {
            value | PU_MATRIX_CONF_X8_DISABLE | (1 << combo_bit)
        });
    }

    // 清除 U3 覆盖位, 使能硬件自动控制 MPU 上电
    update32(phy, PCIE_PHY_OVERRIDE, |value| {
        value & !(OVRD_MPU_U3 | CFG_MPU_U3)
    });

    // 已经初始化过则跳过 (防止重复探测时 PLL 配置被打断)
    let initial_clk_cfg = read32(phy, PHY_CLK_CFG);
    if initial_clk_cfg & CFG_SW_INIT_DONE != 0 {
        error!("[k3-usb] USB3 PHY PLL lock: already initialized, CLK_CFG={initial_clk_cfg:#010x}");
        return k3_usb3_phy_power_on(phy);
    }

    // 触发 PHY RCAL 校准并等待完成
    update32(apb_spare, APB_SPARE_PU_CAL, |value| value | PU_CAL);
    if !poll_until(
        apb_spare,
        APB_SPARE_RCAL_HSIO,
        PU_CAL_DONE,
        POLL_DELAY,
        PU_CAL_TIMEOUT,
    ) {
        warn!("k3 usb3-phy: PU PHY RCAL timeout, use trim override");
        update32(apb_spare, APB_SPARE_RCAL_HSIO, |value| {
            (value
                & !(R_CAL_OVRD_TRIM_MASK
                    | R_CAL_OVRD_NTRIM_MASK
                    | R_CAL_OVRD_PTRIM_MASK
                    | R_CAL_OVRD_STABLE_VAL))
                | R_CAL_OVRD_TRIM_EN
                | R_CAL_OVRD_STABLE_VAL
                | R_CAL_OVRD_NTRIM_DEFAULT
                | R_CAL_OVRD_PTRIM_DEFAULT
        });
        update32(apb_spare, APB_SPARE_RCAL_HSIO, |value| {
            value | R_CAL_OVRD_STABLE_EN
        });
    }

    // 不等 CDR lock 即采样数据
    update32(phy, PHY_RESET_CFG, |value| {
        value & !EN_SAMPLE_DATA_AFTER_LOCK
    });
    // 关闭 100MHz refclk 缓冲 (使用 24MHz)
    update32(phy, PHY_PU_CK_REG, |value| value & !PU_REFCLK_100);
    // PLL1: SSC 下行扩频 5000ppm, 24MHz 参考
    write32(phy, PHY_PLL_REG1, SSC_DOWN_SPREAD1 | SSC_5000PPM | FREF_24M);
    // 不选择 100MHz PLL 参考
    update32(phy, PHY_PLL_REG2, |value| value & !SEL_REF100);
    // USB LFPS 周期配置
    update32(phy, PHY_MODE_CFG, |value| {
        (value & !CFG_LFPS_TPERIOD_MASK) | LFPS_TPERIOD_USB
    });
    // 强制 AFE 适配复位
    update32(phy, PHY_ADPT_CFG0, |value| {
        value | AFE_ADPT_RST_OVRD_EN | AFE_ADPT_RST_OVRD_VAL
    });
    // 驱动幅度覆盖为 900mV
    update32(phy, PHY_RXEQ_TIME, |value| {
        value | RXEQ_TIME_OVRD_AMP_SOC | AMP_SOC_900M
    });
    // RX 参数整写 (等价于 Linux 中对 RX_REG0..RX_REG6 的 6 次 update_bits)
    write32(phy, PHY_RX_REG_A, rx_reg_a_value());
    write32(phy, PHY_RX_REG_B, rx_reg_b_value());

    // USB3 PHY +0x08：选择 24MHz 参考、打开 RX/TX/PCLK/PIPE 时钟并置位
    // SW_INIT_DONE；这个写入通知 PHY 配置完成，随后 PLL 开始锁定。
    let pll_clk_cfg = CFG_SW_INIT_DONE
        | CFG_PU_SSC_OUT
        | CFG_REFCLK_24M
        | CFG_RXCLK_EN
        | CFG_PCLK_EN
        | CFG_PIPE_PCLK_EN
        | CFG_TXCLK_EN;
    write32(phy, PHY_CLK_CFG, pll_clk_cfg);
    error!(
        "[k3-usb] USB3 PHY PLL configured: PLL_REG1={:#010x}, PLL_REG2={:#010x}, \
         CLK_CFG={pll_clk_cfg:#010x}",
        read32(phy, PHY_PLL_REG1),
        read32(phy, PHY_PLL_REG2),
    );

    // PLL_READY 是 USB3 PHY 的硬件 lock 结果；读到它才能确认 PIPE3 的
    // 参考时钟和 PLL 已经稳定，之后才继续 DWC3 host 初始化。
    if !poll_until(phy, PHY_CLK_CFG, PLL_READY, POLL_DELAY, PLL_TIMEOUT) {
        return Err(OnProbeError::other("k3 usb3-phy: PLL lock timeout"));
    }
    error!(
        "[k3-usb] USB3 PHY PLL lock: CLK_CFG={:#010x}",
        read32(phy, PHY_CLK_CFG)
    );
    k3_usb3_phy_power_on(phy)
}

/// phy_power_on: 使能 LFPS RX 滤波 (Linux 在 dwc3_core_init 末尾调用)
/// 参考: drivers/phy/spacemit/phy-k3-usb3.c:224-228
fn k3_usb3_phy_power_on(phy: NonNull<u8>) -> Result<(), OnProbeError> {
    update32(phy, PHY_MODE_CFG, |value| value | CFG_LFPS_RX_FILTER_EN);
    Ok(())
}

/// RX_REG_A 整写值: 参考 phy-k3-usb3.c:166-173 的 RX_REG0..RX_REG3 更新序列
fn rx_reg_a_value() -> u32 {
    RX_REG0_RLOAD
        | RX_REG1_RC_CALI_DEFAULT
        | RX_REG1_RTERM_DEFAULT
        | RX_REG2_PSEL_DEFAULT
        | RX_REG2_FORCE_CSEL
        | RX_REG2_CSEL_DEFAULT
        | RX_REG3_RDEG1_DEFAULT
        | RX_REG3_ADJ_BIAS_DEFAULT
        | RX_REG3_SEL_CBOOST_CODE
        | RX_REG3_I_LOAD_DEFAULT
}

/// RX_REG_B 整写值: 参考 phy-k3-usb3.c:174-180 的 RX_REG4..RX_REG6 更新序列
fn rx_reg_b_value() -> u32 {
    RX_REG4_MANUAL_CFG
        | RX_REG4_RTERM_SEL
        | RX_REG4_ENVOS
        | RX_REG4_RDEG2_DEFAULT
        | RX_REG5_RCELL_BIAS_DEFAULT
        | RX_REG5_RCELL_VCM_DEFAULT
        | RX_REG6_ADAPT_GAIN_DEFAULT
        | RX_REG6_H1_REG_DEFAULT
}

/// DWC3 PHY 接口预配置 (必须在 PHY 初始化前调用, 保证 SUSPHY=0 使 PHY 处于 P0)
///
/// 参考: drivers/usb/dwc3/core.c:666-709 (dwc3_ss_phy_setup)
///       drivers/usb/dwc3/core.c:713-786 (dwc3_hs_phy_setup)
fn dwc3_phy_iface_setup(ctrl: NonNull<u8>, quirks: &K3Dwc3Quirks) {
    // GUSB3PIPECTL0: 清 SUSPHY / UX_EXIT_PX
    update32(ctrl, DWC3_GUSB3PIPECTL0, |value| {
        value & !(GUSB3PIPECTL_SUSPHY | GUSB3PIPECTL_UX_EXIT_PX)
    });

    // GUSB2PHYCFG0: UTMI 8-bit 接口
    update32(ctrl, DWC3_GUSB2PHYCFG0, |value| {
        let value = (value
            & !(GUSB2PHYCFG_PHYIF_MASK
                | GUSB2PHYCFG_USBTRDTIM_MASK
                | GUSB2PHYCFG_SUSPHY
                | GUSB2PHYCFG_TOUTCAL_MASK))
            | GUSB2PHYCFG_USBTRDTIM_UTMI_8BIT
            | GUSB2PHYCFG_TOUTCAL_MASK;
        if quirks.dis_enblslpm {
            value & !GUSB2PHYCFG_ENBLSLPM
        } else {
            value | GUSB2PHYCFG_ENBLSLPM
        }
    });
}

/// DWC3 全局寄存器 host 模式配置
///
/// 镜像 Linux host-only 路径: dwc3_core_init (core.c:1335-1420) +
/// dwc3_core_setup_global_control (core.c:1016-1105) +
/// dwc3_set_prtcap(HOST) (core.c:162-190)。
///
/// 注意:
/// - host 模式下 dwc3_core_soft_reset 直接返回 (core.c:328-333, xHCI 的 HCRST
///   由 crab-usb xHCI 后端在初始化时执行), 这里不做 DCTL.CSFTRST。
/// - DWC3 GEVNT 事件缓冲 (dwc3_event_buffers_setup, core.c:561+) 仅服务于
///   device/OTG 事件; host-only 下 xHCI 事件环由 ERST 寄存器驱动, 无需 GEVNT。
///   TODO: 若未来支持 OTG 模式, 需补充 GEVNTADRLO/HI + GEVNTSIZ + GEVNTCOUNT 设置。
fn dwc3_core_host_setup(ctrl: NonNull<u8>, quirks: &K3Dwc3Quirks) {
    // GCTL: 清除 SCALEDOWN + 按 EN_PWROPT 处理时钟门控 (core.c:1016-1105)
    update32(ctrl, DWC3_GCTL, |value| {
        let mut value = value & !GCTL_SCALEDOWN_MASK;
        let power_opt = (read32(ctrl, DWC3_GHWPARAMS1) >> 24) & 0x3;
        match power_opt {
            1 => {
                // DWC3 2.10a-2.50a 在 host 模式下必须禁用时钟门控 (STAR#9000588375)
                let revision = read32(ctrl, DWC3_GSNPSID);
                if (DWC3_REVISION_210A..=DWC3_REVISION_250A).contains(&revision) {
                    value |= GCTL_DSBLCLKGTNG | GCTL_SOFITPSYNC;
                } else {
                    value &= !GCTL_DSBLCLKGTNG;
                }
            }
            2 => value |= GCTL_GBLHIBERNATIONEN,
            _ => {}
        }
        value
    });

    // GUCTL1: 应用 DT quirk (core.c:1464-1480)
    update32(ctrl, DWC3_GUCTL1, |value| {
        let mut value = value;
        if quirks.dis_tx_ipgap_linecheck {
            value |= GUCTL1_TX_IPGAP_LINECHECK_DIS;
        }
        if quirks.parkmode_disable_ss {
            value |= GUCTL1_PARKMODE_DISABLE_SS;
        }
        value
    });

    // 最后设置 PRTCAPDIR = HOST (dwc3_set_prtcap, 必须在 xHCI HCRST 之前)
    update32(ctrl, DWC3_GCTL, |value| {
        (value & !GCTL_PRTCAPDIR_MASK) | GCTL_PRTCAP_HOST
    });
}

/// 在 xHCI HCRST 之前关闭所有根端口电源, 避免复位期间 VBUS 抖动
/// 导致已枚举设备产生 connect/disconnect 毛刺 (K3 板级要求)。
///
/// 参考: drivers/usb/dwc3/host.c:26-56 (dwc3_power_off_all_roothub_ports)
fn dwc3_power_off_roothub_ports(ctrl: NonNull<u8>) {
    let op_regs_base = (read32(ctrl, XHCI_CAPLENGTH) & XHCI_CAPLENGTH_OPREGS_MASK) as usize;
    let port_num = (read32(ctrl, XHCI_HCSPARAMS1) & XHCI_HCSPARAMS1_MAX_PORTS_MASK) >> 24;
    for port in 0..port_num {
        let offset = op_regs_base + XHCI_PORTSC_BASE + XHCI_PORTSC_STRIDE * port as usize;
        update32(ctrl, offset, |value| value & !XHCI_PORTSC_PORT_POWER);
    }
}

// ======================== MMIO 基础工具 ========================

fn map_reg(reg: RegFixed, default_size: usize) -> Result<NonNull<u8>, OnProbeError> {
    let size = align_up_4k((reg.size.unwrap_or(default_size as u64) as usize).max(1));
    iomap(reg.address as usize, size)
}

fn read32(base: NonNull<u8>, offset: usize) -> u32 {
    unsafe {
        // SAFETY: `base` 为 ioremap 的寄存器块指针, `offset` 为 regs.rs 中
        // 定义且位于映射范围内的寄存器偏移。
        (base.as_ptr().add(offset) as *const u32).read_volatile()
    }
}

fn write32(base: NonNull<u8>, offset: usize, value: u32) {
    unsafe {
        // SAFETY: 同 read32。
        (base.as_ptr().add(offset) as *mut u32).write_volatile(value)
    }
}

#[inline(always)]
fn io_write_fence() {
    #[cfg(target_arch = "riscv64")]
    unsafe {
        // Order the preceding MMIO write before the reset delay or next MMIO write.
        core::arch::asm!("fence o, rw", options(nostack));
    }
    #[cfg(not(target_arch = "riscv64"))]
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
}

fn update32(base: NonNull<u8>, offset: usize, f: impl FnOnce(u32) -> u32) {
    let value = read32(base, offset);
    write32(base, offset, f(value));
}

/// 轮询寄存器位, 超时返回 false
fn poll_until(
    base: NonNull<u8>,
    offset: usize,
    mask: u32,
    interval: Duration,
    timeout: Duration,
) -> bool {
    let mut waited = Duration::ZERO;
    while read32(base, offset) & mask == 0 {
        if waited >= timeout {
            return false;
        }
        axklib::time::busy_wait(interval);
        waited += interval;
    }
    true
}

fn align_up_4k(size: usize) -> usize {
    const MASK: usize = 0xfff;
    (size + MASK) & !MASK
}

fn prop_str<'a>(node: &'a Node, name: &str) -> Option<&'a str> {
    node.get_property(name).and_then(|prop| prop.as_str())
}

fn prop_u32(node: &Node, name: &str) -> Option<u32> {
    node.get_property(name).and_then(|prop| prop.get_u32())
}

fn prop_phandle(node: &Node, name: &str) -> Option<Phandle> {
    node.get_property(name)
        .and_then(|prop| prop.get_u32_iter().next())
        .map(Phandle::from)
}

fn has_prop(node: &Node, name: &str) -> bool {
    node.get_property(name).is_some()
}
