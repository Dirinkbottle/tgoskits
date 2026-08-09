//! SpacemiT K3 (k1-dwc3) USB3 外设寄存器定义
//!
//! 本文件只负责存放 SpacemiT K3 USB3/DWC3 子系统涉及的全部寄存器偏移与位域定义,
//! 所有定义均以 spacemit-k3-linux-6.18 源码为准并给出精确出处。
//! 驱动逻辑 (mod.rs) 中禁止出现未经本文件定义的裸偏移量。
//!
//! 涉及的 Linux 源码文件:
//! - include/soc/spacemit/k3-syscon.h                 (APMU 寄存器偏移)
//! - drivers/clk/spacemit/ccu-k3.c                    (USB3 时钟门控位)
//! - drivers/reset/reset-spacemit.c                   (USB3 复位位)
//! - drivers/phy/spacemit/phy-k3-usb3.c               (USB3 PIPE3 PHY)
//! - drivers/phy/spacemit/phy-k1-usb2.c               (USB2 UTMI PHY)
//! - drivers/usb/dwc3/core.h, core.c, host.c          (DWC3 核心/xHCI)

// 寄存器定义文件: 部分常量仅作文档用途 (基址由 FDT 动态解析), 允许未被引用。
#![allow(dead_code)]

// ======================== APMU 系统控制器 ========================
// DT 节点: syscon_apmu: system-controller@d4282800
// 参考: arch/riscv/boot/dts/spacemit/k3.dtsi:327-335

/// APMU 映射大小 (DT reg 0x400)
pub const APMU_SIZE: usize = 0x400;

/// APMU 时钟/复位混合控制寄存器, USB2/USB3 各端口的时钟门控与复位位均在此
/// 参考: include/soc/spacemit/k3-syscon.h:155  (APMU_USB_CLK_RES_CTRL = 0x05c)
pub const APMU_USB_CLK_RES_CTRL: usize = 0x05c;

/// USB3 Port B 总线时钟门控 (写 1 使能时钟)
/// 参考: drivers/clk/spacemit/ccu-k3.c:908 usb3_portb_bus_clk CCU_GATE_DEFINE(..., BIT(8), 0)
pub const USB3_PORTB_BUS_CLK_GATE: u32 = 1 << 8;

/// USB3 Port B 复位位组: assert(写 0) / deassert(写 1)
/// 参考: drivers/reset/reset-spacemit.c:331-332
///       [RESET_APMU_USB3_PORTB] = RESET_DATA(APMU_USB_CLK_RES_CTRL, 0, BIT(9)|BIT(10)|BIT(11))
pub const USB3_PORTB_RESET_DEASSERT: u32 = (1 << 9) | (1 << 10) | (1 << 11);

/// PCIE/USB3 组合 PHY 通道复用矩阵配置寄存器
/// 参考: drivers/phy/spacemit/phy-k3-usb3.c:62-64
pub const PMUA_PCIE_SUBSYS_MGMT: usize = 0x1d8;
/// bit4 = 0: PCIe A X8 模式; bit4 = 1: 关闭 PCIe A X8, PHY 通道按 [3:0] 分配
/// 参考: drivers/phy/spacemit/phy-k3-usb3.c:63
pub const PU_MATRIX_CONF_X8_DISABLE: u32 = 1 << 4;
/// 组合 PHY 的 USB 通道选择位掩码
/// 参考: drivers/phy/spacemit/phy-k3-usb3.c:64
pub const PU_MATRIX_CONF_USB_MASK: u32 = 0b111;

// ======================== APB_SPARE 系统控制器 ========================
// DT 节点: pll: system-controller@d4090000 (compatible "spacemit,k3-pll"),
//          u3phy 节点通过 spacemit,syscon-apb-spare 引用
// 参考: arch/riscv/boot/dts/spacemit/k3.dtsi:320-324

/// spacemit,k3-pll (APB_SPARE) 寄存器基址
pub const APB_SPARE_BASE: usize = 0xd409_0000;
/// APB_SPARE 映射大小 (DT reg 0x10000)
pub const APB_SPARE_SIZE: usize = 0x10000;

/// PHY 校准上电控制寄存器
/// 参考: drivers/phy/spacemit/phy-k3-usb3.c:73-74
pub const APB_SPARE_PU_CAL: usize = 0x178;
/// PU_CAL: 触发 PHY RCAL 校准
pub const PU_CAL: u32 = 1 << 17;

/// PHY RCAL 状态/覆盖寄存器
/// 参考: drivers/phy/spacemit/phy-k3-usb3.c:76-88
pub const APB_SPARE_RCAL_HSIO: usize = 0x17c;
/// PU_CAL_DONE: RCAL 校准完成标志
pub const PU_CAL_DONE: u32 = 1 << 8;
/// R_CAL_OVRD_STABLE_EN: 使能 trim 覆盖 (覆盖值写入后置位)
pub const R_CAL_OVRD_STABLE_EN: u32 = 1 << 31;
/// R_CAL_OVRD_STABLE_VAL: 覆盖值立即生效
pub const R_CAL_OVRD_STABLE_VAL: u32 = 1 << 30;
/// R_CAL_OVRD_NTRIM_EN / R_CAL_OVRD_PTRIM_EN: 使能 N/P trim 覆盖
pub const R_CAL_OVRD_NTRIM_EN: u32 = 1 << 29;
pub const R_CAL_OVRD_PTRIM_EN: u32 = 1 << 28;
/// TRIM_EN 组合掩码
/// 参考: drivers/phy/spacemit/phy-k3-usb3.c:84
pub const R_CAL_OVRD_TRIM_EN: u32 = R_CAL_OVRD_NTRIM_EN | R_CAL_OVRD_PTRIM_EN;
/// NTRIM 位域 [27:24], 默认 0x6; PTRIM 位域 [23:20], 默认 0xa
pub const R_CAL_OVRD_NTRIM_MASK: u32 = 0xf << 24;
pub const R_CAL_OVRD_NTRIM_DEFAULT: u32 = 0x6 << 24;
pub const R_CAL_OVRD_PTRIM_MASK: u32 = 0xf << 20;
pub const R_CAL_OVRD_PTRIM_DEFAULT: u32 = 0xa << 20;
/// TRIM 位域组合掩码 (NTRIM+PTRIM)
/// 参考: drivers/phy/spacemit/phy-k3-usb3.c:85-87
pub const R_CAL_OVRD_TRIM_MASK: u32 = R_CAL_OVRD_NTRIM_MASK | R_CAL_OVRD_PTRIM_MASK;

// ======================== USB3 PIPE3 PHY ========================
// DT 节点: usb3_portb_u3phy: phy@81f00000 (compatible "spacemit,k3-usb3-phy")
// 参考: drivers/phy/spacemit/phy-k3-usb3.c:101-189

/// USB3 PHY 版本寄存器 (保留, 仅调试用)
pub const PHY_VERSION: usize = 0x00;
/// PHY 复位配置寄存器
pub const PHY_RESET_CFG: usize = 0x04;
/// EN_SAMPLE_DATA_AFTER_LOCK: 锁定后采样数据使能 (清除以不等 CDR lock)
pub const EN_SAMPLE_DATA_AFTER_LOCK: u32 = 1 << 6;

/// PHY 时钟配置寄存器 (PLL 状态与时钟使能)
pub const PHY_CLK_CFG: usize = 0x08;
/// PLL_READY: PLL 锁定标志 (轮询)
pub const PLL_READY: u32 = 1 << 0;
pub const CFG_RXCLK_EN: u32 = 1 << 3;
pub const CFG_TXCLK_EN: u32 = 1 << 4;
pub const CFG_PCLK_EN: u32 = 1 << 5;
pub const CFG_PIPE_PCLK_EN: u32 = 1 << 6;
/// CFG_REFCLK_FREQ [10:7], REFCLK_24M = 0x2 (24MHz 参考时钟)
pub const CFG_REFCLK_FREQ_MASK: u32 = 0xf << 7;
pub const CFG_REFCLK_24M: u32 = 0x2 << 7;
/// CFG_SW_INIT_DONE: 软件初始化完成 (置位后 PLL 才开始锁定; 只读, 跳过初始化判据)
pub const CFG_SW_INIT_DONE: u32 = 1 << 11;
/// CFG_PU_SSC_OUT: 使能 SSC 输出
pub const CFG_PU_SSC_OUT: u32 = 1 << 23;

/// PHY 模式配置寄存器
pub const PHY_MODE_CFG: usize = 0x0c;
/// CFG_LFPS_RX_FILTER_EN: USB LFPS RX 滤波使能 (power_on 时置位)
pub const CFG_LFPS_RX_FILTER_EN: u32 = 1 << 11;
/// CFG_LFPS_TPERIOD [9:8], USB 模式 = 0x3
pub const CFG_LFPS_TPERIOD_MASK: u32 = 0x3 << 8;
pub const LFPS_TPERIOD_USB: u32 = 0x3 << 8;

/// PHY 覆盖配置寄存器
pub const PCIE_PHY_OVERRIDE: usize = 0x18;
pub const OVRD_MPU_U3: u32 = 1 << 17;
pub const CFG_MPU_U3: u32 = 1 << 16;

/// PHY 上电选择寄存器 (set_speed 用, 本驱动未使用, 保留定义)
pub const PHY_PU_SEL: usize = 0x40;

/// PHY 时钟电源控制寄存器
pub const PHY_PU_CK_REG: usize = 0x54;
/// PU_REFCLK_100: 100MHz refclk buffer 上电 (清除: 关闭 100MHz 缓冲)
pub const PU_REFCLK_100: u32 = 1 << 25;

/// PHY PLL 配置寄存器 1
pub const PHY_PLL_REG1: usize = 0x58;
/// FREF_SEL [15:13], 24MHz = 0x1
pub const FREF_SEL_MASK: u32 = 0x7 << 13;
pub const FREF_24M: u32 = 0x1 << 13;
/// SSC_DEP_SEL [27:24], 5000ppm = 0xa
pub const SSC_DEP_SEL_MASK: u32 = 0xf << 24;
pub const SSC_5000PPM: u32 = 0xa << 24;
/// SSC_MODE [29:28], DOWN_SPREAD1 = 0x3
pub const SSC_MODE_MASK: u32 = 0x3 << 28;
pub const SSC_DOWN_SPREAD1: u32 = 0x3 << 28;

/// PHY PLL 配置寄存器 2
pub const PHY_PLL_REG2: usize = 0x5c;
/// SEL_REF100: 选择 100MHz PLL 参考 (清除: 不选)
pub const SEL_REF100: u32 = 1 << 21;

/// PHY RX 参数寄存器 A (0x60) 与 B (0x64), 各 8 位字段一次整写
pub const PHY_RX_REG_A: usize = 0x60;
/// RX_REG0 [7:0]: RX_REG0_RLOAD = 1<<4
pub const RX_REG0_MASK: u32 = 0xff;
pub const RX_REG0_RLOAD: u32 = 1 << 4;
/// RX_REG1 [15:8]: RC_CALI=7<<12 | RTERM=8<<8
pub const RX_REG1_MASK: u32 = 0xff << 8;
pub const RX_REG1_RC_CALI_DEFAULT: u32 = 0x7 << 12;
pub const RX_REG1_RTERM_DEFAULT: u32 = 0x8 << 8;
/// RX_REG2 [23:16]: PSEL=4<<21 | FORCE_CSEL=1<<20 | CSEL=8<<16
pub const RX_REG2_MASK: u32 = 0xff << 16;
pub const RX_REG2_PSEL_DEFAULT: u32 = 0x4 << 21;
pub const RX_REG2_FORCE_CSEL: u32 = 1 << 20;
pub const RX_REG2_CSEL_DEFAULT: u32 = 0x8 << 16;
/// RX_REG3 [31:24]: RDEG1=3<<30 | ADJ_BIAS=1<<28 | SEL_CBOOST_CODE=1<<27 | I_LOAD=7<<24
pub const RX_REG3_MASK: u32 = 0xff << 24;
pub const RX_REG3_RDEG1_DEFAULT: u32 = 0x3 << 30;
pub const RX_REG3_ADJ_BIAS_DEFAULT: u32 = 0x1 << 28;
pub const RX_REG3_SEL_CBOOST_CODE: u32 = 1 << 27;
pub const RX_REG3_I_LOAD_DEFAULT: u32 = 0x7 << 24;

/// PHY RX 参数寄存器 B
pub const PHY_RX_REG_B: usize = 0x64;
/// RX_REG4 [7:0]: MANUAL_CFG=1<<7 | RTERM_SEL=1<<5 | ENVOS=1<<4 | RDEG2=2<<1
pub const RX_REG4_MASK: u32 = 0xff;
pub const RX_REG4_MANUAL_CFG: u32 = 1 << 7;
pub const RX_REG4_RTERM_SEL: u32 = 1 << 5;
pub const RX_REG4_ENVOS: u32 = 1 << 4;
pub const RX_REG4_RDEG2_DEFAULT: u32 = 0x2 << 1;
/// RX_REG5 [15:8]: RCELL_BIAS=8<<12 | RCELL_VCM=8<<8
pub const RX_REG5_MASK: u32 = 0xff << 8;
pub const RX_REG5_RCELL_BIAS_DEFAULT: u32 = 0x8 << 12;
pub const RX_REG5_RCELL_VCM_DEFAULT: u32 = 0x8 << 8;
/// RX_REG6 [23:16]: ADAPT_GAIN=2<<20 | H1_REG=8<<16
pub const RX_REG6_MASK: u32 = 0xff << 16;
pub const RX_REG6_ADAPT_GAIN_DEFAULT: u32 = 0x2 << 20;
pub const RX_REG6_H1_REG_DEFAULT: u32 = 0x8 << 16;

/// PHY RXEQ 时间常数寄存器
pub const PHY_RXEQ_TIME: usize = 0xb4;
/// RXEQ_TIME_OVRD_AMP_SOC + CFG_AMP_SOC [23:22] = AMP_SOC_900M(0x3)
/// 参考: drivers/phy/spacemit/phy-k3-usb3.c:182-186
pub const RXEQ_TIME_OVRD_AMP_SOC: u32 = 1 << 24;
pub const RXEQ_TIME_CFG_AMP_SOC_MASK: u32 = 0x3 << 22;
pub const AMP_SOC_900M: u32 = 0x3 << 22;

/// PHY AFE 适配复位控制寄存器
pub const PHY_ADPT_CFG0: usize = 0x140;
/// AFE_ADPT_RST_OVRD_EN | AFE_ADPT_RST_OVRD_VAL: 强制 AFE 适配复位
pub const AFE_ADPT_RST_OVRD_EN: u32 = 1 << 1;
pub const AFE_ADPT_RST_OVRD_VAL: u32 = 1 << 4;

// ======================== USB2 UTMI PHY ========================
// DT 节点: usb3_portb_u2phy: phy@81500000 (compatible "spacemit,k3-usb2-phy")
// 参考: drivers/phy/spacemit/phy-k1-usb2.c:19-56

/// USB2 PHY 复位/时钟模式控制寄存器
pub const U2PHY_RST_MODE_CTRL: usize = 0x04;
/// PLL_RDY: PLL 就绪 (轮询, 超时 50ms)
pub const U2PHY_PLL_RDY: u32 = 1 << 0;
pub const U2PHY_CLK_CDR_EN: u32 = 1 << 1;
pub const U2PHY_CLK_PLL_EN: u32 = 1 << 2;
pub const U2PHY_CLK_MAC_EN: u32 = 1 << 3;
pub const U2PHY_MAC_RSTN: u32 = 1 << 5;
pub const U2PHY_CDR_RSTN: u32 = 1 << 6;
pub const U2PHY_PLL_RSTN: u32 = 1 << 7;
pub const U2PHY_HS_LINE_TX_MODE: u32 = 1 << 13;
pub const U2PHY_FS_LINE_TX_MODE: u32 = 1 << 14;

/// USB2 PHY TX 主机控制寄存器
pub const U2PHY_TX_HOST_CTRL: usize = 0x10;
/// HST_DISC_AUTO_CLR: 重连时自动清除 HS host disconnect
pub const U2PHY_HST_DISC_AUTO_CLR: u32 = 1 << 2;

/// USB2 PHY HSTXP 时钟控制寄存器
pub const U2PHY_HSTXP_HW_CTRL: usize = 0x34;
pub const U2PHY_HSTXP_RSTN: u32 = 1 << 2;
pub const U2PHY_CLK_HSTXP_EN: u32 = 1 << 3;
pub const U2PHY_HSTXP_MODE: u32 = 1 << 4;

/// USB2 PHY PLL 分频配置寄存器
pub const U2PHY_PLL_DIV_CFG: usize = 0x98;
/// FDIV_REG [12:0] 覆盖值 0x1ec4 (0x100 段对应 24MHz, 其余为默认)
pub const U2PHY_FDIV_REG_MASK: u32 = (1 << 12) | (0xf << 8) | 0xff;
pub const U2PHY_FDIV_REG_VAL: u32 = 0x1ec4;
/// FDIV_FRACT_0_1 [14:13]: freq_sel<1:0>, 24MHz = 0x1
pub const U2PHY_FDIV_FRACT_0_1_MASK: u32 = 0x3 << 13;
pub const U2PHY_SEL_FREQ_24MHZ: u32 = 0x1 << 13;
/// DIV_LOCAL_EN: 使用内部默认分频值 (1) 或被 fdiv_reg 覆盖 (0)
pub const U2PHY_DIV_LOCAL_EN: u32 = 1 << 15;

// ======================== DWC3 核心寄存器 ========================
// k1-dwc3 = DesignWare DWC3, host 模式 xHCI 寄存器区位于偏移 0x0-0x7fff,
// 全局寄存器区位于 0xc100 起。
// 参考: drivers/usb/dwc3/core.h:88-89, 97-120

/// xHCI 寄存器区起始/结束偏移 (host 模式下整个 DWC3 块的低地址区)
/// 参考: drivers/usb/dwc3/core.h:88-89 (DWC3_XHCI_REGS_START/END)
pub const DWC3_XHCI_REGS_END: usize = 0x7fff;

/// 全局寄存器偏移
/// 参考: drivers/usb/dwc3/core.h:97-120
pub const DWC3_GSBUSCFG0: usize = 0xc100;
pub const DWC3_GCTL: usize = 0xc110;
pub const DWC3_GUCTL1: usize = 0xc11c;
pub const DWC3_GSNPSID: usize = 0xc120;
pub const DWC3_GHWPARAMS0: usize = 0xc140;
pub const DWC3_GHWPARAMS1: usize = 0xc144;
pub const DWC3_GUSB2PHYCFG0: usize = 0xc200;
pub const DWC3_GUSB3PIPECTL0: usize = 0xc2c0;

/// GCTL 位域
/// 参考: drivers/usb/dwc3/core.h:255-275
pub const GCTL_PRTCAPDIR_MASK: u32 = 0x3 << 12;
/// PRTCAPDIR = 1 (HOST) / 2 (DEVICE) / 3 (OTG)
pub const GCTL_PRTCAP_HOST: u32 = 0x1 << 12;
/// SCALEDOWN 位域 [6:4] (正常运行必须为 0)
pub const GCTL_SCALEDOWN_MASK: u32 = 0x3 << 4;
/// DSBLCLKGTNG: 禁用时钟门控 (时钟节流 workaround)
pub const GCTL_DSBLCLKGTNG: u32 = 1 << 0;
/// SOFITPSYNC: SOF/ITP 同步使能
pub const GCTL_SOFITPSYNC: u32 = 1 << 10;
/// GBLHIBERNATIONEN: 全局休眠使能 (EN_PWROPT=HIB 时置位)
pub const GCTL_GBLHIBERNATIONEN: u32 = 1 << 1;

/// GUCTL1 位域 (host 相关 quirk)
/// 参考: drivers/usb/dwc3/core.h:278-283
/// snps,dis-tx-ipgap-linecheck-quirk: 关闭 TX IPGAP linecheck
pub const GUCTL1_TX_IPGAP_LINECHECK_DIS: u32 = 1 << 28;
/// snps,parkmode-disable-ss-quirk: 关闭 SS park mode
pub const GUCTL1_PARKMODE_DISABLE_SS: u32 = 1 << 17;

/// GUSB2PHYCFG0 位域
/// 参考: drivers/usb/dwc3/core.h:301-311, core.c:713-786
/// PHYIF [4:3]: UTMI 数据宽度 (0 = 8-bit)
pub const GUSB2PHYCFG_PHYIF_MASK: u32 = 1 << 3;
/// USBTRDTIM [13:10]: UTMI 8-bit 传输恢复时间 = 9
pub const GUSB2PHYCFG_USBTRDTIM_MASK: u32 = 0xf << 10;
pub const GUSB2PHYCFG_USBTRDTIM_UTMI_8BIT: u32 = 9 << 10;
/// SUSPHY: USB2 挂起使能 (phy init 前必须清除)
pub const GUSB2PHYCFG_SUSPHY: u32 = 1 << 6;
/// ENBLSLPM: LPM 使能 (snps,dis_enblslpm_quirk 时清除)
pub const GUSB2PHYCFG_ENBLSLPM: u32 = 1 << 8;
/// TOUTCAL [2:0]: SpacemiT K3 强制置 0x7
/// 参考: drivers/usb/dwc3/core.c:784-785 (CONFIG_SOC_SPACEMIT_K3)
pub const GUSB2PHYCFG_TOUTCAL_MASK: u32 = 0x7;

/// GUSB3PIPECTL0 位域
/// 参考: drivers/usb/dwc3/core.h:330-336, core.c:666-709
/// SUSPHY: USB3 挂起使能 (phy init 前必须清除)
pub const GUSB3PIPECTL_SUSPHY: u32 = 1 << 17;
/// UX_EXIT_PX: 非正常工作位, 必须清除
pub const GUSB3PIPECTL_UX_EXIT_PX: u32 = 1 << 27;

/// DWC3 版本常量 (GSNPSID 低 16 位为版本号, 高 16 位为 IP ID)
/// 参考: drivers/usb/dwc3/core.h:1282-1297
pub const DWC3_REVISION_210A: u32 = 0x5533_210a;
pub const DWC3_REVISION_250A: u32 = 0x5533_250a;

// ======================== xHCI 寄存器 (DWC3 host 块) ========================
// DWC3 块的 xHCI 寄存器区从偏移 0x0 起 (DWC3_XHCI_REGS_START)。
// 参考: drivers/usb/dwc3/host.c:60-61, drivers/usb/host/xhci{,-caps,-port}.h

/// CAPLENGTH: xHCI 能力寄存器区首寄存器 (bit[7:0] = 操作寄存器区偏移)
/// 参考: drivers/usb/host/xhci.h (XHCI_CAPLENGTH = 0x00)
pub const XHCI_CAPLENGTH: usize = 0x00;
/// USBSTS: xHCI operational status register, relative to the operational base.
pub const XHCI_USBSTS: usize = 0x04;
/// HCSPARAMS1: 结构/端口能力参数寄存器
/// 参考: drivers/usb/dwc3/host.c:60 (XHCI_HCSPARAMS1 = 0x4)
pub const XHCI_HCSPARAMS1: usize = 0x04;
/// PORTSC 寄存器区起始偏移 (操作寄存器区内)
/// 参考: drivers/usb/dwc3/host.c:61 (XHCI_PORTSC_BASE = 0x400)
pub const XHCI_PORTSC_BASE: usize = 0x400;
/// 相邻 PORTSC 寄存器步进 0x10
/// 参考: drivers/usb/host/xhci.h (XHCI_PORTSC_REG_OFFSET 0x10)
pub const XHCI_PORTSC_STRIDE: usize = 0x10;

/// CAPLENGTH 位域 [7:0]: 操作寄存器区偏移
/// 参考: drivers/usb/host/xhci-ext-caps.h:29 (XHCI_HC_LENGTH)
pub const XHCI_CAPLENGTH_OPREGS_MASK: u32 = 0xff;
/// HCSPARAMS1 [30:24]: 根端口数
/// 参考: drivers/usb/host/xhci-caps.h:16 (HCS_MAX_PORTS)
pub const XHCI_HCSPARAMS1_MAX_PORTS_MASK: u32 = 0x7f << 24;
/// PORTSC [9]: 端口电源控制 (1 = 上电)
/// 参考: drivers/usb/host/xhci-port.h:33 (PORT_POWER)
pub const XHCI_PORTSC_PORT_POWER: u32 = 1 << 9;
