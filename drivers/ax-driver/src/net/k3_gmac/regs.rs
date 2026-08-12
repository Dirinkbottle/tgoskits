//! DWMAC4/5 寄存器常量与 MMIO 辅助。
//!
//! 所有偏移/位域值来自 Synopsys DWMAC4/5 手册，参考 Linux `dwmac4.h` /
//! `dwmac4_dma.h`。旧实现通过 c2rust 生成的 `generated/bindings.rs` 引用，
//! 这里直接手写为命名常量并注释来源，避免生成代码残留。

use core::ptr::NonNull;

// ---------------------------------------------------------------------------
// MAC 寄存器偏移（相对 GMAC MMIO 基址）
// 来源：dwmac4.h
// ---------------------------------------------------------------------------
pub const GMAC_CONFIG: u32 = 0x000; // MAC_CTRL_REG
pub const GMAC_PACKET_FILTER: u32 = 0x008;
pub const GMAC_HW_FEATURE0: u32 = 0x11c;
pub const GMAC_HW_FEATURE1: u32 = 0x120;
pub const GMAC_HW_FEATURE2: u32 = 0x124;
pub const GMAC_HW_FEATURE3: u32 = 0x128;
pub const GMAC_MDIO_ADDR: u32 = 0x200; // = GMAC_GMII_ADDR
pub const GMAC_MDIO_DATA: u32 = 0x204; // = GMAC_GMII_DATA
pub const GMAC_VERSION: u32 = 0x110;

// MAC 地址寄存器 0（n=0：0x300/high, 0x304/low）
pub const GMAC_ADDR_HIGH0: u32 = 0x300;
pub const GMAC_ADDR_LOW0: u32 = 0x304;

// ---------------------------------------------------------------------------
// GMAC_CONFIG 位域（dwmac4.h L159-185）
// bit0=RE, bit1=TE, bit12=DCRS, bit13=DM, bit14=FES, bit15=PS,
// bit16=JE, bit17=JD, bit18=BE, bit27=IPC
// ---------------------------------------------------------------------------
pub const GMAC_CONFIG_RE: u32 = 1 << 0;
pub const GMAC_CONFIG_TE: u32 = 1 << 1;
pub const GMAC_CONFIG_DCRS: u32 = 1 << 9; // Disable Carrier Sense During TX
pub const GMAC_CONFIG_DM: u32 = 1 << 13; // Duplex Mode（1=全双工）
pub const GMAC_CONFIG_FES: u32 = 1 << 14; // Fast Ethernet Speed
pub const GMAC_CONFIG_PS: u32 = 1 << 15; // Port Select（1=MII/RMII, 0=GMII/RGMII）
pub const GMAC_CONFIG_JE: u32 = 1 << 16; // Jumbo Frame Enable
pub const GMAC_CONFIG_JD: u32 = 1 << 17; // Jabber Disable
pub const GMAC_CONFIG_BE: u32 = 1 << 18; // Backoff Limit Enable
pub const GMAC_CONFIG_IPC: u32 = 1 << 27; // Checksum Offload Enable

/// GMAC 核心初始化位（dwmac4.h GMAC_CORE_INIT = JD | PS | BE | DCRS | JE）。
/// 不含 FES/DM 速率位（由 apply_link_speed 按协商结果写）。
pub const GMAC_CORE_INIT: u32 =
    GMAC_CONFIG_JD | GMAC_CONFIG_PS | GMAC_CONFIG_BE | GMAC_CONFIG_DCRS | GMAC_CONFIG_JE;

/// 速率位掩码（FES|PS），清零后重写以切换速率。
pub const GMAC_SPEED_MASK: u32 = GMAC_CONFIG_FES | GMAC_CONFIG_PS;

// ---------------------------------------------------------------------------
// GMAC_PACKET_FILTER 位域（dwmac4.h）
// ---------------------------------------------------------------------------
pub const GMAC_PACKET_FILTER_PR: u32 = 1 << 0; // Promiscuous Mode
pub const GMAC_PACKET_FILTER_PM: u32 = 1 << 4; // Pass All Multicast

// MAC 地址高位 bit31 = Address Enable
pub const GMAC_HI_REG_AE: u32 = 1 << 31;

// ---------------------------------------------------------------------------
// MDIO 寄存器位域（dwmac4_core.c stmmac_mdio，与 Linux 一致）
// bit0 = GBUSY（MII busy）
// bit1 = C45E（Clause 45 enable，置 1 则走 C45 而非 C22）
// bit3:2 = GOC（操作码：读=0b11，写=0b01）
// bit7:4 = SKAP 等
// bit11:8 = CR（CSR Clock Range）
// bit20:16 = RDA（register devaddr，C22 时是寄存器号）
// bit25:21 = PA（PHY 地址）
// ---------------------------------------------------------------------------
pub const MII_ADDR_GBUSY: u32 = 1 << 0;
#[allow(dead_code)] // C45E 用于 Clause 45，当前只走 C22；保留供将来 C22/C45 切换
pub const MII_GMAC4_C45E: u32 = 1 << 1; // 置 1 走 Clause 45（默认 C22 不置）
pub const MII_GMAC4_GOC_SHIFT: u32 = 2;
/// GOC 读操作码（已左移到 bit3:2）。注意：必须含 shift，否则 bit1 会误置 C45E。
pub const MII_GMAC4_READ: u32 = 0b11 << MII_GMAC4_GOC_SHIFT; // = 0xc
pub const MII_GMAC4_REG_ADDR_SHIFT: u32 = 16;

/// MDC CSR 时钟分频：CR 值放在 GMAC_MDIO_ADDR bit11:8。
/// 来源：U-Boot K3 eqos 配置 EQOS_MAC_MDIO_ADDRESS_CR_250_300 = 5。
/// K3 的 stmmaceth/master_bus CSR 时钟在 250-300MHz 范围。
pub const STMMAC_CSR_250_300M: u32 = 5;
pub const MDIO_CSR_CLK_SHIFT: u32 = 8;

// ---------------------------------------------------------------------------
// MTL 寄存器（dwmac4.h MTL_CHAN_BASE_ADDR = 0xc00，每通道 stride=0x40）
// ---------------------------------------------------------------------------
pub const MTL_CHAN_BASE_ADDR: u32 = 0xc00;
pub const MTL_CHAN_BASE_OFFSET: u32 = 0x40;

// MTL TX 操作模式位（MTL_CHAN_TX_OP_MODE = mtl_chan_base + 0x00）
pub const MTL_OP_MODE_TSF: u32 = 1 << 0; // TX Store-and-Forward
pub const MTL_OP_MODE_TXQEN_MASK: u32 = 0b11 << 2; // bit3:2
pub const MTL_OP_MODE_TXQEN: u32 = 1 << 3; // TX Queue Enable（AV/普通模式）
pub const MTL_OP_MODE_TQS_SHIFT: u32 = 16;
pub const MTL_OP_MODE_TQS_MASK: u32 = 0x1ff << 16;

// MTL RX 操作模式位（MTL_CHAN_RX_OP_MODE = mtl_chan_base + 0x30）
pub const MTL_OP_MODE_RSF: u32 = 1 << 5; // RX Store-and-Forward
pub const MTL_OP_MODE_DIS_TCP_EF: u32 = 1 << 6; // Disable TCP/UDP Checksum
pub const MTL_OP_MODE_EHFC: u32 = 1 << 2; // HW Flow Control
pub const MTL_OP_MODE_RQS_SHIFT: u32 = 20;
pub const MTL_OP_MODE_RQS_MASK: u32 = 0x3ff << 20;
pub const MTL_OP_MODE_RFD_SHIFT: u32 = 14;
pub const MTL_OP_MODE_RFD_MASK: u32 = 0x3f << 14;
pub const MTL_OP_MODE_RFA_SHIFT: u32 = 8;
pub const MTL_OP_MODE_RFA_MASK: u32 = 0x3f << 8;

// ---------------------------------------------------------------------------
// DMA 寄存器（dwmac4_dma.h，DMA 块基址偏移 0x1000）
// ---------------------------------------------------------------------------
pub const DMA_BUS_MODE: u32 = 0x1000; // 含 SFT_RESET bit0
pub const DMA_SYS_BUS_MODE: u32 = 0x1004;
pub const DMA_AXI_BUS_MODE: u32 = 0x1038;

/// DMA 通道寄存器组基址（每通道 stride=0x80，dwmac4_dma.h）。
pub const DMA_CHAN_BASE_ADDR: u32 = 0x1100;
pub const DMA_CHAN_BASE_OFFSET: u32 = 0x80;

// DMA_BUS_MODE 位
pub const DMA_BUS_MODE_SFT_RESET: u32 = 1 << 0;

// DMA_SYS_BUS_MODE 位
pub const DMA_SYS_BUS_FB: u32 = 1 << 0; // Fixed Burst
pub const DMA_SYS_BUS_AAL: u32 = 1 << 12; // Address Aligned Beats
pub const DMA_SYS_BUS_MB: u32 = 1 << 14; // Mixed Burst

/// AXI 突发长度默认值（dwmac4_dma.c stmmac_axi_setup，支持 256/128/64/32/16）。
pub const DMA_AXI_BURST_LEN_DEFAULT: u32 = 0xfe; // blen[1..5]

// DMA 通道控制位
pub const DMA_CONTROL_ST: u32 = 1 << 0; // TX 启动（DMA_CHAN_TX_CONTROL）
pub const DMA_CONTROL_SR: u32 = 1 << 0; // RX 启动（DMA_CHAN_RX_CONTROL）
pub const DMA_CONTROL_OSP: u32 = 1 << 4; // Operate on Second Packet（TX 提前读描述符）

/// RX 缓冲大小编码在 DMA_CHAN_RX_CONTROL bit14:1。
pub const DMA_RBSZ_MASK: u32 = 0x7ffe;
pub const DMA_RBSZ_SHIFT: u32 = 1;

// DMA 通道状态位（DMA_CHAN_STATUS，write-to-clear）
pub const DMA_CHAN_STATUS_NIS: u32 = 1 << 15; // Normal Interrupt Summary
pub const DMA_CHAN_STATUS_AIS: u32 = 1 << 14; // Abnormal Interrupt Summary
pub const DMA_CHAN_STATUS_FBE: u32 = 1 << 12; // Fatal Bus Error
pub const DMA_CHAN_STATUS_RBU: u32 = 1 << 7; // RX Buffer Unavailable
pub const DMA_CHAN_STATUS_RI: u32 = 1 << 6; // RX Interrupt
pub const DMA_CHAN_STATUS_TBU: u32 = 1 << 2; // TX Buffer Unavailable
pub const DMA_CHAN_STATUS_TI: u32 = 1 << 0; // TX Interrupt

/// DMA 通道中断默认掩码（normal + abnormal）。
/// 来源：dwmac4_dma.h DMA_CHAN_INTR_DEFAULT_MASK。
pub const DMA_CHAN_INTR_DEFAULT_MASK: u32 = 0x19001; // NIS|AIS|RIE|TIE|RBUE|TBUE|FBE

// ---------------------------------------------------------------------------
// SpacemiT K3 syscon glue 位域（dwmac-spacemit-ethqos.c L116-152）
// CTRL 寄存器（apmu_base + ctrl_offset = APMU_EMACx_CLK_RES_CTRL）
// 位定义来源：ccu-k3.c（CCU_GATE_DEFINE emacx_bus_clk BIT(0)）+
//            reset-spacemit.c（RESET_DATA deassert_mask BIT(1)）+
//            用户手册 14.3.4.1。
// ---------------------------------------------------------------------------
/// AXI 总线时钟使能（1=开）。Linux 由 CCF 经 DTS `clocks` 自动处理；
/// StarryOS 无 CCF，必须在 glue 里显式置 1，否则 GMAC DMA 寄存器无时钟。
pub const EMAC_BUS_CLK_EN: u32 = 1 << 0;
/// AXI 总线复位（1=释放复位，0=保持复位）。同上需显式置 1。
pub const EMAC_BUS_RST_DEASSERT: u32 = 1 << 1;
/// 总线时钟 + 复位：使 GMAC 可用的最小 CTRL 位集合。
#[allow(dead_code)]
pub const EMAC_BUS_ENABLE: u32 = EMAC_BUS_CLK_EN | EMAC_BUS_RST_DEASSERT;

pub const PHY_INTF_MODE_MASK: u32 = 0b11 << 3; // bit4:3
pub const PHY_INTF_RMII: u32 = 0b00 << 3;
pub const PHY_INTF_RGMII: u32 = 0b01 << 3;
pub const PHY_INTF_MII: u32 = 0b11 << 3;
pub const WOL_WAKE_IRQ_EN: u32 = 1 << 12; // PHY PMT 中断使能（WoL）

// DLINE 寄存器（apmu_base + dline_offset）
pub const EMAC_RX_DLINE_EN: u32 = 1 << 0; // RX 延迟线使能
pub const EMAC_TX_DLINE_EN: u32 = 1 << 16; // TX 延迟线使能
pub const EMAC_RX_DLINE_CODE_MASK: u32 = 0xff << 8; // RX 延迟码 bit15:8
pub const EMAC_TX_DLINE_CODE_MASK: u32 = 0xff << 24; // TX 延迟码 bit31:24

// ---------------------------------------------------------------------------
// K3 GPIO 寄存器（用于 PHY 硬件复位，gpio-spacemit-k1.c K3 布局）
// GPIO 控制器基址 0xd4019000，每 bank stride=0x40（K3 bank_offsets）。
// 寄存器 offset 相对 bank 基址（K3 reg_offsets）。
// ---------------------------------------------------------------------------
/// K3 GPIO 每 bank 的步长（K3: bank0=0x0, bank1=0x40, bank2=0x80, bank3=0x100）。
pub const K3_GPIO_BANK_STRIDE: u32 = 0x40;
#[allow(dead_code)] // GPDR 是方向 R/W 寄存器，PHY 复位用 GSDR；保留供将来 GPIO 驱动
pub const K3_GPIO_GPDR: u32 = 0x04; // 端口方向 R/W（1=output）
pub const K3_GPIO_GPSR: u32 = 0x08; // 端口置位 W（写 1 设高）
pub const K3_GPIO_GPCR: u32 = 0x0c; // 端口清零 W（写 1 设低）
pub const K3_GPIO_GSDR: u32 = 0x1c; // 设方向 W（写 1 设为 output）
#[allow(dead_code)] // GCDR 是清方向寄存器（设 input），PHY 复位后保持 output
pub const K3_GPIO_GCDR: u32 = 0x20; // 清方向 W（写 1 设为 input）

// ---------------------------------------------------------------------------
// Mmio：volatile MMIO 读写封装
// ---------------------------------------------------------------------------

/// volatile MMIO 句柄。offset 单位为字节，相对 GMAC MMIO 基址。
#[derive(Debug)]
pub struct Mmio {
    base: NonNull<u8>,
}

// SAFETY: Mmio 仅持有一个设备 MMIO 基址指针，通过 read_volatile/write_volatile
// 访问设备内存；本身无可变状态，可安全跨线程共享（Send + Sync 由调用方保证
// 同一时间只有一个可变引用）。
unsafe impl Send for Mmio {}

impl Mmio {
    /// 由已映射的 MMIO 基址指针构造。
    ///
    /// # Safety
    /// 调用方需保证 `base` 指向一段已 ioremap 的有效设备 MMIO 区域，且生命周期
    /// 不短于本 `Mmio` 的所有使用。
    pub const unsafe fn new(base: NonNull<u8>) -> Self {
        Self { base }
    }

    pub fn read(&self, offset: u32) -> u32 {
        // SAFETY: 调用方保证 base 指向有效 MMIO；read_volatile 保证不被优化消除。
        unsafe {
            self.base
                .as_ptr()
                .add(offset as usize)
                .cast::<u32>()
                .read_volatile()
        }
    }

    pub fn write(&self, offset: u32, value: u32) {
        // SAFETY: 同 read。
        unsafe {
            self.base
                .as_ptr()
                .add(offset as usize)
                .cast::<u32>()
                .write_volatile(value);
        }
    }

    /// 读-改-写：`(old & !clear) | set`。
    pub fn update(&self, offset: u32, clear: u32, set: u32) {
        let value = (self.read(offset) & !clear) | set;
        self.write(offset, value);
    }
}

// ---------------------------------------------------------------------------
// DMA 通道寄存器地址计算（每通道 0x80 字节，dwmac4_dma.h）
// ---------------------------------------------------------------------------

/// DMA 通道 chan 的基址偏移。
pub const fn dma_chan_base(chan: u32) -> u32 {
    DMA_CHAN_BASE_ADDR + chan * DMA_CHAN_BASE_OFFSET
}

pub const fn dma_chan_tx_control(chan: u32) -> u32 {
    dma_chan_base(chan) + 0x04
}
pub const fn dma_chan_rx_control(chan: u32) -> u32 {
    dma_chan_base(chan) + 0x08
}
pub const fn dma_chan_tx_base_hi(chan: u32) -> u32 {
    dma_chan_base(chan) + 0x10
}
pub const fn dma_chan_tx_base(chan: u32) -> u32 {
    dma_chan_base(chan) + 0x14
}
pub const fn dma_chan_rx_base_hi(chan: u32) -> u32 {
    dma_chan_base(chan) + 0x18
}
pub const fn dma_chan_rx_base(chan: u32) -> u32 {
    dma_chan_base(chan) + 0x1c
}
pub const fn dma_chan_tx_end(chan: u32) -> u32 {
    dma_chan_base(chan) + 0x20
}
pub const fn dma_chan_rx_end(chan: u32) -> u32 {
    dma_chan_base(chan) + 0x28
}
pub const fn dma_chan_tx_ring_len(chan: u32) -> u32 {
    dma_chan_base(chan) + 0x2c
}
pub const fn dma_chan_rx_ring_len(chan: u32) -> u32 {
    dma_chan_base(chan) + 0x30
}
pub const fn dma_chan_intr_ena(chan: u32) -> u32 {
    dma_chan_base(chan) + 0x34
}
pub const fn dma_chan_status(chan: u32) -> u32 {
    dma_chan_base(chan) + 0x60
}

// ---------------------------------------------------------------------------
// MTL 通道寄存器地址计算（每队列 0x40 字节，dwmac4.h）
// ---------------------------------------------------------------------------

/// MTL 通道 chan 的基址偏移。
pub const fn mtl_chan_base(chan: u32) -> u32 {
    MTL_CHAN_BASE_ADDR + chan * MTL_CHAN_BASE_OFFSET
}

pub const fn mtl_chan_tx_op_mode(chan: u32) -> u32 {
    mtl_chan_base(chan)
}
pub const fn mtl_chan_rx_op_mode(chan: u32) -> u32 {
    mtl_chan_base(chan) + 0x30
}

/// 编译期检查描述符大小（DWMAC4 描述符为 4×u32 = 16 字节）。
const _: () = {
    assert!(core::mem::size_of::<super::desc::DmaDesc>() == 16);
};
