//! DWMAC4/5 register constants and MMIO helpers.

use core::ptr::NonNull;

use super::generated::bindings as b;

pub const GMAC_CONFIG: u32 = b::GMAC_CONFIG;
pub const GMAC_PACKET_FILTER: u32 = b::GMAC_PACKET_FILTER;
pub const GMAC_HW_FEATURE0: u32 = b::GMAC_HW_FEATURE0;
pub const GMAC_HW_FEATURE1: u32 = b::GMAC_HW_FEATURE1;
pub const GMAC_HW_FEATURE2: u32 = b::GMAC_HW_FEATURE2;
pub const GMAC_HW_FEATURE3: u32 = b::GMAC_HW_FEATURE3;
pub const GMAC_MDIO_ADDR: u32 = b::GMAC_MDIO_ADDR;
pub const GMAC_MDIO_DATA: u32 = b::GMAC_MDIO_DATA;
pub const GMAC_ADDR_HIGH0: u32 = k(b::k3gmac_generated_values_K3GMAC_GMAC_ADDR_HIGH0);
pub const GMAC_ADDR_LOW0: u32 = k(b::k3gmac_generated_values_K3GMAC_GMAC_ADDR_LOW0);

pub const MTL_CHAN_BASE_ADDR: u32 = b::MTL_CHAN_BASE_ADDR;
pub const MTL_CHAN_BASE_OFFSET: u32 = b::MTL_CHAN_BASE_OFFSET;

pub const DMA_BUS_MODE: u32 = b::DMA_BUS_MODE;
pub const DMA_SYS_BUS_MODE: u32 = b::DMA_SYS_BUS_MODE;
pub const DMA_AXI_BUS_MODE: u32 = b::DMA_AXI_BUS_MODE;
pub const DMA_CHAN_BASE_ADDR: u32 = b::DMA_CHAN_BASE_ADDR;
pub const DMA_CHAN_BASE_OFFSET: u32 = b::DMA_CHAN_BASE_OFFSET;

pub const GMAC_CONFIG_TE: u32 = k(b::k3gmac_generated_values_K3GMAC_GMAC_CONFIG_TE);
pub const GMAC_CONFIG_RE: u32 = k(b::k3gmac_generated_values_K3GMAC_GMAC_CONFIG_RE);
pub const GMAC_CONFIG_DM: u32 = k(b::k3gmac_generated_values_K3GMAC_GMAC_CONFIG_DM);
pub const GMAC_CONFIG_FES: u32 = k(b::k3gmac_generated_values_K3GMAC_GMAC_CONFIG_FES);
pub const GMAC_CONFIG_PS: u32 = k(b::k3gmac_generated_values_K3GMAC_GMAC_CONFIG_PS);
pub const GMAC_CONFIG_IPC: u32 = k(b::k3gmac_generated_values_K3GMAC_GMAC_CONFIG_IPC);
pub const GMAC_PACKET_FILTER_PR: u32 = k(b::k3gmac_generated_values_K3GMAC_GMAC_PACKET_FILTER_PR);
pub const GMAC_PACKET_FILTER_PM: u32 = k(b::k3gmac_generated_values_K3GMAC_GMAC_PACKET_FILTER_PM);
pub const GMAC_HI_REG_AE: u32 = k(b::k3gmac_generated_values_K3GMAC_GMAC_HI_REG_AE);

pub const MTL_OP_MODE_RSF: u32 = k(b::k3gmac_generated_values_K3GMAC_MTL_OP_MODE_RSF);
pub const MTL_OP_MODE_TXQEN_MASK: u32 = k(b::k3gmac_generated_values_K3GMAC_MTL_OP_MODE_TXQEN_MASK);
pub const MTL_OP_MODE_TXQEN: u32 = k(b::k3gmac_generated_values_K3GMAC_MTL_OP_MODE_TXQEN);
pub const MTL_OP_MODE_TSF: u32 = k(b::k3gmac_generated_values_K3GMAC_MTL_OP_MODE_TSF);
pub const MTL_OP_MODE_TQS_MASK: u32 = k(b::k3gmac_generated_values_K3GMAC_MTL_OP_MODE_TQS_MASK);
pub const MTL_OP_MODE_TQS_SHIFT: u32 = k(b::k3gmac_generated_values_K3GMAC_MTL_OP_MODE_TQS_SHIFT);
pub const MTL_OP_MODE_RQS_MASK: u32 = k(b::k3gmac_generated_values_K3GMAC_MTL_OP_MODE_RQS_MASK);
pub const MTL_OP_MODE_RQS_SHIFT: u32 = k(b::k3gmac_generated_values_K3GMAC_MTL_OP_MODE_RQS_SHIFT);
pub const MTL_OP_MODE_RFD_MASK: u32 = k(b::k3gmac_generated_values_K3GMAC_MTL_OP_MODE_RFD_MASK);
pub const MTL_OP_MODE_RFD_SHIFT: u32 = k(b::k3gmac_generated_values_K3GMAC_MTL_OP_MODE_RFD_SHIFT);
pub const MTL_OP_MODE_RFA_MASK: u32 = k(b::k3gmac_generated_values_K3GMAC_MTL_OP_MODE_RFA_MASK);
pub const MTL_OP_MODE_RFA_SHIFT: u32 = k(b::k3gmac_generated_values_K3GMAC_MTL_OP_MODE_RFA_SHIFT);
pub const MTL_OP_MODE_EHFC: u32 = k(b::k3gmac_generated_values_K3GMAC_MTL_OP_MODE_EHFC);
pub const MTL_OP_MODE_DIS_TCP_EF: u32 = k(b::k3gmac_generated_values_K3GMAC_MTL_OP_MODE_DIS_TCP_EF);

pub const DMA_BUS_MODE_SFT_RESET: u32 = k(b::k3gmac_generated_values_K3GMAC_DMA_BUS_MODE_SFT_RESET);
pub const DMA_SYS_BUS_MB: u32 = k(b::k3gmac_generated_values_K3GMAC_DMA_SYS_BUS_MB);
pub const DMA_SYS_BUS_FB: u32 = k(b::k3gmac_generated_values_K3GMAC_DMA_SYS_BUS_FB);
pub const DMA_SYS_BUS_AAL: u32 = k(b::k3gmac_generated_values_K3GMAC_DMA_SYS_BUS_AAL);
pub const DMA_AXI_BURST_LEN_DEFAULT: u32 =
    k(b::k3gmac_generated_values_K3GMAC_DMA_AXI_BURST_LEN_DEFAULT);
pub const DMA_CONTROL_ST: u32 = k(b::k3gmac_generated_values_K3GMAC_DMA_CONTROL_ST);
pub const DMA_CONTROL_SR: u32 = k(b::k3gmac_generated_values_K3GMAC_DMA_CONTROL_SR);
pub const DMA_CONTROL_OSP: u32 = k(b::k3gmac_generated_values_K3GMAC_DMA_CONTROL_OSP);
pub const DMA_RBSZ_MASK: u32 = k(b::k3gmac_generated_values_K3GMAC_DMA_RBSZ_MASK);
pub const DMA_RBSZ_SHIFT: u32 = b::DMA_RBSZ_SHIFT;
pub const DMA_CHAN_STATUS_NIS: u32 = k(b::k3gmac_generated_values_K3GMAC_DMA_CHAN_STATUS_NIS);
pub const DMA_CHAN_STATUS_AIS: u32 = k(b::k3gmac_generated_values_K3GMAC_DMA_CHAN_STATUS_AIS);
pub const DMA_CHAN_STATUS_RI: u32 = k(b::k3gmac_generated_values_K3GMAC_DMA_CHAN_STATUS_RI);
pub const DMA_CHAN_STATUS_TI: u32 = k(b::k3gmac_generated_values_K3GMAC_DMA_CHAN_STATUS_TI);
pub const DMA_CHAN_STATUS_RBU: u32 = k(b::k3gmac_generated_values_K3GMAC_DMA_CHAN_STATUS_RBU);
pub const DMA_CHAN_STATUS_TBU: u32 = k(b::k3gmac_generated_values_K3GMAC_DMA_CHAN_STATUS_TBU);
pub const DMA_CHAN_STATUS_FBE: u32 = k(b::k3gmac_generated_values_K3GMAC_DMA_CHAN_STATUS_FBE);
pub const DMA_CHAN_INTR_DEFAULT_MASK: u32 =
    k(b::k3gmac_generated_values_K3GMAC_DMA_CHAN_INTR_DEFAULT_MASK);

pub const MII_ADDR_GBUSY: u32 = k(b::k3gmac_generated_values_K3GMAC_MII_ADDR_GBUSY);
pub const MII_GMAC4_READ: u32 = k(b::k3gmac_generated_values_K3GMAC_MII_GMAC4_READ);
pub const MII_GMAC4_REG_ADDR_SHIFT: u32 =
    k(b::k3gmac_generated_values_K3GMAC_MII_GMAC4_REG_ADDR_SHIFT);

pub const PHY_INTF_MODE_MASK: u32 = k(b::k3gmac_generated_values_K3GMAC_PHY_INTF_MODE_MASK);
pub const PHY_INTF_RMII: u32 = k(b::k3gmac_generated_values_K3GMAC_PHY_INTF_RMII);
pub const PHY_INTF_RGMII: u32 = k(b::k3gmac_generated_values_K3GMAC_PHY_INTF_RGMII);
pub const PHY_INTF_MII: u32 = k(b::k3gmac_generated_values_K3GMAC_PHY_INTF_MII);
pub const WOL_WAKE_IRQ_EN: u32 = k(b::k3gmac_generated_values_K3GMAC_WOL_WAKE_IRQ_EN);
pub const EMAC_RX_DLINE_EN: u32 = k(b::k3gmac_generated_values_K3GMAC_EMAC_RX_DLINE_EN);
pub const EMAC_TX_DLINE_EN: u32 = k(b::k3gmac_generated_values_K3GMAC_EMAC_TX_DLINE_EN);
pub const EMAC_RX_DLINE_CODE_MASK: u32 =
    k(b::k3gmac_generated_values_K3GMAC_EMAC_RX_DLINE_CODE_MASK);
pub const EMAC_TX_DLINE_CODE_MASK: u32 =
    k(b::k3gmac_generated_values_K3GMAC_EMAC_TX_DLINE_CODE_MASK);

const fn k(value: b::k3gmac_generated_values) -> u32 {
    value
}

#[derive(Debug)]
pub struct Mmio {
    base: NonNull<u8>,
}

unsafe impl Send for Mmio {}

impl Mmio {
    pub const unsafe fn new(base: NonNull<u8>) -> Self {
        Self { base }
    }

    pub fn read(&self, offset: u32) -> u32 {
        unsafe {
            self.base
                .as_ptr()
                .add(offset as usize)
                .cast::<u32>()
                .read_volatile()
        }
    }

    pub fn write(&self, offset: u32, value: u32) {
        unsafe {
            self.base
                .as_ptr()
                .add(offset as usize)
                .cast::<u32>()
                .write_volatile(value);
        }
    }

    pub fn update(&self, offset: u32, clear: u32, set: u32) {
        let value = (self.read(offset) & !clear) | set;
        self.write(offset, value);
    }
}

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

pub const fn mtl_chan_base(chan: u32) -> u32 {
    MTL_CHAN_BASE_ADDR + chan * MTL_CHAN_BASE_OFFSET
}

pub const fn mtl_chan_tx_op_mode(chan: u32) -> u32 {
    mtl_chan_base(chan)
}

pub const fn mtl_chan_rx_op_mode(chan: u32) -> u32 {
    mtl_chan_base(chan) + 0x30
}
