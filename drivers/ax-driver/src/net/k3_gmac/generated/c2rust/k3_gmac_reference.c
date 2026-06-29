/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Freestanding C extraction for c2rust.
 *
 * This file intentionally keeps only the K3 syscon glue and DWMAC4/5 register
 * sequences used by the Rust no_std driver. The original Linux files are kept
 * under ../../linux/ and depend on Linux netdev, clk, regmap, DMA, phylink, and
 * platform-device APIs.
 */

typedef unsigned char u8;
typedef unsigned int u32;
typedef unsigned long long u64;
typedef unsigned long long dma_addr_t;
typedef int bool;

#define true 1
#define false 0
#define BIT(nr) (1U << (nr))
#define GENMASK(h, l) (((~0U) - ((1U << (l)) - 1U)) & (~0U >> (31U - (h))))
#define FIELD_PREP(mask, val) (((val) << (__builtin_ffs(mask) - 1)) & (mask))

struct dma_desc {
	u32 des0;
	u32 des1;
	u32 des2;
	u32 des3;
};

struct k3_mmio {
	u32 regs[2048];
};

static u32 readl(struct k3_mmio *io, u32 offset)
{
	return io->regs[offset / 4];
}

static void writel(u32 value, struct k3_mmio *io, u32 offset)
{
	io->regs[offset / 4] = value;
}

static void update_bits(struct k3_mmio *io, u32 offset, u32 mask, u32 value)
{
	u32 old = readl(io, offset);

	writel((old & ~mask) | (value & mask), io, offset);
}

#define PHY_INTF_MODE_MASK GENMASK(4, 3)
#define PHY_INTF_RMII FIELD_PREP(PHY_INTF_MODE_MASK, 0x0)
#define PHY_INTF_RGMII FIELD_PREP(PHY_INTF_MODE_MASK, 0x1)
#define PHY_INTF_MII FIELD_PREP(PHY_INTF_MODE_MASK, 0x3)
#define RMII_TX_CLK_SEL BIT(6)
#define RMII_RX_CLK_SEL BIT(7)
#define LPI_IRQ_EN BIT(9)
#define WAKE_IRQ_EN BIT(12)
#define AXI_SINGLE_ID BIT(13)
#define EMAC_RX_DLINE_EN BIT(0)
#define EMAC_TX_DLINE_EN BIT(16)
#define RMII_TX_PHASE_MASK GENMASK(18, 16)
#define RMII_RX_PHASE_MASK GENMASK(22, 20)
#define RGMII_RX_PHASE_MASK GENMASK(22, 20)
#define RGMII_TX_PHASE_MASK GENMASK(26, 24)
#define EMAC_RX_DLINE_CODE_MASK GENMASK(15, 8)
#define EMAC_TX_DLINE_CODE_MASK GENMASK(31, 24)
#define CLK_PHASE_REVERT 180

enum k3_phy_mode {
	K3_PHY_MII,
	K3_PHY_RMII,
	K3_PHY_RGMII,
	K3_PHY_RGMII_ID,
	K3_PHY_RGMII_RXID,
	K3_PHY_RGMII_TXID,
};

enum k3_clk_tuning_way {
	K3_CLK_TUNING_BY_REG,
	K3_CLK_TUNING_BY_DLINE,
	K3_CLK_TUNING_BY_CLK_REVERT,
};

struct k3_glue {
	enum k3_phy_mode phy_mode;
	enum k3_clk_tuning_way tuning_way;
	bool clk_tuning_enable;
	bool wol_irq_enable;
	u8 tx_clk_phase;
	u8 rx_clk_phase;
	u32 ctrl_off;
	u32 dline_off;
};

void k3_eqos_iface_config_ref(struct k3_mmio *apmu, const struct k3_glue *eqos)
{
	u32 val = PHY_INTF_RGMII;

	if (eqos->phy_mode == K3_PHY_MII)
		val = PHY_INTF_MII;
	else if (eqos->phy_mode == K3_PHY_RMII)
		val = PHY_INTF_RMII;

	update_bits(apmu, eqos->ctrl_off, PHY_INTF_MODE_MASK, val);
}

void k3_eqos_wol_config_ref(struct k3_mmio *apmu, const struct k3_glue *eqos)
{
	u32 val = eqos->wol_irq_enable ? WAKE_IRQ_EN : 0;

	update_bits(apmu, eqos->ctrl_off, WAKE_IRQ_EN, val);
}

void k3_delayline_init_ref(struct k3_mmio *apmu, const struct k3_glue *eqos)
{
	u32 mask = EMAC_TX_DLINE_EN | EMAC_RX_DLINE_EN |
		   EMAC_TX_DLINE_CODE_MASK | EMAC_RX_DLINE_CODE_MASK;
	u32 val = EMAC_TX_DLINE_EN | EMAC_RX_DLINE_EN;

	update_bits(apmu, eqos->dline_off, mask, val);
}

void k3_fix_mac_speed_ref(struct k3_mmio *apmu, const struct k3_glue *eqos)
{
	if (!eqos->clk_tuning_enable)
		return;

	if (eqos->phy_mode == K3_PHY_RGMII_ID || eqos->phy_mode == K3_PHY_MII)
		return;

	if (eqos->phy_mode == K3_PHY_RMII) {
		if (eqos->tuning_way == K3_CLK_TUNING_BY_REG) {
			update_bits(apmu, eqos->ctrl_off, RMII_TX_PHASE_MASK,
				    FIELD_PREP(RMII_TX_PHASE_MASK,
					       eqos->tx_clk_phase));
			update_bits(apmu, eqos->ctrl_off, RMII_RX_PHASE_MASK,
				    FIELD_PREP(RMII_RX_PHASE_MASK,
					       eqos->rx_clk_phase));
		} else if (eqos->tuning_way == K3_CLK_TUNING_BY_CLK_REVERT) {
			update_bits(apmu, eqos->ctrl_off, RMII_TX_CLK_SEL,
				    eqos->tx_clk_phase == CLK_PHASE_REVERT ?
					    RMII_TX_CLK_SEL :
					    0);
			update_bits(apmu, eqos->ctrl_off, RMII_RX_CLK_SEL,
				    eqos->rx_clk_phase == CLK_PHASE_REVERT ?
					    RMII_RX_CLK_SEL :
					    0);
		}
		return;
	}

	if (eqos->phy_mode != K3_PHY_RGMII_TXID) {
		if (eqos->tuning_way == K3_CLK_TUNING_BY_DLINE)
			update_bits(apmu, eqos->dline_off,
				    EMAC_RX_DLINE_CODE_MASK,
				    FIELD_PREP(EMAC_RX_DLINE_CODE_MASK,
					       eqos->rx_clk_phase));
		else
			update_bits(apmu, eqos->ctrl_off, RGMII_RX_PHASE_MASK,
				    FIELD_PREP(RGMII_RX_PHASE_MASK,
					       eqos->rx_clk_phase));
	}

	if (eqos->phy_mode != K3_PHY_RGMII_RXID) {
		if (eqos->tuning_way == K3_CLK_TUNING_BY_DLINE)
			update_bits(apmu, eqos->dline_off,
				    EMAC_TX_DLINE_CODE_MASK,
				    FIELD_PREP(EMAC_TX_DLINE_CODE_MASK,
					       eqos->tx_clk_phase));
		else
			update_bits(apmu, eqos->ctrl_off, RGMII_TX_PHASE_MASK,
				    FIELD_PREP(RGMII_TX_PHASE_MASK,
					       eqos->tx_clk_phase));
	}
}

#define TDES2_BUFFER1_SIZE_MASK GENMASK(13, 0)
#define TDES2_INTERRUPT_ON_COMPLETION BIT(31)
#define TDES3_PACKET_SIZE_MASK GENMASK(14, 0)
#define TDES3_CHECKSUM_INSERTION_SHIFT 16
#define TDES3_CHECKSUM_INSERTION_MASK GENMASK(17, 16)
#define TDES3_ERROR_SUMMARY BIT(15)
#define TDES3_LAST_DESCRIPTOR BIT(28)
#define TDES3_FIRST_DESCRIPTOR BIT(29)
#define TDES3_OWN BIT(31)
#define TX_CIC_FULL 3
#define RDES3_PACKET_SIZE_MASK GENMASK(14, 0)
#define RDES3_ERROR_SUMMARY BIT(15)
#define RDES3_BUFFER1_VALID_ADDR BIT(24)
#define RDES3_LAST_DESCRIPTOR BIT(28)
#define RDES3_FIRST_DESCRIPTOR BIT(29)
#define RDES3_INT_ON_COMPLETION_EN BIT(30)
#define RDES3_OWN BIT(31)

void dwmac4_set_addr_ref(struct dma_desc *p, dma_addr_t addr)
{
	p->des0 = (u32)addr;
	p->des1 = (u32)(addr >> 32);
}

void dwmac4_rd_init_rx_desc_ref(struct dma_desc *p, dma_addr_t addr,
				bool disable_rx_ic)
{
	u32 flags = RDES3_OWN | RDES3_BUFFER1_VALID_ADDR;

	dwmac4_set_addr_ref(p, addr);
	p->des2 = 0;
	if (!disable_rx_ic)
		flags |= RDES3_INT_ON_COMPLETION_EN;
	p->des3 = flags;
}

void dwmac4_rd_prepare_tx_desc_ref(struct dma_desc *p, dma_addr_t addr, u32 len,
				   bool csum)
{
	u32 tdes3 = len & TDES3_PACKET_SIZE_MASK;

	dwmac4_set_addr_ref(p, addr);
	p->des2 = (len & TDES2_BUFFER1_SIZE_MASK) |
		  TDES2_INTERRUPT_ON_COMPLETION;
	tdes3 |= TDES3_FIRST_DESCRIPTOR | TDES3_LAST_DESCRIPTOR;
	if (csum)
		tdes3 |= (TX_CIC_FULL << TDES3_CHECKSUM_INSERTION_SHIFT) &
			 TDES3_CHECKSUM_INSERTION_MASK;
	p->des3 = tdes3 | TDES3_OWN;
}

bool dwmac4_tx_owned_ref(const struct dma_desc *p)
{
	return (p->des3 & TDES3_OWN) != 0;
}

bool dwmac4_rx_ready_ref(const struct dma_desc *p)
{
	return !(p->des3 & RDES3_OWN) && (p->des3 & RDES3_LAST_DESCRIPTOR) &&
	       (p->des3 & RDES3_FIRST_DESCRIPTOR);
}

u32 dwmac4_rx_len_ref(const struct dma_desc *p)
{
	return p->des3 & RDES3_PACKET_SIZE_MASK;
}

#define GMAC_CONFIG 0x0000
#define GMAC_PACKET_FILTER 0x0008
#define GMAC_ADDR_HIGH0 0x0300
#define GMAC_ADDR_LOW0 0x0304
#define GMAC_CONFIG_RE BIT(0)
#define GMAC_CONFIG_TE BIT(1)
#define GMAC_CONFIG_DM BIT(13)
#define GMAC_CONFIG_FES BIT(14)
#define GMAC_CONFIG_PS BIT(15)
#define GMAC_CONFIG_IPC BIT(27)
#define GMAC_PACKET_FILTER_PR BIT(0)
#define GMAC_PACKET_FILTER_PM BIT(4)
#define GMAC_HI_REG_AE BIT(31)

#define MTL_CHAN_BASE_ADDR 0x0d00
#define MTL_CHAN_BASE_OFFSET 0x40
#define MTL_CHAN_TX_OP_MODE(chan) (MTL_CHAN_BASE_ADDR + (chan) * MTL_CHAN_BASE_OFFSET)
#define MTL_CHAN_RX_OP_MODE(chan) (MTL_CHAN_TX_OP_MODE(chan) + 0x30)
#define MTL_OP_MODE_RSF BIT(5)
#define MTL_OP_MODE_TXQEN_MASK GENMASK(3, 2)
#define MTL_OP_MODE_TXQEN BIT(3)
#define MTL_OP_MODE_TSF BIT(1)
#define MTL_OP_MODE_TQS_MASK GENMASK(24, 16)
#define MTL_OP_MODE_TQS_SHIFT 16
#define MTL_OP_MODE_RQS_MASK GENMASK(29, 20)
#define MTL_OP_MODE_RQS_SHIFT 20
#define MTL_OP_MODE_RFD_MASK GENMASK(19, 14)
#define MTL_OP_MODE_RFD_SHIFT 14
#define MTL_OP_MODE_RFA_MASK GENMASK(13, 8)
#define MTL_OP_MODE_RFA_SHIFT 8
#define MTL_OP_MODE_EHFC BIT(7)
#define MTL_OP_MODE_DIS_TCP_EF BIT(6)

#define DMA_BUS_MODE 0x1000
#define DMA_SYS_BUS_MODE 0x1004
#define DMA_AXI_BUS_MODE 0x1028
#define DMA_CHAN_BASE_ADDR 0x1100
#define DMA_CHAN_BASE_OFFSET 0x80
#define DMA_CHAN_BASE(chan) (DMA_CHAN_BASE_ADDR + (chan) * DMA_CHAN_BASE_OFFSET)
#define DMA_CHAN_TX_CONTROL(chan) (DMA_CHAN_BASE(chan) + 0x04)
#define DMA_CHAN_RX_CONTROL(chan) (DMA_CHAN_BASE(chan) + 0x08)
#define DMA_CHAN_TX_BASE_HI(chan) (DMA_CHAN_BASE(chan) + 0x10)
#define DMA_CHAN_TX_BASE(chan) (DMA_CHAN_BASE(chan) + 0x14)
#define DMA_CHAN_RX_BASE_HI(chan) (DMA_CHAN_BASE(chan) + 0x18)
#define DMA_CHAN_RX_BASE(chan) (DMA_CHAN_BASE(chan) + 0x1c)
#define DMA_CHAN_TX_END(chan) (DMA_CHAN_BASE(chan) + 0x20)
#define DMA_CHAN_RX_END(chan) (DMA_CHAN_BASE(chan) + 0x28)
#define DMA_CHAN_TX_RING_LEN(chan) (DMA_CHAN_BASE(chan) + 0x2c)
#define DMA_CHAN_RX_RING_LEN(chan) (DMA_CHAN_BASE(chan) + 0x30)
#define DMA_CHAN_INTR_ENA(chan) (DMA_CHAN_BASE(chan) + 0x34)
#define DMA_BUS_MODE_SFT_RESET BIT(0)
#define DMA_SYS_BUS_FB BIT(0)
#define DMA_SYS_BUS_AAL BIT(12)
#define DMA_SYS_BUS_MB BIT(14)
#define DMA_BURST_LEN_DEFAULT 0xfe
#define DMA_CONTROL_ST BIT(0)
#define DMA_CONTROL_SR BIT(0)
#define DMA_CONTROL_OSP BIT(4)
#define DMA_RBSZ_MASK GENMASK(14, 1)
#define DMA_RBSZ_SHIFT 1

void dwmac4_dma_init_ref(struct k3_mmio *io)
{
	update_bits(io, DMA_SYS_BUS_MODE, 0,
		    DMA_SYS_BUS_FB | DMA_SYS_BUS_MB | DMA_SYS_BUS_AAL);
	writel(DMA_BURST_LEN_DEFAULT, io, DMA_AXI_BUS_MODE);
}

void dwmac4_dma_init_channel_ref(struct k3_mmio *io, u32 chan, u64 tx_base,
				 u64 rx_base, u32 ring_len, u32 rx_buf_size)
{
	writel((u32)(tx_base >> 32), io, DMA_CHAN_TX_BASE_HI(chan));
	writel((u32)tx_base, io, DMA_CHAN_TX_BASE(chan));
	writel((u32)(rx_base >> 32), io, DMA_CHAN_RX_BASE_HI(chan));
	writel((u32)rx_base, io, DMA_CHAN_RX_BASE(chan));
	writel(ring_len - 1, io, DMA_CHAN_TX_RING_LEN(chan));
	writel(ring_len - 1, io, DMA_CHAN_RX_RING_LEN(chan));
	writel((u32)tx_base, io, DMA_CHAN_TX_END(chan));
	writel((u32)rx_base, io, DMA_CHAN_RX_END(chan));
	update_bits(io, DMA_CHAN_TX_CONTROL(chan), 0, DMA_CONTROL_OSP);
	update_bits(io, DMA_CHAN_RX_CONTROL(chan), DMA_RBSZ_MASK,
		    (rx_buf_size << DMA_RBSZ_SHIFT) & DMA_RBSZ_MASK);
}

void dwmac4_dma_rx_chan_op_mode_ref(struct k3_mmio *io, u32 chan, u32 fifosz)
{
	u32 rqs = fifosz / 256 - 1;
	u32 val = readl(io, MTL_CHAN_RX_OP_MODE(chan));

	val |= MTL_OP_MODE_DIS_TCP_EF | MTL_OP_MODE_RSF;
	val &= ~MTL_OP_MODE_RQS_MASK;
	val |= rqs << MTL_OP_MODE_RQS_SHIFT;
	if (fifosz >= 4096) {
		val |= MTL_OP_MODE_EHFC;
		val &= ~MTL_OP_MODE_RFD_MASK;
		val |= ((fifosz == 4096) ? 0x03 : 0x07)
		       << MTL_OP_MODE_RFD_SHIFT;
		val &= ~MTL_OP_MODE_RFA_MASK;
		val |= ((fifosz == 4096) ? 0x01 : 0x04)
		       << MTL_OP_MODE_RFA_SHIFT;
	}
	writel(val, io, MTL_CHAN_RX_OP_MODE(chan));
}

void dwmac4_dma_tx_chan_op_mode_ref(struct k3_mmio *io, u32 chan, u32 fifosz)
{
	u32 tqs = fifosz / 256 - 1;
	u32 val = readl(io, MTL_CHAN_TX_OP_MODE(chan));

	val |= MTL_OP_MODE_TSF;
	val &= ~MTL_OP_MODE_TXQEN_MASK;
	val |= MTL_OP_MODE_TXQEN;
	val &= ~MTL_OP_MODE_TQS_MASK;
	val |= tqs << MTL_OP_MODE_TQS_SHIFT;
	writel(val, io, MTL_CHAN_TX_OP_MODE(chan));
}

void dwmac4_program_mac_ref(struct k3_mmio *io, const u8 mac[6])
{
	u32 low = mac[0] | (mac[1] << 8) | (mac[2] << 16) | (mac[3] << 24);
	u32 high = mac[4] | (mac[5] << 8) | GMAC_HI_REG_AE;

	writel(low, io, GMAC_ADDR_LOW0);
	writel(high, io, GMAC_ADDR_HIGH0);
}

void dwmac4_enable_mac_ref(struct k3_mmio *io)
{
	update_bits(io, GMAC_PACKET_FILTER, 0,
		    GMAC_PACKET_FILTER_PM | GMAC_PACKET_FILTER_PR);
	update_bits(io, GMAC_CONFIG, GMAC_CONFIG_FES | GMAC_CONFIG_PS,
		    GMAC_CONFIG_DM | GMAC_CONFIG_IPC | GMAC_CONFIG_TE |
			    GMAC_CONFIG_RE);
}
