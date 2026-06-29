pub type u8_0 = ::core::ffi::c_uchar;
pub type u32_0 = ::core::ffi::c_uint;
pub type u64_0 = ::core::ffi::c_ulonglong;
pub type dma_addr_t = ::core::ffi::c_ulonglong;
pub type bool_0 = ::core::ffi::c_int;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct dma_desc {
    pub des0: u32_0,
    pub des1: u32_0,
    pub des2: u32_0,
    pub des3: u32_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct k3_mmio {
    pub regs: [u32_0; 2048],
}
pub type k3_phy_mode = ::core::ffi::c_uint;
pub const K3_PHY_RGMII_TXID: k3_phy_mode = 5;
pub const K3_PHY_RGMII_RXID: k3_phy_mode = 4;
pub const K3_PHY_RGMII_ID: k3_phy_mode = 3;
pub const K3_PHY_RGMII: k3_phy_mode = 2;
pub const K3_PHY_RMII: k3_phy_mode = 1;
pub const K3_PHY_MII: k3_phy_mode = 0;
pub type k3_clk_tuning_way = ::core::ffi::c_uint;
pub const K3_CLK_TUNING_BY_CLK_REVERT: k3_clk_tuning_way = 2;
pub const K3_CLK_TUNING_BY_DLINE: k3_clk_tuning_way = 1;
pub const K3_CLK_TUNING_BY_REG: k3_clk_tuning_way = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct k3_glue {
    pub phy_mode: k3_phy_mode,
    pub tuning_way: k3_clk_tuning_way,
    pub clk_tuning_enable: bool_0,
    pub wol_irq_enable: bool_0,
    pub tx_clk_phase: u8_0,
    pub rx_clk_phase: u8_0,
    pub ctrl_off: u32_0,
    pub dline_off: u32_0,
}
unsafe extern "C" fn readl(mut io: *mut k3_mmio, mut offset: u32_0) -> u32_0 {
    return (*io).regs[offset.wrapping_div(4 as u32_0) as usize];
}
unsafe extern "C" fn writel(mut value: u32_0, mut io: *mut k3_mmio, mut offset: u32_0) {
    (*io).regs[offset.wrapping_div(4 as u32_0) as usize] = value;
}
unsafe extern "C" fn update_bits(
    mut io: *mut k3_mmio,
    mut offset: u32_0,
    mut mask: u32_0,
    mut value: u32_0,
) {
    let mut old: u32_0 = readl(io, offset);
    writel(old & !mask | value & mask, io, offset);
}
pub const PHY_INTF_MODE_MASK: ::core::ffi::c_uint = (!(0 as ::core::ffi::c_uint)).wrapping_sub(
    ((1 as ::core::ffi::c_uint) << 3 as ::core::ffi::c_int).wrapping_sub(1 as ::core::ffi::c_uint),
) & !(0 as ::core::ffi::c_uint)
    >> (31 as ::core::ffi::c_uint).wrapping_sub(4 as ::core::ffi::c_uint);
pub const PHY_INTF_RMII: ::core::ffi::c_uint = ((0 as ::core::ffi::c_int)
    << if ((!(0 as ::core::ffi::c_uint)).wrapping_sub(
        ((1 as ::core::ffi::c_uint) << 3 as ::core::ffi::c_int)
            .wrapping_sub(1 as ::core::ffi::c_uint),
    ) & !(0 as ::core::ffi::c_uint)
        >> (31 as ::core::ffi::c_uint).wrapping_sub(4 as ::core::ffi::c_uint))
        as ::core::ffi::c_int
        == 0
    {
        0
    } else {
        (((!(0 as ::core::ffi::c_uint)).wrapping_sub(
            ((1 as ::core::ffi::c_uint) << 3 as ::core::ffi::c_int)
                .wrapping_sub(1 as ::core::ffi::c_uint),
        ) & !(0 as ::core::ffi::c_uint)
            >> (31 as ::core::ffi::c_uint).wrapping_sub(4 as ::core::ffi::c_uint))
            as ::core::ffi::c_int)
            .trailing_zeros() as i32
            + 1
    } - 1 as ::core::ffi::c_int)
    as ::core::ffi::c_uint
    & ((!(0 as ::core::ffi::c_uint)).wrapping_sub(
        ((1 as ::core::ffi::c_uint) << 3 as ::core::ffi::c_int)
            .wrapping_sub(1 as ::core::ffi::c_uint),
    ) & !(0 as ::core::ffi::c_uint)
        >> (31 as ::core::ffi::c_uint).wrapping_sub(4 as ::core::ffi::c_uint));
pub const PHY_INTF_RGMII: ::core::ffi::c_uint = ((0x1 as ::core::ffi::c_int)
    << if ((!(0 as ::core::ffi::c_uint)).wrapping_sub(
        ((1 as ::core::ffi::c_uint) << 3 as ::core::ffi::c_int)
            .wrapping_sub(1 as ::core::ffi::c_uint),
    ) & !(0 as ::core::ffi::c_uint)
        >> (31 as ::core::ffi::c_uint).wrapping_sub(4 as ::core::ffi::c_uint))
        as ::core::ffi::c_int
        == 0
    {
        0
    } else {
        (((!(0 as ::core::ffi::c_uint)).wrapping_sub(
            ((1 as ::core::ffi::c_uint) << 3 as ::core::ffi::c_int)
                .wrapping_sub(1 as ::core::ffi::c_uint),
        ) & !(0 as ::core::ffi::c_uint)
            >> (31 as ::core::ffi::c_uint).wrapping_sub(4 as ::core::ffi::c_uint))
            as ::core::ffi::c_int)
            .trailing_zeros() as i32
            + 1
    } - 1 as ::core::ffi::c_int)
    as ::core::ffi::c_uint
    & ((!(0 as ::core::ffi::c_uint)).wrapping_sub(
        ((1 as ::core::ffi::c_uint) << 3 as ::core::ffi::c_int)
            .wrapping_sub(1 as ::core::ffi::c_uint),
    ) & !(0 as ::core::ffi::c_uint)
        >> (31 as ::core::ffi::c_uint).wrapping_sub(4 as ::core::ffi::c_uint));
pub const PHY_INTF_MII: ::core::ffi::c_uint = ((0x3 as ::core::ffi::c_int)
    << if ((!(0 as ::core::ffi::c_uint)).wrapping_sub(
        ((1 as ::core::ffi::c_uint) << 3 as ::core::ffi::c_int)
            .wrapping_sub(1 as ::core::ffi::c_uint),
    ) & !(0 as ::core::ffi::c_uint)
        >> (31 as ::core::ffi::c_uint).wrapping_sub(4 as ::core::ffi::c_uint))
        as ::core::ffi::c_int
        == 0
    {
        0
    } else {
        (((!(0 as ::core::ffi::c_uint)).wrapping_sub(
            ((1 as ::core::ffi::c_uint) << 3 as ::core::ffi::c_int)
                .wrapping_sub(1 as ::core::ffi::c_uint),
        ) & !(0 as ::core::ffi::c_uint)
            >> (31 as ::core::ffi::c_uint).wrapping_sub(4 as ::core::ffi::c_uint))
            as ::core::ffi::c_int)
            .trailing_zeros() as i32
            + 1
    } - 1 as ::core::ffi::c_int)
    as ::core::ffi::c_uint
    & ((!(0 as ::core::ffi::c_uint)).wrapping_sub(
        ((1 as ::core::ffi::c_uint) << 3 as ::core::ffi::c_int)
            .wrapping_sub(1 as ::core::ffi::c_uint),
    ) & !(0 as ::core::ffi::c_uint)
        >> (31 as ::core::ffi::c_uint).wrapping_sub(4 as ::core::ffi::c_uint));
pub const RMII_TX_CLK_SEL: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 6 as ::core::ffi::c_int;
pub const RMII_RX_CLK_SEL: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 7 as ::core::ffi::c_int;
pub const WAKE_IRQ_EN: ::core::ffi::c_uint = (1 as ::core::ffi::c_uint) << 12 as ::core::ffi::c_int;
pub const EMAC_RX_DLINE_EN: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 0 as ::core::ffi::c_int;
pub const EMAC_TX_DLINE_EN: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 16 as ::core::ffi::c_int;
pub const RMII_TX_PHASE_MASK: ::core::ffi::c_uint = (!(0 as ::core::ffi::c_uint)).wrapping_sub(
    ((1 as ::core::ffi::c_uint) << 16 as ::core::ffi::c_int).wrapping_sub(1 as ::core::ffi::c_uint),
) & !(0 as ::core::ffi::c_uint)
    >> (31 as ::core::ffi::c_uint).wrapping_sub(18 as ::core::ffi::c_uint);
pub const RMII_RX_PHASE_MASK: ::core::ffi::c_uint = (!(0 as ::core::ffi::c_uint)).wrapping_sub(
    ((1 as ::core::ffi::c_uint) << 20 as ::core::ffi::c_int).wrapping_sub(1 as ::core::ffi::c_uint),
) & !(0 as ::core::ffi::c_uint)
    >> (31 as ::core::ffi::c_uint).wrapping_sub(22 as ::core::ffi::c_uint);
pub const RGMII_RX_PHASE_MASK: ::core::ffi::c_uint = (!(0 as ::core::ffi::c_uint)).wrapping_sub(
    ((1 as ::core::ffi::c_uint) << 20 as ::core::ffi::c_int).wrapping_sub(1 as ::core::ffi::c_uint),
) & !(0 as ::core::ffi::c_uint)
    >> (31 as ::core::ffi::c_uint).wrapping_sub(22 as ::core::ffi::c_uint);
pub const RGMII_TX_PHASE_MASK: ::core::ffi::c_uint = (!(0 as ::core::ffi::c_uint)).wrapping_sub(
    ((1 as ::core::ffi::c_uint) << 24 as ::core::ffi::c_int).wrapping_sub(1 as ::core::ffi::c_uint),
) & !(0 as ::core::ffi::c_uint)
    >> (31 as ::core::ffi::c_uint).wrapping_sub(26 as ::core::ffi::c_uint);
pub const EMAC_RX_DLINE_CODE_MASK: ::core::ffi::c_uint =
    (!(0 as ::core::ffi::c_uint)).wrapping_sub(
        ((1 as ::core::ffi::c_uint) << 8 as ::core::ffi::c_int)
            .wrapping_sub(1 as ::core::ffi::c_uint),
    ) & !(0 as ::core::ffi::c_uint)
        >> (31 as ::core::ffi::c_uint).wrapping_sub(15 as ::core::ffi::c_uint);
pub const EMAC_TX_DLINE_CODE_MASK: ::core::ffi::c_uint =
    (!(0 as ::core::ffi::c_uint)).wrapping_sub(
        ((1 as ::core::ffi::c_uint) << 24 as ::core::ffi::c_int)
            .wrapping_sub(1 as ::core::ffi::c_uint),
    ) & !(0 as ::core::ffi::c_uint)
        >> (31 as ::core::ffi::c_uint).wrapping_sub(31 as ::core::ffi::c_uint);
pub const CLK_PHASE_REVERT: ::core::ffi::c_int = 180 as ::core::ffi::c_int;
#[no_mangle]
pub unsafe extern "C" fn k3_eqos_iface_config_ref(
    mut apmu: *mut k3_mmio,
    mut eqos: *const k3_glue,
) {
    let mut val: u32_0 = PHY_INTF_RGMII;
    if (*eqos).phy_mode as ::core::ffi::c_uint
        == K3_PHY_MII as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        val = PHY_INTF_MII as u32_0;
    } else if (*eqos).phy_mode as ::core::ffi::c_uint
        == K3_PHY_RMII as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        val = PHY_INTF_RMII as u32_0;
    }
    update_bits(apmu, (*eqos).ctrl_off, PHY_INTF_MODE_MASK, val);
}
#[no_mangle]
pub unsafe extern "C" fn k3_eqos_wol_config_ref(mut apmu: *mut k3_mmio, mut eqos: *const k3_glue) {
    let mut val: u32_0 = if (*eqos).wol_irq_enable != 0 {
        WAKE_IRQ_EN
    } else {
        0 as u32_0
    };
    update_bits(apmu, (*eqos).ctrl_off, WAKE_IRQ_EN, val);
}
#[no_mangle]
pub unsafe extern "C" fn k3_delayline_init_ref(mut apmu: *mut k3_mmio, mut eqos: *const k3_glue) {
    let mut mask: u32_0 =
        EMAC_TX_DLINE_EN | EMAC_RX_DLINE_EN | EMAC_TX_DLINE_CODE_MASK | EMAC_RX_DLINE_CODE_MASK;
    let mut val: u32_0 = EMAC_TX_DLINE_EN | EMAC_RX_DLINE_EN;
    update_bits(apmu, (*eqos).dline_off, mask, val);
}
#[no_mangle]
pub unsafe extern "C" fn k3_fix_mac_speed_ref(mut apmu: *mut k3_mmio, mut eqos: *const k3_glue) {
    if (*eqos).clk_tuning_enable == 0 {
        return;
    }
    if (*eqos).phy_mode as ::core::ffi::c_uint
        == K3_PHY_RGMII_ID as ::core::ffi::c_int as ::core::ffi::c_uint
        || (*eqos).phy_mode as ::core::ffi::c_uint
            == K3_PHY_MII as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return;
    }
    if (*eqos).phy_mode as ::core::ffi::c_uint
        == K3_PHY_RMII as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        if (*eqos).tuning_way as ::core::ffi::c_uint
            == K3_CLK_TUNING_BY_REG as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            update_bits(
                apmu,
                (*eqos).ctrl_off,
                RMII_TX_PHASE_MASK,
                (((*eqos).tx_clk_phase as ::core::ffi::c_int)
                    << if ((!(0 as ::core::ffi::c_uint)).wrapping_sub(
                        ((1 as ::core::ffi::c_uint) << 16 as ::core::ffi::c_int)
                            .wrapping_sub(1 as ::core::ffi::c_uint),
                    ) & !(0 as ::core::ffi::c_uint)
                        >> (31 as ::core::ffi::c_uint).wrapping_sub(18 as ::core::ffi::c_uint))
                        as ::core::ffi::c_int
                        == 0
                    {
                        0
                    } else {
                        (((!(0 as ::core::ffi::c_uint)).wrapping_sub(
                            ((1 as ::core::ffi::c_uint) << 16 as ::core::ffi::c_int)
                                .wrapping_sub(1 as ::core::ffi::c_uint),
                        ) & !(0 as ::core::ffi::c_uint)
                            >> (31 as ::core::ffi::c_uint).wrapping_sub(18 as ::core::ffi::c_uint))
                            as ::core::ffi::c_int)
                            .trailing_zeros() as i32
                            + 1
                    } - 1 as ::core::ffi::c_int) as u32_0
                    & ((!(0 as u32_0)).wrapping_sub(
                        ((1 as u32_0) << 16 as ::core::ffi::c_int).wrapping_sub(1 as u32_0),
                    ) & !(0 as u32_0)
                        >> (31 as ::core::ffi::c_uint).wrapping_sub(18 as ::core::ffi::c_uint)),
            );
            update_bits(
                apmu,
                (*eqos).ctrl_off,
                RMII_RX_PHASE_MASK,
                (((*eqos).rx_clk_phase as ::core::ffi::c_int)
                    << if ((!(0 as ::core::ffi::c_uint)).wrapping_sub(
                        ((1 as ::core::ffi::c_uint) << 20 as ::core::ffi::c_int)
                            .wrapping_sub(1 as ::core::ffi::c_uint),
                    ) & !(0 as ::core::ffi::c_uint)
                        >> (31 as ::core::ffi::c_uint).wrapping_sub(22 as ::core::ffi::c_uint))
                        as ::core::ffi::c_int
                        == 0
                    {
                        0
                    } else {
                        (((!(0 as ::core::ffi::c_uint)).wrapping_sub(
                            ((1 as ::core::ffi::c_uint) << 20 as ::core::ffi::c_int)
                                .wrapping_sub(1 as ::core::ffi::c_uint),
                        ) & !(0 as ::core::ffi::c_uint)
                            >> (31 as ::core::ffi::c_uint).wrapping_sub(22 as ::core::ffi::c_uint))
                            as ::core::ffi::c_int)
                            .trailing_zeros() as i32
                            + 1
                    } - 1 as ::core::ffi::c_int) as u32_0
                    & ((!(0 as u32_0)).wrapping_sub(
                        ((1 as u32_0) << 20 as ::core::ffi::c_int).wrapping_sub(1 as u32_0),
                    ) & !(0 as u32_0)
                        >> (31 as ::core::ffi::c_uint).wrapping_sub(22 as ::core::ffi::c_uint)),
            );
        } else if (*eqos).tuning_way as ::core::ffi::c_uint
            == K3_CLK_TUNING_BY_CLK_REVERT as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            update_bits(
                apmu,
                (*eqos).ctrl_off,
                RMII_TX_CLK_SEL,
                if (*eqos).tx_clk_phase as ::core::ffi::c_int == CLK_PHASE_REVERT {
                    RMII_TX_CLK_SEL
                } else {
                    0 as u32_0
                },
            );
            update_bits(
                apmu,
                (*eqos).ctrl_off,
                RMII_RX_CLK_SEL,
                if (*eqos).rx_clk_phase as ::core::ffi::c_int == CLK_PHASE_REVERT {
                    RMII_RX_CLK_SEL
                } else {
                    0 as u32_0
                },
            );
        }
        return;
    }
    if (*eqos).phy_mode as ::core::ffi::c_uint
        != K3_PHY_RGMII_TXID as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        if (*eqos).tuning_way as ::core::ffi::c_uint
            == K3_CLK_TUNING_BY_DLINE as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            update_bits(
                apmu,
                (*eqos).dline_off,
                EMAC_RX_DLINE_CODE_MASK,
                (((*eqos).rx_clk_phase as ::core::ffi::c_int)
                    << if ((!(0 as ::core::ffi::c_uint)).wrapping_sub(
                        ((1 as ::core::ffi::c_uint) << 8 as ::core::ffi::c_int)
                            .wrapping_sub(1 as ::core::ffi::c_uint),
                    ) & !(0 as ::core::ffi::c_uint)
                        >> (31 as ::core::ffi::c_uint).wrapping_sub(15 as ::core::ffi::c_uint))
                        as ::core::ffi::c_int
                        == 0
                    {
                        0
                    } else {
                        (((!(0 as ::core::ffi::c_uint)).wrapping_sub(
                            ((1 as ::core::ffi::c_uint) << 8 as ::core::ffi::c_int)
                                .wrapping_sub(1 as ::core::ffi::c_uint),
                        ) & !(0 as ::core::ffi::c_uint)
                            >> (31 as ::core::ffi::c_uint).wrapping_sub(15 as ::core::ffi::c_uint))
                            as ::core::ffi::c_int)
                            .trailing_zeros() as i32
                            + 1
                    } - 1 as ::core::ffi::c_int) as u32_0
                    & ((!(0 as u32_0)).wrapping_sub(
                        ((1 as u32_0) << 8 as ::core::ffi::c_int).wrapping_sub(1 as u32_0),
                    ) & !(0 as u32_0)
                        >> (31 as ::core::ffi::c_uint).wrapping_sub(15 as ::core::ffi::c_uint)),
            );
        } else {
            update_bits(
                apmu,
                (*eqos).ctrl_off,
                RGMII_RX_PHASE_MASK,
                (((*eqos).rx_clk_phase as ::core::ffi::c_int)
                    << if ((!(0 as ::core::ffi::c_uint)).wrapping_sub(
                        ((1 as ::core::ffi::c_uint) << 20 as ::core::ffi::c_int)
                            .wrapping_sub(1 as ::core::ffi::c_uint),
                    ) & !(0 as ::core::ffi::c_uint)
                        >> (31 as ::core::ffi::c_uint).wrapping_sub(22 as ::core::ffi::c_uint))
                        as ::core::ffi::c_int
                        == 0
                    {
                        0
                    } else {
                        (((!(0 as ::core::ffi::c_uint)).wrapping_sub(
                            ((1 as ::core::ffi::c_uint) << 20 as ::core::ffi::c_int)
                                .wrapping_sub(1 as ::core::ffi::c_uint),
                        ) & !(0 as ::core::ffi::c_uint)
                            >> (31 as ::core::ffi::c_uint).wrapping_sub(22 as ::core::ffi::c_uint))
                            as ::core::ffi::c_int)
                            .trailing_zeros() as i32
                            + 1
                    } - 1 as ::core::ffi::c_int) as u32_0
                    & ((!(0 as u32_0)).wrapping_sub(
                        ((1 as u32_0) << 20 as ::core::ffi::c_int).wrapping_sub(1 as u32_0),
                    ) & !(0 as u32_0)
                        >> (31 as ::core::ffi::c_uint).wrapping_sub(22 as ::core::ffi::c_uint)),
            );
        }
    }
    if (*eqos).phy_mode as ::core::ffi::c_uint
        != K3_PHY_RGMII_RXID as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        if (*eqos).tuning_way as ::core::ffi::c_uint
            == K3_CLK_TUNING_BY_DLINE as ::core::ffi::c_int as ::core::ffi::c_uint
        {
            update_bits(
                apmu,
                (*eqos).dline_off,
                EMAC_TX_DLINE_CODE_MASK,
                (((*eqos).tx_clk_phase as ::core::ffi::c_int)
                    << if ((!(0 as ::core::ffi::c_uint)).wrapping_sub(
                        ((1 as ::core::ffi::c_uint) << 24 as ::core::ffi::c_int)
                            .wrapping_sub(1 as ::core::ffi::c_uint),
                    ) & !(0 as ::core::ffi::c_uint)
                        >> (31 as ::core::ffi::c_uint).wrapping_sub(31 as ::core::ffi::c_uint))
                        as ::core::ffi::c_int
                        == 0
                    {
                        0
                    } else {
                        (((!(0 as ::core::ffi::c_uint)).wrapping_sub(
                            ((1 as ::core::ffi::c_uint) << 24 as ::core::ffi::c_int)
                                .wrapping_sub(1 as ::core::ffi::c_uint),
                        ) & !(0 as ::core::ffi::c_uint)
                            >> (31 as ::core::ffi::c_uint).wrapping_sub(31 as ::core::ffi::c_uint))
                            as ::core::ffi::c_int)
                            .trailing_zeros() as i32
                            + 1
                    } - 1 as ::core::ffi::c_int) as u32_0
                    & ((!(0 as u32_0)).wrapping_sub(
                        ((1 as u32_0) << 24 as ::core::ffi::c_int).wrapping_sub(1 as u32_0),
                    ) & !(0 as u32_0)
                        >> (31 as ::core::ffi::c_uint).wrapping_sub(31 as ::core::ffi::c_uint)),
            );
        } else {
            update_bits(
                apmu,
                (*eqos).ctrl_off,
                RGMII_TX_PHASE_MASK,
                (((*eqos).tx_clk_phase as ::core::ffi::c_int)
                    << if ((!(0 as ::core::ffi::c_uint)).wrapping_sub(
                        ((1 as ::core::ffi::c_uint) << 24 as ::core::ffi::c_int)
                            .wrapping_sub(1 as ::core::ffi::c_uint),
                    ) & !(0 as ::core::ffi::c_uint)
                        >> (31 as ::core::ffi::c_uint).wrapping_sub(26 as ::core::ffi::c_uint))
                        as ::core::ffi::c_int
                        == 0
                    {
                        0
                    } else {
                        (((!(0 as ::core::ffi::c_uint)).wrapping_sub(
                            ((1 as ::core::ffi::c_uint) << 24 as ::core::ffi::c_int)
                                .wrapping_sub(1 as ::core::ffi::c_uint),
                        ) & !(0 as ::core::ffi::c_uint)
                            >> (31 as ::core::ffi::c_uint).wrapping_sub(26 as ::core::ffi::c_uint))
                            as ::core::ffi::c_int)
                            .trailing_zeros() as i32
                            + 1
                    } - 1 as ::core::ffi::c_int) as u32_0
                    & ((!(0 as u32_0)).wrapping_sub(
                        ((1 as u32_0) << 24 as ::core::ffi::c_int).wrapping_sub(1 as u32_0),
                    ) & !(0 as u32_0)
                        >> (31 as ::core::ffi::c_uint).wrapping_sub(26 as ::core::ffi::c_uint)),
            );
        }
    }
}
pub const TDES2_BUFFER1_SIZE_MASK: ::core::ffi::c_uint =
    (!(0 as ::core::ffi::c_uint)).wrapping_sub(
        ((1 as ::core::ffi::c_uint) << 0 as ::core::ffi::c_int)
            .wrapping_sub(1 as ::core::ffi::c_uint),
    ) & !(0 as ::core::ffi::c_uint)
        >> (31 as ::core::ffi::c_uint).wrapping_sub(13 as ::core::ffi::c_uint);
pub const TDES2_INTERRUPT_ON_COMPLETION: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 31 as ::core::ffi::c_int;
pub const TDES3_PACKET_SIZE_MASK: ::core::ffi::c_uint = (!(0 as ::core::ffi::c_uint)).wrapping_sub(
    ((1 as ::core::ffi::c_uint) << 0 as ::core::ffi::c_int).wrapping_sub(1 as ::core::ffi::c_uint),
) & !(0 as ::core::ffi::c_uint)
    >> (31 as ::core::ffi::c_uint).wrapping_sub(14 as ::core::ffi::c_uint);
pub const TDES3_CHECKSUM_INSERTION_SHIFT: ::core::ffi::c_int = 16 as ::core::ffi::c_int;
pub const TDES3_CHECKSUM_INSERTION_MASK: ::core::ffi::c_uint =
    (!(0 as ::core::ffi::c_uint)).wrapping_sub(
        ((1 as ::core::ffi::c_uint) << 16 as ::core::ffi::c_int)
            .wrapping_sub(1 as ::core::ffi::c_uint),
    ) & !(0 as ::core::ffi::c_uint)
        >> (31 as ::core::ffi::c_uint).wrapping_sub(17 as ::core::ffi::c_uint);
pub const TDES3_LAST_DESCRIPTOR: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 28 as ::core::ffi::c_int;
pub const TDES3_FIRST_DESCRIPTOR: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 29 as ::core::ffi::c_int;
pub const TDES3_OWN: ::core::ffi::c_uint = (1 as ::core::ffi::c_uint) << 31 as ::core::ffi::c_int;
pub const TX_CIC_FULL: ::core::ffi::c_int = 3 as ::core::ffi::c_int;
pub const RDES3_PACKET_SIZE_MASK: ::core::ffi::c_uint = (!(0 as ::core::ffi::c_uint)).wrapping_sub(
    ((1 as ::core::ffi::c_uint) << 0 as ::core::ffi::c_int).wrapping_sub(1 as ::core::ffi::c_uint),
) & !(0 as ::core::ffi::c_uint)
    >> (31 as ::core::ffi::c_uint).wrapping_sub(14 as ::core::ffi::c_uint);
pub const RDES3_BUFFER1_VALID_ADDR: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 24 as ::core::ffi::c_int;
pub const RDES3_LAST_DESCRIPTOR: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 28 as ::core::ffi::c_int;
pub const RDES3_FIRST_DESCRIPTOR: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 29 as ::core::ffi::c_int;
pub const RDES3_INT_ON_COMPLETION_EN: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 30 as ::core::ffi::c_int;
pub const RDES3_OWN: ::core::ffi::c_uint = (1 as ::core::ffi::c_uint) << 31 as ::core::ffi::c_int;
#[no_mangle]
pub unsafe extern "C" fn dwmac4_set_addr_ref(mut p: *mut dma_desc, mut addr: dma_addr_t) {
    (*p).des0 = addr as u32_0;
    (*p).des1 = (addr >> 32 as ::core::ffi::c_int) as u32_0;
}
#[no_mangle]
pub unsafe extern "C" fn dwmac4_rd_init_rx_desc_ref(
    mut p: *mut dma_desc,
    mut addr: dma_addr_t,
    mut disable_rx_ic: bool_0,
) {
    let mut flags: u32_0 = RDES3_OWN | RDES3_BUFFER1_VALID_ADDR;
    dwmac4_set_addr_ref(p, addr);
    (*p).des2 = 0 as u32_0;
    if disable_rx_ic == 0 {
        flags |= RDES3_INT_ON_COMPLETION_EN;
    }
    (*p).des3 = flags;
}
#[no_mangle]
pub unsafe extern "C" fn dwmac4_rd_prepare_tx_desc_ref(
    mut p: *mut dma_desc,
    mut addr: dma_addr_t,
    mut len: u32_0,
    mut csum: bool_0,
) {
    let mut tdes3: u32_0 = len & TDES3_PACKET_SIZE_MASK;
    dwmac4_set_addr_ref(p, addr);
    (*p).des2 = (len as ::core::ffi::c_uint & TDES2_BUFFER1_SIZE_MASK
        | TDES2_INTERRUPT_ON_COMPLETION) as u32_0;
    tdes3 |= TDES3_FIRST_DESCRIPTOR | TDES3_LAST_DESCRIPTOR;
    if csum != 0 {
        tdes3 |= (TX_CIC_FULL << TDES3_CHECKSUM_INSERTION_SHIFT) as ::core::ffi::c_uint
            & TDES3_CHECKSUM_INSERTION_MASK;
    }
    (*p).des3 = (tdes3 as ::core::ffi::c_uint | TDES3_OWN) as u32_0;
}
#[no_mangle]
pub unsafe extern "C" fn dwmac4_tx_owned_ref(mut p: *const dma_desc) -> bool_0 {
    return ((*p).des3 as ::core::ffi::c_uint & TDES3_OWN != 0 as ::core::ffi::c_uint)
        as ::core::ffi::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn dwmac4_rx_ready_ref(mut p: *const dma_desc) -> bool_0 {
    return ((*p).des3 as ::core::ffi::c_uint & RDES3_OWN == 0
        && (*p).des3 as ::core::ffi::c_uint & RDES3_LAST_DESCRIPTOR != 0
        && (*p).des3 as ::core::ffi::c_uint & RDES3_FIRST_DESCRIPTOR != 0)
        as ::core::ffi::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn dwmac4_rx_len_ref(mut p: *const dma_desc) -> u32_0 {
    return (*p).des3 & RDES3_PACKET_SIZE_MASK;
}
pub const GMAC_CONFIG: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const GMAC_PACKET_FILTER: ::core::ffi::c_int = 0x8 as ::core::ffi::c_int;
pub const GMAC_ADDR_HIGH0: ::core::ffi::c_int = 0x300 as ::core::ffi::c_int;
pub const GMAC_ADDR_LOW0: ::core::ffi::c_int = 0x304 as ::core::ffi::c_int;
pub const GMAC_CONFIG_RE: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 0 as ::core::ffi::c_int;
pub const GMAC_CONFIG_TE: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 1 as ::core::ffi::c_int;
pub const GMAC_CONFIG_DM: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 13 as ::core::ffi::c_int;
pub const GMAC_CONFIG_FES: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 14 as ::core::ffi::c_int;
pub const GMAC_CONFIG_PS: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 15 as ::core::ffi::c_int;
pub const GMAC_CONFIG_IPC: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 27 as ::core::ffi::c_int;
pub const GMAC_PACKET_FILTER_PR: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 0 as ::core::ffi::c_int;
pub const GMAC_PACKET_FILTER_PM: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 4 as ::core::ffi::c_int;
pub const GMAC_HI_REG_AE: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 31 as ::core::ffi::c_int;
pub const MTL_CHAN_BASE_ADDR: ::core::ffi::c_int = 0xd00 as ::core::ffi::c_int;
pub const MTL_CHAN_BASE_OFFSET: ::core::ffi::c_int = 0x40 as ::core::ffi::c_int;
pub const MTL_OP_MODE_RSF: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 5 as ::core::ffi::c_int;
pub const MTL_OP_MODE_TXQEN_MASK: ::core::ffi::c_uint = (!(0 as ::core::ffi::c_uint)).wrapping_sub(
    ((1 as ::core::ffi::c_uint) << 2 as ::core::ffi::c_int).wrapping_sub(1 as ::core::ffi::c_uint),
) & !(0 as ::core::ffi::c_uint)
    >> (31 as ::core::ffi::c_uint).wrapping_sub(3 as ::core::ffi::c_uint);
pub const MTL_OP_MODE_TXQEN: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 3 as ::core::ffi::c_int;
pub const MTL_OP_MODE_TSF: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 1 as ::core::ffi::c_int;
pub const MTL_OP_MODE_TQS_MASK: ::core::ffi::c_uint = (!(0 as ::core::ffi::c_uint)).wrapping_sub(
    ((1 as ::core::ffi::c_uint) << 16 as ::core::ffi::c_int).wrapping_sub(1 as ::core::ffi::c_uint),
) & !(0 as ::core::ffi::c_uint)
    >> (31 as ::core::ffi::c_uint).wrapping_sub(24 as ::core::ffi::c_uint);
pub const MTL_OP_MODE_TQS_SHIFT: ::core::ffi::c_int = 16 as ::core::ffi::c_int;
pub const MTL_OP_MODE_RQS_MASK: ::core::ffi::c_uint = (!(0 as ::core::ffi::c_uint)).wrapping_sub(
    ((1 as ::core::ffi::c_uint) << 20 as ::core::ffi::c_int).wrapping_sub(1 as ::core::ffi::c_uint),
) & !(0 as ::core::ffi::c_uint)
    >> (31 as ::core::ffi::c_uint).wrapping_sub(29 as ::core::ffi::c_uint);
pub const MTL_OP_MODE_RQS_SHIFT: ::core::ffi::c_int = 20 as ::core::ffi::c_int;
pub const MTL_OP_MODE_RFD_MASK: ::core::ffi::c_uint = (!(0 as ::core::ffi::c_uint)).wrapping_sub(
    ((1 as ::core::ffi::c_uint) << 14 as ::core::ffi::c_int).wrapping_sub(1 as ::core::ffi::c_uint),
) & !(0 as ::core::ffi::c_uint)
    >> (31 as ::core::ffi::c_uint).wrapping_sub(19 as ::core::ffi::c_uint);
pub const MTL_OP_MODE_RFD_SHIFT: ::core::ffi::c_int = 14 as ::core::ffi::c_int;
pub const MTL_OP_MODE_RFA_MASK: ::core::ffi::c_uint = (!(0 as ::core::ffi::c_uint)).wrapping_sub(
    ((1 as ::core::ffi::c_uint) << 8 as ::core::ffi::c_int).wrapping_sub(1 as ::core::ffi::c_uint),
) & !(0 as ::core::ffi::c_uint)
    >> (31 as ::core::ffi::c_uint).wrapping_sub(13 as ::core::ffi::c_uint);
pub const MTL_OP_MODE_RFA_SHIFT: ::core::ffi::c_int = 8 as ::core::ffi::c_int;
pub const MTL_OP_MODE_EHFC: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 7 as ::core::ffi::c_int;
pub const MTL_OP_MODE_DIS_TCP_EF: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 6 as ::core::ffi::c_int;
pub const DMA_SYS_BUS_MODE: ::core::ffi::c_int = 0x1004 as ::core::ffi::c_int;
pub const DMA_AXI_BUS_MODE: ::core::ffi::c_int = 0x1028 as ::core::ffi::c_int;
pub const DMA_CHAN_BASE_ADDR: ::core::ffi::c_int = 0x1100 as ::core::ffi::c_int;
pub const DMA_CHAN_BASE_OFFSET: ::core::ffi::c_int = 0x80 as ::core::ffi::c_int;
pub const DMA_SYS_BUS_FB: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 0 as ::core::ffi::c_int;
pub const DMA_SYS_BUS_AAL: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 12 as ::core::ffi::c_int;
pub const DMA_SYS_BUS_MB: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 14 as ::core::ffi::c_int;
pub const DMA_BURST_LEN_DEFAULT: ::core::ffi::c_int = 0xfe as ::core::ffi::c_int;
pub const DMA_CONTROL_OSP: ::core::ffi::c_uint =
    (1 as ::core::ffi::c_uint) << 4 as ::core::ffi::c_int;
pub const DMA_RBSZ_MASK: ::core::ffi::c_uint = (!(0 as ::core::ffi::c_uint)).wrapping_sub(
    ((1 as ::core::ffi::c_uint) << 1 as ::core::ffi::c_int).wrapping_sub(1 as ::core::ffi::c_uint),
) & !(0 as ::core::ffi::c_uint)
    >> (31 as ::core::ffi::c_uint).wrapping_sub(14 as ::core::ffi::c_uint);
pub const DMA_RBSZ_SHIFT: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
#[no_mangle]
pub unsafe extern "C" fn dwmac4_dma_init_ref(mut io: *mut k3_mmio) {
    update_bits(
        io,
        DMA_SYS_BUS_MODE as u32_0,
        0 as u32_0,
        DMA_SYS_BUS_FB | DMA_SYS_BUS_MB | DMA_SYS_BUS_AAL,
    );
    writel(
        DMA_BURST_LEN_DEFAULT as u32_0,
        io,
        DMA_AXI_BUS_MODE as u32_0,
    );
}
#[no_mangle]
pub unsafe extern "C" fn dwmac4_dma_init_channel_ref(
    mut io: *mut k3_mmio,
    mut chan: u32_0,
    mut tx_base: u64_0,
    mut rx_base: u64_0,
    mut ring_len: u32_0,
    mut rx_buf_size: u32_0,
) {
    writel(
        (tx_base >> 32 as ::core::ffi::c_int) as u32_0,
        io,
        (DMA_CHAN_BASE_ADDR as u32_0)
            .wrapping_add(chan.wrapping_mul(DMA_CHAN_BASE_OFFSET as u32_0))
            .wrapping_add(0x10 as u32_0),
    );
    writel(
        tx_base as u32_0,
        io,
        (DMA_CHAN_BASE_ADDR as u32_0)
            .wrapping_add(chan.wrapping_mul(DMA_CHAN_BASE_OFFSET as u32_0))
            .wrapping_add(0x14 as u32_0),
    );
    writel(
        (rx_base >> 32 as ::core::ffi::c_int) as u32_0,
        io,
        (DMA_CHAN_BASE_ADDR as u32_0)
            .wrapping_add(chan.wrapping_mul(DMA_CHAN_BASE_OFFSET as u32_0))
            .wrapping_add(0x18 as u32_0),
    );
    writel(
        rx_base as u32_0,
        io,
        (DMA_CHAN_BASE_ADDR as u32_0)
            .wrapping_add(chan.wrapping_mul(DMA_CHAN_BASE_OFFSET as u32_0))
            .wrapping_add(0x1c as u32_0),
    );
    writel(
        ring_len.wrapping_sub(1 as u32_0),
        io,
        (DMA_CHAN_BASE_ADDR as u32_0)
            .wrapping_add(chan.wrapping_mul(DMA_CHAN_BASE_OFFSET as u32_0))
            .wrapping_add(0x2c as u32_0),
    );
    writel(
        ring_len.wrapping_sub(1 as u32_0),
        io,
        (DMA_CHAN_BASE_ADDR as u32_0)
            .wrapping_add(chan.wrapping_mul(DMA_CHAN_BASE_OFFSET as u32_0))
            .wrapping_add(0x30 as u32_0),
    );
    writel(
        tx_base as u32_0,
        io,
        (DMA_CHAN_BASE_ADDR as u32_0)
            .wrapping_add(chan.wrapping_mul(DMA_CHAN_BASE_OFFSET as u32_0))
            .wrapping_add(0x20 as u32_0),
    );
    writel(
        rx_base as u32_0,
        io,
        (DMA_CHAN_BASE_ADDR as u32_0)
            .wrapping_add(chan.wrapping_mul(DMA_CHAN_BASE_OFFSET as u32_0))
            .wrapping_add(0x28 as u32_0),
    );
    update_bits(
        io,
        (DMA_CHAN_BASE_ADDR as u32_0)
            .wrapping_add(chan.wrapping_mul(DMA_CHAN_BASE_OFFSET as u32_0))
            .wrapping_add(0x4 as u32_0),
        0 as u32_0,
        DMA_CONTROL_OSP,
    );
    update_bits(
        io,
        (DMA_CHAN_BASE_ADDR as u32_0)
            .wrapping_add(chan.wrapping_mul(DMA_CHAN_BASE_OFFSET as u32_0))
            .wrapping_add(0x8 as u32_0),
        DMA_RBSZ_MASK,
        rx_buf_size << DMA_RBSZ_SHIFT & DMA_RBSZ_MASK,
    );
}
#[no_mangle]
pub unsafe extern "C" fn dwmac4_dma_rx_chan_op_mode_ref(
    mut io: *mut k3_mmio,
    mut chan: u32_0,
    mut fifosz: u32_0,
) {
    let mut rqs: u32_0 = fifosz.wrapping_div(256 as u32_0).wrapping_sub(1 as u32_0);
    let mut val: u32_0 = readl(
        io,
        (MTL_CHAN_BASE_ADDR as u32_0)
            .wrapping_add(chan.wrapping_mul(MTL_CHAN_BASE_OFFSET as u32_0))
            .wrapping_add(0x30 as u32_0),
    );
    val |= MTL_OP_MODE_DIS_TCP_EF | MTL_OP_MODE_RSF;
    val &= !MTL_OP_MODE_RQS_MASK;
    val |= rqs << MTL_OP_MODE_RQS_SHIFT;
    if fifosz >= 4096 as u32_0 {
        val |= MTL_OP_MODE_EHFC;
        val &= !MTL_OP_MODE_RFD_MASK;
        val |= ((if fifosz == 4096 as u32_0 {
            0x3 as ::core::ffi::c_int
        } else {
            0x7 as ::core::ffi::c_int
        }) << MTL_OP_MODE_RFD_SHIFT) as u32_0;
        val &= !MTL_OP_MODE_RFA_MASK;
        val |= ((if fifosz == 4096 as u32_0 {
            0x1 as ::core::ffi::c_int
        } else {
            0x4 as ::core::ffi::c_int
        }) << MTL_OP_MODE_RFA_SHIFT) as u32_0;
    }
    writel(
        val,
        io,
        (MTL_CHAN_BASE_ADDR as u32_0)
            .wrapping_add(chan.wrapping_mul(MTL_CHAN_BASE_OFFSET as u32_0))
            .wrapping_add(0x30 as u32_0),
    );
}
#[no_mangle]
pub unsafe extern "C" fn dwmac4_dma_tx_chan_op_mode_ref(
    mut io: *mut k3_mmio,
    mut chan: u32_0,
    mut fifosz: u32_0,
) {
    let mut tqs: u32_0 = fifosz.wrapping_div(256 as u32_0).wrapping_sub(1 as u32_0);
    let mut val: u32_0 = readl(
        io,
        (MTL_CHAN_BASE_ADDR as u32_0)
            .wrapping_add(chan.wrapping_mul(MTL_CHAN_BASE_OFFSET as u32_0)),
    );
    val |= MTL_OP_MODE_TSF;
    val &= !MTL_OP_MODE_TXQEN_MASK;
    val |= MTL_OP_MODE_TXQEN;
    val &= !MTL_OP_MODE_TQS_MASK;
    val |= tqs << MTL_OP_MODE_TQS_SHIFT;
    writel(
        val,
        io,
        (MTL_CHAN_BASE_ADDR as u32_0)
            .wrapping_add(chan.wrapping_mul(MTL_CHAN_BASE_OFFSET as u32_0)),
    );
}
#[no_mangle]
pub unsafe extern "C" fn dwmac4_program_mac_ref(mut io: *mut k3_mmio, mut mac: *const u8_0) {
    let mut low: u32_0 = (*mac.offset(0 as ::core::ffi::c_int as isize) as ::core::ffi::c_int
        | (*mac.offset(1 as ::core::ffi::c_int as isize) as ::core::ffi::c_int)
            << 8 as ::core::ffi::c_int
        | (*mac.offset(2 as ::core::ffi::c_int as isize) as ::core::ffi::c_int)
            << 16 as ::core::ffi::c_int
        | (*mac.offset(3 as ::core::ffi::c_int as isize) as ::core::ffi::c_int)
            << 24 as ::core::ffi::c_int) as u32_0;
    let mut high: u32_0 = (*mac.offset(4 as ::core::ffi::c_int as isize) as ::core::ffi::c_int
        | (*mac.offset(5 as ::core::ffi::c_int as isize) as ::core::ffi::c_int)
            << 8 as ::core::ffi::c_int) as u32_0
        | GMAC_HI_REG_AE;
    writel(low, io, GMAC_ADDR_LOW0 as u32_0);
    writel(high, io, GMAC_ADDR_HIGH0 as u32_0);
}
#[no_mangle]
pub unsafe extern "C" fn dwmac4_enable_mac_ref(mut io: *mut k3_mmio) {
    update_bits(
        io,
        GMAC_PACKET_FILTER as u32_0,
        0 as u32_0,
        GMAC_PACKET_FILTER_PM | GMAC_PACKET_FILTER_PR,
    );
    update_bits(
        io,
        GMAC_CONFIG as u32_0,
        GMAC_CONFIG_FES | GMAC_CONFIG_PS,
        GMAC_CONFIG_DM | GMAC_CONFIG_IPC | GMAC_CONFIG_TE | GMAC_CONFIG_RE,
    );
}
