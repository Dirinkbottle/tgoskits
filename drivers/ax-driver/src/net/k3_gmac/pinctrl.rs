//! Minimal SpacemiT K3 pinctrl programming for COM260 GMAC1.
//!
//! The board DT selects `pinctrl-0 = <&gmac1_cfg>` for `eth1`, but this tree
//! does not currently have a generic SpacemiT pinctrl driver. Until that exists,
//! the GMAC glue applies only the Linux COM260 `gmac1_cfg` state needed for
//! MDIO/RGMII bring-up.

use alloc::{format, vec::Vec};
use core::ptr::NonNull;

use fdt_edit::{Node, Phandle};
use log::{info, warn};
use rdrive::{probe::OnProbeError, register::FdtInfo};

use crate::mmio::iomap;

const K3_GMAC1_CTRL_OFFSET: u32 = 0x3ec;

const PAD_MUX: u32 = 0x7;
const PAD_DRIVE_K3: u32 = 0xf << 9;

const APBC_AIB_CLK_RST: u32 = 0x3c;
const APBC_AIB_CLK_EN: u32 = 1 << 1;
const APBC_AIB_BUS_CLK_EN: u32 = 1 << 0;

const APBC_ASFAR: u32 = 0x00;
const APBC_ASSAR: u32 = 0x04;
const APBC_ASFAR_AKEY: u32 = 0xbaba;
const APBC_ASSAR_AKEY: u32 = 0xeb10;

const IO_PWR_DOMAIN_V18EN: u32 = 1 << 2;

const GMAC1_PINS: &[K3PinGroup] = &[
    K3PinGroup {
        name: "gmac1-0-pins/base",
        drive_ma: 9,
        power_mv: 1800,
        pins: &[
            K3PinMux::new(21, 1, "rx_ctl"),
            K3PinMux::new(22, 1, "rx_d0"),
            K3PinMux::new(23, 1, "rx_d1"),
            K3PinMux::new(24, 1, "rx_clk"),
            K3PinMux::new(27, 1, "tx_d0"),
            K3PinMux::new(28, 1, "tx_d1"),
            K3PinMux::new(32, 1, "tx_ctl"),
            K3PinMux::new(33, 1, "mdc"),
            K3PinMux::new(34, 1, "mdio"),
        ],
    },
    K3PinGroup {
        name: "gmac1-1-pins/rgmii-extra",
        drive_ma: 9,
        power_mv: 1800,
        pins: &[
            K3PinMux::new(25, 1, "rx_d2"),
            K3PinMux::new(26, 1, "rx_d3"),
            K3PinMux::new(29, 1, "tx_clk"),
            K3PinMux::new(30, 1, "tx_d2"),
            K3PinMux::new(31, 1, "tx_d3"),
        ],
    },
    K3PinGroup {
        name: "gmac1-3-pins/phy-int",
        drive_ma: 9,
        power_mv: 1800,
        pins: &[K3PinMux::new(35, 1, "phy_int")],
    },
    K3PinGroup {
        name: "gmac1-6-pins/com260-gpio37",
        drive_ma: 25,
        power_mv: 1800,
        pins: &[K3PinMux::new(37, 0, "com260_gpio37")],
    },
];

#[derive(Clone, Copy)]
struct K3PinMux {
    pin: u16,
    mux: u16,
    signal: &'static str,
}

impl K3PinMux {
    const fn new(pin: u16, mux: u16, signal: &'static str) -> Self {
        Self { pin, mux, signal }
    }
}

#[derive(Clone, Copy)]
struct K3PinGroup {
    name: &'static str,
    drive_ma: u32,
    power_mv: u32,
    pins: &'static [K3PinMux],
}

/// Mapped K3 pinctrl and IO power-domain windows.
pub(super) struct K3GmacPinctrl {
    pinctrl_base: NonNull<u8>,
    pinctrl_phys: u64,
    io_pd_base: NonNull<u8>,
    io_pd_phys: u64,
    apbc_base: NonNull<u8>,
    apbc_phys: u64,
    apbc_unlock_offset: u32,
}

impl K3GmacPinctrl {
    /// Parses the GMAC node's pinctrl reference and maps the K3 pinctrl MMIO
    /// resources. Only the COM260 GMAC1 state is currently implemented.
    pub(super) fn parse(
        info: &FdtInfo<'_>,
        node: &Node,
        ctrl_offset: u32,
    ) -> Result<Option<Self>, OnProbeError> {
        let phandles = prop_u32_list(node, "pinctrl-0")
            .into_iter()
            .map(Phandle::from)
            .collect::<Vec<_>>();
        info!(
            "k3-gmac pinctrl: parse ctrl={:#x} pinctrl-0={:?}",
            ctrl_offset, phandles
        );

        if ctrl_offset != K3_GMAC1_CTRL_OFFSET {
            warn!(
                "k3-gmac pinctrl: ctrl offset {:#x} is not COM260 GMAC1; leaving pinctrl to \
                 firmware",
                ctrl_offset
            );
            return Ok(None);
        }

        let phandle = *phandles.first().ok_or_else(|| {
            OnProbeError::other("k3-gmac pinctrl: COM260 GMAC1 missing pinctrl-0")
        })?;
        let state = info.get_by_phandle(phandle).ok_or_else(|| {
            OnProbeError::other(format!(
                "k3-gmac pinctrl: pinctrl-0 phandle {phandle:?} not found"
            ))
        })?;
        let controller = state.parent().ok_or_else(|| {
            OnProbeError::other(format!(
                "k3-gmac pinctrl: state {} has no parent controller",
                state.path()
            ))
        })?;

        if !has_compatible(controller.as_node(), "spacemit,k3-pinctrl") {
            return Err(OnProbeError::other(format!(
                "k3-gmac pinctrl: state {} parent {} is not spacemit,k3-pinctrl",
                state.path(),
                controller.path()
            )));
        }

        let regs = controller.regs();
        let pinctrl_reg = regs.first().copied().ok_or_else(|| {
            OnProbeError::other(format!(
                "k3-gmac pinctrl: controller {} missing pinctrl reg",
                controller.path()
            ))
        })?;
        let io_pd_reg = regs.get(1).copied().ok_or_else(|| {
            OnProbeError::other(format!(
                "k3-gmac pinctrl: controller {} missing IO power-domain reg",
                controller.path()
            ))
        })?;
        let pinctrl_base = iomap(
            pinctrl_reg.address as usize,
            pinctrl_reg.size.unwrap_or(0x400).max(0x400) as usize,
        )?;
        let io_pd_base = iomap(
            io_pd_reg.address as usize,
            io_pd_reg.size.unwrap_or(0x34).max(0x34) as usize,
        )?;

        let (apbc_base, apbc_phys, apbc_unlock_offset) = map_apbc(info, controller.as_node())?;

        info!(
            "k3-gmac pinctrl: state={} controller={} pinctrl={:#x}+{:#x} io-pd={:#x}+{:#x} \
             apbc={:#x} unlock_offset={:#x}",
            state.path(),
            controller.path(),
            pinctrl_reg.address,
            pinctrl_reg.size.unwrap_or(0x400),
            io_pd_reg.address,
            io_pd_reg.size.unwrap_or(0x34),
            apbc_phys,
            apbc_unlock_offset
        );

        Ok(Some(Self {
            pinctrl_base,
            pinctrl_phys: pinctrl_reg.address,
            io_pd_base,
            io_pd_phys: io_pd_reg.address,
            apbc_base,
            apbc_phys,
            apbc_unlock_offset,
        }))
    }

    /// Applies the COM260 `gmac1_cfg` pin state.
    pub(super) fn apply(&self) {
        info!("k3-gmac pinctrl: enable AIB/APBC clocks before pin writes");
        self.enable_aib_clocks();

        for group in GMAC1_PINS {
            info!(
                "k3-gmac pinctrl: apply group={} pins={} bias=disable drive={}mA power={}mV",
                group.name,
                group.pins.len(),
                group.drive_ma,
                group.power_mv
            );
            for pin in group.pins {
                self.set_io_power_1v8(pin.pin);
                self.apply_pin(*pin, group.drive_ma);
            }
        }

        info!("k3-gmac pinctrl: COM260 GMAC1 pin state complete");
    }

    fn enable_aib_clocks(&self) {
        let offset = APBC_AIB_CLK_RST as usize;
        let before = self.read_apbc(offset);
        let next = before | APBC_AIB_CLK_EN | APBC_AIB_BUS_CLK_EN;
        self.write_apbc(offset, next);
        let after = self.read_apbc(offset);
        info!(
            "k3-gmac pinctrl: APBC AIB clock gate phys={:#x} offset={:#x} before={:#010x} \
             wrote={:#010x} after={:#010x}",
            self.apbc_phys, APBC_AIB_CLK_RST, before, next, after
        );
    }

    fn set_io_power_1v8(&self, pin: u16) {
        let offset = io_pd_offset(pin);
        if offset == 0 {
            warn!(
                "k3-gmac pinctrl: pin={} has no K3 IO power-domain offset; skip 1.8V write",
                pin
            );
            return;
        }

        self.write_apbc(
            (self.apbc_unlock_offset + APBC_ASFAR) as usize,
            APBC_ASFAR_AKEY,
        );
        self.write_apbc(
            (self.apbc_unlock_offset + APBC_ASSAR) as usize,
            APBC_ASSAR_AKEY,
        );
        self.write_io_pd(offset as usize, IO_PWR_DOMAIN_V18EN);
        self.write_apbc(
            (self.apbc_unlock_offset + APBC_ASFAR) as usize,
            APBC_ASFAR_AKEY,
        );
        self.write_apbc(
            (self.apbc_unlock_offset + APBC_ASSAR) as usize,
            APBC_ASSAR_AKEY,
        );
        let after = self.read_io_pd(offset as usize);
        info!(
            "k3-gmac pinctrl: IO power pin={} phys={:#x} offset={:#x} set=1v8 after={:#010x}",
            pin, self.io_pd_phys, offset, after
        );
    }

    fn apply_pin(&self, pin: K3PinMux, drive_ma: u32) {
        let offset = pin_reg_offset(pin.pin) as usize;
        let before = self.read_pin(offset);
        let config = bias_disable_config() | drive_strength_config_1v8(drive_ma);
        let with_config = (before & PAD_MUX) | config;
        self.write_pin(offset, with_config);
        let after_config = self.read_pin(offset);
        let with_mux = (after_config & !PAD_MUX) | u32::from(pin.mux);
        self.write_pin(offset, with_mux);
        let after_mux = self.read_pin(offset);
        info!(
            "k3-gmac pinctrl: pin={} signal={} mux={} reg_phys={:#x} offset={:#x} before={:#010x} \
             config={:#010x} after_config={:#010x} after_mux={:#010x}",
            pin.pin,
            pin.signal,
            pin.mux,
            self.pinctrl_phys,
            offset,
            before,
            with_config,
            after_config,
            after_mux
        );
    }

    fn read_pin(&self, offset: usize) -> u32 {
        read32(self.pinctrl_base, offset)
    }

    fn write_pin(&self, offset: usize, value: u32) {
        write32(self.pinctrl_base, offset, value);
    }

    fn read_io_pd(&self, offset: usize) -> u32 {
        read32(self.io_pd_base, offset)
    }

    fn write_io_pd(&self, offset: usize, value: u32) {
        write32(self.io_pd_base, offset, value);
    }

    fn read_apbc(&self, offset: usize) -> u32 {
        read32(self.apbc_base, offset)
    }

    fn write_apbc(&self, offset: usize, value: u32) {
        write32(self.apbc_base, offset, value);
    }
}

fn map_apbc(
    info: &FdtInfo<'_>,
    pinctrl_node: &Node,
) -> Result<(NonNull<u8>, u64, u32), OnProbeError> {
    let cells = prop_u32_list(pinctrl_node, "spacemit,apbc");
    if cells.len() < 2 {
        return Err(OnProbeError::other(format!(
            "k3-gmac pinctrl: malformed spacemit,apbc={cells:#x?}"
        )));
    }

    let apbc = info
        .get_by_phandle(Phandle::from(cells[0]))
        .ok_or_else(|| {
            OnProbeError::other(format!(
                "k3-gmac pinctrl: APBC phandle {:?} not found",
                Phandle::from(cells[0])
            ))
        })?;
    let reg = apbc.regs().into_iter().next().ok_or_else(|| {
        OnProbeError::other(format!(
            "k3-gmac pinctrl: APBC node {} missing reg",
            apbc.path()
        ))
    })?;
    let size = reg.size.unwrap_or(0x1000).max(0x1000) as usize;
    let base = iomap(reg.address as usize, size)?;
    Ok((base, reg.address, cells[1]))
}

fn has_compatible(node: &Node, compatible: &str) -> bool {
    node.compatibles().any(|value| value == compatible)
}

fn bias_disable_config() -> u32 {
    0
}

fn drive_strength_config_1v8(ma: u32) -> u32 {
    let value = match ma {
        0..=2 => 0,
        3..=4 => 1,
        5..=6 => 2,
        7 => 3,
        8..=9 => 4,
        10..=11 => 5,
        12..=13 => 6,
        14 => 7,
        15..=21 => 8,
        22..=23 => 9,
        24..=25 => 10,
        26 => 11,
        27..=28 => 12,
        29..=30 => 13,
        31 => 14,
        _ => 15,
    };
    (value << PAD_DRIVE_K3.trailing_zeros()) & PAD_DRIVE_K3
}

fn pin_reg_offset(pin: u16) -> u32 {
    let mut pin = u32::from(pin);
    if pin > 130 {
        pin += 2;
    }
    pin << 2
}

fn io_pd_offset(pin: u16) -> u32 {
    match pin {
        0..=20 => 0x04,
        21..=41 => 0x0c,
        76..=98 => 0x20,
        99..=127 => 0x10,
        132..=137 => 0x1c,
        138..=144 => 0x2c,
        _ => 0,
    }
}

fn read32(base: NonNull<u8>, offset: usize) -> u32 {
    // SAFETY: callers provide offsets copied from Linux K3 pinctrl/syscon
    // register definitions into live MMIO mappings returned by `iomap`.
    unsafe { base.as_ptr().add(offset).cast::<u32>().read_volatile() }
}

fn write32(base: NonNull<u8>, offset: usize, value: u32) {
    // SAFETY: callers provide offsets copied from Linux K3 pinctrl/syscon
    // register definitions into live MMIO mappings returned by `iomap`.
    unsafe {
        base.as_ptr()
            .add(offset)
            .cast::<u32>()
            .write_volatile(value)
    };
}

fn prop_u32_list(node: &Node, name: &str) -> Vec<u32> {
    node.get_property(name)
        .map(|prop| prop.get_u32_iter().collect())
        .unwrap_or_default()
}
