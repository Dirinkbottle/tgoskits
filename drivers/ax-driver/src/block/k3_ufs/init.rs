//! MPHY / UNIPRO / link-startup / HS power-mode initialization sequence.
//!
//! These steps bring the UFS physical and link layers up and then upgrade
//! the link from the default PWM gear to the negotiated HS gear, following
//! the SpacemiT K3 sequence from Linux `drivers/ufs/host/ufs-spacemit.c`
//! (upstream series v2, 2026-07-25) and the generic UFSHCI flow from
//! `drivers/ufs/core/ufshcd.c`. The UIC command channel these steps use is
//! implemented in [`super::uic`].

use core::time::Duration;

use log::{info, warn};

use super::{
    K3UfsHost,
    error::UfsError,
    regs::{
        ANA_EQ_CTRL_REG_ATTR, ANA_HSGEAR_CTRL_ATTR, DEVICE_PRESENT, DL_AFC0REQTIMEOUTVAL,
        DME_LOCAL_AFC0_REQ_TIMEOUT, DME_LOCAL_FC0_PROTECTION_TIMEOUT, DME_LOCAL_TC0_REPLAY_TIMEOUT,
        MPHY_DEVICE_RESET_DEASSERT, MPHY_PLL_LOCK_BIT, MPHY_PU_ALL, MPHY_PU_WITH_HB8_RESET,
        PA_ACTIVERXDATALANES, PA_ACTIVETXDATALANES, PA_AVAILRXDATALANES, PA_AVAILTXDATALANES,
        PA_CONNECTEDRXDATALANES, PA_CONNECTEDTXDATALANES, PA_GRANULARITY, PA_HS_MODE_A,
        PA_HS_MODE_B, PA_HSSERIES, PA_LOCAL_TX_LCC_ENABLE, PA_MAXRXHSGEAR, PA_MAXRXPWMGEAR,
        PA_MK2EXTENSIONGUARDBAND, PA_PEER_TX_LCC_ENABLE, PA_PEERSCRAMBLING, PA_PWR_FAST_MODE,
        PA_PWR_SLOW_MODE, PA_PWRMODE, PA_PWRMODEUSERDATA0, PA_PWRMODEUSERDATA1,
        PA_PWRMODEUSERDATA2, PA_PWRMODEUSERDATA3, PA_PWRMODEUSERDATA4, PA_PWRMODEUSERDATA5,
        PA_RXGEAR, PA_RXTERMINATION, PA_SCRAMBLING, PA_STALLNOCONFIGTIME, PA_TACTIVATE, PA_TXGEAR,
        PA_TXHSG1PREPARELENGTH, PA_TXHSG1SYNCLENGTH, PA_TXHSG2PREPARELENGTH, PA_TXHSG2SYNCLENGTH,
        PA_TXHSG3PREPARELENGTH, PA_TXHSG3SYNCLENGTH, PA_TXMK2EXTENSION, PA_TXSKIP, PA_TXSKIPPERIOD,
        PA_TXTERMINATION, PA_TXTRAILINGCLOCKS, REG_CONTROLLER_CAPABILITIES, REG_CONTROLLER_ENABLE,
        REG_CONTROLLER_STATUS, REG_UFS_VERSION, REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER,
        RX_GARBAGE_COUNT_OFFSET, RX_HIBERN8TIME_CAP, RX_LANE_HB8_BKDOOR_ATTR, RX_LS_PRE_LEN_CAP,
        RX_MIN_STALL_CAP, RX_PWRM_CLOSURE_LEN_CAP, TX_HIBERN8TIME_CAP, TX_LCC_ENABLE,
        TX_MIN_ACTIVATETIME, UFS_ATOP_BASE, UFS_DEVICE_IO_CTRL, UFS_DL_AFC0REQTIMEOUTVAL_MAX,
        UFS_HS_G1, UFS_HS_G3, UFS_MPHY_BKDR_CTRL, UFS_MPHY_PU_CTRL, UFS_PA_LINK_STARTUP_TIMER,
        UFS_PHY_MNG_BASE, UFS_SPACEMIT_GEAR3_ATTR, UFS_SYS1CLK_1US, UFS_TX_SYMBO_CLK,
        UFS_TX_SYMBOL_CLK_NS_US, UIC_CMD_DME_LINK_STARTUP, rx_lane_sel, uic_arg_mib,
        uic_arg_mib_sel,
    },
};

/// UniPro DL timeout values programmed before a power-mode change (Linux:
/// `DL_*_Default` in include/ufs/unipro.h).
const DL_FC0_PROTECTION_TIMEOUT_DEFAULT: u32 = 8191;
const DL_TC0_REPLAY_TIMEOUT_DEFAULT: u32 = 65535;
const DL_AFC0_REQ_TIMEOUT_DEFAULT: u32 = 32767;

/// A negotiated (or target) UniPro power mode, mirroring the PA layer
/// attributes of Linux `struct ufs_pa_layer_attr`.
struct PwrModeParams {
    gear_rx: u32,
    gear_tx: u32,
    lane_rx: u32,
    lane_tx: u32,
    pwr_rx: u32,
    pwr_tx: u32,
    hs_rate: u32,
}

/// Link direction used to pick the DME access side for max-gear reads: the
/// host RX max HS gear is read locally, the host TX max HS gear from the
/// peer (device-side) `PA_MAXRXHSGEAR` (Linux: ufshcd_get_max_pwr_mode).
#[derive(Clone, Copy)]
enum DmeSide {
    Rx,
    Tx,
}

impl DmeSide {
    fn name(self) -> &'static str {
        match self {
            DmeSide::Rx => "RX",
            DmeSide::Tx => "TX",
        }
    }
}

impl K3UfsHost {
    /// Power up the MPHY and wait for PLL lock (Linux: ufs_spacemit_mphy_init).
    pub(super) fn mphy_init(&self) -> Result<(), UfsError> {
        info!("[k3-ufs] Initializing MPHY...");

        // SAFETY: every offset below is a constant inside the mapped MMIO
        // window (`UFS_PHY_MNG_BASE`/`UFS_ATOP_BASE` + small displacement),
        // and access is exclusive to this host instance; see `read32`/`write32`.
        unsafe {
            // Reset all MPHY logical blocks.
            self.write32(UFS_PHY_MNG_BASE, 0x003);

            // Power up all, then assert ana_rx_hb8_reset.
            self.write32(UFS_PHY_MNG_BASE + UFS_MPHY_PU_CTRL, MPHY_PU_ALL);
            self.write32(UFS_PHY_MNG_BASE + UFS_MPHY_PU_CTRL, MPHY_PU_WITH_HB8_RESET);
            axklib::time::busy_wait(Duration::from_micros(500));

            // Deassert ana_rx_hb8_reset and the UFS device reset, enabling the
            // reference clock output.
            self.write32(UFS_PHY_MNG_BASE + UFS_MPHY_PU_CTRL, MPHY_PU_ALL);
            self.write32(
                UFS_PHY_MNG_BASE + UFS_DEVICE_IO_CTRL,
                MPHY_DEVICE_RESET_DEASSERT,
            );
            axklib::time::busy_wait(Duration::from_millis(1));

            self.wait_mphy_pll_lock()?;

            // Configure the ATOP analog register 0xC2 via the backdoor.
            self.write32(UFS_PHY_MNG_BASE + UFS_MPHY_BKDR_CTRL, 0x1);
            self.write32(UFS_ATOP_BASE + (0xC2 << 2), 0x40);
            self.write32(UFS_PHY_MNG_BASE + UFS_MPHY_BKDR_CTRL, 0x0);
            axklib::time::busy_wait(Duration::from_millis(2));
        }

        info!("[k3-ufs] MPHY init completed");
        Ok(())
    }

    /// Wait for the MPHY PLL to lock (Linux: ufs_spacemit_wait_mphy_pll_lock).
    ///
    /// Used before link startup, after applying device quirks, and after each
    /// power-mode change (the PLL must re-lock at the new gear rate).
    fn wait_mphy_pll_lock(&self) -> Result<(), UfsError> {
        // SAFETY: `UFS_PHY_MNG_BASE + UFS_MPHY_PU_CTRL` is a constant inside
        // the mapped MMIO window, and access is exclusive to this host.
        unsafe {
            for _ in 0..10000 {
                let pu_ctrl = self.read32(UFS_PHY_MNG_BASE + UFS_MPHY_PU_CTRL);
                if pu_ctrl & MPHY_PLL_LOCK_BIT != 0 {
                    return Ok(());
                }
                axklib::time::busy_wait(Duration::from_micros(10));
            }
        }

        Err(UfsError::Init("MPHY PLL lock timeout"))
    }

    /// Enable the host controller and log the controller capabilities.
    pub(super) fn host_init(&self) -> Result<(), UfsError> {
        info!("[k3-ufs] Initializing host controller...");

        // SAFETY: all offsets are UFSHCI register constants inside the mapped
        // MMIO window, and access is exclusive to this host instance; see
        // `read32`/`write32`.
        unsafe {
            let cap = self.read32(REG_CONTROLLER_CAPABILITIES);
            let version = self.read32(REG_UFS_VERSION);
            info!("[k3-ufs] CAP: 0x{:08x}, VER: 0x{:08x}", cap, version);

            self.write32(REG_CONTROLLER_ENABLE, 0);
            axklib::time::busy_wait(Duration::from_millis(1));

            self.write32(REG_CONTROLLER_ENABLE, 1);
            axklib::time::busy_wait(Duration::from_millis(1));

            let hce = self.read32(REG_CONTROLLER_ENABLE);
            if hce & 1 == 0 {
                return Err(UfsError::Init("Controller enable failed"));
            }
        }

        info!("[k3-ufs] Host controller enabled");
        Ok(())
    }

    /// Program the K3 timer registers before link startup.
    ///
    /// Linux: `ufs_spacemit_link_startup_pre_change()`. The link-startup
    /// timer clears its low 4 bits to select the b0 design, and the sysclk /
    /// TX symbol clock registers are derived from the UFS ACLK
    /// (491.52 MHz, per the `freq-table-hz` in the board dts).
    pub(super) fn link_startup_pre(&self) -> Result<(), UfsError> {
        // SAFETY: all offsets are K3 vendor register constants inside the
        // mapped MMIO window, and access is exclusive to this host instance.
        unsafe {
            self.write32(UFS_PA_LINK_STARTUP_TIMER, 0xFFFF_FFF0);
            self.write32(UFS_SYS1CLK_1US, self.clock_freq / 1_000_000);
            self.write32(UFS_TX_SYMBOL_CLK_NS_US, UFS_TX_SYMBO_CLK);

            info!(
                "[k3-ufs] SYS1CLK_1US=0x{:x}, TX_SYMBOL_CLK=0x{:x}",
                self.read32(UFS_SYS1CLK_1US),
                self.read32(UFS_TX_SYMBOL_CLK_NS_US)
            );
        }

        Ok(())
    }

    /// UNIPRO v1.6 initialization - critical for link startup.
    pub(super) fn unipro_init(&self) -> Result<(), UfsError> {
        info!("[k3-ufs] Initializing UNIPRO v1.6...");

        // PA layer attributes (Linux: ufs_spacemit_uniprov1p6_init()).
        self.dme_set(uic_arg_mib(PA_TXHSG1SYNCLENGTH), 0x4f)?;
        self.dme_set(uic_arg_mib(PA_TXHSG1PREPARELENGTH), 0x0f)?;
        self.dme_set(uic_arg_mib(PA_TXHSG2SYNCLENGTH), 0x4f)?;
        self.dme_set(uic_arg_mib(PA_TXHSG2PREPARELENGTH), 0x0f)?;
        self.dme_set(uic_arg_mib(PA_TXHSG3SYNCLENGTH), 0x4f)?;
        self.dme_set(uic_arg_mib(PA_TXHSG3PREPARELENGTH), 0x0f)?;
        self.dme_set(uic_arg_mib(PA_TXMK2EXTENSION), 0x0)?;
        self.dme_set(uic_arg_mib(PA_PEERSCRAMBLING), 0x1)?;
        self.dme_set(uic_arg_mib(PA_TXSKIP), 0x1)?;
        self.dme_set(uic_arg_mib(PA_TXSKIPPERIOD), 250)?;
        self.dme_set(uic_arg_mib(PA_LOCAL_TX_LCC_ENABLE), 0x0)?;
        self.dme_set(uic_arg_mib(PA_PEER_TX_LCC_ENABLE), 0x0)?;
        self.dme_set(uic_arg_mib(PA_SCRAMBLING), 0x1)?;
        self.dme_set(uic_arg_mib(PA_GRANULARITY), 0x1)?;
        self.dme_set(uic_arg_mib(PA_MK2EXTENSIONGUARDBAND), 0x0)?;
        self.dme_set(uic_arg_mib(PA_STALLNOCONFIGTIME), 15)?;
        self.dme_set(uic_arg_mib(PA_TACTIVATE), 0x64)?;
        self.dme_set(uic_arg_mib(PA_TXTRAILINGCLOCKS), 0x64)?;

        // RX lane 0 & 1 attributes (RX lane selector = 4 + lane).
        self.dme_set(uic_arg_mib_sel(RX_LS_PRE_LEN_CAP, rx_lane_sel(0)), 0x0b)?;
        self.dme_set(uic_arg_mib_sel(RX_LS_PRE_LEN_CAP, rx_lane_sel(1)), 0x0b)?;
        self.dme_set(
            uic_arg_mib_sel(RX_LANE_HB8_BKDOOR_ATTR, rx_lane_sel(0)),
            0x9f,
        )?;
        self.dme_set(
            uic_arg_mib_sel(RX_LANE_HB8_BKDOOR_ATTR, rx_lane_sel(1)),
            0x9f,
        )?;
        self.dme_set(uic_arg_mib_sel(RX_PWRM_CLOSURE_LEN_CAP, rx_lane_sel(0)), 15)?;
        self.dme_set(uic_arg_mib_sel(RX_PWRM_CLOSURE_LEN_CAP, rx_lane_sel(1)), 15)?;
        self.dme_set(uic_arg_mib_sel(RX_MIN_STALL_CAP, rx_lane_sel(0)), 0xff)?;
        self.dme_set(uic_arg_mib_sel(RX_MIN_STALL_CAP, rx_lane_sel(1)), 0xff)?;

        // TX lane 0 & 1 hibernate time (TX lane selector = lane).
        self.dme_set(uic_arg_mib_sel(TX_HIBERN8TIME_CAP, 0), 0x64)?;
        self.dme_set(uic_arg_mib_sel(TX_HIBERN8TIME_CAP, 1), 0x64)?;

        // RX lane 0 & 1 hibernate time.
        self.dme_set(uic_arg_mib_sel(RX_HIBERN8TIME_CAP, rx_lane_sel(0)), 0x64)?;
        self.dme_set(uic_arg_mib_sel(RX_HIBERN8TIME_CAP, rx_lane_sel(1)), 0x64)?;

        // TX EQ on TX lane 0 and RX garbage count on both RX lanes.
        self.dme_set(uic_arg_mib_sel(ANA_EQ_CTRL_REG_ATTR, 0), 0x5)?;
        self.dme_set(
            uic_arg_mib_sel(RX_GARBAGE_COUNT_OFFSET, rx_lane_sel(0)),
            0x9f,
        )?;
        self.dme_set(
            uic_arg_mib_sel(RX_GARBAGE_COUNT_OFFSET, rx_lane_sel(1)),
            0x9f,
        )?;

        info!("[k3-ufs] UNIPRO v1.6 init completed");
        Ok(())
    }

    /// Run the UFS link startup and wait for the device to appear.
    pub(super) fn link_startup(&self) -> Result<(), UfsError> {
        info!("[k3-ufs] Starting UFS link...");

        // LINK_STARTUP success is judged by DEVICE_PRESENT below; the UIC
        // command only needs to complete (ARG2 result checked in `uic_cmd`).
        self.uic_cmd(UIC_CMD_DME_LINK_STARTUP, 0, 0, 0)?;

        // SAFETY: `REG_CONTROLLER_STATUS` is a UFSHCI register inside the
        // mapped MMIO window, and access is exclusive to this host instance.
        unsafe {
            for _ in 0..1000 {
                let status = self.read32(REG_CONTROLLER_STATUS);
                if status & DEVICE_PRESENT != 0 {
                    info!(
                        "[k3-ufs] Link active, device present. Status=0x{:08x}",
                        status
                    );
                    return Ok(());
                }
                axklib::time::busy_wait(Duration::from_millis(1));
            }
        }

        Err(UfsError::Init("Device not present after link startup"))
    }

    /// Link startup post processing (Linux: ufs_spacemit_link_startup_post_change).
    pub(super) fn link_startup_post(&self) -> Result<(), UfsError> {
        // The 0xe8 attribute makes a UFS2.1 device run at GEAR3 + 2 lanes
        // (Linux: "add 0xe8 make UFS2.1 run GEAR3 + 2Lane@409M"). The vendor
        // driver ignores DME errors here, so failures are only logged.
        for value in [0x97, 0xd7, 0x17] {
            if let Err(e) = self.dme_set(uic_arg_mib_sel(UFS_SPACEMIT_GEAR3_ATTR, 0), value) {
                warn!("[k3-ufs] GEAR3 attr 0xe8 write failed: {}", e);
            }
        }

        self.dme_set(
            uic_arg_mib(DL_AFC0REQTIMEOUTVAL),
            UFS_DL_AFC0REQTIMEOUTVAL_MAX,
        )?;

        // Read back the negotiated TX lane count for diagnostics
        // (Linux: ufs_spacemit_get_connected_tx_lanes).
        match self.dme_get(uic_arg_mib(PA_CONNECTEDTXDATALANES)) {
            Ok(lanes) => info!("[k3-ufs] Connected TX data lanes: {}", lanes),
            Err(e) => warn!("[k3-ufs] Read connected TX lanes failed: {}", e),
        }

        // The LINERESET during LINK_STARTUP latches sticky UECPA error bits
        // in the UIC error code register; reading the register consumes
        // (clears) them so a later ISR does not report a spurious
        // PHY-adapter-layer error (Linux: ufshcd.c after POST_CHANGE).
        // SAFETY: `REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER` is a UFSHCI register
        // inside the mapped MMIO window, and access is exclusive to this host.
        unsafe {
            let _ = self.read32(REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER);
        }

        info!("[k3-ufs] Link startup post processing completed");
        Ok(())
    }

    /// Apply device quirks before the HS power-mode change.
    ///
    /// Linux: `ufs_spacemit_apply_dev_quirks()`. The backdoor
    /// `ANA_HSGEAR_CTRL_ATTR` write pre-sets TX rate/gear so the M-PHY PLL
    /// locks at the target HS rate before the PA power-mode switch.
    pub(super) fn apply_dev_quirks(&self) -> Result<(), UfsError> {
        info!("[k3-ufs] Applying device quirks...");

        // Disable TX LCC and TX min-activate-time on both lanes.
        self.dme_set(uic_arg_mib_sel(TX_LCC_ENABLE, 0), 0)?;
        self.dme_set(uic_arg_mib_sel(TX_LCC_ENABLE, 1), 0)?;
        self.dme_set(uic_arg_mib_sel(TX_MIN_ACTIVATETIME, 0), 0)?;
        self.dme_set(uic_arg_mib_sel(TX_MIN_ACTIVATETIME, 1), 0)?;

        self.dme_set(uic_arg_mib(ANA_HSGEAR_CTRL_ATTR), 0x25)?;
        self.wait_mphy_pll_lock()?;

        info!("[k3-ufs] Device quirks applied");
        Ok(())
    }

    /// Read the device's advertised power-mode caps.
    ///
    /// Linux: `ufshcd_get_max_pwr_mode()`. The TX gear is read from the peer
    /// (device-side) PA_MAXRXHSGEAR; a zero HS gear falls back to the max PWM
    /// gear in SLOW mode. Returns `None` when neither HS nor PWM gear is
    /// reported, in which case the link stays at its default PWM gear.
    fn read_device_pwr_caps(&self) -> Option<PwrModeParams> {
        // Lane counts are local-side attributes on both directions (Linux:
        // ufshcd_get_max_pwr_mode reads PA_CONNECTEDRXDATALANES and
        // PA_CONNECTEDTXDATALANES locally).
        let lane_rx = self.read_lane_count(PA_CONNECTEDRXDATALANES, PA_AVAILRXDATALANES, "RX");
        let lane_tx = self.read_lane_count(PA_CONNECTEDTXDATALANES, PA_AVAILTXDATALANES, "TX");
        let lanes = lane_rx.min(lane_tx).max(1);

        let (gear_rx, pwr_rx) =
            self.read_side_gear(PA_MAXRXHSGEAR, PA_MAXRXPWMGEAR, DmeSide::Rx)?;
        let (gear_tx, pwr_tx) =
            self.read_side_gear(PA_MAXRXHSGEAR, PA_MAXRXPWMGEAR, DmeSide::Tx)?;

        info!(
            "[k3-ufs] Device pwr caps: gear_rx={}, gear_tx={}, lanes={}, pwr_rx={}, pwr_tx={}",
            gear_rx, gear_tx, lanes, pwr_rx, pwr_tx
        );

        Some(PwrModeParams {
            gear_rx,
            gear_tx,
            lane_rx: lanes,
            lane_tx: lanes,
            pwr_rx,
            pwr_tx,
            hs_rate: PA_HS_MODE_B,
        })
    }

    /// Resolve the lane count for one direction: prefer the connected-lane
    /// attribute, then the available-lane capability, then the lane count
    /// declared in the dts (`lanes-per-direction`), cached in `max_lanes`.
    fn read_lane_count(&self, connected_attr: u32, avail_attr: u32, name: &'static str) -> u32 {
        if let Ok(lanes) = self.dme_get(uic_arg_mib(connected_attr))
            && lanes != 0
        {
            info!("[k3-ufs] {} connected lanes: {}", name, lanes);
            return lanes;
        }
        if let Ok(lanes) = self.dme_get(uic_arg_mib(avail_attr))
            && lanes != 0
        {
            warn!(
                "[k3-ufs] {} connected lanes=0, using available lanes: {}",
                name, lanes
            );
            return lanes;
        }
        warn!(
            "[k3-ufs] {} connected and available lanes=0, assuming {} (dts)",
            name, self.max_lanes
        );
        self.max_lanes
    }

    /// Read the max gear for one side: the HS gear in FAST mode, else the PWM
    /// gear in SLOW mode, else `None`.
    fn read_side_gear(&self, hs_attr: u32, pwm_attr: u32, side: DmeSide) -> Option<(u32, u32)> {
        if let Ok(gear) = self.dme_get_side(uic_arg_mib(hs_attr), side)
            && gear != 0
        {
            return Some((gear, PA_PWR_FAST_MODE));
        }
        if let Ok(gear) = self.dme_get_side(uic_arg_mib(pwm_attr), side)
            && gear != 0
        {
            return Some((gear, PA_PWR_SLOW_MODE));
        }
        warn!("[k3-ufs] {} max HS/PWM gear reads zero", side.name());
        None
    }

    /// DME_GET (or DME_PEER_GET) a MIB attribute for the given direction.
    fn dme_get_side(&self, attr: u32, side: DmeSide) -> Result<u32, UfsError> {
        match side {
            DmeSide::Rx => self.dme_get(attr),
            DmeSide::Tx => self.dme_peer_get(attr),
        }
    }

    /// Perform the PA power-mode change to the given target.
    ///
    /// Linux: `ufshcd_dme_change_power_mode()` followed by the vendor
    /// `pwr_change_notify` POST_CHANGE (wait for PLL lock and restore the
    /// analog HS gear control). Writing [`PA_PWRMODE`] triggers the actual
    /// in-band power-mode change.
    fn config_pwr_mode(&self, params: &PwrModeParams) -> Result<(), UfsError> {
        // Linux (ufshcd_dme_change_power_mode) ignores failures of these
        // attribute writes: the device may reject them until the PA_PWRMODE
        // change below actually starts, so only that command is fatal.
        for (attr, val) in [
            (PA_RXGEAR, params.gear_rx),
            (PA_ACTIVERXDATALANES, params.lane_rx),
            (PA_RXTERMINATION, 1),
            (PA_TXGEAR, params.gear_tx),
            (PA_ACTIVETXDATALANES, params.lane_tx),
            (PA_TXTERMINATION, 1),
            (PA_HSSERIES, params.hs_rate),
        ] {
            if let Err(e) = self.dme_set(uic_arg_mib(attr), val) {
                warn!(
                    "[k3-ufs] PWR pre-set DME_SET(0x{:04x})={} failed: {}",
                    attr, val, e
                );
            }
        }

        // UniPro DL timeouts for the new power mode (Linux defaults).
        for (attr, val) in [
            (PA_PWRMODEUSERDATA0, DL_FC0_PROTECTION_TIMEOUT_DEFAULT),
            (PA_PWRMODEUSERDATA1, DL_TC0_REPLAY_TIMEOUT_DEFAULT),
            (PA_PWRMODEUSERDATA2, DL_AFC0_REQ_TIMEOUT_DEFAULT),
            (PA_PWRMODEUSERDATA3, DL_FC0_PROTECTION_TIMEOUT_DEFAULT),
            (PA_PWRMODEUSERDATA4, DL_TC0_REPLAY_TIMEOUT_DEFAULT),
            (PA_PWRMODEUSERDATA5, DL_AFC0_REQ_TIMEOUT_DEFAULT),
            (
                DME_LOCAL_FC0_PROTECTION_TIMEOUT,
                DL_FC0_PROTECTION_TIMEOUT_DEFAULT,
            ),
            (DME_LOCAL_TC0_REPLAY_TIMEOUT, DL_TC0_REPLAY_TIMEOUT_DEFAULT),
            (DME_LOCAL_AFC0_REQ_TIMEOUT, DL_AFC0_REQ_TIMEOUT_DEFAULT),
        ] {
            let _ = self.dme_set(uic_arg_mib(attr), val);
        }

        // Trigger the power-mode change; the mode packs RX in the high nibble.
        let mode = (params.pwr_rx << 4) | params.pwr_tx;
        self.dme_set(uic_arg_mib(PA_PWRMODE), mode)?;

        // POST_CHANGE: the PLL must re-lock at the new gear rate and the link
        // must stay up (Linux: ufs_spacemit_pwr_change_notify).
        self.wait_mphy_pll_lock()?;
        // SAFETY: `REG_CONTROLLER_STATUS` is a UFSHCI register inside the
        // mapped MMIO window, and access is exclusive to this host instance.
        let hcs = unsafe { self.read32(REG_CONTROLLER_STATUS) };
        info!(
            "[k3-ufs] Post-change HCS=0x{:08x}, UPMCRS={}",
            hcs,
            (hcs >> 8) & 0x7
        );
        if hcs & DEVICE_PRESENT == 0 {
            return Err(UfsError::Init("Device lost after power mode change"));
        }

        // Restore the analog HS gear control to its default value.
        self.dme_set(uic_arg_mib(ANA_HSGEAR_CTRL_ATTR), 0x00)?;
        Ok(())
    }

    /// Negotiate and apply the fastest supported HS power mode.
    ///
    /// Linux: `ufshcd_post_device_init()` (via `ufshcd_config_pwr_mode`).
    /// Host caps follow `ufs_spacemit_set_dev_cap()`: HS-G3, 2 lanes, Rate B,
    /// FAST mode. Like Linux, a device whose caps cannot be read, or that has
    /// no HS gear, simply stays at the default PWM gear; this never fails the
    /// probe. If the primary mode change fails, lower-rate and lower-gear
    /// candidates are tried before falling back to a working PWM link.
    pub(super) fn upgrade_link_to_hs(&self) -> Result<(), UfsError> {
        let Some(dev) = self.read_device_pwr_caps() else {
            self.restore_ana_hsgear();
            warn!("[k3-ufs] Cannot read device power caps, staying at PWM gear");
            return Ok(());
        };

        if dev.pwr_rx != PA_PWR_FAST_MODE || dev.pwr_tx != PA_PWR_FAST_MODE {
            self.restore_ana_hsgear();
            warn!("[k3-ufs] Device has no HS gear capability, staying at PWM gear");
            return Ok(());
        }

        let gear = dev.gear_rx.min(dev.gear_tx).min(UFS_HS_G3);
        let lanes = dev.lane_rx.min(dev.lane_tx).min(2);
        if gear == 0 || lanes == 0 {
            self.restore_ana_hsgear();
            warn!("[k3-ufs] Negotiated zero gear or lanes, staying at PWM gear");
            return Ok(());
        }

        // Try the negotiated gear at the device series first, then the same
        // gear at Rate A, then one gear down at Rate A (most capable first).
        // After each failed change the link is re-trained at the default PWM
        // gear before trying the next candidate, so a partially-applied HS
        // change cannot wedge the M-PHY.
        let params = |gear, lanes, rate| PwrModeParams {
            gear_rx: gear,
            gear_tx: gear,
            lane_rx: lanes,
            lane_tx: lanes,
            pwr_rx: PA_PWR_FAST_MODE,
            pwr_tx: PA_PWR_FAST_MODE,
            hs_rate: rate,
        };

        let candidates = [
            Some((gear, lanes, dev.hs_rate)),
            (dev.hs_rate != PA_HS_MODE_A).then_some((gear, lanes, PA_HS_MODE_A)),
            (gear > UFS_HS_G1).then_some((gear - 1, lanes, PA_HS_MODE_A)),
        ];

        for (g, l, rate) in candidates.into_iter().flatten() {
            match self.config_pwr_mode(&params(g, l, rate)) {
                Ok(()) => {
                    info!(
                        "[k3-ufs] Link upgraded to HS-G{} ({} lanes, rate {})",
                        g, l, rate
                    );
                    return Ok(());
                }
                Err(e) => {
                    warn!("[k3-ufs] HS-G{} rate {} change failed: {}", g, rate, e);
                    // Re-establish a working PWM link before any next attempt.
                    if let Err(re) = self.restore_pwm_link() {
                        warn!(
                            "[k3-ufs] PWM link restore after failed HS change failed: {}",
                            re
                        );
                        return Ok(());
                    }
                }
            }
        }

        warn!("[k3-ufs] HS negotiation failed, staying at PWM gear");
        Ok(())
    }

    /// Re-establish the link at the default PWM gear after a failed HS change.
    ///
    /// `apply_dev_quirks` leaves `ANA_HSGEAR_CTRL_ATTR` pre-set for HS, which
    /// would break a PWM re-train, so it is restored before link startup.
    fn restore_pwm_link(&self) -> Result<(), UfsError> {
        self.restore_ana_hsgear();
        self.link_startup()?;
        let _ = self.link_startup_post();
        Ok(())
    }

    /// Clear the analog HS gear pre-set (`apply_dev_quirks` writes 0x25) so a
    /// link left at the PWM gear does not keep the M-PHY configured for HS.
    fn restore_ana_hsgear(&self) {
        let _ = self.dme_set(uic_arg_mib(ANA_HSGEAR_CTRL_ATTR), 0x00);
    }

    /// Dump the main host controller registers (debug helper).
    pub(super) fn dump_regs(&self) {
        // SAFETY: all offsets are UFSHCI register constants inside the mapped
        // MMIO window, and access is exclusive to this host instance.
        unsafe {
            info!("[k3-ufs] Register dump:");
            info!(
                "  CAP:     0x{:08x}",
                self.read32(REG_CONTROLLER_CAPABILITIES)
            );
            info!("  VER:     0x{:08x}", self.read32(REG_UFS_VERSION));
            info!("  HCS:     0x{:08x}", self.read32(REG_CONTROLLER_STATUS));
            info!("  HCE:     0x{:08x}", self.read32(REG_CONTROLLER_ENABLE));
        }
    }
}
