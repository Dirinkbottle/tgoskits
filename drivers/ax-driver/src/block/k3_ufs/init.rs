//! MPHY / UNIPRO / link-startup initialization sequence.
//!
//! These steps bring the UFS physical and link layers up, following the
//! SpacemiT K3 sequence from Linux `drivers/ufs/host/ufs-spacemit.c` and the
//! generic UFSHCI flow from `drivers/ufs/core/ufshcd.c`.

use core::time::Duration;

use log::{info, warn};

use super::{
    K3UfsHost,
    error::UfsError,
    regs::{
        ANA_EQ_CTRL_REG_ATTR, DEVICE_PRESENT, DL_AFC0REQTIMEOUTVAL, MPHY_DEVICE_RESET_DEASSERT,
        MPHY_PLL_LOCK_BIT, MPHY_PU_ALL, MPHY_PU_WITH_HB8_RESET, PA_GRANULARITY,
        PA_LOCAL_TX_LCC_ENABLE, PA_MK2EXTENSIONGUARDBAND, PA_PEER_TX_LCC_ENABLE, PA_PEERSCRAMBLING,
        PA_SCRAMBLING, PA_STALLNOCONFIGTIME, PA_TACTIVATE, PA_TXHSG1PREPARELENGTH,
        PA_TXHSG1SYNCLENGTH, PA_TXHSG2PREPARELENGTH, PA_TXHSG2SYNCLENGTH, PA_TXHSG3PREPARELENGTH,
        PA_TXHSG3SYNCLENGTH, PA_TXMK2EXTENSION, PA_TXSKIP, PA_TXSKIPPERIOD, PA_TXTRAILINGCLOCKS,
        REG_CONTROLLER_CAPABILITIES, REG_CONTROLLER_ENABLE, REG_CONTROLLER_STATUS,
        REG_INTERRUPT_STATUS, REG_UFS_VERSION, REG_UIC_COMMAND, REG_UIC_COMMAND_ARG1,
        REG_UIC_COMMAND_ARG2, REG_UIC_COMMAND_ARG3, REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER,
        RX_GARBAGE_COUNT_OFFSET, RX_HIBERN8TIME_CAP, RX_LANE_HB8_BKDOOR_ATTR, RX_LS_PRE_LEN_CAP,
        RX_MIN_STALL_CAP, RX_PWRM_CLOSURE_LEN_CAP, TX_HIBERN8TIME_CAP, UFS_ATOP_BASE,
        UFS_DEVICE_IO_CTRL, UFS_DL_AFC0REQTIMEOUTVAL_MAX, UFS_HCLKDIV_REG, UFS_MPHY_BKDR_CTRL,
        UFS_MPHY_PU_CTRL, UFS_PA_LINK_STARTUP_TIMER, UFS_PHY_MNG_BASE, UFS_SYS1CLK_1US,
        UFS_TX_SYMBOL_CLK_NS_US, UIC_CMD_DME_LINK_STARTUP, UIC_CMD_DME_SET, UIC_COMMAND_COMPL,
        uic_arg_mib, uic_arg_mib_sel,
    },
};

impl K3UfsHost {
    /// Power up the MPHY and wait for PLL lock (Linux: ufs_spacemit_mphy_init).
    pub(super) fn mphy_init(&self) -> Result<(), UfsError> {
        info!("[k3-ufs] Initializing MPHY...");

        // SAFETY: every offset below is a constant inside the mapped MMIO
        // window (`UFS_PHY_MNG_BASE`/`UFS_ATOP_BASE` + small displacement),
        // and access is exclusive to this host instance; see `read32`/`write32`.
        unsafe {
            // Reset all MPHY logical
            self.write32(UFS_PHY_MNG_BASE, 0x003);
            axklib::time::busy_wait(Duration::from_millis(1));

            // Power up all
            self.write32(UFS_PHY_MNG_BASE + UFS_MPHY_PU_CTRL, MPHY_PU_ALL);
            axklib::time::busy_wait(Duration::from_millis(1));

            // Assert ana_rx_hb8_reset
            self.write32(UFS_PHY_MNG_BASE + UFS_MPHY_PU_CTRL, MPHY_PU_WITH_HB8_RESET);
            axklib::time::busy_wait(Duration::from_millis(1));

            // Deassert ana_rx_hb8_reset
            self.write32(UFS_PHY_MNG_BASE + UFS_MPHY_PU_CTRL, MPHY_PU_ALL);
            axklib::time::busy_wait(Duration::from_millis(1));

            // Deassert UFS device reset & enable reference clock output
            self.write32(
                UFS_PHY_MNG_BASE + UFS_DEVICE_IO_CTRL,
                MPHY_DEVICE_RESET_DEASSERT,
            );
            axklib::time::busy_wait(Duration::from_millis(1));

            // Wait for PLL lock
            for _ in 0..10000 {
                let pu_ctrl = self.read32(UFS_PHY_MNG_BASE + UFS_MPHY_PU_CTRL);
                if pu_ctrl & MPHY_PLL_LOCK_BIT != 0 {
                    info!("[k3-ufs] MPHY PLL locked: 0x{:08x}", pu_ctrl);

                    // Configure ATOP registers via backdoor
                    self.write32(UFS_PHY_MNG_BASE + UFS_MPHY_BKDR_CTRL, 0x1);
                    axklib::time::busy_wait(Duration::from_micros(20));

                    self.write32(UFS_ATOP_BASE + (0xC1 << 2), 0x00);
                    self.write32(UFS_ATOP_BASE + (0xC2 << 2), 0x00);
                    axklib::time::busy_wait(Duration::from_micros(20));

                    self.write32(UFS_PHY_MNG_BASE + UFS_MPHY_BKDR_CTRL, 0x0);
                    axklib::time::busy_wait(Duration::from_micros(20));

                    return Ok(());
                }
                axklib::time::busy_wait(Duration::from_micros(1));
            }

            Err(UfsError::Init("MPHY PLL lock timeout"))
        }
    }

    /// Enable the host controller and program the K3 timer registers.
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

            let sys1clk = self.clock_freq / 1_000_000;
            self.write32(UFS_SYS1CLK_1US, sys1clk);

            let tx_clk = 1000 / (self.clock_freq / 1_000_000);
            self.write32(UFS_TX_SYMBOL_CLK_NS_US, tx_clk << 10);

            self.write32(UFS_PA_LINK_STARTUP_TIMER, 0xFFFFFFFF);

            self.write32(REG_CONTROLLER_ENABLE, 1);
            axklib::time::busy_wait(Duration::from_millis(1));

            let hce = self.read32(REG_CONTROLLER_ENABLE);
            if hce & 1 == 0 {
                return Err(UfsError::Init("Controller enable failed"));
            }

            info!("[k3-ufs] Host controller enabled");
        }

        Ok(())
    }

    /// Issue a UIC command and wait for UIC_COMMAND_COMPL.
    fn uic_cmd(&self, cmd: u32, arg1: u32, arg2: u32, arg3: u32) -> Result<u32, UfsError> {
        // SAFETY: all offsets are UFSHCI register constants inside the mapped
        // MMIO window, and access is exclusive to this host instance; see
        // `read32`/`write32`.
        unsafe {
            self.write32(REG_INTERRUPT_STATUS, UIC_COMMAND_COMPL);
            self.write32(REG_UIC_COMMAND_ARG1, arg1);
            self.write32(REG_UIC_COMMAND_ARG2, arg2);
            self.write32(REG_UIC_COMMAND_ARG3, arg3);
            self.write32(REG_UIC_COMMAND, cmd);

            for _ in 0..5000 {
                let is = self.read32(REG_INTERRUPT_STATUS);
                if is & UIC_COMMAND_COMPL != 0 {
                    self.write32(REG_INTERRUPT_STATUS, UIC_COMMAND_COMPL);
                    return Ok(self.read32(REG_UIC_COMMAND_ARG2));
                }
                axklib::time::busy_wait(Duration::from_micros(100));
            }

            Err(UfsError::Init("UIC command timeout"))
        }
    }

    /// DME_SET a UNIPRO attribute.
    fn dme_set(&self, attr: u32, value: u32) -> Result<(), UfsError> {
        let result = self.uic_cmd(UIC_CMD_DME_SET, attr, 0, value)?;
        if result != 0 {
            warn!(
                "[k3-ufs] DME_SET(0x{:04x})={} failed: 0x{:08x}",
                attr, value, result
            );
            return Err(UfsError::Init("DME_SET failed"));
        }
        Ok(())
    }

    /// UNIPRO v1.6 initialization - critical for link startup.
    pub(super) fn unipro_init(&self) -> Result<(), UfsError> {
        info!("[k3-ufs] Initializing UNIPRO v1.6...");

        // PA layer attributes
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

        // RX lane 0 & 1 attributes
        self.dme_set(uic_arg_mib_sel(RX_LS_PRE_LEN_CAP, 0), 0x0b)?;
        self.dme_set(uic_arg_mib_sel(RX_LS_PRE_LEN_CAP, 1), 0x0b)?;
        self.dme_set(uic_arg_mib_sel(RX_LANE_HB8_BKDOOR_ATTR, 0), 0x9f)?;
        self.dme_set(uic_arg_mib_sel(RX_LANE_HB8_BKDOOR_ATTR, 1), 0x9f)?;
        self.dme_set(uic_arg_mib_sel(RX_PWRM_CLOSURE_LEN_CAP, 0), 15)?;
        self.dme_set(uic_arg_mib_sel(RX_PWRM_CLOSURE_LEN_CAP, 1), 15)?;
        self.dme_set(uic_arg_mib_sel(RX_MIN_STALL_CAP, 0), 0xff)?;
        self.dme_set(uic_arg_mib_sel(RX_MIN_STALL_CAP, 1), 0xff)?;

        // TX lane 0 & 1 hibernate time
        self.dme_set(uic_arg_mib_sel(TX_HIBERN8TIME_CAP, 0), 0x64)?;
        self.dme_set(uic_arg_mib_sel(TX_HIBERN8TIME_CAP, 1), 0x64)?;

        // RX lane 0 & 1 hibernate time
        self.dme_set(uic_arg_mib_sel(RX_HIBERN8TIME_CAP, 0), 0x64)?;
        self.dme_set(uic_arg_mib_sel(RX_HIBERN8TIME_CAP, 1), 0x64)?;

        // TX EQ and RX garbage count
        self.dme_set(uic_arg_mib_sel(ANA_EQ_CTRL_REG_ATTR, 0), 0x5)?;
        self.dme_set(uic_arg_mib_sel(RX_GARBAGE_COUNT_OFFSET, 0), 0x9f)?;
        self.dme_set(uic_arg_mib_sel(RX_GARBAGE_COUNT_OFFSET, 1), 0x9f)?;

        // HCLKDIV register (via DME, not direct register write)
        self.dme_set(uic_arg_mib(UFS_HCLKDIV_REG as u32), 0xfc)?;

        info!("[k3-ufs] UNIPRO v1.6 init completed");
        Ok(())
    }

    /// Run the UFS link startup and wait for the device to appear.
    pub(super) fn link_startup(&self) -> Result<(), UfsError> {
        info!("[k3-ufs] Starting UFS link...");

        let result = self.uic_cmd(UIC_CMD_DME_LINK_STARTUP, 0, 0, 0)?;

        if result != 0 {
            warn!(
                "[k3-ufs] Link startup command failed: result=0x{:08x}",
                result
            );
            return Err(UfsError::Init("Link startup failed"));
        }

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
        // Set DL_AFC0REQTIMEOUTVAL_MAX (required by Linux driver)
        self.dme_set(
            uic_arg_mib(DL_AFC0REQTIMEOUTVAL),
            UFS_DL_AFC0REQTIMEOUTVAL_MAX,
        )?;

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
