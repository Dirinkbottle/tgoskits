//! SpacemiT K3 SDHCI Host Controller Driver
//!
//! Portable driver core for the SpacemiT K3/K1 SDHCI host controller:
//! PHY configuration, HS200/HS400 timing, and software RX tuning. OS glue
//! (FDT probe, IRQ, block registration) lives in the consuming layer.

#![no_std]

use log::{debug, warn};
use mmio_api::MmioRaw;

mod vendor_ext;
pub use vendor_ext::SdhciVendorExt;

/// Highest RX delay-code step scanned during software tuning.
///
/// The delay line supports 256 codes (`0..=0xFF`), matching the 8-bit
/// `RX_DLINE_CODE` field in `DLINE_CTRL`.
const RX_DELAY_CODE_MAX: u8 = 0xff;

/// Polling budget for PHY DLL lock, in MMIO read iterations.
///
/// Each MMIO read takes on the order of a microsecond on the K3 bus, which
/// gives the DLL time to settle between polls.
const PHY_DLL_POLL_BUDGET: u32 = 50;

/// Default PHY pad drive strength selection programmed during eMMC reset.
const DEFAULT_PHY_DRIVER_SEL: u8 = 4;

/// Minimum consecutive passing delay codes required for a reliable window.
const DEFAULT_TUNING_WINDOW_LIMIT: u16 = 50;

/// K3 SDHCI register offsets and bit fields (SpacemiT vendor block).
///
/// Offsets and field names follow the reference Linux driver
/// `drivers/mmc/host/sdhci-of-k1.c`.
mod regs {
    pub const OP_EXT: usize = 0x108;
    pub const MMC_CTRL: usize = 0x114;
    pub const RX_CFG: usize = 0x118;
    pub const TX_CFG: usize = 0x11C;
    pub const DLINE_CTRL: usize = 0x130;
    pub const DLINE_CFG: usize = 0x134;
    pub const PHY_CTRL: usize = 0x160;
    pub const PHY_FUNC: usize = 0x164;
    pub const PHY_DLLCFG: usize = 0x168;
    pub const PHY_DLLCFG1: usize = 0x16C;
    pub const PHY_DLLSTS: usize = 0x170;
    pub const PHY_PADCFG: usize = 0x178;

    // PHY_DLLCFG fields.
    pub const DLL_PREDLY_NUM_MASK: u32 = 0b11 << 2;
    pub const DLL_FULLDLY_RANGE_MASK: u32 = 0b11 << 4;
    pub const DLL_VREG_CTRL_MASK: u32 = 0b11 << 6;
    pub const DLL_ENABLE: u32 = 1 << 31;

    // Values programmed into the DLL fields during PHY DLL initialization.
    // Each field is set to 1 as required by the reference driver.
    pub const DLL_PREDLY_NUM_VALUE_1: u32 = 1 << 2;
    pub const DLL_FULLDLY_RANGE_VALUE_1: u32 = 1 << 4;
    pub const DLL_VREG_CTRL_VALUE_1: u32 = 1 << 6;

    // PHY_DLLCFG1 fields.
    pub const DLL_REG1_CTRL_MASK: u32 = 0xff;
    pub const DLL_REG1_CTRL_VALUE: u32 = 0x92;

    // PHY_DLLSTS fields.
    pub const DLL_LOCK_STATE: u32 = 1 << 0;

    // PHY_PADCFG fields.
    pub const PHY_DRIVE_SEL_MASK: u32 = 0b111;
    pub const RX_BIAS_CTRL: u32 = 1 << 5;

    // RX_CFG fields.
    pub const RX_SDCLK_SEL1_MASK: u32 = 0b11 << 2;
    pub const RX_SDCLK_SEL1_VALUE_1: u32 = 1 << 2;

    // DLINE_CTRL fields.
    pub const DLINE_PU: u32 = 1 << 0;
    pub const RX_DLINE_CODE_MASK: u32 = 0xff << 16;

    /// Returns the `RX_DLINE_CODE` field value for an RX delay code.
    pub const fn rx_dline_code(delay: u8) -> u32 {
        (delay as u32) << 16
    }

    // DLINE_CFG fields.
    pub const RX_DLINE_REG_MASK: u32 = 0xff;
}

/// Failure modes of PHY/tuning operations.
#[derive(Debug, Copy, Clone, PartialEq, Eq, thiserror::Error)]
pub enum TuningError {
    /// The PHY DLL did not report lock within the polling budget.
    #[error("PHY DLL lock timeout")]
    DllLockTimeout,
    /// The tuning pass found no window wide enough to be reliable.
    #[error("tuning window too narrow: max_windows={max_windows}")]
    WindowTooNarrow { max_windows: u16 },
}

/// SDHCI timing modes the K3 controller supports.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Timing {
    /// Default (legacy) single-data-rate mode.
    Legacy,
    /// MMC high-speed mode.
    MmcHs,
    /// SD high-speed mode.
    SdHs,
    /// UHS-I SDR12 (12.5 MB/s).
    UhsSdr12,
    /// UHS-I SDR25 (25 MB/s).
    UhsSdr25,
    /// UHS-I SDR50 (50 MB/s).
    UhsSdr50,
    /// UHS-I SDR104 (104 MB/s).
    UhsSdr104,
    /// eMMC HS200 mode.
    MmcHs200,
    /// eMMC HS400 mode.
    MmcHs400,
}

/// Whether the attached card is an embedded eMMC device or a removable
/// SD/SDIO card. The K3 PHY configuration differs between the two.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum SdMode {
    /// Removable SD/SDIO card.
    Sd,
    /// Embedded eMMC device.
    Emmc,
}

/// Clock gating policy for the card clock output.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ClockGate {
    /// Manual override: the clock is forced on and never gated.
    Manual,
    /// The controller may auto-gate the clock when the bus is idle.
    Auto,
}

/// Whether to enable or disable the HS400 enhanced strobe.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Hs400Strobe {
    /// Disable the enhanced strobe.
    Disable,
    /// Enable the enhanced strobe (re-initializes the PHY DLL).
    Enable,
}

bitflags::bitflags! {
    #[derive(Copy, Clone, Debug)]
    pub(crate) struct OpExtFlags: u32 {
        const OVRRD_CLK_OEN = 1 << 11;
        const FORCE_CLK_ON = 1 << 12;
    }

    #[derive(Copy, Clone, Debug)]
    pub(crate) struct MmcCtrlFlags: u32 {
        const ENHANCE_STROBE_EN = 1 << 8;
        const MMC_HS400 = 1 << 9;
        const MMC_HS200 = 1 << 10;
        const MMC_CARD_MODE = 1 << 12;
    }

    #[derive(Copy, Clone, Debug)]
    pub(crate) struct TxCfgFlags: u32 {
        const TX_INT_CLK_SEL = 1 << 30;
    }

    #[derive(Copy, Clone, Debug)]
    pub(crate) struct PhyCtrlFlags: u32 {
        const PHY_FUNC_EN = 1 << 0;
        const PHY_PLL_LOCK = 1 << 1;
    }

    #[derive(Copy, Clone, Debug)]
    pub(crate) struct PhyFuncFlags: u32 {
        const HS200_USE_RFIFO = 1 << 15;
    }
}

/// A contiguous range of RX delay codes that pass the tuning check.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct TuningWindow {
    /// First delay code of the widest passing window.
    min_delay: u8,
    /// Last delay code of the widest passing window.
    max_delay: u8,
}

/// Result of scanning the full RX delay-code space for the widest run of
/// passing delays.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
struct TuningWindowScan {
    /// Number of consecutive passing delay codes in the widest run.
    max_windows: u16,
    /// Delay code at the end of the widest run.
    max_delay: u8,
}

/// Per-controller state carried across software RX tuning passes.
#[derive(Debug, Clone)]
pub(crate) struct RxTuning {
    rx_dline_reg: u8,
    windows: TuningWindow,
    select_delay: u8,
    window_limit: u16,
}

impl Default for RxTuning {
    fn default() -> Self {
        Self {
            rx_dline_reg: 0,
            windows: TuningWindow {
                min_delay: 0,
                max_delay: 0,
            },
            select_delay: 0,
            window_limit: DEFAULT_TUNING_WINDOW_LIMIT,
        }
    }
}

/// Portable driver core for the K3 SDHCI host controller.
///
/// Owns the MMIO window and the per-controller PHY/tuning state. The host
/// does not depend on any OS glue; the consuming layer is responsible for
/// mapping the MMIO window (via `mmio-api`), registering interrupts and
/// wiring the block stack.
pub struct K3SdhciHost {
    mmio: MmioRaw,
    phy_driver_sel: u8,
    rxtuning: RxTuning,
}

impl K3SdhciHost {
    /// Creates a host controller over an already-mapped MMIO window.
    ///
    /// The caller must keep the mapping alive for the lifetime of the host.
    pub fn new(mmio: MmioRaw) -> Self {
        Self {
            mmio,
            phy_driver_sel: DEFAULT_PHY_DRIVER_SEL,
            rxtuning: RxTuning::default(),
        }
    }

    #[inline]
    fn read32(&self, offset: usize) -> u32 {
        self.mmio.read(offset)
    }

    #[inline]
    fn write32(&self, offset: usize, val: u32) {
        self.mmio.write(offset, val);
    }

    #[inline]
    fn setbits(&mut self, offset: usize, bits: u32) {
        let val = self.read32(offset);
        self.write32(offset, val | bits);
    }

    #[inline]
    fn clrbits(&mut self, offset: usize, bits: u32) {
        let val = self.read32(offset);
        self.write32(offset, val & !bits);
    }

    #[inline]
    fn clrsetbits(&mut self, offset: usize, clr: u32, set: u32) {
        let val = self.read32(offset);
        self.write32(offset, (val & !clr) | set);
    }

    /// Resets the controller and applies the K3 PHY configuration for the
    /// given bus mode.
    ///
    /// For [`SdMode::Emmc`] this enables the PHY, programs the pad drive
    /// strength and switches to card mode. For [`SdMode::Sd`] it selects the
    /// internal transmit clock.
    pub fn reset(&mut self, mode: SdMode) {
        match mode {
            SdMode::Emmc => {
                self.setbits(
                    regs::PHY_CTRL,
                    PhyCtrlFlags::PHY_FUNC_EN.bits() | PhyCtrlFlags::PHY_PLL_LOCK.bits(),
                );
                self.clrsetbits(
                    regs::PHY_PADCFG,
                    regs::PHY_DRIVE_SEL_MASK,
                    regs::RX_BIAS_CTRL | (self.phy_driver_sel as u32 & regs::PHY_DRIVE_SEL_MASK),
                );
                self.setbits(regs::MMC_CTRL, MmcCtrlFlags::MMC_CARD_MODE.bits());
            }
            SdMode::Sd => {
                self.setbits(regs::TX_CFG, TxCfgFlags::TX_INT_CLK_SEL.bits());
            }
        }
    }

    /// Selects the MMC_CTRL timing bits for the given `timing`.
    ///
    /// Only [`Timing::MmcHs200`] and [`Timing::MmcHs400`] program a dedicated
    /// timing bit; all other timings leave the timing bits untouched.
    pub fn set_timing(&mut self, timing: Timing) {
        match timing {
            Timing::MmcHs200 => {
                self.setbits(regs::MMC_CTRL, MmcCtrlFlags::MMC_HS200.bits());
            }
            Timing::MmcHs400 => {
                self.setbits(regs::MMC_CTRL, MmcCtrlFlags::MMC_HS400.bits());
            }
            _ => {}
        }
    }

    /// Applies the clock gating policy for the card clock output.
    pub fn set_clock_gate(&mut self, gate: ClockGate) {
        let flags = OpExtFlags::OVRRD_CLK_OEN | OpExtFlags::FORCE_CLK_ON;
        match gate {
            ClockGate::Manual => self.setbits(regs::OP_EXT, flags.bits()),
            ClockGate::Auto => self.clrbits(regs::OP_EXT, flags.bits()),
        }
    }

    /// Selects the internal transmit clock source for the given `timing`.
    ///
    /// Timings at or below UHS SDR50 use the internal clock; HS200/HS400
    /// timing modes use the card clock directly.
    pub fn set_clock(&mut self, timing: Timing) {
        if matches!(
            timing,
            Timing::Legacy
                | Timing::MmcHs
                | Timing::SdHs
                | Timing::UhsSdr12
                | Timing::UhsSdr25
                | Timing::UhsSdr50
        ) {
            self.setbits(regs::TX_CFG, TxCfgFlags::TX_INT_CLK_SEL.bits());
        } else {
            self.clrbits(regs::TX_CFG, TxCfgFlags::TX_INT_CLK_SEL.bits());
        }
    }

    /// Brings the PHY DLL up and waits for it to report lock.
    ///
    /// # Errors
    ///
    /// Returns [`TuningError::DllLockTimeout`] if the DLL does not lock
    /// within [`PHY_DLL_POLL_BUDGET`] MMIO polls.
    fn phy_dll_init(&mut self) -> Result<(), TuningError> {
        self.clrsetbits(
            regs::PHY_DLLCFG,
            regs::DLL_PREDLY_NUM_MASK | regs::DLL_FULLDLY_RANGE_MASK | regs::DLL_VREG_CTRL_MASK,
            regs::DLL_PREDLY_NUM_VALUE_1
                | regs::DLL_FULLDLY_RANGE_VALUE_1
                | regs::DLL_VREG_CTRL_VALUE_1,
        );
        self.clrsetbits(
            regs::PHY_DLLCFG1,
            regs::DLL_REG1_CTRL_MASK,
            regs::DLL_REG1_CTRL_VALUE,
        );
        self.setbits(regs::PHY_DLLCFG, regs::DLL_ENABLE);

        for _ in 0..PHY_DLL_POLL_BUDGET {
            if self.read32(regs::PHY_DLLSTS) & regs::DLL_LOCK_STATE != 0 {
                return Ok(());
            }
            // Each MMIO read takes on the order of a microsecond on the K3
            // bus, which gives the DLL time to settle between polls.
        }
        warn!("PHY DLL lock timeout");
        Err(TuningError::DllLockTimeout)
    }

    /// Enables or disables the HS400 enhanced strobe.
    ///
    /// Enabling the strobe also brings the PHY DLL back up so that the data
    /// strobe is sampled reliably.
    ///
    /// # Errors
    ///
    /// Returns [`TuningError::DllLockTimeout`] if the PHY DLL fails to lock
    /// while enabling the strobe.
    pub fn enable_hs400_strobe(&mut self, strobe: Hs400Strobe) -> Result<(), TuningError> {
        match strobe {
            Hs400Strobe::Enable => {
                self.setbits(regs::MMC_CTRL, MmcCtrlFlags::ENHANCE_STROBE_EN.bits());
                self.phy_dll_init()?;
            }
            Hs400Strobe::Disable => {
                self.clrbits(regs::MMC_CTRL, MmcCtrlFlags::ENHANCE_STROBE_EN.bits());
            }
        }
        Ok(())
    }

    /// Selects HS400 timing in preparation for the mode switch.
    pub fn prepare_hs400(&mut self) {
        self.setbits(regs::MMC_CTRL, MmcCtrlFlags::MMC_HS400.bits());
    }

    /// Completes the HS400 mode switch by re-initializing the PHY DLL.
    ///
    /// # Errors
    ///
    /// Returns [`TuningError::DllLockTimeout`] if the PHY DLL fails to lock.
    pub fn post_hs400_config(&mut self) -> Result<(), TuningError> {
        self.phy_dll_init()
    }

    /// Downgrades the bus from HS400 back to HS200.
    ///
    /// Takes the PHY down, clears the HS400/HS200/strobe timing bits and
    /// re-enables the PHY.
    pub fn hs400_to_hs200(&mut self) {
        self.clrbits(
            regs::PHY_CTRL,
            PhyCtrlFlags::PHY_FUNC_EN.bits() | PhyCtrlFlags::PHY_PLL_LOCK.bits(),
        );
        self.clrbits(
            regs::MMC_CTRL,
            MmcCtrlFlags::MMC_HS400.bits()
                | MmcCtrlFlags::MMC_HS200.bits()
                | MmcCtrlFlags::ENHANCE_STROBE_EN.bits(),
        );
        self.clrbits(regs::PHY_FUNC, PhyFuncFlags::HS200_USE_RFIFO.bits());
        // PHY re-enable is preceded by MMIO register accesses that give the
        // bus time to settle after the PHY is taken down.
        self.setbits(
            regs::PHY_CTRL,
            PhyCtrlFlags::PHY_FUNC_EN.bits() | PhyCtrlFlags::PHY_PLL_LOCK.bits(),
        );
    }

    /// Routes the RX delay line to `dline_reg` and selects the tuned clock.
    fn sw_rx_tuning_prepare(&mut self, dline_reg: u8) {
        self.clrsetbits(regs::DLINE_CFG, regs::RX_DLINE_REG_MASK, dline_reg as u32);
        self.setbits(regs::DLINE_CTRL, regs::DLINE_PU);
        // MMIO read/write spacing gives the delay line controller time to
        // latch the new configuration before RX switching.
        self.clrsetbits(
            regs::RX_CFG,
            regs::RX_SDCLK_SEL1_MASK,
            regs::RX_SDCLK_SEL1_VALUE_1,
        );
    }

    /// Programs the RX delay line with the given delay code.
    fn sw_rx_set_delaycode(&mut self, delay: u8) {
        self.clrsetbits(
            regs::DLINE_CTRL,
            regs::RX_DLINE_CODE_MASK,
            regs::rx_dline_code(delay),
        );
    }

    /// Runs software RX tuning by scanning the full delay-code space.
    ///
    /// Tuning is performed for [`Timing::MmcHs200`], [`Timing::UhsSdr50`]
    /// and [`Timing::UhsSdr104`]; other timings return `Ok(())` without
    /// touching the controller.
    ///
    /// `test_fn` is invoked once per delay code with the code being tested
    /// and must report whether the card accepts that delay. The widest
    /// consecutive run of passing codes becomes the tuning window; the
    /// midpoint of the window is programmed back into the RX delay line.
    ///
    /// # Errors
    ///
    /// Returns [`TuningError::WindowTooNarrow`] when the widest passing run
    /// is shorter than the configured window limit.
    pub fn execute_tuning<F>(&mut self, timing: Timing, mut test_fn: F) -> Result<(), TuningError>
    where
        F: FnMut(u8) -> bool,
    {
        if !matches!(
            timing,
            Timing::MmcHs200 | Timing::UhsSdr50 | Timing::UhsSdr104
        ) {
            return Ok(());
        }

        self.sw_rx_tuning_prepare(self.rxtuning.rx_dline_reg);

        let scan = scan_tuning_windows(&mut test_fn);
        let Some(window) = select_tuning_window(scan, self.rxtuning.window_limit) else {
            warn!("Tuning failed: max_windows={}", scan.max_windows);
            return Err(TuningError::WindowTooNarrow {
                max_windows: scan.max_windows,
            });
        };

        self.rxtuning.windows = window;
        // Midpoint of the passing window: the element at index
        // `max_windows / 2` of the run, which for even-sized runs is the
        // upper-middle element.
        self.rxtuning.select_delay = window.min_delay + (scan.max_windows / 2) as u8;

        self.sw_rx_set_delaycode(self.rxtuning.select_delay);
        debug!(
            "Tuning done: delay={}, window=[{}, {}]",
            self.rxtuning.select_delay, window.min_delay, window.max_delay
        );

        Ok(())
    }
}

/// Scans every RX delay code from `0` to [`RX_DELAY_CODE_MAX`] and returns
/// the widest consecutive run of delays for which `test_fn` returns `true`.
///
/// When several runs share the same length, the first one wins. The counters
/// are `u16` so that an all-passing sweep (256 codes) cannot overflow.
fn scan_tuning_windows<F>(test_fn: &mut F) -> TuningWindowScan
where
    F: FnMut(u8) -> bool,
{
    let mut max_windows: u16 = 0;
    let mut cur_windows: u16 = 0;
    let mut max_delay: u8 = 0;

    for delay in 0..=RX_DELAY_CODE_MAX {
        if test_fn(delay) {
            cur_windows += 1;
            if cur_windows > max_windows {
                max_windows = cur_windows;
                max_delay = delay;
            }
        } else {
            cur_windows = 0;
        }
    }

    TuningWindowScan {
        max_windows,
        max_delay,
    }
}

/// Converts a tuning scan into a usable window.
///
/// Returns `None` when the widest passing run is shorter than
/// `window_limit`; otherwise returns the `[min_delay, max_delay]` interval
/// of the run.
fn select_tuning_window(scan: TuningWindowScan, window_limit: u16) -> Option<TuningWindow> {
    if scan.max_windows < window_limit {
        return None;
    }
    let min_delay = scan
        .max_delay
        .saturating_sub(scan.max_windows.saturating_sub(1) as u8);
    Some(TuningWindow {
        min_delay,
        max_delay: scan.max_delay,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scan_finds_widest_run_in_middle() {
        let scan = scan_tuning_windows(&mut |delay| (5..=14).contains(&delay));
        assert_eq!(scan.max_windows, 10);
        assert_eq!(scan.max_delay, 14);
    }

    #[test]
    fn scan_no_pass_returns_zero() {
        let scan = scan_tuning_windows(&mut |_| false);
        assert_eq!(scan.max_windows, 0);
        assert_eq!(scan.max_delay, 0);
    }

    #[test]
    fn scan_single_delay_at_zero() {
        let scan = scan_tuning_windows(&mut |delay| delay == 0);
        assert_eq!(scan.max_windows, 1);
        assert_eq!(scan.max_delay, 0);
    }

    #[test]
    fn scan_window_at_upper_boundary() {
        let scan = scan_tuning_windows(&mut |delay| delay >= 250);
        assert_eq!(scan.max_windows, 6);
        assert_eq!(scan.max_delay, RX_DELAY_CODE_MAX);
    }

    #[test]
    fn scan_keeps_first_run_on_tie() {
        let scan = scan_tuning_windows(&mut |delay| {
            (10..=19).contains(&delay) || (40..=49).contains(&delay)
        });
        assert_eq!(scan.max_windows, 10);
        assert_eq!(scan.max_delay, 19);
    }

    #[test]
    fn select_rejects_window_below_limit() {
        let scan = TuningWindowScan {
            max_windows: 4,
            max_delay: 10,
        };
        assert_eq!(select_tuning_window(scan, 50), None);
    }

    #[test]
    fn select_computes_min_and_max_delay() {
        let scan = TuningWindowScan {
            max_windows: 10,
            max_delay: 50,
        };
        let window = select_tuning_window(scan, 5).expect("window wide enough");
        assert_eq!(window.min_delay, 41);
        assert_eq!(window.max_delay, 50);
    }

    #[test]
    fn select_window_at_lower_boundary() {
        let scan = TuningWindowScan {
            max_windows: 8,
            max_delay: 7,
        };
        let window = select_tuning_window(scan, 5).expect("window wide enough");
        assert_eq!(window.min_delay, 0);
        assert_eq!(window.max_delay, 7);
    }

    #[test]
    fn select_single_delay_window() {
        let scan = TuningWindowScan {
            max_windows: 1,
            max_delay: 200,
        };
        let window = select_tuning_window(scan, 1).expect("window wide enough");
        assert_eq!(window.min_delay, 200);
        assert_eq!(window.max_delay, 200);
    }

    fn host_on_backing_store() -> K3SdhciHost {
        // Enough space to cover the highest vendor register offset (0x178).
        let mut backing = [0u8; 0x200];
        let virt = core::ptr::NonNull::from(&mut backing[0]);
        let mmio = unsafe { MmioRaw::new(0u64.into(), virt, backing.len()) };
        K3SdhciHost::new(mmio)
    }

    #[test]
    fn execute_tuning_skips_unsupported_timings() {
        let mut host = host_on_backing_store();
        let result = host.execute_tuning(Timing::Legacy, |_| true);
        assert!(result.is_ok());
        // No delay was selected for a skipped timing.
        assert_eq!(host.rxtuning.select_delay, 0);
    }

    #[test]
    fn execute_tuning_rejects_narrow_window() {
        let mut host = host_on_backing_store();
        let result = host.execute_tuning(Timing::UhsSdr104, |delay| delay < 10);
        assert_eq!(
            result,
            Err(TuningError::WindowTooNarrow { max_windows: 10 })
        );
    }

    #[test]
    fn execute_tuning_sets_selected_delay() {
        let mut host = host_on_backing_store();
        let result = host.execute_tuning(Timing::MmcHs200, |delay| (50..=109).contains(&delay));
        assert!(result.is_ok());

        let window = host.rxtuning.windows;
        assert_eq!(window.min_delay, 50);
        assert_eq!(window.max_delay, 109);
        // Midpoint of the 60-code window, rounded toward the low edge.
        assert_eq!(host.rxtuning.select_delay, 80);
    }
}
