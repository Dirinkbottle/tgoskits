//! Vendor-specific SDHCI extension trait for the K3 host controller.

use crate::{K3SdhciHost, SdMode, Timing, TuningError};

/// Vendor-specific operations that the SDHCI stack can rely on regardless of
/// the underlying host implementation.
///
/// The K3 controller needs vendor register sequences beyond the standard
/// SDHCI command path for reset, timing selection and RX tuning; these are
/// exposed so that OS glue layers can drive the controller uniformly.
pub trait SdhciVendorExt {
    /// Resets the host and applies the K3 PHY setup for the given bus mode.
    fn vendor_reset(&mut self, mode: SdMode);

    /// Selects the MMC_CTRL timing bits for the given `timing`.
    fn vendor_set_timing(&mut self, timing: Timing);

    /// Selects the internal transmit clock for the given `timing`.
    fn vendor_set_clock(&mut self, timing: Timing);

    /// Runs software RX tuning and returns the selected delay.
    ///
    /// # Errors
    ///
    /// Returns [`TuningError`] when no sufficiently wide window is found.
    fn vendor_execute_tuning<F>(&mut self, timing: Timing, test_fn: F) -> Result<(), TuningError>
    where
        F: FnMut(u8) -> bool;
}

impl SdhciVendorExt for K3SdhciHost {
    fn vendor_reset(&mut self, mode: SdMode) {
        self.reset(mode);
    }

    fn vendor_set_timing(&mut self, timing: Timing) {
        self.set_timing(timing);
    }

    fn vendor_set_clock(&mut self, timing: Timing) {
        self.set_clock(timing);
    }

    fn vendor_execute_tuning<F>(&mut self, timing: Timing, test_fn: F) -> Result<(), TuningError>
    where
        F: FnMut(u8) -> bool,
    {
        self.execute_tuning(timing, test_fn)
    }
}

/// Maps a raw SDHCI timing index (the Linux `MMC_TIMING_*` encoding, as
/// found in the device tree or the SDHCI core) to the portable [`Timing`]
/// enum.
///
/// Unknown values fall back to [`Timing::Legacy`]. This mirrors the reference
/// Linux driver, which treats unrecognized timing indices as the default
/// legacy mode, so a stale or unknown timing never faults the host.
impl From<u32> for Timing {
    fn from(raw: u32) -> Self {
        match raw {
            0 => Self::Legacy,
            1 => Self::MmcHs,
            2 => Self::SdHs,
            3 => Self::UhsSdr12,
            4 => Self::UhsSdr25,
            5 => Self::UhsSdr50,
            6 => Self::UhsSdr104,
            7 => Self::MmcHs200,
            8 => Self::MmcHs400,
            _ => Self::Legacy,
        }
    }
}
