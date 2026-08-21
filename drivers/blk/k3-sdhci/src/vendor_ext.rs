//! K3 SDHCI vendor extensions integration

use crate::{K3SdhciHost, Timing};

/// Vendor-specific extensions for SDHCI host
pub trait SdhciVendorExt {
    fn vendor_reset(&mut self, is_emmc: bool);
    fn vendor_set_timing(&mut self, timing: Timing);
    fn vendor_set_clock(&mut self, timing: Timing);
    fn vendor_execute_tuning<F>(&mut self, timing: Timing, test_fn: F) -> Result<(), ()>
    where
        F: FnMut(u8) -> bool;
}

impl SdhciVendorExt for K3SdhciHost {
    fn vendor_reset(&mut self, is_emmc: bool) {
        self.reset(is_emmc);
    }

    fn vendor_set_timing(&mut self, timing: Timing) {
        self.set_timing(timing);
    }

    fn vendor_set_clock(&mut self, timing: Timing) {
        self.set_clock(timing);
    }

    fn vendor_execute_tuning<F>(&mut self, timing: Timing, test_fn: F) -> Result<(), ()>
    where
        F: FnMut(u8) -> bool,
    {
        self.execute_tuning(timing, test_fn)
    }
}

/// K3-specific timing to generic timing conversion
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
