mod binding;

#[cfg(feature = "aic8800-wifi")]
pub mod aic8800;
#[cfg(feature = "fxmac")]
pub mod fxmac;
#[cfg(feature = "intel-net")]
pub mod intel;
#[cfg(feature = "k3-gmac")]
pub mod k3_gmac;
#[cfg(feature = "ls2k1000-gmac")]
pub mod loongson_gmac;
#[cfg(feature = "realtek-rtl8125")]
pub mod realtek;

pub use binding::*;
