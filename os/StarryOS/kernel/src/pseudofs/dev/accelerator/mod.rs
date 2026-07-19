//! Hardware AI accelerator character devices.

// The SpacemiT K3 (`k3_com260kit`) AI runner lives under its own submodule so
// its control plane, memory bookkeeping and scheduler glue can be split into
// focused files instead of one monolithic device module.
#[cfg(feature = "k3_com260kit")]
#[allow(non_snake_case)]
pub mod k3AiCore;
