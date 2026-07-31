//! Driver error type for the K3 UFS host.
//!
//! [`UfsError`] distinguishes transfer failures that need a controller reset
//! (timeout, OCS, controller fatal) from init and protocol failures that do
//! not, so the transfer path can decide when host recovery is required.

/// UPIU submission and host initialization outcome.
///
/// `Timeout`, `OcsError` and `ControllerFatal` describe failures that require
/// controller recovery; `Init` covers hardware init/link-sequence failures and
/// `Other` covers driver-internal and SCSI-protocol failures that do not.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub(crate) enum UfsError {
    /// The doorbell bit was never cleared before the poll timeout.
    #[error("UFS transfer timeout")]
    Timeout,
    /// The UTRD reported a non-zero Overall Command Status.
    #[error("UFS OCS error")]
    OcsError,
    /// A fatal error interrupt fired while a transfer was outstanding.
    #[error("UFS controller fatal error")]
    ControllerFatal,
    /// A hardware init/link startup sequence step failed. The payload is the
    /// failing step's static reason string.
    #[error("UFS host init failed: {0}")]
    Init(&'static str),
    /// Driver-internal or SCSI-protocol failure that does not require host
    /// recovery (allocation, invalid parameters, unexpected response).
    #[error("UFS driver error: {0}")]
    Other(&'static str),
}
