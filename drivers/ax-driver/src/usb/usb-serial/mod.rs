//! USB-to-serial interface probe drivers.
//!
//! These drivers only identify supported interfaces and report them through
//! the USB host driver's probe path.  Data transfers remain implemented by
//! the kernel-side USB serial transport.

mod cdc_acm;
mod ch34x;
mod cp210x;

use crab_usb::USBHost;

pub(super) fn register(host: &mut USBHost) {
    ch34x::register(host);
    cp210x::register(host);
    cdc_acm::register(host);
}
