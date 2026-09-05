//! CH34x USB-to-serial protocol support.
//!
//! Request and register names follow Linux `drivers/usb/serial/ch341.c`.
//! The initialization sequence is intentionally incomplete while it is being
//! implemented step by step.

use log::{error, info, warn};
use usb_if::descriptor::InterfaceDescriptor;

use crate::{
    ControlTransfer, UsbDeviceId, UsbSerialPort, bulk_pair_for_interface,
    device_id_from_descriptor_blob,
};

pub const VENDOR_ID: u16 = 0x1a86;
pub const PRODUCT_ID_CH340: u16 = 0x7523;

const USB_DIR_OUT: u8 = 0x00;
const USB_DIR_IN: u8 = 0x80;
const USB_TYPE_VENDOR: u8 = 0x40;
const USB_RECIP_DEVICE: u8 = 0x00;
const VENDOR_DEVICE_OUT: u8 = USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
const VENDOR_DEVICE_IN: u8 = USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE;

pub const CH341_REQ_READ_VERSION: u8 = 0x5f;
pub const CH341_REQ_WRITE_REG: u8 = 0x9a;
pub const CH341_REQ_READ_REG: u8 = 0x95;
pub const CH341_REQ_SERIAL_INIT: u8 = 0xa1;
pub const CH341_REQ_MODEM_CTRL: u8 = 0xa4;

pub const CH341_REG_BREAK: u8 = 0x05;
pub const CH341_REG_PRESCALER: u8 = 0x12;
pub const CH341_REG_DIVISOR: u8 = 0x13;
pub const CH341_REG_LCR: u8 = 0x18;
pub const CH341_REG_LCR2: u8 = 0x25;
pub const CH341_REG_FLOW_CTL: u8 = 0x27;

pub const CH341_BIT_RTS: u8 = 1 << 6;
pub const CH341_BIT_DTR: u8 = 1 << 5;
pub const CH341_LCR_ENABLE_RX: u8 = 0x80;
pub const CH341_LCR_ENABLE_TX: u8 = 0x40;
pub const CH341_LCR_MARK_SPACE: u8 = 0x20;
pub const CH341_LCR_PAR_EVEN: u8 = 0x10;
pub const CH341_LCR_ENABLE_PAR: u8 = 0x08;
pub const CH341_LCR_STOP_BITS_2: u8 = 0x04;
pub const CH341_LCR_CS8: u8 = 0x03;
pub const CH341_LCR_CS7: u8 = 0x02;
pub const CH341_LCR_CS6: u8 = 0x01;
pub const CH341_LCR_CS5: u8 = 0x00;
pub const CH341_FLOW_CTL_NONE: u8 = 0x00;
pub const CH341_FLOW_CTL_RTSCTS: u8 = 0x01;

pub const CH341_CLKRATE: u32 = 48_000_000;
pub const CH341_MIN_BPS: u32 = 46;
pub const CH341_MAX_BPS: u32 = 3_000_000;

const USB_CLASS_VENDOR_SPECIFIC: u8 = 0xff;

pub fn probe(descriptor_blob: &[u8]) -> Option<UsbSerialPort> {
    let UsbDeviceId {
        vendor_id,
        product_id,
    } = device_id_from_descriptor_blob(descriptor_blob)?;
    if !matches!((vendor_id, product_id), (VENDOR_ID, PRODUCT_ID_CH340)) {
        return None;
    }

    bulk_pair_for_interface(descriptor_blob, is_data_interface)
}

/// Starts the CH34x initialization sequence.
///
/// This currently performs the Linux version read, serial initialization,
/// baud-rate/LCR setup, and quirk probe. Modem handshake and full termios
/// support are still separate implementation steps.
pub fn init<T: ControlTransfer>(
    control: &T,
    _port: &UsbSerialPort,
    baud: u32,
) -> Result<(), T::Error> {
    let version = ch341_read_version(control)?;
    info!("ch34x: chip version {version:#04x}");

    ch341_control_out(control, CH341_REQ_SERIAL_INIT, 0, 0)?;

    ch341_set_baudrate_lcr(
        control,
        version,
        baud,
        CH341_LCR_ENABLE_RX | CH341_LCR_ENABLE_TX | CH341_LCR_CS8,
    )?;
    ch341_detect_quirks(control)?;
    Ok(())
}

/// Changes the CH34x baud rate.
pub fn set_baud<T: ControlTransfer>(
    control: &T,
    _port: &UsbSerialPort,
    baud: u32,
) -> Result<(), T::Error> {
    let version = ch341_read_version(control)?;
    ch341_set_baudrate_lcr(
        control,
        version,
        baud,
        CH341_LCR_ENABLE_RX | CH341_LCR_ENABLE_TX | CH341_LCR_CS8,
    )?;
    Ok(())
}

fn ch341_read_version<T: ControlTransfer>(control: &T) -> Result<u8, T::Error> {
    let mut response = [0u8; 2];
    let actual = ch341_control_in(control, CH341_REQ_READ_VERSION, 0, 0, &mut response)?;
    if actual != response.len() {
        warn!(
            "ch34x: version response is short: expected {} bytes, received {actual}",
            response.len()
        );
    }
    Ok(response[0])
}

fn ch341_set_baudrate_lcr<T: ControlTransfer>(
    control: &T,
    version: u8,
    baud: u32,
    lcr: u8,
) -> Result<(), T::Error> {
    if baud == 0 {
        return Ok(());
    }

    let mut divisor = ch341_get_divisor(baud);
    // CH341A buffers data until a complete 32-byte packet arrives unless bit
    // 7 is set. Linux keeps this bit for newer chip versions.
    if version > 0x27 {
        divisor |= 1 << 7;
    }

    ch341_control_out(
        control,
        CH341_REQ_WRITE_REG,
        (u16::from(CH341_REG_DIVISOR) << 8) | u16::from(CH341_REG_PRESCALER),
        divisor,
    )?;

    // Versions below 0x30 use the default line-control registers selected by
    // SERIAL_INIT; Linux returns before writing LCR/LCR2 for those versions.
    if version < 0x30 {
        return Ok(());
    }

    ch341_control_out(
        control,
        CH341_REQ_WRITE_REG,
        (u16::from(CH341_REG_LCR2) << 8) | u16::from(CH341_REG_LCR),
        u16::from(lcr),
    )?;
    Ok(())
}

/// Probes the optional CH34x break-control register.
///
/// Linux uses a stalled read here to identify devices with limited prescaler
/// and simulated-break support. The current portable state does not retain
/// those quirk flags yet, so this step only performs the probe and propagates
/// failures after logging them.
fn ch341_detect_quirks<T: ControlTransfer>(control: &T) -> Result<(), T::Error> {
    let mut response = [0u8; 2];
    match ch341_control_in(
        control,
        CH341_REQ_READ_REG,
        u16::from(CH341_REG_BREAK),
        0,
        &mut response,
    ) {
        Ok(_) => Ok(()),
        Err(error) => {
            error!("ch34x: failed to read break control while detecting quirks");
            Err(error)
        }
    }
}

fn ch341_get_divisor(baud: u32) -> u16 {
    let speed = u64::from(baud.clamp(CH341_MIN_BPS, CH341_MAX_BPS));
    let mut fact = 1u8;
    let mut prescaler = 0u8;

    // Select the highest base clock whose minimum rate is below the request.
    for candidate in (0u8..=3).rev() {
        if speed > ch341_min_rate(candidate) {
            prescaler = candidate;
            break;
        }
    }

    let mut clock_divisor = ch341_clk_div(prescaler, fact);
    let mut divisor = u64::from(CH341_CLKRATE) / (clock_divisor * speed);

    // A fact=1 divisor must be in the 9..255 range. Fall back to fact=0 when
    // the initially selected clock would produce a divisor outside it.
    if !(9..=255).contains(&divisor) {
        divisor /= 2;
        clock_divisor *= 2;
        fact = 0;
    }

    // Pick the adjacent divisor when it gives a closer baud rate.
    let lower_error = 16 * u64::from(CH341_CLKRATE) / (clock_divisor * divisor) - 16 * speed;
    let upper_error = 16 * speed - 16 * u64::from(CH341_CLKRATE) / (clock_divisor * (divisor + 1));
    if lower_error >= upper_error {
        divisor += 1;
    }

    // Prefer the lower base clock when the divisor is even; this improves
    // receiver tolerance to baud-rate error.
    if fact == 1 && divisor.is_multiple_of(2) {
        divisor /= 2;
        fact = 0;
    }

    (((0x100 - divisor) << 8) | (u64::from(fact) << 2) | u64::from(prescaler)) as u16
}

fn ch341_clk_div(prescaler: u8, fact: u8) -> u64 {
    1u64 << (12 - 3 * u32::from(prescaler) - u32::from(fact))
}

fn ch341_min_rate(prescaler: u8) -> u64 {
    u64::from(CH341_CLKRATE) / (ch341_clk_div(prescaler, 1) * 512)
}

/// Sends a CH341 device-recipient vendor OUT request with no data stage.
pub fn ch341_control_out<T: ControlTransfer>(
    control: &T,
    request: u8,
    value: u16,
    index: u16,
) -> Result<usize, T::Error> {
    control.control_out(VENDOR_DEVICE_OUT, request, value, index, &mut [])
}

/// Sends a CH341 device-recipient vendor IN request.
pub fn ch341_control_in<T: ControlTransfer>(
    control: &T,
    request: u8,
    value: u16,
    index: u16,
    data: &mut [u8],
) -> Result<usize, T::Error> {
    control.control_in(VENDOR_DEVICE_IN, request, value, index, data)
}

fn is_data_interface(interface: &InterfaceDescriptor) -> bool {
    interface.alternate_setting == 0
        && interface.class == USB_CLASS_VENDOR_SPECIFIC
        && interface.subclass == 0x01
        && interface.protocol == 0x02
}

#[cfg(test)]
mod tests {
    use alloc::{vec, vec::Vec};
    use core::cell::RefCell;

    use super::*;

    struct VersionControl {
        requests: RefCell<Vec<(u8, u8, u16, u16)>>,
    }

    impl ControlTransfer for VersionControl {
        type Error = ();

        fn control_out(
            &self,
            request_type: u8,
            request: u8,
            value: u16,
            index: u16,
            _data: &mut [u8],
        ) -> Result<usize, Self::Error> {
            self.requests
                .borrow_mut()
                .push((request_type, request, value, index));
            Ok(0)
        }

        fn control_in(
            &self,
            request_type: u8,
            request: u8,
            value: u16,
            index: u16,
            data: &mut [u8],
        ) -> Result<usize, Self::Error> {
            self.requests
                .borrow_mut()
                .push((request_type, request, value, index));
            data.copy_from_slice(&[0x27, 0x00]);
            Ok(data.len())
        }
    }

    #[test]
    fn init_reads_version_with_a_device_vendor_request() {
        let control = VersionControl {
            requests: RefCell::new(Vec::new()),
        };
        let port = UsbSerialPort {
            interface: 0,
            bulk_in: 0x82,
            bulk_out: 0x02,
        };

        init(&control, &port, 115_200).unwrap();

        assert_eq!(
            control.requests.into_inner(),
            vec![
                (VENDOR_DEVICE_IN, CH341_REQ_READ_VERSION, 0, 0),
                (VENDOR_DEVICE_OUT, CH341_REQ_SERIAL_INIT, 0, 0),
                (
                    VENDOR_DEVICE_OUT,
                    CH341_REQ_WRITE_REG,
                    (u16::from(CH341_REG_DIVISOR) << 8) | u16::from(CH341_REG_PRESCALER),
                    0xcc03,
                ),
                (
                    VENDOR_DEVICE_IN,
                    CH341_REQ_READ_REG,
                    u16::from(CH341_REG_BREAK),
                    0,
                ),
            ]
        );
    }

    struct FailingControl;

    impl ControlTransfer for FailingControl {
        type Error = &'static str;

        fn control_out(
            &self,
            _request_type: u8,
            _request: u8,
            _value: u16,
            _index: u16,
            _data: &mut [u8],
        ) -> Result<usize, Self::Error> {
            Ok(0)
        }

        fn control_in(
            &self,
            _request_type: u8,
            _request: u8,
            _value: u16,
            _index: u16,
            _data: &mut [u8],
        ) -> Result<usize, Self::Error> {
            Err("break register is unavailable")
        }
    }

    #[test]
    fn detect_quirks_propagates_break_register_errors() {
        assert_eq!(
            ch341_detect_quirks(&FailingControl),
            Err("break register is unavailable")
        );
    }

    #[test]
    fn divisor_matches_linux_formula_for_115200_baud() {
        assert_eq!(ch341_get_divisor(115_200), 0xcc03);
    }

    #[test]
    fn divisor_clamps_outside_supported_baud_range() {
        assert_eq!(ch341_get_divisor(0), ch341_get_divisor(CH341_MIN_BPS));
        assert_eq!(
            ch341_get_divisor(u32::MAX),
            ch341_get_divisor(CH341_MAX_BPS)
        );
    }

    #[test]
    fn newer_version_writes_lcr_after_divisor() {
        let control = VersionControl {
            requests: RefCell::new(Vec::new()),
        };

        ch341_set_baudrate_lcr(&control, 0x30, 115_200, 0xc3).unwrap();

        assert_eq!(
            control.requests.into_inner(),
            vec![
                (
                    VENDOR_DEVICE_OUT,
                    CH341_REQ_WRITE_REG,
                    (u16::from(CH341_REG_DIVISOR) << 8) | u16::from(CH341_REG_PRESCALER),
                    0xcc83,
                ),
                (
                    VENDOR_DEVICE_OUT,
                    CH341_REQ_WRITE_REG,
                    (u16::from(CH341_REG_LCR2) << 8) | u16::from(CH341_REG_LCR),
                    0x00c3,
                ),
            ]
        );
    }
}
