//! USB CDC ACM descriptor matching.
//!
//! The kernel-side transport is deliberately kept outside this crate.  This
//! module only turns the configuration descriptor into the interface and
//! endpoint numbers needed by an ACM transport.

use usb_if::{
    descriptor::{ConfigurationDescriptor, DeviceDescriptor, EndpointType, InterfaceDescriptor},
    transfer::Direction,
};

use crate::{ControlTransfer, UsbSerialPort};

pub const USB_CLASS_COMM: u8 = 0x02;
pub const USB_CDC_SUBCLASS_ACM: u8 = 0x02;
pub const USB_CLASS_CDC_DATA: u8 = 0x0a;

const CDC_REQUEST_SET_LINE_CODING: u8 = 0x20;
const CDC_REQUEST_TYPE_INTERFACE_OUT: u8 = 0x21;
const CDC_LINE_CODING_LEN: usize = 7;

/// Finds one CDC ACM function in a raw device/configuration descriptor blob.
///
/// The returned port contains the communication-class control interface, the
/// data-class interface, and the two bulk endpoints used for serial data.
pub fn probe(descriptor_blob: &[u8]) -> Option<UsbSerialPort> {
    let mut rest = descriptor_blob.get(DeviceDescriptor::LEN..)?;
    while !rest.is_empty() {
        let configuration = ConfigurationDescriptor::parse(rest)?;
        if let Some(port) = find_port_in_configuration(&configuration) {
            return Some(port);
        }
        let consumed = configuration.raw.len();
        if consumed == 0 || consumed > rest.len() {
            return None;
        }
        rest = &rest[consumed..];
    }
    None
}

fn find_port_in_configuration(configuration: &ConfigurationDescriptor) -> Option<UsbSerialPort> {
    // CDC ACM keeps class-specific control requests on the communication
    // interface.  Its interrupt endpoint carries notifications; data itself
    // travels through the separate data interface below.
    let control = configuration
        .interfaces
        .iter()
        .flat_map(|group| group.alt_settings.iter())
        .find(|interface| is_control_interface(interface))?;
    if !has_interrupt_in(control) {
        return None;
    }

    // The data interface must provide one bulk endpoint in each direction.
    let (data_interface, bulk_in, bulk_out) = configuration
        .interfaces
        .iter()
        .flat_map(|group| group.alt_settings.iter())
        .filter(|interface| is_data_interface(interface))
        .find_map(|interface| {
            let (bulk_in, bulk_out) = bulk_pair(interface)?;
            Some((interface.interface_number, bulk_in, bulk_out))
        })?;

    Some(UsbSerialPort {
        control_interface: control.interface_number,
        data_interface,
        bulk_in,
        bulk_out,
    })
}

fn is_control_interface(interface: &InterfaceDescriptor) -> bool {
    interface.alternate_setting == 0
        && interface.class == USB_CLASS_COMM
        && interface.subclass == USB_CDC_SUBCLASS_ACM
}

fn has_interrupt_in(interface: &InterfaceDescriptor) -> bool {
    interface.endpoints.iter().any(|endpoint| {
        endpoint.transfer_type == EndpointType::Interrupt && endpoint.direction == Direction::In
    })
}

fn is_data_interface(interface: &InterfaceDescriptor) -> bool {
    interface.alternate_setting == 0 && interface.class == USB_CLASS_CDC_DATA
}

/// Initializes the CDC ACM line using the default 8N1 format.
pub fn init<T: ControlTransfer>(
    control: &T,
    port: &UsbSerialPort,
    baudrate: u32,
) -> Result<(), T::Error> {
    set_line_coding(control, port, baudrate)
}

/// Updates the CDC ACM baud rate and line format.
///
/// CDC ACM uses the same `SET_LINE_CODING` request during initial attach and
/// later termios changes.  Keeping this operation under one descriptive name
/// avoids treating it as a separate device-specific baud register write.
pub fn set_line_coding<T: ControlTransfer>(
    control: &T,
    port: &UsbSerialPort,
    baudrate: u32,
) -> Result<(), T::Error> {
    let mut line_coding = [0u8; CDC_LINE_CODING_LEN];
    line_coding[..4].copy_from_slice(&baudrate.to_le_bytes());
    // bCharFormat=0 (one stop bit), bParityType=0 (none), bDataBits=8.
    line_coding[4..].copy_from_slice(&[0, 0, 8]);
    control.control_out(
        CDC_REQUEST_TYPE_INTERFACE_OUT,
        CDC_REQUEST_SET_LINE_CODING,
        0,
        u16::from(port.control_interface),
        &mut line_coding,
    )?;
    Ok(())
}

fn bulk_pair(interface: &InterfaceDescriptor) -> Option<(u8, u8)> {
    let bulk_in = interface
        .endpoints
        .iter()
        .find(|endpoint| {
            endpoint.transfer_type == EndpointType::Bulk && endpoint.direction == Direction::In
        })
        .map(|endpoint| endpoint.address)?;
    let bulk_out = interface
        .endpoints
        .iter()
        .find(|endpoint| {
            endpoint.transfer_type == EndpointType::Bulk && endpoint.direction == Direction::Out
        })
        .map(|endpoint| endpoint.address)?;
    Some((bulk_in, bulk_out))
}
