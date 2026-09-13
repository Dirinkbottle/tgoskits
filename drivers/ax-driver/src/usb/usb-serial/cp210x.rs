use crab_usb::{
    USBHost,
    driver::{UsbDriver, UsbId, UsbInterface},
    usb_if::err::USBError,
};

const USB_CLASS_VENDOR_SPECIFIC: u8 = 0xff;
const USB_SUBCLASS_CP210X: u8 = 0x00;
const USB_PROTOCOL_CP210X: u8 = 0x00;

struct Cp210xProbeDriver;

static CP210X_DRIVER: Cp210xProbeDriver = Cp210xProbeDriver;
static CP210X_IDS: &[UsbId] = &[UsbId {
    vid: Some(::usb_serial::cp210x::VENDOR_ID),
    pid: Some(::usb_serial::cp210x::PRODUCT_ID_EA60),
    class: Some(USB_CLASS_VENDOR_SPECIFIC),
    subclass: Some(USB_SUBCLASS_CP210X),
    protocol: Some(USB_PROTOCOL_CP210X),
}];

pub(super) fn register(host: &mut USBHost) {
    host.register_usb_driver(&CP210X_DRIVER, CP210X_IDS);
}

impl UsbDriver for Cp210xProbeDriver {
    fn name(&self) -> &'static str {
        "cp210x"
    }

    fn probe(&self, interface: &UsbInterface) -> Result<(), USBError> {
        if interface.alternate_setting() != 0
            || interface.class() != USB_CLASS_VENDOR_SPECIFIC
            || interface.subclass() != USB_SUBCLASS_CP210X
            || interface.protocol() != USB_PROTOCOL_CP210X
        {
            return Err(USBError::NotSupported);
        }

        log::info!(
            "usb-serial: cp210x recognized {:04x}:{:04x} interface {} alt {}",
            interface.device().vendor_id(),
            interface.device().product_id(),
            interface.interface_number(),
            interface.alternate_setting(),
        );
        Ok(())
    }
}
