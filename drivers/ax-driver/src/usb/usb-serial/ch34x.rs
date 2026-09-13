use crab_usb::{
    USBHost,
    driver::{UsbDriver, UsbId, UsbInterface},
    usb_if::err::USBError,
};

const USB_CLASS_VENDOR_SPECIFIC: u8 = 0xff;
const USB_SUBCLASS_CH34X: u8 = 0x01;
const USB_PROTOCOL_CH34X: u8 = 0x02;

struct Ch34xProbeDriver;

static CH34X_DRIVER: Ch34xProbeDriver = Ch34xProbeDriver;
static CH34X_IDS: &[UsbId] = &[UsbId {
    vid: Some(::usb_serial::ch34x::VENDOR_ID),
    pid: Some(::usb_serial::ch34x::PRODUCT_ID_CH340),
    class: Some(USB_CLASS_VENDOR_SPECIFIC),
    subclass: Some(USB_SUBCLASS_CH34X),
    protocol: Some(USB_PROTOCOL_CH34X),
}];

pub(super) fn register(host: &mut USBHost) {
    host.register_usb_driver(&CH34X_DRIVER, CH34X_IDS);
}

impl UsbDriver for Ch34xProbeDriver {
    fn name(&self) -> &'static str {
        "ch34x"
    }

    fn probe(&self, interface: &UsbInterface) -> Result<(), USBError> {
        if interface.alternate_setting() != 0
            || interface.class() != USB_CLASS_VENDOR_SPECIFIC
            || interface.subclass() != USB_SUBCLASS_CH34X
            || interface.protocol() != USB_PROTOCOL_CH34X
        {
            return Err(USBError::NotSupported);
        }

        log::info!(
            "usb-serial: ch34x recognized {:04x}:{:04x} interface {} alt {}",
            interface.device().vendor_id(),
            interface.device().product_id(),
            interface.interface_number(),
            interface.alternate_setting(),
        );
        Ok(())
    }
}
