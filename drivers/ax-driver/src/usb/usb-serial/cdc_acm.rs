use crab_usb::{
    USBHost,
    driver::{UsbDriver, UsbId, UsbInterface},
    usb_if::{descriptor::EndpointType, err::USBError, transfer::Direction},
};

static CDC_ACM_DRIVER: CdcAcmProbeDriver = CdcAcmProbeDriver;
static CDC_ACM_IDS: &[UsbId] = &[UsbId {
    vid: None,
    pid: None,
    class: Some(::usb_serial::cdc_acm::USB_CLASS_COMM),
    subclass: Some(::usb_serial::cdc_acm::USB_CDC_SUBCLASS_ACM),
    protocol: None,
}];

struct CdcAcmProbeDriver;

pub(super) fn register(host: &mut USBHost) {
    host.register_usb_driver(&CDC_ACM_DRIVER, CDC_ACM_IDS);
}

impl UsbDriver for CdcAcmProbeDriver {
    fn name(&self) -> &'static str {
        "cdc-acm"
    }

    fn probe(&self, interface: &UsbInterface) -> Result<(), USBError> {
        let is_control_interface = interface.alternate_setting() == 0
            && interface.class() == ::usb_serial::cdc_acm::USB_CLASS_COMM
            && interface.subclass() == ::usb_serial::cdc_acm::USB_CDC_SUBCLASS_ACM;
        let has_interrupt_in = interface.endpoints().iter().any(|endpoint| {
            endpoint.transfer_type == EndpointType::Interrupt && endpoint.direction == Direction::In
        });

        if !is_control_interface || !has_interrupt_in {
            return Err(USBError::NotSupported);
        }

        log::info!(
            "usb-serial: cdc-acm recognized {:04x}:{:04x} interface {} alt {}",
            interface.device().vendor_id(),
            interface.device().product_id(),
            interface.interface_number(),
            interface.alternate_setting(),
        );
        Ok(())
    }
}
