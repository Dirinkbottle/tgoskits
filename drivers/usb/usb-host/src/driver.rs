//! USB interface-driver matching and binding.

use alloc::{sync::Arc, vec::Vec};
use core::fmt;

use crossbeam::atomic::AtomicCell;
use usb_if::{
    descriptor::{DeviceDescriptor, EndpointDescriptor, InterfaceDescriptor},
    err::USBError,
};

/// A USB device and interface descriptor match rule.
///
/// Every `None` field is a wildcard.  Device fields match the parent device;
/// class fields match one interface alternate setting.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UsbId {
    pub vid: Option<u16>,
    pub pid: Option<u16>,
    pub class: Option<u8>,
    pub subclass: Option<u8>,
    pub protocol: Option<u8>,
}

impl UsbId {
    /// Returns whether this ID rule accepts `interface`.
    pub fn matches(&self, interface: &UsbInterface) -> bool {
        let device = interface.device();
        self.vid.is_none_or(|vid| vid == device.vendor_id())
            && self.pid.is_none_or(|pid| pid == device.product_id())
            && self.class.is_none_or(|class| class == interface.class())
            && self
                .subclass
                .is_none_or(|subclass| subclass == interface.subclass())
            && self
                .protocol
                .is_none_or(|protocol| protocol == interface.protocol())
    }
}

/// Immutable USB device descriptor data shared by all of its interfaces.
#[derive(Debug)]
pub struct UsbDevice {
    descriptor: DeviceDescriptor,
}

impl UsbDevice {
    pub(crate) fn new(descriptor: DeviceDescriptor) -> Self {
        Self { descriptor }
    }

    /// Returns the enumerated device descriptor.
    pub fn descriptor(&self) -> &DeviceDescriptor {
        &self.descriptor
    }

    /// Returns the USB vendor ID.
    pub fn vendor_id(&self) -> u16 {
        self.descriptor.vendor_id
    }

    /// Returns the USB product ID.
    pub fn product_id(&self) -> u16 {
        self.descriptor.product_id
    }
}

/// One enumerated USB interface alternate setting.
pub struct UsbInterface {
    device: Arc<UsbDevice>,
    descriptor: InterfaceDescriptor,
    driver: AtomicCell<Option<&'static dyn UsbDriver>>,
}

impl UsbInterface {
    pub(crate) fn new(device: Arc<UsbDevice>, descriptor: InterfaceDescriptor) -> Self {
        Self {
            device,
            descriptor,
            driver: AtomicCell::new(None),
        }
    }

    /// Returns the immutable descriptor data of the parent device.
    pub fn device(&self) -> &UsbDevice {
        &self.device
    }

    /// Returns the enumerated interface descriptor, including its endpoints.
    pub fn descriptor(&self) -> &InterfaceDescriptor {
        &self.descriptor
    }

    /// Returns the interface's endpoint descriptors.
    pub fn endpoints(&self) -> &[EndpointDescriptor] {
        &self.descriptor.endpoints
    }

    /// Returns the interface number.
    pub fn interface_number(&self) -> u8 {
        self.descriptor.interface_number
    }

    /// Returns the selected alternate setting.
    pub fn alternate_setting(&self) -> u8 {
        self.descriptor.alternate_setting
    }

    /// Returns the interface class.
    pub fn class(&self) -> u8 {
        self.descriptor.class
    }

    /// Returns the interface subclass.
    pub fn subclass(&self) -> u8 {
        self.descriptor.subclass
    }

    /// Returns the interface protocol.
    pub fn protocol(&self) -> u8 {
        self.descriptor.protocol
    }

    /// Returns the driver that successfully claimed this interface.
    pub fn driver(&self) -> Option<&'static dyn UsbDriver> {
        self.driver.load()
    }

    /// Returns the claiming driver's name, if the interface is bound.
    pub fn driver_name(&self) -> Option<&'static str> {
        self.driver().map(UsbDriver::name)
    }

    pub(crate) fn bind_driver(&self, driver: &'static dyn UsbDriver) -> bool {
        if self.driver.load().is_some() {
            return false;
        }
        self.driver.store(Some(driver));
        true
    }

    pub(crate) fn take_driver(&self) -> Option<&'static dyn UsbDriver> {
        self.driver.swap(None)
    }
}

impl fmt::Debug for UsbInterface {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("UsbInterface")
            .field("device", &self.device)
            .field("descriptor", &self.descriptor)
            .field("driver", &self.driver_name())
            .finish()
    }
}

/// A static USB interface driver.
///
/// `probe` receives descriptor data after an ID rule matched.  Returning an
/// error declines the interface and lets the registry try the next matching
/// entry.  Returning `Ok(())` binds this driver to the interface.
pub trait UsbDriver: Sync {
    /// Returns a stable name used for diagnostics and binding state.
    fn name(&self) -> &'static str;

    /// Validates an interface after its ID rule matched.
    ///
    /// # Errors
    ///
    /// Returning an error declines the interface and allows another matching
    /// driver to probe it.
    fn probe(&self, interface: &UsbInterface) -> Result<(), USBError>;

    /// Releases driver state when the bound interface disconnects.
    fn disconnect(&self, _interface: &UsbInterface) {}
}

/// One flattened USB ID rule and its owning driver.
pub struct UsbIdEntry {
    pub id: UsbId,
    pub driver: &'static dyn UsbDriver,
}

/// The interface-driver registry for one USB host.
///
/// The host owns this registry and probes it only while it has exclusive
/// access to enumeration state, so callbacks execute without a registry lock.
#[derive(Default)]
pub struct UsbDriverRegistry {
    entries: Vec<UsbIdEntry>,
}

impl UsbDriverRegistry {
    /// Registers every ID rule in `ids` for `driver`.
    pub fn register_usb_driver(&mut self, driver: &'static dyn UsbDriver, ids: &'static [UsbId]) {
        self.entries
            .extend(ids.iter().copied().map(|id| UsbIdEntry { id, driver }));
    }

    pub(crate) fn probe_interface(&self, interface: &Arc<UsbInterface>) {
        if interface.driver().is_some() {
            return;
        }

        for entry in &self.entries {
            if !entry.id.matches(interface) {
                continue;
            }

            info!(
                "usb: interface {} alt {} matched driver {}",
                interface.interface_number(),
                interface.alternate_setting(),
                entry.driver.name()
            );
            if let Err(error) = entry.driver.probe(interface) {
                debug!(
                    "usb: driver {} declined interface {} alt {}: {error:?}",
                    entry.driver.name(),
                    interface.interface_number(),
                    interface.alternate_setting(),
                );
                continue;
            }

            if interface.bind_driver(entry.driver) {
                info!(
                    "usb: interface {} alt {} claimed by {}",
                    interface.interface_number(),
                    interface.alternate_setting(),
                    entry.driver.name()
                );
            }
            return;
        }

        debug!(
            "usb: interface {} alt {} has no driver",
            interface.interface_number(),
            interface.alternate_setting(),
        );
    }

    pub(crate) fn disconnect_interfaces(&self, interfaces: &[Arc<UsbInterface>]) {
        for interface in interfaces {
            let Some(driver) = interface.take_driver() else {
                continue;
            };
            driver.disconnect(interface);
        }
    }
}

#[cfg(test)]
mod tests {
    use alloc::{sync::Arc, vec};
    use core::sync::atomic::{AtomicUsize, Ordering};

    use super::*;

    static REJECTED_PROBES: AtomicUsize = AtomicUsize::new(0);
    static CLAIMED_PROBES: AtomicUsize = AtomicUsize::new(0);
    static DISCONNECTS: AtomicUsize = AtomicUsize::new(0);

    struct RejectingVideoDriver;

    impl UsbDriver for RejectingVideoDriver {
        fn name(&self) -> &'static str {
            "rejecting-video"
        }

        fn probe(&self, _interface: &UsbInterface) -> Result<(), USBError> {
            REJECTED_PROBES.fetch_add(1, Ordering::Relaxed);
            Err(USBError::NotSupported)
        }
    }

    struct UvcDriver;

    impl UsbDriver for UvcDriver {
        fn name(&self) -> &'static str {
            "uvc"
        }

        fn probe(&self, _interface: &UsbInterface) -> Result<(), USBError> {
            CLAIMED_PROBES.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }

        fn disconnect(&self, _interface: &UsbInterface) {
            DISCONNECTS.fetch_add(1, Ordering::Relaxed);
        }
    }

    static REJECTING_VIDEO_DRIVER: RejectingVideoDriver = RejectingVideoDriver;
    static UVC_DRIVER: UvcDriver = UvcDriver;
    static VIDEO_CONTROL_ID: UsbId = UsbId {
        vid: None,
        pid: None,
        class: Some(0x0e),
        subclass: Some(0x01),
        protocol: None,
    };
    static VIDEO_CONTROL_IDS: &[UsbId] = &[VIDEO_CONTROL_ID];

    #[test]
    fn id_matches_device_and_interface_descriptor_fields() {
        let interface = test_video_control_interface();
        let exact = UsbId {
            vid: Some(0x1b17),
            pid: Some(0x0211),
            class: Some(0x0e),
            subclass: Some(0x01),
            protocol: Some(0),
        };

        assert!(exact.matches(&interface));
        assert!(
            !UsbId {
                vid: Some(0),
                ..exact
            }
            .matches(&interface)
        );
        assert!(
            !UsbId {
                pid: Some(0),
                ..exact
            }
            .matches(&interface)
        );
        assert!(
            !UsbId {
                class: Some(0),
                ..exact
            }
            .matches(&interface)
        );
        assert!(
            !UsbId {
                subclass: Some(0),
                ..exact
            }
            .matches(&interface)
        );
        assert!(
            !UsbId {
                protocol: Some(1),
                ..exact
            }
            .matches(&interface)
        );
    }

    #[test]
    fn binds_next_matching_driver_after_probe_rejects_interface() {
        REJECTED_PROBES.store(0, Ordering::Relaxed);
        CLAIMED_PROBES.store(0, Ordering::Relaxed);

        let interface = test_video_control_interface();
        let mut registry = UsbDriverRegistry::default();
        registry.register_usb_driver(&REJECTING_VIDEO_DRIVER, VIDEO_CONTROL_IDS);
        registry.register_usb_driver(&UVC_DRIVER, VIDEO_CONTROL_IDS);

        registry.probe_interface(&interface);

        assert_eq!(REJECTED_PROBES.load(Ordering::Relaxed), 1);
        assert_eq!(CLAIMED_PROBES.load(Ordering::Relaxed), 1);
        assert_eq!(interface.driver_name(), Some("uvc"));
    }

    #[test]
    fn disconnects_each_bound_interface_once() {
        DISCONNECTS.store(0, Ordering::Relaxed);

        let interface = test_video_control_interface();
        let mut registry = UsbDriverRegistry::default();
        registry.register_usb_driver(&UVC_DRIVER, VIDEO_CONTROL_IDS);
        registry.probe_interface(&interface);

        registry.disconnect_interfaces(&[interface.clone()]);
        registry.disconnect_interfaces(&[interface]);

        assert_eq!(DISCONNECTS.load(Ordering::Relaxed), 1);
    }

    fn test_video_control_interface() -> Arc<UsbInterface> {
        let device = Arc::new(UsbDevice::new(DeviceDescriptor {
            usb_version: 0x0200,
            class: 0,
            subclass: 0,
            protocol: 0,
            max_packet_size_0: 64,
            vendor_id: 0x1b17,
            product_id: 0x0211,
            device_version: 0x0100,
            manufacturer_string_index: None,
            product_string_index: None,
            serial_number_string_index: None,
            num_configurations: 1,
        }));
        Arc::new(UsbInterface::new(
            device,
            InterfaceDescriptor {
                interface_number: 0,
                alternate_setting: 0,
                class: 0x0e,
                subclass: 0x01,
                protocol: 0,
                string_index: None,
                string: None,
                num_endpoints: 1,
                endpoints: vec![],
            },
        ))
    }
}
