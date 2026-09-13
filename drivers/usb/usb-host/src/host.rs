use alloc::{boxed::Box, collections::BTreeMap, sync::Arc, vec::Vec};

#[cfg(kmod)]
pub use super::backend::kmod::*;
#[cfg(umod)]
pub use super::backend::umod::*;
pub use crate::device::{Device, DeviceInfo, HubDeviceInfo, ProbeChanges, ProbedDevice};
use crate::{
    backend::{BackendOp, ty::*},
    driver::{UsbDriver, UsbDriverRegistry, UsbId, UsbInterface},
    err::Result,
};

/// USB 主机控制器
pub struct USBHost {
    pub(crate) backend: Box<dyn BackendOp>,
    pub(crate) initialized: bool,
    driver_registry: UsbDriverRegistry,
    bound_interfaces: BTreeMap<usize, Vec<Arc<UsbInterface>>>,
    connected_devices: BTreeMap<usize, DeviceInfo>,
}

impl USBHost {
    pub(crate) fn with_backend(backend: Box<dyn BackendOp>) -> Self {
        Self {
            backend,
            initialized: false,
            driver_registry: UsbDriverRegistry::default(),
            bound_interfaces: BTreeMap::new(),
            connected_devices: BTreeMap::new(),
        }
    }

    /// Registers static interface driver IDs for this host.
    ///
    /// Register drivers before the first [`Self::probe_changes`] call so each
    /// newly enumerated interface is considered exactly once.
    pub fn register_usb_driver(&mut self, driver: &'static dyn UsbDriver, ids: &'static [UsbId]) {
        self.driver_registry.register_usb_driver(driver, ids);
    }

    /// 初始化主机控制器
    pub async fn init(&mut self) -> Result<()> {
        if self.initialized {
            return Ok(());
        }
        self.backend.init().await?;
        self.initialized = true;
        Ok(())
    }

    #[cfg(any(kmod, umod))]
    pub async fn probe_devices(&mut self) -> Result<Vec<ProbedDevice>> {
        Ok(self.probe_changes().await?.connected)
    }

    #[cfg(any(kmod, umod))]
    /// Returns connection and disconnection transitions since the last scan.
    pub async fn probe_changes(&mut self) -> Result<ProbeChanges> {
        let changes = self.backend.device_list().await?;
        self.disconnect_removed_interfaces(&changes.disconnected);
        let mut connected = Vec::new();
        for dev in changes.connected {
            let dev_info = match dev {
                ProbedDeviceInfoOp::Device(inner) => {
                    let info = DeviceInfo::new(inner);
                    self.probe_device_interfaces(&info);
                    self.connected_devices.insert(info.id(), info.clone());
                    ProbedDevice::Device(info)
                }
                ProbedDeviceInfoOp::Hub(inner) => ProbedDevice::Hub(HubDeviceInfo { inner }),
            };
            connected.push(dev_info);
        }
        Ok(ProbeChanges {
            connected,
            disconnected: changes.disconnected,
        })
    }

    fn probe_device_interfaces(&mut self, device: &DeviceInfo) {
        let interfaces = device.interfaces().to_vec();
        for interface in &interfaces {
            self.driver_registry.probe_interface(interface);
        }
        self.bound_interfaces.insert(device.id(), interfaces);
    }

    fn disconnect_removed_interfaces(&mut self, disconnected: &[usize]) {
        for device_id in disconnected {
            self.connected_devices.remove(device_id);
            let Some(interfaces) = self.bound_interfaces.remove(device_id) else {
                continue;
            };
            self.driver_registry.disconnect_interfaces(&interfaces);
        }
    }

    /// Returns descriptions for devices retained in the connected USB topology.
    ///
    /// Unlike [`Self::probe_changes`], this does not consume a connection
    /// transition and remains valid after another subsystem handles hotplug.
    pub fn connected_devices(&self) -> impl Iterator<Item = &DeviceInfo> {
        self.connected_devices.values()
    }

    #[cfg(kmod)]
    pub fn create_event_handler(&mut self) -> EventHandler {
        let handler = self.backend.create_event_handler();
        EventHandler { handler }
    }

    pub fn enable_irq(&mut self) -> Result {
        self.backend.enable_irq()
    }

    pub fn disable_irq(&mut self) -> Result {
        self.backend.disable_irq()
    }

    #[cfg(kmod)]
    pub fn dwc2_transfer_stats(&self) -> Option<Dwc2TransferStats> {
        self.backend.dwc2_transfer_stats()
    }

    #[cfg(kmod)]
    pub fn reset_dwc2_transfer_stats(&self) {
        self.backend.reset_dwc2_transfer_stats();
    }

    pub async fn open_device(&mut self, dev: &DeviceInfo) -> Result<Device> {
        let device = self.backend.open_device(dev.inner.as_ref()).await?;
        let mut device: Device = device.into();
        device.init().await?;
        Ok(device)
    }
}

pub struct EventHandler {
    handler: Box<dyn EventHandlerOp>,
}

impl EventHandler {
    /// 处理事件
    pub fn handle_event(&self) -> Event {
        self.handler.handle_event()
    }
}

#[cfg(test)]
mod tests {
    use alloc::{sync::Arc, vec};
    use core::{
        future::Future,
        pin::Pin,
        ptr,
        sync::atomic::{AtomicUsize, Ordering},
        task::{Context, Poll, RawWaker, RawWakerVTable, Waker},
    };

    use futures::{FutureExt, future::LocalBoxFuture};
    use usb_if::{
        descriptor::{
            ConfigurationDescriptor, DeviceDescriptor, InterfaceDescriptor, InterfaceDescriptors,
        },
        err::USBError,
    };

    use super::*;
    use crate::backend::{
        BackendOp,
        ty::{DeviceInfoOp, DeviceOp, ProbeChangesOp, ProbedDeviceInfoOp},
    };

    #[derive(Default)]
    struct IrqCalls {
        init: AtomicUsize,
        enable: AtomicUsize,
        disable: AtomicUsize,
    }

    struct TestBackend {
        calls: Arc<IrqCalls>,
        connected: Vec<ProbedDeviceInfoOp>,
        disconnected: Vec<usize>,
    }

    impl BackendOp for TestBackend {
        fn init<'a>(&'a mut self) -> futures::future::BoxFuture<'a, crate::err::Result> {
            self.calls.init.fetch_add(1, Ordering::Relaxed);
            async { Ok(()) }.boxed()
        }

        #[cfg(any(kmod, umod))]
        fn device_list<'a>(
            &'a mut self,
        ) -> futures::future::BoxFuture<'a, crate::err::Result<ProbeChangesOp>> {
            let connected = core::mem::take(&mut self.connected);
            let disconnected = core::mem::take(&mut self.disconnected);
            async {
                Ok(ProbeChangesOp {
                    connected,
                    disconnected,
                })
            }
            .boxed()
        }

        fn open_device<'a>(
            &'a mut self,
            _dev: &'a dyn crate::backend::ty::DeviceInfoOp,
        ) -> LocalBoxFuture<'a, crate::err::Result<Box<dyn DeviceOp>>> {
            async { Err(USBError::NotSupported) }.boxed_local()
        }

        #[cfg(kmod)]
        fn create_event_handler(&mut self) -> Box<dyn crate::backend::ty::EventHandlerOp> {
            Box::new(TestEventHandler)
        }

        fn enable_irq(&mut self) -> crate::err::Result {
            self.calls.enable.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }

        fn disable_irq(&mut self) -> crate::err::Result {
            self.calls.disable.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }
    }

    #[cfg(kmod)]
    struct TestEventHandler;

    #[cfg(kmod)]
    impl crate::backend::ty::EventHandlerOp for TestEventHandler {
        fn handle_event(&self) -> crate::backend::ty::Event {
            crate::backend::ty::Event::Nothing
        }
    }

    #[derive(Debug)]
    struct TestDeviceInfo {
        descriptor: DeviceDescriptor,
        configurations: Vec<ConfigurationDescriptor>,
    }

    impl DeviceInfoOp for TestDeviceInfo {
        fn id(&self) -> usize {
            1
        }

        fn backend_name(&self) -> &str {
            "test"
        }

        fn descriptor(&self) -> &DeviceDescriptor {
            &self.descriptor
        }

        fn configuration_descriptors(&self) -> &[ConfigurationDescriptor] {
            &self.configurations
        }
    }

    struct TestUvcDriver;

    impl UsbDriver for TestUvcDriver {
        fn name(&self) -> &'static str {
            "test-uvc"
        }

        fn probe(&self, _interface: &UsbInterface) -> Result<()> {
            Ok(())
        }
    }

    static TEST_UVC_DRIVER: TestUvcDriver = TestUvcDriver;
    static TEST_UVC_IDS: &[UsbId] = &[UsbId {
        vid: None,
        pid: None,
        class: Some(0x0e),
        subclass: Some(0x01),
        protocol: None,
    }];

    fn block_on_ready<F: Future>(mut future: F) -> F::Output {
        let waker = noop_waker();
        let mut context = Context::from_waker(&waker);
        match unsafe { Pin::new_unchecked(&mut future) }.poll(&mut context) {
            Poll::Ready(output) => output,
            Poll::Pending => panic!("test future unexpectedly pending"),
        }
    }

    fn noop_waker() -> Waker {
        unsafe fn clone(_: *const ()) -> RawWaker {
            RawWaker::new(ptr::null(), &VTABLE)
        }
        unsafe fn wake(_: *const ()) {}
        unsafe fn wake_by_ref(_: *const ()) {}
        unsafe fn drop(_: *const ()) {}

        static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, wake, wake_by_ref, drop);

        unsafe { Waker::from_raw(RawWaker::new(ptr::null(), &VTABLE)) }
    }

    #[test]
    fn host_irq_control_forwards_to_backend() {
        let calls = Arc::new(IrqCalls::default());
        let mut host = USBHost::with_backend(Box::new(TestBackend {
            calls: calls.clone(),
            connected: Vec::new(),
            disconnected: Vec::new(),
        }));

        host.enable_irq().unwrap();
        host.disable_irq().unwrap();

        assert_eq!(calls.enable.load(Ordering::Relaxed), 1);
        assert_eq!(calls.disable.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn host_init_is_idempotent() {
        let calls = Arc::new(IrqCalls::default());
        let mut host = USBHost::with_backend(Box::new(TestBackend {
            calls: calls.clone(),
            connected: Vec::new(),
            disconnected: Vec::new(),
        }));

        block_on_ready(host.init()).unwrap();
        block_on_ready(host.init()).unwrap();

        assert_eq!(calls.init.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn host_preserves_disconnected_device_ids() {
        let mut host = USBHost::with_backend(Box::new(TestBackend {
            calls: Arc::new(IrqCalls::default()),
            connected: Vec::new(),
            disconnected: vec![7, 9],
        }));
        host.initialized = true;

        let changes = block_on_ready(host.probe_changes()).unwrap();

        assert!(changes.connected.is_empty());
        assert_eq!(changes.disconnected, vec![7, 9]);
    }

    #[test]
    fn probe_changes_binds_registered_driver_to_enumerated_interface() {
        let mut host = USBHost::with_backend(Box::new(TestBackend {
            calls: Arc::new(IrqCalls::default()),
            connected: vec![ProbedDeviceInfoOp::Device(Box::new(test_device_info()))],
            disconnected: Vec::new(),
        }));
        host.register_usb_driver(&TEST_UVC_DRIVER, TEST_UVC_IDS);

        let changes = block_on_ready(host.probe_changes()).unwrap();
        let ProbedDevice::Device(device) = changes.connected.first().unwrap() else {
            panic!("test backend must enumerate a non-hub device");
        };

        assert_eq!(device.interfaces().len(), 1);
        assert_eq!(device.interfaces()[0].driver_name(), Some("test-uvc"));
        assert_eq!(host.connected_devices().count(), 1);
        assert_eq!(host.connected_devices().next().unwrap().id(), device.id());

        let changes = block_on_ready(host.probe_changes()).unwrap();
        assert!(changes.connected.is_empty());
        assert_eq!(host.connected_devices().count(), 1);
    }

    fn test_device_info() -> TestDeviceInfo {
        TestDeviceInfo {
            descriptor: DeviceDescriptor {
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
            },
            configurations: vec![ConfigurationDescriptor {
                num_interfaces: 1,
                configuration_value: 1,
                attributes: 0x80,
                max_power: 50,
                string_index: None,
                string: None,
                interfaces: vec![InterfaceDescriptors {
                    interface_number: 0,
                    alt_settings: vec![InterfaceDescriptor {
                        interface_number: 0,
                        alternate_setting: 0,
                        class: 0x0e,
                        subclass: 0x01,
                        protocol: 0,
                        string_index: None,
                        string: None,
                        num_endpoints: 0,
                        endpoints: Vec::new(),
                    }],
                }],
                raw: Vec::new(),
            }],
        }
    }
}
