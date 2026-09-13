use alloc::{boxed::Box, collections::btree_map::BTreeMap, vec, vec::Vec};

use futures::{
    FutureExt,
    future::{BoxFuture, LocalBoxFuture},
};
use usb_if::{
    descriptor::{ConfigurationDescriptor, DeviceDescriptor},
    err::USBError,
};

use super::osal::Kernel;
use crate::{
    DeviceAddressInfo,
    backend::{
        BackendOp,
        kmod::hub::{Hub, HubDevice, HubId, HubInfo, HubOp, PortChangeInfo, PortEvent},
        ty::{DeviceInfoOp, DeviceOp, EventHandlerOp, ProbeChangesOp, ProbedDeviceInfoOp},
    },
};

pub trait CoreOp: Send + 'static {
    /// 初始化后端
    fn init<'a>(&'a mut self) -> BoxFuture<'a, Result<(), USBError>>;

    fn root_hub(&mut self) -> Box<dyn HubOp>;

    fn new_addressed_device<'a>(
        &'a mut self,
        addr: DeviceAddressInfo,
    ) -> BoxFuture<'a, Result<Box<dyn DeviceOp>, USBError>>;

    fn create_event_handler(&mut self) -> Box<dyn EventHandlerOp>;

    fn enable_irq(&mut self) -> Result<(), USBError> {
        Err(USBError::NotSupported)
    }

    fn disable_irq(&mut self) -> Result<(), USBError> {
        Err(USBError::NotSupported)
    }

    fn dwc2_transfer_stats(&self) -> Option<crate::Dwc2TransferStats> {
        None
    }

    fn reset_dwc2_transfer_stats(&self) {}

    fn kernel(&self) -> &Kernel;
}

pub struct Core {
    pub(crate) backend: Box<dyn CoreOp>,
    hubs: BTreeMap<HubId, Hub>,
    root_hub: Option<HubId>,
    topology: BTreeMap<(HubId, u8), TopologyDevice>,
    inited_devices: BTreeMap<usize, Box<dyn DeviceOp>>,
    next_hub_id: usize,
    next_device_id: usize,
}

#[derive(Clone, Copy)]
struct TopologyDevice {
    device_id: usize,
    child_hub: Option<HubId>,
}

impl Core {
    pub(crate) fn new(backend: impl CoreOp) -> Self {
        Self {
            root_hub: None,
            backend: Box::new(backend),
            hubs: BTreeMap::new(),
            topology: BTreeMap::new(),
            inited_devices: BTreeMap::new(),
            next_hub_id: 1,
            next_device_id: 1,
        }
    }

    fn hub_infos(&self) -> BTreeMap<HubId, HubInfo> {
        self.hubs
            .iter()
            .map(|(id, hub)| (*id, hub.info.clone()))
            .collect()
    }

    fn allocate_hub_id(&mut self) -> HubId {
        let id = HubId::new(self.next_hub_id);
        self.next_hub_id += 1;
        id
    }

    fn allocate_device_id(&mut self) -> usize {
        let id = self.next_device_id;
        self.next_device_id += 1;
        id
    }

    async fn _probe_devices(&mut self) -> Result<(bool, ProbeChangesOp), USBError> {
        let mut is_have_new_hub = false;
        let mut connected = Vec::new();
        let mut disconnected = Vec::new();

        let hub_ids = self.hubs.keys().copied().collect::<Vec<_>>();
        trace!("[usb-probe] enter: hubs={}", hub_ids.len());

        for id in hub_ids {
            if !self.hubs.contains_key(&id) {
                continue;
            }
            trace!("[usb-probe] hub {id:?}: changed_ports begin");
            let events = self.hub_changed_ports(id).await.map_err(|error| {
                trace!("[usb-probe] hub {id:?}: changed_ports failed: {error:?}");
                error
            })?;
            if events.is_empty() {
                trace!("[usb-probe] hub {id:?}: changed_ports complete, changes=0");
            } else {
                trace!(
                    "[usb-probe] hub {id:?}: changed_ports complete, changes={}",
                    events.len()
                );
            }
            for event in events {
                match event {
                    PortEvent::Connected(addr_info) => {
                        trace!(
                            "[usb-probe] address device begin: root_port={}, parent={id:?}, \
                             port={}, speed={:?}",
                            addr_info.root_port_id, addr_info.port_id, addr_info.port_speed
                        );
                        let (device, added_hub) = self.connect_port(id, addr_info).await?;
                        connected.push(device);
                        is_have_new_hub |= added_hub;
                    }
                    PortEvent::Disconnected { port_id } => {
                        trace!("[usb-probe] disconnect begin: parent={id:?}, port={port_id}");
                        let mut removed = self.disconnect_port(id, port_id).await?;
                        trace!(
                            "[usb-probe] disconnect complete: parent={id:?}, port={port_id}, \
                             devices={removed:?}"
                        );
                        disconnected.append(&mut removed);
                    }
                }
            }
        }

        if connected.is_empty() && disconnected.is_empty() {
            trace!("[usb-probe] pass complete: no topology changes");
        } else {
            trace!(
                "[usb-probe] pass complete: new_hub={is_have_new_hub}, discovered={}, removed={}",
                connected.len(),
                disconnected.len()
            );
        }
        Ok((
            is_have_new_hub,
            ProbeChangesOp {
                connected,
                disconnected,
            },
        ))
    }

    async fn connect_port(
        &mut self,
        parent_hub: HubId,
        address: PortChangeInfo,
    ) -> Result<(ProbedDeviceInfoOp, bool), USBError> {
        if self.topology.contains_key(&(parent_hub, address.port_id)) {
            return Err(USBError::from("USB topology port is already occupied"));
        }
        let parent_slot_id = self
            .hubs
            .get(&parent_hub)
            .ok_or(USBError::NotFound)?
            .backend
            .slot_id();
        let device = self
            .backend
            .new_addressed_device(DeviceAddressInfo {
                root_port_id: address.root_port_id,
                port_speed: address.port_speed,
                parent_hub: Some(parent_hub),
                port_id: address.port_id,
                infos: self.hub_infos(),
            })
            .await
            .map_err(|error| {
                trace!(
                    "[usb-probe] address device failed: root_port={}, parent={parent_hub:?}, \
                     port={}, speed={:?}, error={error:?}",
                    address.root_port_id, address.port_id, address.port_speed
                );
                error
            })?;
        let device_id = self.allocate_device_id();
        trace!(
            "[usb-probe] address device complete: logical_id={device_id}, backend_id={}",
            device.id()
        );
        let desc = device.descriptor().clone();
        let configs = device.configuration_descriptors().to_vec();

        if let Some(settings) =
            HubDevice::is_hub(device.descriptor(), device.configuration_descriptors())
        {
            let hub_device = HubDevice::new(
                device.into(),
                settings,
                address.root_port_id,
                parent_slot_id,
                self.backend.kernel(),
            )
            .await?;
            let mut hub = Hub::new(
                Box::new(hub_device),
                &self.hub_infos(),
                address.port_id,
                Some(parent_hub),
            );
            hub.info = hub.backend.init(hub.info.clone()).await?;
            let hub_id = self.allocate_hub_id();
            self.hubs.insert(hub_id, hub);
            self.topology.insert(
                (parent_hub, address.port_id),
                TopologyDevice {
                    device_id,
                    child_hub: Some(hub_id),
                },
            );
            trace!("[usb-probe] added external hub {hub_id:?}, device_id={device_id}");
            Ok((
                ProbedDeviceInfoOp::Hub(Box::new(DeviceInfo::new(device_id, desc, &configs))),
                true,
            ))
        } else {
            self.inited_devices.insert(device_id, device);
            self.topology.insert(
                (parent_hub, address.port_id),
                TopologyDevice {
                    device_id,
                    child_hub: None,
                },
            );
            Ok((
                ProbedDeviceInfoOp::Device(Box::new(DeviceInfo::new(device_id, desc, &configs))),
                false,
            ))
        }
    }

    async fn disconnect_port(
        &mut self,
        parent_hub: HubId,
        port_id: u8,
    ) -> Result<Vec<usize>, USBError> {
        let root_key = (parent_hub, port_id);
        let Some(root) = self.topology.get(&root_key).copied() else {
            return Ok(Vec::new());
        };

        let mut subtree = vec![(root_key, root)];
        let mut cursor = 0;
        while cursor < subtree.len() {
            if let Some(child_hub) = subtree[cursor].1.child_hub {
                let children = self
                    .topology
                    .iter()
                    .filter_map(|(key @ (owner, _), device)| {
                        (*owner == child_hub).then_some((*key, *device))
                    })
                    .collect::<Vec<_>>();
                subtree.extend(children);
            }
            cursor += 1;
        }

        let mut disconnected = Vec::with_capacity(subtree.len());
        for (key, device) in subtree.into_iter().rev() {
            if let Some(hub_id) = device.child_hub {
                if let Some(hub) = self.hubs.get_mut(&hub_id) {
                    hub.backend.disconnect().await?;
                }
                self.hubs.remove(&hub_id);
            } else if let Some(unopened) = self.inited_devices.get_mut(&device.device_id) {
                unopened.disconnect().await?;
                self.inited_devices.remove(&device.device_id);
            }
            self.topology.remove(&key);
            disconnected.push(device.device_id);
        }
        Ok(disconnected)
    }

    async fn hub_changed_ports(&mut self, hub_id: HubId) -> Result<Vec<PortEvent>, USBError> {
        let hub = self.hubs.get_mut(&hub_id).ok_or(USBError::NotFound)?;
        hub.backend.changed_ports().await
    }

    async fn probe_devices(&mut self) -> Result<ProbeChangesOp, USBError> {
        let mut result = ProbeChangesOp {
            connected: Vec::new(),
            disconnected: Vec::new(),
        };

        loop {
            let (is_have_new_hub, mut changes) = self._probe_devices().await?;
            result.connected.append(&mut changes.connected);
            result.disconnected.append(&mut changes.disconnected);
            if !is_have_new_hub {
                break;
            }
        }
        Ok(result)
    }
}

impl BackendOp for Core {
    fn init<'a>(&'a mut self) -> BoxFuture<'a, Result<(), USBError>> {
        async {
            self.backend.init().await?;
            let mut root_hub = Hub::new(self.backend.root_hub(), &self.hub_infos(), 0, None);
            let info = root_hub.backend.init(root_hub.info.clone()).await?;
            root_hub.info = info;

            let id = self.allocate_hub_id();
            self.hubs.insert(id, root_hub);
            self.root_hub = Some(id);
            Ok(())
        }
        .boxed()
    }

    fn device_list<'a>(&'a mut self) -> BoxFuture<'a, Result<ProbeChangesOp, USBError>> {
        self.probe_devices().boxed()
    }

    fn open_device<'a>(
        &'a mut self,
        dev: &'a dyn crate::backend::ty::DeviceInfoOp,
    ) -> LocalBoxFuture<'a, Result<Box<dyn DeviceOp>, USBError>> {
        async {
            self.inited_devices
                .remove(&dev.id())
                .ok_or(USBError::NotFound)
        }
        .boxed()
    }

    fn create_event_handler(&mut self) -> Box<dyn EventHandlerOp> {
        self.backend.create_event_handler()
    }

    fn enable_irq(&mut self) -> Result<(), USBError> {
        self.backend.enable_irq()
    }

    fn disable_irq(&mut self) -> Result<(), USBError> {
        self.backend.disable_irq()
    }

    fn dwc2_transfer_stats(&self) -> Option<crate::Dwc2TransferStats> {
        self.backend.dwc2_transfer_stats()
    }

    fn reset_dwc2_transfer_stats(&self) {
        self.backend.reset_dwc2_transfer_stats();
    }
}

#[derive(Debug, Clone)]
pub struct DeviceInfo {
    id: usize,
    desc: DeviceDescriptor,
    config_desc: Vec<ConfigurationDescriptor>,
}

impl DeviceInfo {
    pub fn new(id: usize, desc: DeviceDescriptor, config_desc: &[ConfigurationDescriptor]) -> Self {
        Self {
            id,
            desc,
            config_desc: config_desc.to_vec(),
        }
    }
}

impl DeviceInfoOp for DeviceInfo {
    fn id(&self) -> usize {
        self.id
    }

    fn backend_name(&self) -> &str {
        "kernel"
    }

    fn descriptor(&self) -> &DeviceDescriptor {
        &self.desc
    }

    fn configuration_descriptors(&self) -> &[ConfigurationDescriptor] {
        &self.config_desc
    }
}
