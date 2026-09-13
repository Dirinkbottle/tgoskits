//! xHCI Root Hub 实现
//!
//! 实现 xHCI 控制器的 Root Hub 功能，遵循 xHCI 规范第 4.19 章。

use alloc::{sync::Arc, vec::Vec};
use core::{
    cell::UnsafeCell,
    sync::atomic::{AtomicBool, Ordering},
};

use futures::{FutureExt, future::BoxFuture, task::AtomicWaker};
use usb_if::{err::USBError, host::hub::Speed};

use super::reg::{MemMapper, PortStatusRegisters, XhciRegisters};
use crate::backend::kmod::hub::{HubInfo, HubOp, PortChangeInfo, PortEvent, PortState};

pub struct PortChangeWaker {
    ports: Arc<UnsafeCell<Vec<Port>>>,
}

unsafe impl Send for PortChangeWaker {}
unsafe impl Sync for PortChangeWaker {}

impl PortChangeWaker {
    #[allow(clippy::arc_with_non_send_sync)]
    pub fn new(port_num: u8) -> Self {
        let mut ports = Vec::with_capacity(port_num as usize);
        for i in 0..port_num {
            ports.push(Port {
                port_id: i + 1,
                change_waker: AtomicWaker::new(),
                changed: AtomicBool::new(false),
                state: PortState::Uninit,
            });
        }
        Self {
            ports: Arc::new(UnsafeCell::new(ports)),
        }
    }

    pub fn set_port_changed(&self, port_id: u8) {
        let ports = unsafe { &*self.ports.get() };
        let idx = (port_id - 1) as usize;
        debug!("Setting port {} changed", port_id);
        ports[idx].changed.store(true, Ordering::Release);
        ports[idx].change_waker.wake();
    }
}

pub struct Port {
    port_id: u8,
    change_waker: AtomicWaker,
    changed: AtomicBool,
    state: PortState,
}

/// xHCI Root Hub
///
/// Root Hub 是集成在 xHCI 控制器中的虚拟 Hub。
pub struct XhciRootHub {
    portsc: PortStatusRegisters<MemMapper>,

    ports: Arc<UnsafeCell<Vec<Port>>>,
}

unsafe impl Send for XhciRootHub {}

impl XhciRootHub {
    fn ports(&self) -> &[Port] {
        unsafe { &*self.ports.get() }
    }

    fn ports_mut(&mut self) -> &mut [Port] {
        unsafe { &mut *self.ports.get() }
    }
}

impl HubOp for XhciRootHub {
    fn changed_ports(&mut self) -> BoxFuture<'_, Result<Vec<PortEvent>, USBError>> {
        self._changed_ports().boxed()
    }

    fn init(&mut self, info: HubInfo) -> BoxFuture<'_, Result<HubInfo, USBError>> {
        async {
            let mut info = info;
            info.speed = Speed::SuperSpeedPlus;
            debug!("Resetting all ports of xHCI Root Hub");

            for idx in 0..self.portsc.len() {
                self.portsc.update_volatile_at(idx, |portsc| {
                    if !portsc.port_power() {
                        trace!("Powering on port {}", idx + 1);
                        portsc.set_port_power();
                    }
                });
            }

            for idx in 0..self.portsc.len() {
                self.portsc.update_volatile_at(idx, |portsc| {
                    portsc.set_0_port_enabled_disabled();
                    portsc.set_port_reset();
                });
            }

            Ok(info)
        }
        .boxed()
    }

    fn slot_id(&self) -> u8 {
        0
    }
}

impl XhciRootHub {
    /// 创建新的 xHCI Root Hub
    pub fn new(reg: XhciRegisters) -> Result<Self, USBError> {
        let portsc = reg.port_status_registers();
        let port_num = portsc.len();
        let ports = PortChangeWaker::new(port_num as _).ports.clone();

        Ok(Self { portsc, ports })
    }

    pub fn waker(&self) -> PortChangeWaker {
        PortChangeWaker {
            ports: self.ports.clone(),
        }
    }

    async fn _changed_ports(&mut self) -> Result<Vec<PortEvent>, USBError> {
        let mut events = self.handle_disconnected();
        self.handle_uninit().await?;
        events.extend(
            self.handle_reseted()
                .await?
                .into_iter()
                .map(PortEvent::Connected),
        );
        self.acknowledge_port_changes();
        Ok(events)
    }

    fn acknowledge_port_changes(&mut self) {
        for index in 0..self.portsc.len() {
            let status = self.portsc.read_volatile_at(index);
            if status.port_reset() || status.warm_port_reset() {
                continue;
            }
            let has_change = status.connect_status_change()
                || status.port_enabled_disabled_change()
                || status.warm_port_reset_change()
                || status.over_current_change()
                || status.port_reset_change()
                || status.port_link_state_change()
                || status.port_config_error_change();
            if !has_change {
                continue;
            }
            self.portsc.update_volatile_at(index, |portsc| {
                // PORTSC change bits are RW1C and retain their read value in
                // this update, so the write acknowledges every reported
                // change. Do not disable an enabled port while acknowledging
                // them. Reset-in-progress ports are skipped above because the
                // xHCI register API intentionally exposes reset as write-one.
                portsc.set_0_port_enabled_disabled();
            });
        }
    }

    fn handle_disconnected(&mut self) -> Vec<PortEvent> {
        let statuses = (0..self.portsc.len())
            .map(|index| self.portsc.read_volatile_at(index).current_connect_status())
            .collect::<Vec<_>>();
        let mut events = Vec::new();
        for (index, connected) in statuses.into_iter().enumerate() {
            let port_id = index as u8 + 1;
            if let Some(event) = self.ports_mut()[index]
                .state
                .take_disconnect_event(connected, port_id)
            {
                events.push(event);
            }
        }
        events
    }

    async fn handle_uninit(&mut self) -> Result<(), USBError> {
        let uninited = self
            .ports()
            .iter()
            .filter(|port| matches!(port.state, PortState::Uninit))
            .map(|p| p.port_id)
            .collect::<Vec<_>>();

        for &id in &uninited {
            debug!("Waiting for port {id} reset ...");
            let i = (id - 1) as usize;

            let port = self.portsc.read_volatile_at(i);

            if !port.current_connect_status() {
                continue;
            }

            if port.port_reset() {
                self.ports_mut()[i].state = PortState::Reseted;
                continue;
            }

            debug!(
                "Port {} reset complete, enable={}, connect={}",
                id,
                port.port_enabled_disabled(),
                port.current_connect_status()
            );

            if port.port_enabled_disabled() {
                self.ports_mut()[i].state = PortState::Reseted;
                continue;
            }

            self.portsc.update_volatile_at(i, |portsc| {
                portsc.set_0_port_enabled_disabled();
                portsc.set_port_reset();
            });
            self.ports_mut()[i].state = PortState::Reseted;
        }

        Ok(())
    }

    async fn handle_reseted(&mut self) -> Result<Vec<PortChangeInfo>, USBError> {
        let reseted = self
            .ports()
            .iter()
            .filter(|port| matches!(port.state, PortState::Reseted))
            .map(|p| p.port_id)
            .collect::<Vec<_>>();

        let mut out = Vec::new();

        for &id in &reseted {
            let i = (id - 1) as usize;
            let portsc = self.portsc.read_volatile_at(i);
            if !portsc.current_connect_status() {
                self.ports_mut()[i].state = PortState::Uninit;
                continue;
            }
            if portsc.port_reset() {
                continue;
            }
            if !portsc.port_enabled_disabled() {
                self.ports_mut()[i].state = PortState::Uninit;
                continue;
            }
            let speed_raw = portsc.port_speed();
            let speed = Speed::from_xhci_portsc(speed_raw);
            debug!("Port {} device connected at speed {:?}", id, speed);
            debug!("Port {} : \r\n {:?}", id, portsc);
            self.ports_mut()[i].state = PortState::Probed;

            out.push(PortChangeInfo {
                root_port_id: id,
                port_id: id,
                port_speed: speed,
            });
        }

        Ok(out)
    }
}
