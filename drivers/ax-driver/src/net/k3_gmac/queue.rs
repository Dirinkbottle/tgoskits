//! `rdif-eth` ownership split for the K3 GMAC.
//!
//! The complete device is split once into task-context queue/control objects
//! and a move-only hard-IRQ endpoint. Queue polling owns descriptor reclaim;
//! the hard-IRQ path only acknowledges the source and returns a bounded
//! [`NetIrqSnapshot`].

use alloc::{boxed::Box, sync::Arc, vec};

use ax_sync::SpinLock;
use rd_net::{
    DmaBuffer, FixedNetControl, IRxQueue, ITxQueue, NetDevice, NetDeviceInfo, NetDeviceParts,
    NetError, NetHardIrqEndpoint, NetHardIrqHandler, NetHardIrqResult, NetIrqSnapshot,
    NetIrqSourceId, NetPollGroupId, NetPollGroupParts, NetPollIrqControl, NetQueueId,
    NetQueuePairParts, NetRearmResult, QueueConfig, RxCompletion, SubmitError,
};

use super::core::{BUFFER_SIZE, DMA_ALIGN, DMA_MASK, K3GmacCore, QUEUE_ID, QUEUE_SIZE, SharedCore};

const GROUP_ID: NetPollGroupId = NetPollGroupId::new(0);
const IRQ_SOURCE: NetIrqSourceId = NetIrqSourceId::new(0);
const NET_QUEUE_ID: NetQueueId = NetQueueId::new(QUEUE_ID as u16);

/// Portable `rdif-eth` device wrapper around the K3 GMAC core.
pub struct K3GmacNet {
    inner: SharedCore,
    mac: [u8; 6],
}

impl K3GmacNet {
    pub fn new(core: K3GmacCore) -> Self {
        let mac = core.mac_address();
        Self {
            inner: Arc::new(SpinLock::new(core)),
            mac,
        }
    }
}

impl rdrive::DriverGeneric for K3GmacNet {
    fn name(&self) -> &str {
        super::DRIVER_NAME
    }
}

impl NetDevice for K3GmacNet {
    fn into_parts(self: Box<Self>) -> Result<NetDeviceParts, NetError> {
        let Self { inner, mac } = *self;
        Ok(NetDeviceParts {
            info: NetDeviceInfo::new(super::DRIVER_NAME, mac),
            control: Box::new(FixedNetControl::new(mac)),
            wifi_control: None,
            poll_groups: vec![NetPollGroupParts {
                id: GROUP_ID,
                queues: NetQueuePairParts {
                    tx: Box::new(K3GmacTxQueue {
                        inner: Arc::clone(&inner),
                    }),
                    rx: Box::new(K3GmacRxQueue {
                        inner: Arc::clone(&inner),
                    }),
                },
                irq_control: Box::new(K3GmacIrqControl {
                    inner: Arc::clone(&inner),
                }),
                owner_startup: None,
                irq_endpoints: vec![NetHardIrqEndpoint::new(
                    IRQ_SOURCE,
                    Box::new(K3GmacIrqHandler { inner }),
                )],
            }],
        })
    }
}

struct K3GmacIrqControl {
    inner: SharedCore,
}

impl NetPollIrqControl for K3GmacIrqControl {
    fn quiesce(&mut self) -> Result<(), NetError> {
        self.inner.lock().disable_irq();
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NetError> {
        self.inner.lock().shutdown()
    }

    fn rearm_and_check(&mut self, _now_nanos: u64) -> Result<NetRearmResult, NetError> {
        let pending = self.inner.lock().rearm_and_snapshot();
        if pending == NetIrqSnapshot::empty() {
            Ok(NetRearmResult::Idle)
        } else {
            Ok(NetRearmResult::WorkPending(pending))
        }
    }
}

/// Hard IRQ endpoint: acknowledge and classify only; no descriptor reclaim or
/// buffer ownership transfer is performed in interrupt context.
struct K3GmacIrqHandler {
    inner: SharedCore,
}

impl NetHardIrqHandler for K3GmacIrqHandler {
    fn handle_irq(&mut self) -> NetHardIrqResult {
        let Some(mut core) = self.inner.try_lock() else {
            return NetHardIrqResult::ProbeDeferred;
        };
        let snapshot = core.handle_irq();
        if snapshot == NetIrqSnapshot::empty() {
            NetHardIrqResult::Spurious
        } else {
            NetHardIrqResult::Schedule(snapshot)
        }
    }
}

struct K3GmacTxQueue {
    inner: SharedCore,
}

impl ITxQueue for K3GmacTxQueue {
    fn id(&self) -> NetQueueId {
        NET_QUEUE_ID
    }

    fn config(&self) -> QueueConfig {
        queue_config()
    }

    fn submit(&mut self, buffer: DmaBuffer) -> Result<(), SubmitError> {
        self.inner.lock().submit_tx(buffer)
    }

    fn reclaim(&mut self) -> Option<DmaBuffer> {
        self.inner.lock().reclaim_tx_buffer()
    }
}

struct K3GmacRxQueue {
    inner: SharedCore,
}

impl IRxQueue for K3GmacRxQueue {
    fn id(&self) -> NetQueueId {
        NET_QUEUE_ID
    }

    fn config(&self) -> QueueConfig {
        queue_config()
    }

    fn submit(&mut self, buffer: DmaBuffer) -> Result<(), SubmitError> {
        self.inner.lock().submit_rx(buffer)
    }

    fn reclaim(&mut self) -> Option<RxCompletion> {
        self.inner.lock().reclaim_rx_buffer()
    }
}

fn queue_config() -> QueueConfig {
    QueueConfig {
        dma_mask: DMA_MASK,
        align: DMA_ALIGN,
        buf_size: BUFFER_SIZE,
        ring_size: QUEUE_SIZE,
    }
}
