use alloc::{
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
};

use xhci::ring::trb::event::{CompletionCode, TransferEvent};

use super::{reg::XhciRegistersShared, ring::SendRing, sync::IrqLock};
use crate::{BusAddr, queue::Finished};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TransferId(pub(crate) BusAddr);

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Clone, Copy)]
pub struct TransQueueId {
    slot_id: u8,
    ep_id: u8,
}

#[derive(Clone)]
pub struct TransferResultHandler {
    inner: Arc<IrqLock<TransferRoutes>>,
}

#[derive(Default)]
struct TransferRoutes {
    queues: BTreeMap<TransQueueId, Finished<TransferEvent>>,
    enumerating_slots: BTreeSet<u8>,
}

unsafe impl Send for TransferResultHandler {}

impl TransferResultHandler {
    pub fn new(reg: XhciRegistersShared) -> Self {
        Self {
            inner: Arc::new(IrqLock::new(TransferRoutes::default(), reg)),
        }
    }

    pub fn register_queue(&mut self, slot_id: u8, ep_id: u8, ring: &SendRing<TransferEvent>) {
        let id = TransQueueId { slot_id, ep_id };
        let handle = ring.finished_handle();
        self.inner.lock().queues.insert(id, handle);
    }

    pub fn set_enumeration_logging(&self, slot_id: u8, enabled: bool) {
        let mut routes = self.inner.lock();
        if enabled {
            routes.enumerating_slots.insert(slot_id);
        } else {
            routes.enumerating_slots.remove(&slot_id);
        }
    }

    pub fn unregister_slot(&self, slot_id: u8) {
        let mut routes = self.inner.lock();
        routes
            .queues
            .retain(|queue_id, _| queue_id.slot_id != slot_id);
        routes.enumerating_slots.remove(&slot_id);
    }

    /// Marks a queue completion from the xHCI interrupt path.
    ///
    /// This runs while handling an interrupt, so it must not acquire OS-facing
    /// locks or call into device/file managers. Queue registration is protected
    /// by `IrqLock::lock`, which disables this interrupt source before mutating
    /// the map. The IRQ hot path uses `force_use` and only touches the
    /// pre-registered queue completion slot, then wakes queue-local waiters.
    pub unsafe fn set_finished(&self, slot_id: u8, ep_id: u8, ptr: BusAddr, res: TransferEvent) {
        // xHCI reports ISO ring underrun/overrun when the periodic ring is
        // empty. Linux treats these as ring xrun events, not TD completions.
        if is_iso_ring_xrun(res) {
            trace!(
                "xhci: ignore ISO ring xrun event slot={} ep={} ptr={:#x} code={:?}",
                slot_id,
                ep_id,
                ptr.raw(),
                res.completion_code()
            );
            return;
        }

        let queue_id = TransQueueId { slot_id, ep_id };
        let routes = unsafe { self.inner.force_use() };
        if let Some(q) = routes.queues.get(&queue_id) {
            if ep_id == 1 && routes.enumerating_slots.contains(&slot_id) {
                trace!(
                    "[xhci-enum] control transfer event: slot={} ep={} trb={:#x} code={:?} \
                     remaining={}",
                    slot_id,
                    ep_id,
                    ptr.raw(),
                    res.completion_code(),
                    res.trb_transfer_length()
                );
            } else {
                trace!(
                    "xhci: dispatch transfer event slot={} ep={} ptr={:#x} code={:?} len={}",
                    slot_id,
                    ep_id,
                    ptr.raw(),
                    res.completion_code(),
                    res.trb_transfer_length()
                );
            }
            q.set_finished(ptr, res);
        } else {
            warn!(
                "xhci: transfer event has no endpoint queue slot={} ep={} ptr={:#x} code={:?} \
                 len={}",
                slot_id,
                ep_id,
                ptr.raw(),
                res.completion_code(),
                res.trb_transfer_length()
            );
        }
    }
}

fn is_iso_ring_xrun(event: TransferEvent) -> bool {
    matches!(
        event.completion_code(),
        Ok(CompletionCode::RingUnderrun | CompletionCode::RingOverrun)
    )
}
