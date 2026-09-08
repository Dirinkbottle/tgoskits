use alloc::sync::Arc;

use ax_sync::{SpinLock as Mutex, SpinRwLock as RwLock};
use dma_api::DmaDirection;
use mbarrier::wmb;
use usb_if::err::TransferError;
use xhci::{
    registers::doorbell,
    ring::trb::{command, event::CommandCompletion},
};

use super::{reg::XhciRegisters, ring::SendRing};
use crate::{err::ConvertXhciError, osal::Kernel, queue::Finished};

#[derive(Clone)]
pub struct CommandRing(Arc<Mutex<Inner>>);

impl CommandRing {
    pub fn new(
        direction: DmaDirection,
        dma: &Kernel,
        reg: Arc<RwLock<XhciRegisters>>,
    ) -> crate::err::Result<Self> {
        let ring = SendRing::new(direction, dma)?;
        let inner = Inner { ring, reg };
        Ok(Self(Arc::new(Mutex::new(inner))))
    }

    pub fn bus_addr(&self) -> crate::BusAddr {
        // SAFETY: command-ring access excludes local xHCI event re-entry.
        let inner = unsafe { self.0.lock_raw() };
        inner.ring.bus_addr()
    }

    pub fn cycle(&self) -> bool {
        // SAFETY: command-ring access excludes local xHCI event re-entry.
        let inner = unsafe { self.0.lock_raw() };
        inner.ring.cycle()
    }

    pub fn finished_handle(&self) -> Finished<CommandCompletion> {
        // SAFETY: command-ring access excludes local xHCI event re-entry.
        let inner = unsafe { self.0.lock_raw() };
        inner.ring.finished_handle()
    }

    pub async fn cmd_request(
        &mut self,
        trb: command::Allowed,
    ) -> Result<CommandCompletion, TransferError> {
        trace!("[xhci-cmd] submit begin: {trb:?}");
        let fur = {
            // SAFETY: command submission excludes local xHCI event re-entry.
            let mut inner = unsafe { self.0.lock_raw() };
            let trb_addr = inner.ring.enque_command(trb);
            trace!("[xhci-cmd] TRB queued: addr={:#x}", trb_addr.raw());
            let fur = inner.ring.take_finished_future(trb_addr);
            wmb();
            inner
                .reg
                .write()
                .doorbell
                .write_volatile_at(0, doorbell::Register::default());
            trace!(
                "[xhci-cmd] command doorbell rung: trb={:#x}",
                trb_addr.raw()
            );
            fur
        };

        let res = fur.await;
        trace!(
            "[xhci-cmd] completion received: trb={:#x}, slot={}, code={:?}",
            res.command_trb_pointer(),
            res.slot_id(),
            res.completion_code()
        );

        match res.completion_code() {
            Ok(code) => code.to_result()?,
            Err(e) => {
                trace!("[xhci-cmd] completion code decode failed: {e:?}");
                Err(TransferError::Other(anyhow!("Command failed: {e:?}")))?
            }
        }

        Ok(res)
    }
}

struct Inner {
    ring: SendRing<CommandCompletion>,
    reg: Arc<RwLock<XhciRegisters>>,
}
