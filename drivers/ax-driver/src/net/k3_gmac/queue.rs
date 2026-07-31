//! rd-net queue adapters for K3 GMAC.

use alloc::{boxed::Box, sync::Arc};

use ax_kspin::SpinRaw;
use rd_net::{DmaBuffer, Event, IRxQueue, ITxQueue, NetError, QueueConfig};

use super::core::{BUFFER_SIZE, DMA_ALIGN, DMA_MASK, K3GmacCore, QUEUE_ID, QUEUE_SIZE, SharedCore};

pub struct K3GmacNet {
    inner: SharedCore,
    mac: [u8; 6],
    tx_created: bool,
    rx_created: bool,
}

impl K3GmacNet {
    pub fn new(core: K3GmacCore) -> Self {
        let mac = core.mac_address();
        Self {
            inner: Arc::new(SpinRaw::new(core)),
            mac,
            tx_created: false,
            rx_created: false,
        }
    }
}

impl rdrive::DriverGeneric for K3GmacNet {
    fn name(&self) -> &str {
        super::DRIVER_NAME
    }
}

impl rd_net::Interface for K3GmacNet {
    fn mac_address(&self) -> [u8; 6] {
        self.mac
    }

    fn create_tx_queue(&mut self) -> Option<Box<dyn ITxQueue>> {
        if self.tx_created {
            return None;
        }
        self.tx_created = true;
        Some(Box::new(K3GmacTxQueue {
            inner: self.inner.clone(),
        }))
    }

    fn create_rx_queue(&mut self) -> Option<Box<dyn IRxQueue>> {
        if self.rx_created {
            return None;
        }
        self.rx_created = true;
        Some(Box::new(K3GmacRxQueue {
            inner: self.inner.clone(),
        }))
    }

    fn enable_irq(&mut self) {
        self.inner.lock().enable_irq();
    }

    fn disable_irq(&mut self) {
        self.inner.lock().disable_irq();
    }

    fn is_irq_enabled(&self) -> bool {
        self.inner.lock().is_irq_enabled()
    }

    fn handle_irq(&mut self) -> Event {
        self.inner.lock().handle_irq()
    }
}

pub struct K3GmacTxQueue {
    inner: SharedCore,
}

impl ITxQueue for K3GmacTxQueue {
    fn id(&self) -> usize {
        QUEUE_ID
    }

    fn config(&self) -> QueueConfig {
        queue_config()
    }

    fn submit(&mut self, buffer: DmaBuffer) -> Result<(), NetError> {
        self.inner.lock().submit_tx(buffer)
    }

    fn reclaim(&mut self) -> Option<u64> {
        self.inner.lock().reclaim_tx_buffer()
    }
}

pub struct K3GmacRxQueue {
    inner: SharedCore,
}

impl IRxQueue for K3GmacRxQueue {
    fn id(&self) -> usize {
        QUEUE_ID
    }

    fn config(&self) -> QueueConfig {
        queue_config()
    }

    fn submit(&mut self, buffer: DmaBuffer) -> Result<(), NetError> {
        self.inner.lock().submit_rx(buffer)
    }

    fn reclaim(&mut self) -> Option<(u64, usize)> {
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
