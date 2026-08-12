//! rd-net 队列适配：`Interface` + `ITxQueue`/`IRxQueue` + IRQ handler。
//!
//! 修复点（相对旧实现 f7c4081be）：
//! - 新增 `K3GmacIrqHandler` 并实现 `Interface::take_irq_handler`，返回 `Some`。
//!   旧实现缺这个方法，导致设备退化成纯轮询（违反 rd_net 框架契约）。
//! - 共享状态用 `SpinNoIrq`（数据面 lock 关中断，IRQ 上半部 try_lock 避免死锁）。

use alloc::{boxed::Box, sync::Arc};

use ax_kspin::SpinNoIrq;
use rd_net::{DmaBuffer, Event, IRxQueue, ITxQueue, NetError, QueueConfig};

use super::core::{BUFFER_SIZE, DMA_ALIGN, DMA_MASK, K3GmacCore, QUEUE_ID, QUEUE_SIZE, SharedCore};

/// rd_net `Interface` 实现：持有 `Arc<SpinNoIrq<K3GmacCore>>` 共享状态。
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
            inner: Arc::new(SpinNoIrq::new(core)),
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

    /// 修复点：向框架注册 IRQ handler。
    ///
    /// 返回 `Some`（持有共享状态的 Arc clone），由 ax-net 注册到平台 IRQ。
    /// handler 在硬中断上下文被调用，用 `try_lock` 抢锁，失败则返回空事件
    /// （等下一次中断重试），避免在中断里阻塞。
    fn take_irq_handler(&mut self) -> Option<rd_net::BIrqHandler> {
        Some(Box::new(K3GmacIrqHandler {
            inner: self.inner.clone(),
        }))
    }
}

/// 硬中断上半部 handler。持有 `Arc` clone，与 `K3GmacNet`/队列共享同一把锁。
pub struct K3GmacIrqHandler {
    inner: SharedCore,
}

impl rd_net::InterfaceIrqHandler for K3GmacIrqHandler {
    fn handle_irq(&mut self) -> Event {
        // try_lock：硬中断上下文不能睡眠/忙等锁，抢不到就放弃本次。
        let Some(mut core) = self.inner.try_lock() else {
            return Event::none();
        };
        core.handle_irq()
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
