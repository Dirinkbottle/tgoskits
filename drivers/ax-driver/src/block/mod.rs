//! Platform discovery and registration for migrated block controllers.
//!
//! Every exposed controller implements the owned-DMA, interrupt-driven
//! `BlockController`/`HardwareQueue` contract. Low-level driver crates that
//! have not migrated remain unreachable from `ax-driver`.

#[cfg(any(feature = "ahci", feature = "ahci-fdt"))]
mod ahci;
mod binding;
mod irq_bound;

#[cfg(feature = "cv181x-sdhci")]
mod cvsd;
#[cfg(feature = "k230-sdhci")]
pub mod k230_sdhci;
#[cfg(feature = "k3-sdhci")]
pub mod k3_sdhci;
#[cfg(feature = "k3-ufs")]
pub mod k3_ufs;
#[cfg(feature = "nvme")]
pub mod nvme;
#[cfg(feature = "phytium-mci")]
mod phytium_mci;
#[cfg(any(feature = "rockchip-dwmmc", feature = "rockchip-sdhci"))]
mod rockchip;
#[cfg(feature = "starfive-jh7110-dwmmc")]
mod starfive_mmc;

pub use binding::*;
pub use irq_bound::IrqBoundBlock;

#[cfg(sync_block_dev)]
mod sync_block_dev {
    //! Adapter that bridges a synchronous [`SyncBlockOps`] driver to the
    //! async owned-DMA [`BlockController`]/[`HardwareQueue`] contract.
    //!
    //! The controller publishes a single queue on `Start`. Each submitted
    //! request is executed synchronously against the underlying driver and its
    //! DMA backing is returned to the runtime through the completion sink.

    use alloc::{boxed::Box, collections::VecDeque, sync::Arc, vec::Vec};
    use core::slice;

    use ax_kspin::SpinRaw as Mutex;
    use rdif_block::{
        BHardwareQueue, BatchSubmitDisposition, BatchSubmitResult, BlkError, BlockController,
        CompletedRequest, CompletionSink, ControllerEvent, ControllerState, ControllerUpdate,
        DeviceInfo, DriverGeneric, HardwareQueue, OwnedRequest, OwnedRequestBatch, QueueInfo,
        QueueLimits, RequestId, RequestOp, SubmissionSink, dma_api::CompletedDma,
    };

    use crate::block::PlatformDeviceBlock;

    /// Synchronous block operations implemented by a low-level driver.
    ///
    /// Each method blocks until the transfer has fully completed.
    pub(crate) trait SyncBlockOps: Send + 'static {
        fn name(&self) -> &'static str;
        fn num_blocks(&self) -> u64;
        fn block_size(&self) -> usize;
        fn read_blocks(&mut self, block_id: u64, buf: &mut [u8]) -> Result<(), BlkError>;
        fn write_blocks(&mut self, block_id: u64, buf: &[u8]) -> Result<(), BlkError>;
    }

    pub(crate) fn register_sync_block<D: SyncBlockOps>(
        plat_dev: rdrive::PlatformDevice,
        driver: D,
    ) {
        let controller = SyncBlockController::new(driver);
        plat_dev.register_block(controller);
    }

    type StoredCompletion = (RequestId, Result<(), BlkError>, Option<CompletedDma>);

    struct SyncBlockController<D: SyncBlockOps> {
        inner: Arc<Mutex<D>>,
        device_info: DeviceInfo,
        queue_taken: bool,
    }

    impl<D: SyncBlockOps> SyncBlockController<D> {
        fn new(driver: D) -> Self {
            // `SyncBlockOps::name` returns `&'static str`, so it is safe to
            // read before `driver` is moved into the shared lock.
            let name = driver.name();
            let num_blocks = driver.num_blocks();
            let block_size = driver.block_size();
            let device_info = DeviceInfo {
                name: Some(name),
                ..DeviceInfo::new(num_blocks, block_size)
            };
            Self {
                inner: Arc::new(Mutex::new(driver)),
                device_info,
                queue_taken: false,
            }
        }

        fn queue_limits(&self) -> QueueLimits {
            QueueLimits {
                // The synchronous driver accepts flush as a no-op.
                supports_flush: true,
                ..QueueLimits::simple(self.device_info.logical_block_size, u64::MAX)
            }
        }

        fn queue_info(&self) -> QueueInfo {
            QueueInfo {
                id: 0,
                device: self.device_info,
                limits: self.queue_limits(),
            }
        }
    }

    impl<D: SyncBlockOps> DriverGeneric for SyncBlockController<D> {
        fn name(&self) -> &str {
            self.device_info.name.unwrap_or("sync-block")
        }
    }

    impl<D: SyncBlockOps> BlockController for SyncBlockController<D> {
        fn device_info(&self) -> DeviceInfo {
            self.device_info
        }

        fn max_io_queues(&self) -> usize {
            1
        }

        fn advance(&mut self, event: ControllerEvent) -> Result<ControllerUpdate, BlkError> {
            match event {
                ControllerEvent::Start { .. } if !self.queue_taken => {
                    self.queue_taken = true;
                    let queue: BHardwareQueue = Box::new(SyncHardwareQueue::new(
                        Arc::clone(&self.inner),
                        self.queue_info(),
                    ));
                    Ok(ControllerUpdate::with_resources(
                        ControllerState::Ready,
                        alloc::vec![queue],
                        Vec::new(),
                    ))
                }
                ControllerEvent::Shutdown | ControllerEvent::Watchdog { .. } => {
                    Ok(ControllerUpdate::state(ControllerState::Shutdown))
                }
                // Already ready: no register transition, IRQ, or SMP scaling
                // applies to a synchronous single-queue adapter.
                _ => Ok(ControllerUpdate::state(ControllerState::Ready)),
            }
        }
    }

    struct SyncHardwareQueue<D: SyncBlockOps> {
        inner: Arc<Mutex<D>>,
        info: QueueInfo,
        next_id: usize,
        completed: VecDeque<StoredCompletion>,
    }

    impl<D: SyncBlockOps> SyncHardwareQueue<D> {
        fn new(inner: Arc<Mutex<D>>, info: QueueInfo) -> Self {
            Self {
                inner,
                info,
                next_id: 0,
                completed: VecDeque::new(),
            }
        }

        fn execute_sync(
            &self,
            mut request: OwnedRequest,
        ) -> (Result<(), BlkError>, Option<CompletedDma>) {
            match request.op {
                RequestOp::Read => match request.data.take() {
                    Some(prepared) => {
                        let ptr = prepared.cpu_ptr();
                        let len = prepared.len().get();
                        // SAFETY: the prepared backing is exclusively owned by
                        // this queue for the duration of the synchronous
                        // transfer. No other CPU or device access occurs while
                        // `buf` is live, and the synchronous driver fully
                        // completes before `buf` is dropped.
                        let buf = unsafe { slice::from_raw_parts_mut(ptr.as_ptr(), len) };
                        let result = self.inner.lock().read_blocks(request.lba, buf);
                        // `complete_without_device` returns the buffer to CPU
                        // ownership: the synchronous driver filled it through
                        // its own CPU mapping and no bus-master DMA remains
                        // outstanding.
                        let completed = prepared.complete_without_device();
                        (result, Some(completed))
                    }
                    None => (Err(BlkError::InvalidRequest), None),
                },
                RequestOp::Write => match request.data.take() {
                    Some(prepared) => {
                        let ptr = prepared.cpu_ptr();
                        let len = prepared.len().get();
                        // SAFETY: see the read branch; the same exclusive
                        // ownership and synchronous-completion guarantees hold.
                        let buf = unsafe { slice::from_raw_parts(ptr.as_ptr(), len) };
                        let result = self.inner.lock().write_blocks(request.lba, buf);
                        let completed = prepared.complete_without_device();
                        (result, Some(completed))
                    }
                    None => (Err(BlkError::InvalidRequest), None),
                },
                RequestOp::Flush => (Ok(()), None),
            }
        }
    }

    impl<D: SyncBlockOps> HardwareQueue for SyncHardwareQueue<D> {
        fn id(&self) -> usize {
            0
        }

        fn info(&self) -> QueueInfo {
            self.info
        }

        fn submit_batch_owned(
            &mut self,
            requests: &mut OwnedRequestBatch,
            sink: &mut dyn SubmissionSink,
        ) -> BatchSubmitResult {
            let mut accepted = 0usize;
            while let Some(request) = requests.pop_front() {
                let id = RequestId::new(self.next_id);
                self.next_id += 1;
                let (result, data) = self.execute_sync(request);
                self.completed.push_back((id, result, data));
                sink.accepted(id);
                accepted += 1;
            }
            BatchSubmitResult::new(accepted, BatchSubmitDisposition::Continue)
        }

        fn commit_submissions(&mut self) -> Result<(), BlkError> {
            // Every request was executed synchronously during submission, so
            // there is no staged descriptor ring to publish.
            Ok(())
        }

        fn drain_completions(&mut self, sink: &mut dyn CompletionSink) -> Result<(), BlkError> {
            while let Some((id, result, data)) = self.completed.pop_front() {
                sink.complete(CompletedRequest::new(id, result, data));
            }
            Ok(())
        }

        fn shutdown(&mut self, sink: &mut dyn CompletionSink) -> Result<(), BlkError> {
            // Report every not-yet-drained request as failed during quiesce.
            while let Some((id, _result, data)) = self.completed.pop_front() {
                sink.complete(CompletedRequest::new(id, Err(BlkError::Io), data));
            }
            Ok(())
        }
    }
}

#[cfg(sync_block_dev)]
pub(crate) use sync_block_dev::{SyncBlockOps, register_sync_block};
