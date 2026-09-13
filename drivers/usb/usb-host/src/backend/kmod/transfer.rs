use alloc::vec::Vec;
use core::ptr::NonNull;

use dma_api::DmaDirection;
use usb_if::{endpoint::TransferRequest, err::TransferError, transfer::Direction};

use crate::{
    backend::ty::transfer::{Transfer, TransferKind},
    osal::Kernel,
};

const ALIGN: usize = 64;

impl Transfer {
    pub(crate) fn new(
        dma: &Kernel,
        kind: TransferKind,
        direction: Direction,
        buff: Option<(NonNull<u8>, usize)>,
    ) -> Result<Self, TransferError> {
        let dma_direction = match direction {
            Direction::In => DmaDirection::FromDevice,
            Direction::Out => DmaDirection::ToDevice,
        };
        let mapping = if let Some((ptr, len)) = buff.filter(|(_, len)| *len > 0) {
            let slice = unsafe { core::slice::from_raw_parts_mut(ptr.as_ptr(), len) };
            Some(
                dma.map_streaming_slice_for_device(slice, ALIGN, dma_direction)
                    .map_err(|err| TransferError::Other(anyhow!("DMA mapping failed: {err}")))?,
            )
        } else {
            None
        };

        Ok(Self {
            kind,
            direction,
            mapping,
            transfer_len: 0,
            iso_packet_actual_lengths: Vec::new(),
        })
    }

    pub(crate) fn from_request(
        dma: &Kernel,
        request: TransferRequest,
    ) -> Result<Self, TransferError> {
        let (kind, direction, buffer) = request.into();
        let buff = buffer.map(|buffer| (buffer.ptr, buffer.len));
        Self::new(dma, kind, direction, buff)
    }

    pub fn buffer_len(&self) -> usize {
        if let Some(ref mapping) = self.mapping {
            mapping.len()
        } else {
            0
        }
    }

    pub fn dma_addr(&self) -> u64 {
        if let Some(ref mapping) = self.mapping {
            mapping.dma_addr().as_u64()
        } else {
            0
        }
    }

    pub fn complete_for_cpu_all(&self) {
        if let Some(ref mapping) = self.mapping {
            mapping.complete_for_cpu_all();
        }
    }
}

#[cfg(test)]
mod tests {
    use core::{
        alloc::Layout,
        num::NonZeroUsize,
        sync::atomic::{AtomicUsize, Ordering},
        time::Duration,
    };

    use dma_api::{DmaAllocHandle, DmaConstraints, DmaError, DmaMapHandle, DmaOp};

    use super::*;
    use crate::backend::kmod::osal::KernelOp;

    static TEST_KERNEL: TestKernel = TestKernel {
        prepare_count: AtomicUsize::new(0),
    };

    struct TestKernel {
        prepare_count: AtomicUsize,
    }

    impl DmaOp for TestKernel {
        fn page_size(&self) -> usize {
            4096
        }

        unsafe fn alloc_contiguous(
            &self,
            _constraints: DmaConstraints,
            _layout: Layout,
        ) -> Option<DmaAllocHandle> {
            None
        }

        unsafe fn dealloc_contiguous(&self, _handle: DmaAllocHandle) {}

        unsafe fn alloc_coherent(
            &self,
            _constraints: DmaConstraints,
            _layout: Layout,
        ) -> Option<DmaAllocHandle> {
            None
        }

        unsafe fn dealloc_coherent(&self, _handle: DmaAllocHandle) -> Result<(), DmaError> {
            Ok(())
        }

        unsafe fn map_streaming(
            &self,
            constraints: DmaConstraints,
            addr: NonNull<u8>,
            size: NonZeroUsize,
            _direction: DmaDirection,
        ) -> Result<DmaMapHandle, DmaError> {
            let layout = Layout::from_size_align(size.get(), constraints.align)?;
            Ok(unsafe { DmaMapHandle::new(addr, 0x1000.into(), layout, None) })
        }

        unsafe fn unmap_streaming(&self, _handle: DmaMapHandle) {}

        fn sync_map_for_device(
            &self,
            _handle: &DmaMapHandle,
            _offset: usize,
            _size: usize,
            direction: DmaDirection,
        ) {
            assert_eq!(direction, DmaDirection::FromDevice);
            self.prepare_count.fetch_add(1, Ordering::SeqCst);
        }
    }

    impl KernelOp for TestKernel {
        fn delay(&self, _duration: Duration) {}
    }

    #[test]
    fn in_transfer_prepares_streaming_dma_before_submission() {
        TEST_KERNEL.prepare_count.store(0, Ordering::SeqCst);
        let kernel = Kernel::new(u64::MAX, &TEST_KERNEL);
        let mut buffer = [0u8; 4];
        let buffer = NonNull::from(&mut buffer).cast();

        let _transfer = Transfer::new(
            &kernel,
            TransferKind::Bulk,
            Direction::In,
            Some((buffer, 4)),
        )
        .unwrap();

        assert_eq!(TEST_KERNEL.prepare_count.load(Ordering::SeqCst), 1);
    }
}
