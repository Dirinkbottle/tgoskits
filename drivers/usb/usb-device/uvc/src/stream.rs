use alloc::vec::Vec;
use core::{
    future::poll_fn,
    task::{Context, Poll},
};

use crab_usb::Endpoint;
use log::{info, warn};
use usb_if::{
    descriptor::{EndpointDescriptor, EndpointType},
    endpoint::{RequestId, TransferCompletion, TransferRequest, TransferStatus},
    err::{TransferError, USBError},
};

use crate::{
    VideoFormat,
    frame::{FrameEvent, FrameParser},
};

const MAX_PACKETS_PER_TRANSFER: usize = 32;
const MAX_INITIAL_FRAME_CAPACITY: usize = 16 * 1024 * 1024;
const ISO_TRANSFER_QUEUE_DEPTH: usize = 4;

mod completion_order;
use completion_order::CompletionOrder;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct IsoTransferLayout {
    packet_size: usize,
    packets_per_transfer: usize,
}

impl IsoTransferLayout {
    fn new(packet_size: usize) -> Self {
        Self {
            packet_size,
            packets_per_transfer: MAX_PACKETS_PER_TRANSFER,
        }
    }

    fn buffer_size(self) -> usize {
        self.packet_size
            .saturating_mul(self.packets_per_transfer)
            .max(1)
    }
}

struct IsoTransferSlot {
    buffer: Vec<u8>,
    request_id: Option<RequestId>,
}

impl IsoTransferSlot {
    fn new(buffer_size: usize) -> Self {
        Self {
            buffer: vec![0; buffer_size],
            request_id: None,
        }
    }
}

/// A running UVC video payload reader.
pub struct VideoStream {
    ep: Endpoint,
    frame_parser: FrameParser,
    /// Kept for compatibility with the original public API.
    pub vedio_format: VideoFormat,
    transfer_type: EndpointType,
    packet_size: usize,
    buffer: Vec<u8>,
    iso_packet_lengths: Vec<usize>,
    iso_slots: Vec<IsoTransferSlot>,
    completed_iso_transfers: u32,
    completion_order: CompletionOrder,
    iso_slot_completions: [u64; ISO_TRANSFER_QUEUE_DEPTH],
    iso_actual_bytes: u64,
    iso_zero_packets: u64,
    iso_oversized_packets: u64,
}

impl VideoStream {
    /// Creates a stream using the format's computed frame capacity.
    pub fn new(ep: Endpoint, desc: EndpointDescriptor, vfmt: VideoFormat) -> Self {
        let frame_size = vfmt.frame_bytes().max(1);
        Self::new_with_transfer_size(ep, desc, vfmt, frame_size as u32, frame_size as u32)
    }

    /// Creates a stream using the sizes negotiated by the UVC Probe/Commit exchange.
    pub fn new_with_transfer_size(
        ep: Endpoint,
        desc: EndpointDescriptor,
        vfmt: VideoFormat,
        max_payload_transfer_size: u32,
        max_video_frame_size: u32,
    ) -> Self {
        let max_packet_size = usize::from(desc.max_packet_size).max(1);
        let packets_per_microframe = if desc.transfer_type == EndpointType::Isochronous {
            desc.packets_per_microframe.max(1)
        } else {
            1
        };
        let packet_size = max_packet_size
            .saturating_mul(packets_per_microframe)
            .max(1);
        let (packets_per_transfer, buffer, iso_packet_lengths, iso_slots) =
            if desc.transfer_type == EndpointType::Isochronous {
                let transfer_layout = IsoTransferLayout::new(packet_size);
                let iso_packet_lengths = vec![packet_size; transfer_layout.packets_per_transfer];
                let mut iso_slots = Vec::with_capacity(ISO_TRANSFER_QUEUE_DEPTH);
                for _ in 0..ISO_TRANSFER_QUEUE_DEPTH {
                    iso_slots.push(IsoTransferSlot::new(transfer_layout.buffer_size()));
                }
                (
                    transfer_layout.packets_per_transfer,
                    Vec::new(),
                    iso_packet_lengths,
                    iso_slots,
                )
            } else {
                let requested_payload = usize::try_from(max_payload_transfer_size)
                    .unwrap_or(usize::MAX)
                    .max(packet_size);
                let packets_per_transfer = requested_payload
                    .div_ceil(packet_size)
                    .clamp(1, MAX_PACKETS_PER_TRANSFER);
                let buffer_size = packet_size.saturating_mul(packets_per_transfer).max(1);
                (
                    packets_per_transfer,
                    vec![0; buffer_size],
                    Vec::new(),
                    Vec::new(),
                )
            };
        let frame_size = usize::try_from(max_video_frame_size)
            .unwrap_or(usize::MAX)
            .min(MAX_INITIAL_FRAME_CAPACITY)
            .max(vfmt.frame_bytes().min(MAX_INITIAL_FRAME_CAPACITY))
            .max(1);

        info!(
            "VideoStream created: type={:?}, max_packet_size={}, packets_per_microframe={}, \
             packets_per_transfer={}, in_flight_transfers={}, buffer_size={}",
            desc.transfer_type,
            max_packet_size,
            packets_per_microframe,
            packets_per_transfer,
            iso_slots.len(),
            if desc.transfer_type == EndpointType::Isochronous {
                iso_slots.first().map_or(0, |slot| slot.buffer.len())
            } else {
                buffer.len()
            }
        );

        Self {
            ep,
            frame_parser: FrameParser::new(frame_size),
            vedio_format: vfmt,
            transfer_type: desc.transfer_type,
            packet_size,
            buffer,
            iso_packet_lengths,
            iso_slots,
            completed_iso_transfers: 0,
            completion_order: CompletionOrder::default(),
            iso_slot_completions: [0; ISO_TRANSFER_QUEUE_DEPTH],
            iso_actual_bytes: 0,
            iso_zero_packets: 0,
            iso_oversized_packets: 0,
        }
    }

    /// Reads one USB transfer and returns any complete UVC frames it contains.
    pub async fn recv(&mut self) -> Result<Vec<FrameEvent>, USBError> {
        match self.transfer_type {
            EndpointType::Isochronous => self.recv_isochronous().await,
            EndpointType::Bulk => self.recv_bulk().await,
            _ => Err(USBError::NotSupported),
        }
    }

    async fn recv_isochronous(&mut self) -> Result<Vec<FrameEvent>, USBError> {
        self.submit_isochronous_queue()?;
        let (slot_index, completion) = poll_fn(|cx| self.poll_isochronous_completion(cx))
            .await
            .map_err(USBError::TransferError)?;
        let events = self.consume_isochronous_completion(slot_index, completion)?;
        self.submit_isochronous_transfer(slot_index)?;
        Ok(events)
    }

    async fn recv_bulk(&mut self) -> Result<Vec<FrameEvent>, USBError> {
        self.buffer.fill(0);
        let completion = self
            .ep
            .wait(TransferRequest::bulk_in(&mut self.buffer))
            .await?;
        self.consume_bulk_completion(completion)
    }

    fn submit_isochronous_queue(&mut self) -> Result<(), USBError> {
        for slot_index in 0..self.iso_slots.len() {
            if self.iso_slots[slot_index].request_id.is_none() {
                self.submit_isochronous_transfer(slot_index)?;
            }
        }
        Ok(())
    }

    fn submit_isochronous_transfer(&mut self, slot_index: usize) -> Result<(), USBError> {
        let request = {
            let slot = self
                .iso_slots
                .get_mut(slot_index)
                .ok_or(USBError::InvalidParameter)?;
            if slot.request_id.is_some() {
                return Err(USBError::InvalidParameter);
            }

            // `Endpoint` retains a raw DMA pointer until completion. Slots and
            // their buffers are allocated before the first submission, the slot
            // vector is never grown, and this buffer is not accessed again until
            // `poll_isochronous_completion` reclaims the request.
            TransferRequest::iso_in(&mut slot.buffer, &self.iso_packet_lengths)
        };
        let request_id = self.ep.submit(request)?;
        self.iso_slots[slot_index].request_id = Some(request_id);
        self.completion_order.submitted(slot_index);
        Ok(())
    }

    fn poll_isochronous_completion(
        &mut self,
        cx: &mut Context<'_>,
    ) -> Poll<Result<(usize, TransferCompletion), TransferError>> {
        self.completion_order
            .poll(|slot_index| {
                let Some(request_id) = self.iso_slots[slot_index].request_id else {
                    return Poll::Ready(Err(TransferError::InvalidEndpoint));
                };
                self.ep.poll_request(request_id, cx)
            })
            .map(|(slot_index, result)| {
                self.iso_slots[slot_index].request_id = None;
                result.map(|completion| (slot_index, completion))
            })
    }

    fn consume_isochronous_completion(
        &mut self,
        slot_index: usize,
        completion: TransferCompletion,
    ) -> Result<Vec<FrameEvent>, USBError> {
        if completion.status != TransferStatus::Completed {
            return Err(USBError::TransferError(TransferError::Other(
                anyhow::anyhow!(
                    "UVC payload transfer completed with status {:?}",
                    completion.status
                ),
            )));
        }

        let actual_bytes = completion
            .iso_packets
            .iter()
            .map(|packet| {
                packet
                    .actual_length
                    .min(packet.requested_length.min(self.packet_size))
            })
            .sum::<usize>();
        let zero_length_packets = completion
            .iso_packets
            .iter()
            .filter(|packet| packet.actual_length == 0)
            .count();
        let oversized_packets = completion
            .iso_packets
            .iter()
            .filter(|packet| packet.actual_length > packet.requested_length)
            .count();
        // Serial logging here can take longer than the queued ISO coverage.
        // Keep counters only; the owner reports them after stopping the stream.
        self.iso_slot_completions[slot_index] =
            self.iso_slot_completions[slot_index].saturating_add(1);
        self.iso_actual_bytes = self.iso_actual_bytes.saturating_add(actual_bytes as u64);
        self.iso_zero_packets = self
            .iso_zero_packets
            .saturating_add(zero_length_packets as u64);
        self.iso_oversized_packets = self
            .iso_oversized_packets
            .saturating_add(oversized_packets as u64);
        self.completed_iso_transfers = self.completed_iso_transfers.wrapping_add(1);

        let mut events = Vec::new();
        let buffer = &self
            .iso_slots
            .get(slot_index)
            .ok_or(USBError::InvalidParameter)?
            .buffer;
        let mut offset = 0usize;
        for packet in completion.iso_packets {
            if packet.status != TransferStatus::Completed {
                return Err(USBError::TransferError(TransferError::Other(
                    anyhow::anyhow!(
                        "UVC isochronous payload packet completed with status {:?}",
                        packet.status
                    ),
                )));
            }
            let requested_length = packet.requested_length.min(self.packet_size);
            let actual_length = packet.actual_length.min(requested_length);
            let end = offset.saturating_add(actual_length).min(buffer.len());
            if end > offset {
                Self::push_payload(&mut self.frame_parser, &buffer[offset..end], &mut events);
            }
            offset = offset.saturating_add(requested_length).min(buffer.len());
            if offset == buffer.len() {
                break;
            }
        }

        Ok(events)
    }

    fn consume_bulk_completion(
        &mut self,
        completion: TransferCompletion,
    ) -> Result<Vec<FrameEvent>, USBError> {
        if completion.status != TransferStatus::Completed {
            return Err(USBError::TransferError(TransferError::Other(
                anyhow::anyhow!(
                    "UVC payload transfer completed with status {:?}",
                    completion.status
                ),
            )));
        }

        let mut events = Vec::new();
        let actual_length = completion.actual_length.min(self.buffer.len());
        if actual_length > 0 {
            Self::push_payload(
                &mut self.frame_parser,
                &self.buffer[..actual_length],
                &mut events,
            );
        }

        Ok(events)
    }

    fn push_payload(parser: &mut FrameParser, payload: &[u8], events: &mut Vec<FrameEvent>) {
        match parser.push_packet(payload) {
            Ok(Some(frame)) => events.push(frame),
            Ok(None) => {}
            Err(error) => {
                warn!(
                    "[uvc-frame] dropping {}-byte payload after parser error: {error:?}",
                    payload.len()
                );
            }
        }
    }

    /// Returns the number of payloads rejected with the UVC ERR flag.
    pub fn error_packet_count(&self) -> u32 {
        self.frame_parser.error_packet_count()
    }

    /// Resets the UVC ERR counter.
    pub fn reset_error_count(&mut self) {
        self.frame_parser.reset_error_count();
    }

    /// Reports accumulated capture diagnostics. Call after stopping transfers
    /// so synchronous logging cannot delay replenishing the ISO queue.
    pub fn report_diagnostics(&self) {
        info!(
            "[uvc-iso] stopped: transfers={} slots={:?} actual_bytes={} zero_packets={} \
             oversized_packets={} uvc_err_packets={} invalid_headers={}",
            self.completed_iso_transfers,
            self.iso_slot_completions,
            self.iso_actual_bytes,
            self.iso_zero_packets,
            self.iso_oversized_packets,
            self.frame_parser.error_packet_count(),
            self.frame_parser.invalid_header_count(),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::{IsoTransferLayout, MAX_PACKETS_PER_TRANSFER};

    #[test]
    fn isochronous_transfer_spans_multiple_service_intervals() {
        let layout = IsoTransferLayout::new(1_024);

        assert_eq!(layout.packets_per_transfer, MAX_PACKETS_PER_TRANSFER);
    }
}
