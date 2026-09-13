//! `/dev/video0` — a minimal V4L2 MMAP capture adapter for the board UVC camera.
//!
//! UVC negotiation and USB payload assembly stay in `crab-uvc` and `ax-driver`.
//! This module owns only the Linux V4L2 ABI, userspace-visible MMAP buffers, and
//! per-open queue state.  There is one open capture session at a time because a
//! UVC VS interface has one active Probe/Commit configuration and endpoint.

use alloc::{borrow::Cow, string::ToString, sync::Arc, vec::Vec};
#[cfg(test)]
use core::mem::size_of;
use core::{
    any::Any,
    sync::atomic::{AtomicBool, Ordering},
    task::Context,
};

use ax_alloc::GlobalPage;
use ax_errno::{AxError, AxResult, LinuxError};
use ax_memory_addr::{PAGE_SIZE_4K, PhysAddrRange};
use ax_runtime::hal::{mem::virt_to_phys, time::monotonic_time_nanos};
use ax_sync::Mutex;
use axfs_ng_vfs::{NodeFlags, VfsError, VfsResult};
use axpoll::{IoEvents, PollSet, Pollable};
use crab_usb::usb_if::err::{TransferError, USBError};
use crab_uvc::{UncompressedFormat, VideoFormat, VideoFormatType};
use linux_raw_sys::ioctl::{
    VIDIOC_DQBUF, VIDIOC_ENUM_FMT, VIDIOC_ENUM_FRAMEINTERVALS, VIDIOC_ENUM_FRAMESIZES,
    VIDIOC_G_FMT, VIDIOC_G_PARM, VIDIOC_QBUF, VIDIOC_QUERYBUF, VIDIOC_QUERYCAP, VIDIOC_REQBUFS,
    VIDIOC_S_FMT, VIDIOC_S_PARM, VIDIOC_STREAMOFF, VIDIOC_STREAMON, VIDIOC_TRY_FMT,
};
use starry_vm::{VmMutPtr, VmPtr};

use crate::{
    file::{File as KernelFile, FileLike, IoDst, IoSrc, Kstat},
    pseudofs::{DeviceMmap, DeviceOps},
};

const V4L2_BUF_TYPE_VIDEO_CAPTURE: u32 = 1;
const V4L2_MEMORY_MMAP: u32 = 1;
const V4L2_FIELD_NONE: u32 = 1;
const V4L2_FRMSIZE_TYPE_DISCRETE: u32 = 1;
const V4L2_FRMIVAL_TYPE_DISCRETE: u32 = 1;

const V4L2_CAP_VIDEO_CAPTURE: u32 = 0x0000_0001;
const V4L2_CAP_STREAMING: u32 = 0x0400_0000;
const V4L2_CAP_DEVICE_CAPS: u32 = 0x8000_0000;
const V4L2_CAP_TIMEPERFRAME: u32 = 0x0000_1000;
const V4L2_BUF_CAP_SUPPORTS_MMAP: u32 = 0x0000_0001;

const V4L2_BUF_FLAG_MAPPED: u32 = 0x0000_0001;
const V4L2_BUF_FLAG_QUEUED: u32 = 0x0000_0002;
const V4L2_BUF_FLAG_DONE: u32 = 0x0000_0004;
const V4L2_BUF_FLAG_ERROR: u32 = 0x0000_0040;
const V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC: u32 = 0x0000_2000;

const MAX_CAPTURE_BUFFERS: u32 = 4;
const MAX_CAPTURE_BUFFER_SIZE: usize = 16 * 1024 * 1024;
const V4L2_MMAP_OFFSET_STRIDE: u64 = MAX_CAPTURE_BUFFER_SIZE as u64;
const V4L2_DRIVER_VERSION: u32 = (7 << 8) | 6;

const fn fourcc(bytes: [u8; 4]) -> u32 {
    u32::from_le_bytes(bytes)
}

const V4L2_PIX_FMT_MJPEG: u32 = fourcc(*b"MJPG");
const V4L2_PIX_FMT_H264: u32 = fourcc(*b"H264");
const V4L2_PIX_FMT_YUYV: u32 = fourcc(*b"YUYV");
const V4L2_PIX_FMT_NV12: u32 = fourcc(*b"NV12");
const V4L2_PIX_FMT_RGB24: u32 = fourcc(*b"RGB3");
const V4L2_PIX_FMT_RGB32: u32 = fourcc(*b"RGB4");

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct V4l2Capability {
    driver: [u8; 16],
    card: [u8; 32],
    bus_info: [u8; 32],
    version: u32,
    capabilities: u32,
    device_caps: u32,
    reserved: [u32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct V4l2PixFormat {
    width: u32,
    height: u32,
    pixelformat: u32,
    field: u32,
    bytesperline: u32,
    sizeimage: u32,
    colorspace: u32,
    priv_: u32,
    flags: u32,
    ycbcr_enc: u32,
    quantization: u32,
    xfer_func: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct V4l2Format {
    type_: u32,
    // `v4l2_format.fmt` is a C union that contains legacy pointer-bearing
    // members.  Its alignment is 8 on 64-bit Linux ABIs, so the union starts
    // after four bytes of padding there (but immediately after `type` on 32-bit).
    #[cfg(target_pointer_width = "64")]
    alignment_padding: u32,
    pix: V4l2PixFormat,
    reserved: [u8; 152],
}

impl Default for V4l2Format {
    fn default() -> Self {
        Self {
            type_: 0,
            #[cfg(target_pointer_width = "64")]
            alignment_padding: 0,
            pix: V4l2PixFormat::default(),
            reserved: [0; 152],
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct V4l2FmtDesc {
    index: u32,
    type_: u32,
    flags: u32,
    description: [u8; 32],
    pixelformat: u32,
    mbus_code: u32,
    reserved: [u32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct V4l2Fract {
    numerator: u32,
    denominator: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct V4l2Frmsizeenum {
    index: u32,
    pixel_format: u32,
    type_: u32,
    discrete_width: u32,
    discrete_height: u32,
    padding: [u8; 16],
    reserved: [u32; 2],
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct V4l2Frmivalenum {
    index: u32,
    pixel_format: u32,
    width: u32,
    height: u32,
    type_: u32,
    discrete: V4l2Fract,
    padding: [u8; 16],
    reserved: [u32; 2],
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct V4l2Requestbuffers {
    count: u32,
    type_: u32,
    memory: u32,
    capabilities: u32,
    flags: u8,
    reserved: [u8; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct V4l2Timeval {
    tv_sec: i64,
    tv_usec: i64,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct V4l2Timecode {
    type_: u32,
    flags: u32,
    frames: u8,
    seconds: u8,
    minutes: u8,
    hours: u8,
    userbits: [u8; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct V4l2Buffer {
    index: u32,
    type_: u32,
    bytesused: u32,
    flags: u32,
    field: u32,
    timestamp: V4l2Timeval,
    timecode: V4l2Timecode,
    sequence: u32,
    memory: u32,
    memory_offset: usize,
    length: u32,
    reserved2: u32,
    request_fd: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct V4l2CaptureParm {
    capability: u32,
    capturemode: u32,
    timeperframe: V4l2Fract,
    extendedmode: u32,
    readbuffers: u32,
    reserved: [u32; 4],
}

#[repr(C)]
#[derive(Clone, Copy)]
struct V4l2Streamparm {
    type_: u32,
    capture: V4l2CaptureParm,
    padding: [u8; 160],
}

impl Default for V4l2Streamparm {
    fn default() -> Self {
        Self {
            type_: 0,
            capture: V4l2CaptureParm::default(),
            padding: [0; 160],
        }
    }
}

const fn ioctl_size(command: u32) -> usize {
    ((command >> 16) & 0x3fff) as usize
}

// Keep every wire struct tied to the target-specific ioctl encoding supplied
// by linux-raw-sys.  This catches a C-layout drift while compiling the kernel,
// including cross builds where `unsigned long` differs from the host.
const _: () = {
    assert!(core::mem::size_of::<V4l2Capability>() == ioctl_size(VIDIOC_QUERYCAP));
    assert!(core::mem::size_of::<V4l2Format>() == ioctl_size(VIDIOC_G_FMT));
    assert!(core::mem::size_of::<V4l2Format>() == ioctl_size(VIDIOC_S_FMT));
    assert!(core::mem::size_of::<V4l2Format>() == ioctl_size(VIDIOC_TRY_FMT));
    assert!(core::mem::size_of::<V4l2FmtDesc>() == ioctl_size(VIDIOC_ENUM_FMT));
    assert!(core::mem::size_of::<V4l2Frmsizeenum>() == ioctl_size(VIDIOC_ENUM_FRAMESIZES));
    assert!(core::mem::size_of::<V4l2Frmivalenum>() == ioctl_size(VIDIOC_ENUM_FRAMEINTERVALS));
    assert!(core::mem::size_of::<V4l2Requestbuffers>() == ioctl_size(VIDIOC_REQBUFS));
    assert!(core::mem::size_of::<V4l2Buffer>() == ioctl_size(VIDIOC_QUERYBUF));
    assert!(core::mem::size_of::<V4l2Buffer>() == ioctl_size(VIDIOC_QBUF));
    assert!(core::mem::size_of::<V4l2Buffer>() == ioctl_size(VIDIOC_DQBUF));
    assert!(core::mem::size_of::<V4l2Streamparm>() == ioctl_size(VIDIOC_G_PARM));
    assert!(core::mem::size_of::<V4l2Streamparm>() == ioctl_size(VIDIOC_S_PARM));
    assert!(core::mem::size_of::<u32>() == ioctl_size(VIDIOC_STREAMON));
    assert!(core::mem::size_of::<u32>() == ioctl_size(VIDIOC_STREAMOFF));
};

/// Shared `/dev/video0` node.  Per-open capture state lives in [`UvcVideoFile`].
pub(crate) struct UvcVideoDevice {
    active_open: Arc<AtomicBool>,
}

impl UvcVideoDevice {
    pub(crate) fn new() -> Self {
        Self {
            active_open: Arc::new(AtomicBool::new(false)),
        }
    }
}

impl Default for UvcVideoDevice {
    fn default() -> Self {
        Self::new()
    }
}

impl DeviceOps for UvcVideoDevice {
    fn read_at(&self, _buf: &mut [u8], _offset: u64) -> VfsResult<usize> {
        Err(VfsError::InvalidInput)
    }

    fn write_at(&self, _buf: &[u8], _offset: u64) -> VfsResult<usize> {
        Err(VfsError::InvalidInput)
    }

    fn ioctl(&self, _cmd: u32, _arg: usize) -> VfsResult<usize> {
        Err(VfsError::NotATty)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn flags(&self) -> NodeFlags {
        NodeFlags::NON_CACHEABLE | NodeFlags::STREAM
    }
}

pub(crate) fn is_uvc_video_device(inner: &dyn Any) -> bool {
    inner.is::<UvcVideoDevice>()
}

/// Opens the one supported UVC capture device and creates an independent V4L2
/// open-file session.  `dup` and `fork` retain the returned `Arc`, therefore
/// sharing this exact queue and stream as Linux V4L2 file descriptions do.
pub(crate) fn open_uvc_video_file(
    inner: &dyn Any,
    file: ax_fs_ng::File,
    open_flags: u32,
) -> AxResult<Arc<dyn FileLike>> {
    let device = inner
        .downcast_ref::<UvcVideoDevice>()
        .ok_or(AxError::InvalidInput)?;
    let lease = VideoOpenLease::acquire(device.active_open.clone())?;

    if !rdrive::is_initialized() {
        return Err(AxError::NoSuchDevice);
    }
    let host = ax_driver::usb::usb_host_device().ok_or(AxError::NoSuchDevice)?;
    let (capture, formats) = {
        let mut guard = host.lock().map_err(|_| AxError::ResourceBusy)?;
        let capture = ax_task::future::block_on(guard.probe_first_uvc_video())
            .map_err(map_usb_error)?
            .ok_or(AxError::NoSuchDevice)?;
        let mut capture = capture;
        let formats =
            ax_task::future::block_on(capture.supported_formats()).map_err(map_usb_error)?;
        (capture, formats)
    };
    let active_format = formats
        .first()
        .cloned()
        .ok_or(AxError::OperationNotSupported)?;
    if frame_capacity(&active_format).is_err() {
        return Err(AxError::OperationNotSupported);
    }

    Ok(Arc::new(UvcVideoFile {
        base: KernelFile::new(file, open_flags),
        host,
        shared: Arc::new(VideoShared {
            session: Mutex::new(VideoSession {
                capture,
                formats,
                active_format,
                buffers: Vec::new(),
                streaming: false,
                sequence: 0,
            }),
            poll_ready: PollSet::new(),
            worker_running: AtomicBool::new(false),
        }),
        _lease: lease,
    }))
}

struct VideoOpenLease(Arc<AtomicBool>);

impl VideoOpenLease {
    fn acquire(active_open: Arc<AtomicBool>) -> AxResult<Self> {
        active_open
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .map_err(|_| AxError::ResourceBusy)?;
        Ok(Self(active_open))
    }
}

impl Drop for VideoOpenLease {
    fn drop(&mut self) {
        self.0.store(false, Ordering::Release);
    }
}

struct UvcVideoFile {
    base: KernelFile,
    host: ax_driver::usb::UsbHostDevice,
    shared: Arc<VideoShared>,
    _lease: VideoOpenLease,
}

struct VideoShared {
    session: Mutex<VideoSession>,
    poll_ready: PollSet,
    worker_running: AtomicBool,
}

struct VideoSession {
    capture: ax_driver::usb::uvcvideo::UvcVideoCapture,
    formats: Vec<VideoFormat>,
    active_format: VideoFormat,
    buffers: Vec<CaptureBuffer>,
    streaming: bool,
    sequence: u32,
}

struct CaptureBuffer {
    backing: Arc<MmapBuffer>,
    state: CaptureBufferState,
}

#[derive(Clone, Copy)]
enum CaptureBufferState {
    Dequeued,
    Queued,
    Delivering(FrameMetadata),
    Done(FrameMetadata),
}

#[derive(Clone, Copy)]
struct FrameMetadata {
    bytesused: u32,
    timestamp_ns: u64,
    sequence: u32,
    error: bool,
}

struct MmapBuffer {
    pages: Mutex<GlobalPage>,
    frame_capacity: usize,
    mapping_size: usize,
    offset: u64,
}

impl MmapBuffer {
    fn allocate(frame_capacity: usize, index: u32) -> AxResult<Arc<Self>> {
        let mapping_size = frame_capacity
            .checked_add(PAGE_SIZE_4K - 1)
            .ok_or(AxError::NoMemory)?
            / PAGE_SIZE_4K
            * PAGE_SIZE_4K;
        let pages = mapping_size / PAGE_SIZE_4K;
        let mut page =
            GlobalPage::alloc_contiguous(pages, PAGE_SIZE_4K).map_err(|_| AxError::NoMemory)?;
        // These pages are mapped directly into userspace.  Zero them before
        // publication so bytes after `bytesused` never disclose old kernel data.
        page.zero();
        Ok(Arc::new(Self {
            pages: Mutex::new(page),
            frame_capacity,
            mapping_size,
            offset: u64::from(index) * V4L2_MMAP_OFFSET_STRIDE,
        }))
    }

    fn mmap(&self, length: u64) -> AxResult<PhysAddrRange> {
        let length = usize::try_from(length).map_err(|_| AxError::InvalidInput)?;
        if length == 0 || length > self.mapping_size {
            return Err(AxError::InvalidInput);
        }
        let pages = self.pages.lock();
        Ok(PhysAddrRange::from_start_size(
            virt_to_phys(pages.start_vaddr()),
            length,
        ))
    }

    fn write_frame(&self, data: &[u8]) -> (u32, bool) {
        let copy_len = data.len().min(self.frame_capacity);
        let mut pages = self.pages.lock();
        pages.as_slice_mut()[..copy_len].copy_from_slice(&data[..copy_len]);
        // The user mapping is non-cacheable.  Publish stores through the
        // cacheable kernel alias before a V4L2 DONE buffer becomes visible.
        ax_runtime::hal::cache::clean_dcache_to_pou(pages.start_vaddr(), copy_len);
        (
            u32::try_from(copy_len).unwrap_or(u32::MAX),
            copy_len != data.len(),
        )
    }
}

impl VideoSession {
    fn has_queued_buffer(&self) -> bool {
        self.buffers
            .iter()
            .any(|buffer| matches!(buffer.state, CaptureBufferState::Queued))
    }

    fn discard_buffers(&mut self) {
        for buffer in &mut self.buffers {
            buffer.state = CaptureBufferState::Dequeued;
        }
    }
}

impl UvcVideoFile {
    fn read_arg<T: Copy>(&self, arg: usize) -> AxResult<T> {
        if arg == 0 {
            return Err(AxError::BadAddress);
        }
        // SAFETY: VmPtr validates the entire C-layout object against the
        // current userspace address space before copying it into kernel memory.
        Ok(unsafe { (arg as *const T).vm_read_uninit()?.assume_init() })
    }

    fn write_arg<T: Copy>(&self, arg: usize, value: T) -> AxResult {
        if arg == 0 {
            return Err(AxError::BadAddress);
        }
        // SAFETY: VmMutPtr faults safely if this C-layout object is not a
        // writable userspace range.
        Ok((arg as *mut T).vm_write(value)?)
    }

    fn query_capability(&self, arg: usize) -> AxResult<usize> {
        let mut capability = V4l2Capability {
            version: V4L2_DRIVER_VERSION,
            device_caps: V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING,
            ..Default::default()
        };
        capability.capabilities = capability.device_caps | V4L2_CAP_DEVICE_CAPS;
        write_c_string(&mut capability.driver, b"starry-uvc");
        write_c_string(&mut capability.card, b"TGOS UVC Camera");
        write_c_string(&mut capability.bus_info, b"usb-0");
        self.write_arg(arg, capability)?;
        Ok(0)
    }

    fn enum_format(&self, arg: usize) -> AxResult<usize> {
        let request: V4l2FmtDesc = self.read_arg(arg)?;
        ensure_capture_type(request.type_)?;
        let session = self.shared.session.lock();
        let format = unique_formats(&session.formats)
            .get(request.index as usize)
            .copied()
            .ok_or(AxError::InvalidInput)?;
        let mut reply = V4l2FmtDesc {
            index: request.index,
            type_: V4L2_BUF_TYPE_VIDEO_CAPTURE,
            pixelformat: pixel_format(format),
            ..Default::default()
        };
        let (description, compressed) = format_description(format.format_type);
        if compressed {
            reply.flags = 1;
        }
        write_c_string(&mut reply.description, description);
        self.write_arg(arg, reply)?;
        Ok(0)
    }

    fn enum_frame_size(&self, arg: usize) -> AxResult<usize> {
        let request: V4l2Frmsizeenum = self.read_arg(arg)?;
        let session = self.shared.session.lock();
        let formats = unique_frame_sizes(&session.formats, request.pixel_format);
        let format = formats
            .get(request.index as usize)
            .copied()
            .ok_or(AxError::InvalidInput)?;
        let reply = V4l2Frmsizeenum {
            index: request.index,
            pixel_format: request.pixel_format,
            type_: V4L2_FRMSIZE_TYPE_DISCRETE,
            discrete_width: u32::from(format.width),
            discrete_height: u32::from(format.height),
            ..Default::default()
        };
        self.write_arg(arg, reply)?;
        Ok(0)
    }

    fn enum_frame_interval(&self, arg: usize) -> AxResult<usize> {
        let request: V4l2Frmivalenum = self.read_arg(arg)?;
        let session = self.shared.session.lock();
        let formats = matching_modes(
            &session.formats,
            request.pixel_format,
            request.width,
            request.height,
        );
        let format = formats
            .get(request.index as usize)
            .copied()
            .ok_or(AxError::InvalidInput)?;
        let reply = V4l2Frmivalenum {
            index: request.index,
            pixel_format: request.pixel_format,
            width: request.width,
            height: request.height,
            type_: V4L2_FRMIVAL_TYPE_DISCRETE,
            discrete: frame_interval(format),
            ..Default::default()
        };
        self.write_arg(arg, reply)?;
        Ok(0)
    }

    fn get_format(&self, arg: usize) -> AxResult<usize> {
        let request: V4l2Format = self.read_arg(arg)?;
        ensure_capture_type(request.type_)?;
        let session = self.shared.session.lock();
        self.write_arg(arg, format_reply(&session.active_format))?;
        Ok(0)
    }

    fn try_or_set_format(&self, arg: usize, commit: bool) -> AxResult<usize> {
        let request: V4l2Format = self.read_arg(arg)?;
        ensure_capture_type(request.type_)?;
        let mut session = self.shared.session.lock();
        if commit && (session.streaming || !session.buffers.is_empty()) {
            return Err(AxError::ResourceBusy);
        }
        let selected =
            select_format(&session.formats, &request.pix).ok_or(AxError::OperationNotSupported)?;
        if commit {
            session.active_format = selected.clone();
        }
        self.write_arg(arg, format_reply(&selected))?;
        Ok(0)
    }

    fn get_streamparm(&self, arg: usize) -> AxResult<usize> {
        let request: V4l2Streamparm = self.read_arg(arg)?;
        ensure_capture_type(request.type_)?;
        let session = self.shared.session.lock();
        self.write_arg(arg, streamparm_reply(&session.active_format))?;
        Ok(0)
    }

    fn set_streamparm(&self, arg: usize) -> AxResult<usize> {
        let request: V4l2Streamparm = self.read_arg(arg)?;
        ensure_capture_type(request.type_)?;
        let mut session = self.shared.session.lock();
        if session.streaming || !session.buffers.is_empty() {
            return Err(AxError::ResourceBusy);
        }
        let requested_fps = (request.capture.timeperframe.numerator != 0)
            .then(|| {
                request.capture.timeperframe.denominator / request.capture.timeperframe.numerator
            })
            .filter(|fps| *fps != 0);
        if let Some(fps) = requested_fps {
            let active = &session.active_format;
            let candidate = session
                .formats
                .iter()
                .filter(|format| {
                    pixel_format(format) == pixel_format(active)
                        && format.width == active.width
                        && format.height == active.height
                })
                .min_by_key(|format| format.frame_rate.abs_diff(fps))
                .cloned();
            if let Some(candidate) = candidate {
                session.active_format = candidate;
            }
        }
        self.write_arg(arg, streamparm_reply(&session.active_format))?;
        Ok(0)
    }

    fn request_buffers(&self, arg: usize) -> AxResult<usize> {
        let request: V4l2Requestbuffers = self.read_arg(arg)?;
        ensure_capture_type(request.type_)?;
        ensure_mmap_memory(request.memory)?;
        if request.flags != 0 || request.reserved != [0; 3] {
            return Err(AxError::InvalidInput);
        }

        if request.count == 0 {
            self.stop_streaming()?;
            self.shared.session.lock().buffers.clear();
            self.write_arg(
                arg,
                V4l2Requestbuffers {
                    type_: V4L2_BUF_TYPE_VIDEO_CAPTURE,
                    memory: V4L2_MEMORY_MMAP,
                    capabilities: V4L2_BUF_CAP_SUPPORTS_MMAP,
                    ..Default::default()
                },
            )?;
            return Ok(0);
        }

        let (count, frame_size) = {
            let session = self.shared.session.lock();
            if session.streaming || !session.buffers.is_empty() {
                return Err(AxError::ResourceBusy);
            }
            (
                request.count.clamp(1, MAX_CAPTURE_BUFFERS),
                frame_capacity(&session.active_format)?,
            )
        };
        let mut buffers = Vec::new();
        buffers
            .try_reserve_exact(count as usize)
            .map_err(|_| AxError::NoMemory)?;
        for index in 0..count {
            buffers.push(CaptureBuffer {
                backing: MmapBuffer::allocate(frame_size, index)?,
                state: CaptureBufferState::Dequeued,
            });
        }
        let mut session = self.shared.session.lock();
        if session.streaming || !session.buffers.is_empty() {
            return Err(AxError::ResourceBusy);
        }
        session.buffers = buffers;
        self.write_arg(
            arg,
            V4l2Requestbuffers {
                count,
                type_: V4L2_BUF_TYPE_VIDEO_CAPTURE,
                memory: V4L2_MEMORY_MMAP,
                capabilities: V4L2_BUF_CAP_SUPPORTS_MMAP,
                ..Default::default()
            },
        )?;
        Ok(0)
    }

    fn query_buffer(&self, arg: usize) -> AxResult<usize> {
        let request: V4l2Buffer = self.read_arg(arg)?;
        ensure_buffer_request(&request)?;
        let session = self.shared.session.lock();
        let buffer = session
            .buffers
            .get(request.index as usize)
            .ok_or(AxError::InvalidInput)?;
        self.write_arg(
            arg,
            buffer_reply(request.index, buffer, metadata_of(buffer.state)),
        )?;
        Ok(0)
    }

    fn queue_buffer(&self, arg: usize) -> AxResult<usize> {
        let request: V4l2Buffer = self.read_arg(arg)?;
        ensure_buffer_request(&request)?;
        let mut session = self.shared.session.lock();
        let buffer = session
            .buffers
            .get_mut(request.index as usize)
            .ok_or(AxError::InvalidInput)?;
        if !matches!(buffer.state, CaptureBufferState::Dequeued) {
            return Err(AxError::ResourceBusy);
        }
        buffer.state = CaptureBufferState::Queued;
        let reply = buffer_reply(request.index, buffer, None);
        drop(session);
        if let Err(error) = self.write_arg(arg, reply) {
            let mut session = self.shared.session.lock();
            if let Some(buffer) = session.buffers.get_mut(request.index as usize)
                && matches!(buffer.state, CaptureBufferState::Queued)
            {
                buffer.state = CaptureBufferState::Dequeued;
            }
            return Err(error);
        }
        self.start_capture_worker();
        Ok(0)
    }

    fn dequeue_buffer(&self, arg: usize) -> AxResult<usize> {
        let request: V4l2Buffer = self.read_arg(arg)?;
        ensure_buffer_request(&request)?;
        let (reply, index) = ax_task::future::block_on(ax_task::future::poll_io(
            self,
            IoEvents::IN,
            self.nonblocking(),
            || self.claim_done_buffer(),
        ))?;

        let write_result = self.write_arg(arg, reply);
        let mut session = self.shared.session.lock();
        let buffer = session.buffers.get_mut(index).ok_or(AxError::BadState)?;
        let metadata = match buffer.state {
            CaptureBufferState::Delivering(metadata) => metadata,
            _ => return Err(AxError::BadState),
        };
        buffer.state = if write_result.is_ok() {
            CaptureBufferState::Dequeued
        } else {
            CaptureBufferState::Done(metadata)
        };
        let restore_ready = write_result.is_err();
        drop(session);
        if restore_ready {
            // The frame remained available after an EFAULT, so another DQBUF
            // or a repaired caller can retrieve it.
            unsafe { self.shared.poll_ready.wake(IoEvents::IN) };
        }
        write_result?;
        Ok(0)
    }

    fn claim_done_buffer(&self) -> AxResult<(V4l2Buffer, usize)> {
        let mut session = self.shared.session.lock();
        let Some((index, buffer)) = session
            .buffers
            .iter_mut()
            .enumerate()
            .find(|(_, buffer)| matches!(buffer.state, CaptureBufferState::Done(_)))
        else {
            return Err(AxError::WouldBlock);
        };
        let CaptureBufferState::Done(metadata) = buffer.state else {
            return Err(AxError::BadState);
        };
        let reply = buffer_reply(index as u32, buffer, Some(metadata));
        buffer.state = CaptureBufferState::Delivering(metadata);
        Ok((reply, index))
    }

    fn stream_on(&self, arg: usize) -> AxResult<usize> {
        let type_: u32 = self.read_arg(arg)?;
        ensure_capture_type(type_)?;
        let mut session = self.shared.session.lock();
        if session.streaming {
            return Ok(0);
        }
        if session.buffers.is_empty() {
            return Err(AxError::InvalidInput);
        }
        let requested = session.active_format.clone();
        let _host_guard = self.host.lock().map_err(|_| AxError::ResourceBusy)?;
        ax_task::future::block_on(session.capture.start_streaming(requested))
            .map_err(map_usb_error)?;
        let negotiated = match session.capture.current_format().cloned() {
            Some(format) => format,
            None => {
                let _ = ax_task::future::block_on(session.capture.stop_streaming());
                return Err(AxError::BadState);
            }
        };
        let capacity = match frame_capacity(&negotiated) {
            Ok(capacity) => capacity,
            Err(error) => {
                let _ = ax_task::future::block_on(session.capture.stop_streaming());
                return Err(error);
            }
        };
        if session
            .buffers
            .iter()
            .any(|buffer| buffer.backing.frame_capacity < capacity)
        {
            let _ = ax_task::future::block_on(session.capture.stop_streaming());
            return Err(AxError::NoMemory);
        }
        drop(_host_guard);
        session.active_format = negotiated;
        session.streaming = true;
        drop(session);
        self.start_capture_worker();
        Ok(0)
    }

    fn stop_streaming(&self) -> AxResult<()> {
        let mut session = self.shared.session.lock();
        if session.streaming {
            let _host_guard = self.host.lock().map_err(|_| AxError::ResourceBusy)?;
            let stop_result = ax_task::future::block_on(session.capture.stop_streaming());
            drop(_host_guard);
            session.streaming = false;
            session.discard_buffers();
            drop(session);
            unsafe { self.shared.poll_ready.wake(IoEvents::IN) };
            stop_result.map_err(map_usb_error)?;
            return Ok(());
        }
        session.discard_buffers();
        drop(session);
        unsafe { self.shared.poll_ready.wake(IoEvents::IN) };
        Ok(())
    }

    fn stream_off(&self, arg: usize) -> AxResult<usize> {
        let type_: u32 = self.read_arg(arg)?;
        ensure_capture_type(type_)?;
        self.stop_streaming()?;
        Ok(0)
    }

    fn start_capture_worker(&self) {
        let should_start = {
            let session = self.shared.session.lock();
            session.streaming && session.has_queued_buffer()
        };
        if !should_start
            || self
                .shared
                .worker_running
                .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                .is_err()
        {
            return;
        }
        let shared = Arc::downgrade(&self.shared);
        let host = self.host.clone();
        ax_task::spawn_with_name(
            move || capture_worker(shared, host),
            "uvc-v4l2-capture".to_string(),
        );
    }
}

fn capture_worker(shared: alloc::sync::Weak<VideoShared>, host: ax_driver::usb::UsbHostDevice) {
    loop {
        let Some(shared) = shared.upgrade() else {
            return;
        };
        let mut session = shared.session.lock();
        if !session.streaming {
            shared.worker_running.store(false, Ordering::Release);
            return;
        }
        let Some(index) = session
            .buffers
            .iter()
            .position(|buffer| matches!(buffer.state, CaptureBufferState::Queued))
        else {
            shared.worker_running.store(false, Ordering::Release);
            return;
        };

        // This is process context, never the USB IRQ path.  The order is the
        // session mutex then rdrive host lock; no VFS/userspace/poll operation
        // executes while either is held.
        let frame = match host.lock() {
            Ok(_host_guard) => {
                ax_task::future::block_on(session.capture.recv_frame()).map_err(map_usb_error)
            }
            Err(_) => Err(AxError::ResourceBusy),
        };
        match frame {
            Ok(frame) => {
                let (bytesused, truncated) =
                    session.buffers[index].backing.write_frame(&frame.data);
                let is_mjpeg = pixel_format(&session.active_format) == V4L2_PIX_FMT_MJPEG;
                let complete_mjpeg = !is_mjpeg || has_complete_jpeg_markers(&frame.data);
                let error = truncated || frame.has_error || !complete_mjpeg;
                if error {
                    let edge_len = frame.data.len().min(4);
                    let suffix_start = frame.data.len().saturating_sub(edge_len);
                    warn!(
                        "[uvc-v4l2] rejected frame: sequence={} uvc_frame={} bytesused={} \
                         source_bytes={} buffer_capacity={} uvc_error={} truncated={} \
                         complete_mjpeg={} prefix={:02x?} suffix={:02x?}",
                        session.sequence,
                        frame.frame_number,
                        bytesused,
                        frame.data.len(),
                        session.buffers[index].backing.frame_capacity,
                        frame.has_error,
                        truncated,
                        complete_mjpeg,
                        &frame.data[..edge_len],
                        &frame.data[suffix_start..],
                    );
                } else if session.sequence < 2 {
                    debug!(
                        "[uvc-v4l2] delivered frame: sequence={} uvc_frame={} bytesused={}",
                        session.sequence, frame.frame_number, bytesused,
                    );
                }
                let metadata = FrameMetadata {
                    bytesused,
                    timestamp_ns: monotonic_time_nanos(),
                    sequence: session.sequence,
                    error,
                };
                session.sequence = session.sequence.wrapping_add(1);
                session.buffers[index].state = CaptureBufferState::Done(metadata);
                drop(session);
                unsafe { shared.poll_ready.wake(IoEvents::IN) };
            }
            Err(error) => {
                warn!("uvc-v4l2: frame receive failed: {error:?}");
                if let Ok(_host_guard) = host.lock() {
                    let _ = ax_task::future::block_on(session.capture.stop_streaming());
                }
                let metadata = FrameMetadata {
                    bytesused: 0,
                    timestamp_ns: monotonic_time_nanos(),
                    sequence: session.sequence,
                    error: true,
                };
                session.sequence = session.sequence.wrapping_add(1);
                session.buffers[index].state = CaptureBufferState::Done(metadata);
                session.streaming = false;
                shared.worker_running.store(false, Ordering::Release);
                drop(session);
                unsafe { shared.poll_ready.wake(IoEvents::IN) };
                return;
            }
        }
    }
}

impl FileLike for UvcVideoFile {
    fn read(&self, _dst: &mut IoDst) -> AxResult<usize> {
        Err(AxError::InvalidInput)
    }

    fn write(&self, _src: &mut IoSrc) -> AxResult<usize> {
        Err(AxError::InvalidInput)
    }

    fn stat(&self) -> AxResult<Kstat> {
        self.base.stat()
    }

    fn path(&self) -> Cow<'_, str> {
        self.base.path()
    }

    fn device_mmap(&self, offset: u64, length: u64) -> AxResult<DeviceMmap> {
        let session = self.shared.session.lock();
        let buffer = session
            .buffers
            .iter()
            .find(|buffer| buffer.backing.offset == offset)
            .ok_or(AxError::InvalidInput)?;
        let range = buffer.backing.mmap(length)?;
        let retain: Arc<dyn Any + Send + Sync> = buffer.backing.clone();
        Ok(DeviceMmap::PhysicalResolved(range, Some(retain)))
    }

    fn ioctl(&self, cmd: u32, arg: usize) -> AxResult<usize> {
        match cmd {
            VIDIOC_QUERYCAP => self.query_capability(arg),
            VIDIOC_ENUM_FMT => self.enum_format(arg),
            VIDIOC_ENUM_FRAMESIZES => self.enum_frame_size(arg),
            VIDIOC_ENUM_FRAMEINTERVALS => self.enum_frame_interval(arg),
            VIDIOC_G_FMT => self.get_format(arg),
            VIDIOC_S_FMT => self.try_or_set_format(arg, true),
            VIDIOC_TRY_FMT => self.try_or_set_format(arg, false),
            VIDIOC_G_PARM => self.get_streamparm(arg),
            VIDIOC_S_PARM => self.set_streamparm(arg),
            VIDIOC_REQBUFS => self.request_buffers(arg),
            VIDIOC_QUERYBUF => self.query_buffer(arg),
            VIDIOC_QBUF => self.queue_buffer(arg),
            VIDIOC_DQBUF => self.dequeue_buffer(arg),
            VIDIOC_STREAMON => self.stream_on(arg),
            VIDIOC_STREAMOFF => self.stream_off(arg),
            _ => Err(AxError::NotATty),
        }
    }

    fn open_flags(&self) -> u32 {
        self.base.open_flags()
    }

    fn nonblocking(&self) -> bool {
        self.base.nonblocking()
    }

    fn set_nonblocking(&self, nonblocking: bool) -> AxResult {
        self.base.set_nonblocking(nonblocking)
    }
}

impl Pollable for UvcVideoFile {
    fn poll(&self) -> IoEvents {
        let session = self.shared.session.lock();
        if session
            .buffers
            .iter()
            .any(|buffer| matches!(buffer.state, CaptureBufferState::Done(_)))
        {
            IoEvents::IN
        } else {
            IoEvents::empty()
        }
    }

    fn register(&self, context: &mut Context<'_>, events: IoEvents) {
        if events.contains(IoEvents::IN) {
            // Poll registration and wakeup are always task-context operations.
            unsafe {
                self.shared
                    .poll_ready
                    .register(context.waker(), IoEvents::IN)
            };
            if self.poll().contains(IoEvents::IN) {
                context.waker().wake_by_ref();
            }
        }
    }
}

impl Drop for UvcVideoFile {
    fn drop(&mut self) {
        let _ = self.stop_streaming();
    }
}

fn ensure_capture_type(type_: u32) -> AxResult<()> {
    (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE)
        .then_some(())
        .ok_or(AxError::InvalidInput)
}

fn ensure_mmap_memory(memory: u32) -> AxResult<()> {
    (memory == V4L2_MEMORY_MMAP)
        .then_some(())
        .ok_or(AxError::InvalidInput)
}

fn ensure_buffer_request(request: &V4l2Buffer) -> AxResult<()> {
    ensure_capture_type(request.type_)?;
    ensure_mmap_memory(request.memory)
}

fn frame_capacity(format: &VideoFormat) -> AxResult<usize> {
    let size = usize::try_from(format.max_frame_size).map_err(|_| AxError::NoMemory)?;
    if size == 0 || size > MAX_CAPTURE_BUFFER_SIZE {
        return Err(AxError::NoMemory);
    }
    Ok(size)
}

fn pixel_format(format: &VideoFormat) -> u32 {
    match format.format_type {
        VideoFormatType::Mjpeg => V4L2_PIX_FMT_MJPEG,
        VideoFormatType::H264 => V4L2_PIX_FMT_H264,
        VideoFormatType::Uncompressed(UncompressedFormat::Yuy2) => V4L2_PIX_FMT_YUYV,
        VideoFormatType::Uncompressed(UncompressedFormat::Nv12) => V4L2_PIX_FMT_NV12,
        VideoFormatType::Uncompressed(UncompressedFormat::Rgb24) => V4L2_PIX_FMT_RGB24,
        VideoFormatType::Uncompressed(UncompressedFormat::Rgb32) => V4L2_PIX_FMT_RGB32,
    }
}

fn has_complete_jpeg_markers(data: &[u8]) -> bool {
    data.len() >= 4 && data.starts_with(&[0xff, 0xd8]) && data.ends_with(&[0xff, 0xd9])
}

fn format_description(format: VideoFormatType) -> (&'static [u8], bool) {
    match format {
        VideoFormatType::Mjpeg => (b"Motion-JPEG", true),
        VideoFormatType::H264 => (b"H.264", true),
        VideoFormatType::Uncompressed(UncompressedFormat::Yuy2) => (b"YUYV 4:2:2", false),
        VideoFormatType::Uncompressed(UncompressedFormat::Nv12) => (b"NV12", false),
        VideoFormatType::Uncompressed(UncompressedFormat::Rgb24) => (b"RGB24", false),
        VideoFormatType::Uncompressed(UncompressedFormat::Rgb32) => (b"RGB32", false),
    }
}

fn bytes_per_line(format: &VideoFormat) -> u32 {
    match format.format_type {
        VideoFormatType::Mjpeg | VideoFormatType::H264 => 0,
        VideoFormatType::Uncompressed(UncompressedFormat::Yuy2) => u32::from(format.width) * 2,
        VideoFormatType::Uncompressed(UncompressedFormat::Nv12) => u32::from(format.width),
        VideoFormatType::Uncompressed(UncompressedFormat::Rgb24) => u32::from(format.width) * 3,
        VideoFormatType::Uncompressed(UncompressedFormat::Rgb32) => u32::from(format.width) * 4,
    }
}

fn format_reply(format: &VideoFormat) -> V4l2Format {
    V4l2Format {
        type_: V4L2_BUF_TYPE_VIDEO_CAPTURE,
        pix: V4l2PixFormat {
            width: u32::from(format.width),
            height: u32::from(format.height),
            pixelformat: pixel_format(format),
            field: V4L2_FIELD_NONE,
            bytesperline: bytes_per_line(format),
            sizeimage: format.max_frame_size,
            ..Default::default()
        },
        ..Default::default()
    }
}

fn frame_interval(format: &VideoFormat) -> V4l2Fract {
    V4l2Fract {
        numerator: 1,
        denominator: format.frame_rate.max(1),
    }
}

fn streamparm_reply(format: &VideoFormat) -> V4l2Streamparm {
    V4l2Streamparm {
        type_: V4L2_BUF_TYPE_VIDEO_CAPTURE,
        capture: V4l2CaptureParm {
            capability: V4L2_CAP_TIMEPERFRAME,
            timeperframe: frame_interval(format),
            ..Default::default()
        },
        ..Default::default()
    }
}

fn select_format(formats: &[VideoFormat], requested: &V4l2PixFormat) -> Option<VideoFormat> {
    let by_pixel = formats
        .iter()
        .filter(|format| {
            requested.pixelformat == 0 || pixel_format(format) == requested.pixelformat
        })
        .collect::<Vec<_>>();
    let candidates = if by_pixel.is_empty() {
        formats.iter().collect::<Vec<_>>()
    } else {
        by_pixel
    };
    candidates
        .into_iter()
        .min_by_key(|format| {
            u32::from(format.width).abs_diff(requested.width)
                + u32::from(format.height).abs_diff(requested.height)
        })
        .cloned()
}

fn unique_formats(formats: &[VideoFormat]) -> Vec<&VideoFormat> {
    let mut result: Vec<&VideoFormat> = Vec::new();
    for format in formats {
        if result
            .iter()
            .all(|existing| pixel_format(existing) != pixel_format(format))
        {
            result.push(format);
        }
    }
    result
}

fn unique_frame_sizes(formats: &[VideoFormat], pixel: u32) -> Vec<&VideoFormat> {
    let mut result: Vec<&VideoFormat> = Vec::new();
    for format in formats
        .iter()
        .filter(|format| pixel_format(format) == pixel)
    {
        if result
            .iter()
            .all(|existing| existing.width != format.width || existing.height != format.height)
        {
            result.push(format);
        }
    }
    result
}

fn matching_modes(
    formats: &[VideoFormat],
    pixel: u32,
    width: u32,
    height: u32,
) -> Vec<&VideoFormat> {
    formats
        .iter()
        .filter(|format| {
            pixel_format(format) == pixel
                && u32::from(format.width) == width
                && u32::from(format.height) == height
        })
        .collect()
}

fn buffer_reply(index: u32, buffer: &CaptureBuffer, metadata: Option<FrameMetadata>) -> V4l2Buffer {
    let mut flags = V4L2_BUF_FLAG_MAPPED;
    match buffer.state {
        CaptureBufferState::Queued => flags |= V4L2_BUF_FLAG_QUEUED,
        CaptureBufferState::Done(_) | CaptureBufferState::Delivering(_) => {
            flags |= V4L2_BUF_FLAG_DONE
        }
        CaptureBufferState::Dequeued => {}
    }
    let mut reply = V4l2Buffer {
        index,
        type_: V4L2_BUF_TYPE_VIDEO_CAPTURE,
        flags,
        field: V4L2_FIELD_NONE,
        memory: V4L2_MEMORY_MMAP,
        memory_offset: buffer.backing.offset as usize,
        length: u32::try_from(buffer.backing.mapping_size).unwrap_or(u32::MAX),
        ..Default::default()
    };
    if let Some(metadata) = metadata {
        reply.bytesused = metadata.bytesused;
        reply.flags |= V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
        if metadata.error {
            reply.flags |= V4L2_BUF_FLAG_ERROR;
        }
        reply.sequence = metadata.sequence;
        reply.timestamp = V4l2Timeval {
            tv_sec: (metadata.timestamp_ns / 1_000_000_000) as i64,
            tv_usec: ((metadata.timestamp_ns % 1_000_000_000) / 1_000) as i64,
        };
    }
    reply
}

fn metadata_of(state: CaptureBufferState) -> Option<FrameMetadata> {
    match state {
        CaptureBufferState::Delivering(metadata) | CaptureBufferState::Done(metadata) => {
            Some(metadata)
        }
        CaptureBufferState::Dequeued | CaptureBufferState::Queued => None,
    }
}

fn write_c_string(target: &mut [u8], source: &[u8]) {
    let length = source.len().min(target.len().saturating_sub(1));
    target[..length].copy_from_slice(&source[..length]);
}

fn map_transfer_error(error: TransferError) -> AxError {
    match error {
        TransferError::Timeout => AxError::TimedOut,
        TransferError::Cancelled => AxError::from(LinuxError::ENOENT),
        TransferError::Stall => AxError::BrokenPipe,
        TransferError::QueueFull => AxError::ResourceBusy,
        TransferError::InvalidEndpoint => AxError::InvalidInput,
        TransferError::NoDevice => AxError::NoSuchDevice,
        TransferError::NotSupported => AxError::Unsupported,
        TransferError::Other(_) => AxError::Io,
    }
}

fn map_usb_error(error: USBError) -> AxError {
    match error {
        USBError::Timeout => AxError::TimedOut,
        USBError::NoMemory => AxError::NoMemory,
        USBError::TransferError(error) => map_transfer_error(error),
        USBError::NotInitialized | USBError::ConfigurationNotSet => AxError::BadState,
        USBError::NotFound => AxError::NoSuchDevice,
        USBError::InvalidParameter => AxError::InvalidInput,
        USBError::SlotLimitReached => AxError::ResourceBusy,
        USBError::NotSupported => AxError::Unsupported,
        USBError::Other(_) => AxError::Io,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn v4l2_abi_layout_matches_ioctl_encoding() {
        assert_eq!(size_of::<V4l2Capability>(), ioctl_size(VIDIOC_QUERYCAP));
        assert_eq!(size_of::<V4l2Format>(), ioctl_size(VIDIOC_G_FMT));
        assert_eq!(size_of::<V4l2Format>(), ioctl_size(VIDIOC_S_FMT));
        assert_eq!(size_of::<V4l2Format>(), ioctl_size(VIDIOC_TRY_FMT));
        assert_eq!(size_of::<V4l2FmtDesc>(), ioctl_size(VIDIOC_ENUM_FMT));
        assert_eq!(
            size_of::<V4l2Frmsizeenum>(),
            ioctl_size(VIDIOC_ENUM_FRAMESIZES)
        );
        assert_eq!(
            size_of::<V4l2Frmivalenum>(),
            ioctl_size(VIDIOC_ENUM_FRAMEINTERVALS)
        );
        assert_eq!(size_of::<V4l2Requestbuffers>(), ioctl_size(VIDIOC_REQBUFS));
        assert_eq!(size_of::<V4l2Buffer>(), ioctl_size(VIDIOC_QUERYBUF));
        assert_eq!(size_of::<V4l2Buffer>(), ioctl_size(VIDIOC_QBUF));
        assert_eq!(size_of::<V4l2Buffer>(), ioctl_size(VIDIOC_DQBUF));
        assert_eq!(size_of::<V4l2Streamparm>(), ioctl_size(VIDIOC_G_PARM));
        assert_eq!(size_of::<V4l2Streamparm>(), ioctl_size(VIDIOC_S_PARM));
        assert_eq!(size_of::<u32>(), ioctl_size(VIDIOC_STREAMON));
        assert_eq!(size_of::<u32>(), ioctl_size(VIDIOC_STREAMOFF));
    }

    #[test]
    fn format_selection_preserves_requested_fourcc_when_advertised() {
        let formats = [
            VideoFormat {
                width: 640,
                height: 480,
                frame_rate: 30,
                format_type: VideoFormatType::Mjpeg,
                max_frame_size: 1_000_000,
            },
            VideoFormat {
                width: 1280,
                height: 720,
                frame_rate: 30,
                format_type: VideoFormatType::Uncompressed(UncompressedFormat::Yuy2),
                max_frame_size: 2_000_000,
            },
        ];
        let selected = select_format(
            &formats,
            &V4l2PixFormat {
                width: 1920,
                height: 1080,
                pixelformat: V4L2_PIX_FMT_YUYV,
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(pixel_format(&selected), V4L2_PIX_FMT_YUYV);
        assert_eq!(selected.width, 1280);
    }

    #[test]
    fn complete_jpeg_validation_requires_both_frame_markers() {
        assert!(has_complete_jpeg_markers(&[0xff, 0xd8, 0xff, 0xd9]));
        assert!(!has_complete_jpeg_markers(&[0xff, 0xd8, 0x00, 0x00]));
        assert!(!has_complete_jpeg_markers(&[0x00, 0x00, 0xff, 0xd9]));
        assert!(!has_complete_jpeg_markers(&[]));
    }
}
