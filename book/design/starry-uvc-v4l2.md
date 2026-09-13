# StarryOS UVC V4L2 capture interface

## Problem and users

`crab-uvc` and `ax-driver` can negotiate a UVC VideoControl/VideoStreaming
pair and deliver complete frames, but a StarryOS userspace program has no
Linux media ABI with which to consume them.  Camera applications which already
use V4L2 need a `/dev/video0` capture node instead of a private kernel API.

The interface is enabled only for the SpacemiT K3 COM260Kit build. It accepts
the first USB device whose descriptors contain a complete UVC VideoControl and
VideoStreaming pair, regardless of VID:PID. A K3 host without such a camera
still exposes the stable devfs name, but opening it returns `ENODEV`. Other
boards, including SG2002, do not expose this node and retain their own camera
interfaces.

## Scope and non-goals

The first implementation exposes one V4L2 single-planar video-capture session:

- `VIDIOC_QUERYCAP`, format/frame-size/frame-interval enumeration, `G/S/TRY_FMT`,
  `G/S_PARM`, MMAP `REQBUFS`/`QUERYBUF`/`QBUF`/`DQBUF`, and
  `STREAMON`/`STREAMOFF`;
- descriptor-advertised MJPEG, H.264, YUYV, NV12, RGB24, and RGB32 modes;
- a bounded (at most four, at most 16 MiB each) physically contiguous MMAP
  queue, with a UVC descriptor/Probe-derived `sizeimage`.

It deliberately does not claim multi-planar capture, `read(2)`, USERPTR,
DMABUF, V4L2 controls/events, media-controller topology, camera selection,
hot-unplug recovery, or concurrent independent opens. When multiple complete
UVC cameras are present, the first topology entry is selected. A second open
gets `EBUSY`; `dup` and `fork` share the original open-file description and its
queue, matching normal VFS ownership. These exclusions are explicit so a
userspace program cannot mistake an unimplemented ABI for a working one.

## Alternatives considered

1. A private ioctl or a kernel-only callback would avoid UAPI work, but forces
   every application to be Starry-specific.
2. A `read(2)`-only device needs kernel-owned copying and does not support the
   normal zero-copy V4L2 application model.
3. USERPTR/DMABUF would avoid driver-owned pages, but require user-page pinning,
   DMA ownership, and cache-coherency contracts which are not present for this
   UVC path.

MMAP is therefore the smallest standard V4L2 transport.  The V4L2 queue and
stream state follow the Linux V4L2 userspace contract: format changes are
rejected while buffers exist or streaming is active; `STREAMOFF` discards queued
and completed buffers; and a completed buffer is returned only by `DQBUF`.

## Layering and state

`crab-uvc` owns UVC descriptor parsing, VS Probe/Commit, alternate-setting
selection, payload assembly, and UVC errors.  `ax-driver::usb::UvcVideoCapture`
is the portable capture capability.  `pseudofs::dev::video` owns only the
StarryOS-specific V4L2 structs, userspace copies, MMAP pages, and queue states:

```text
Dequeued --QBUF--> Queued --worker/UVC frame--> Done --DQBUF--> Dequeued
                            \--STREAMOFF/error--> Dequeued
```

The worker copies one assembled frame into one queued MMAP buffer and publishes
`Done` before waking poll waiters.  A short-lived `Delivering` state protects a
frame while `DQBUF` writes its descriptor to userspace; a bad userspace pointer
restores it to `Done` rather than losing the frame.

The UVC camera and USB host are exclusively owned by the one V4L2 session.  A
process-context worker holds the UVC session mutex and then the `rdrive` USB-host
lock while a USB transfer is pending, which serializes host control/transfer
operations.  It never accesses userspace memory or wakes poll waiters while
those locks are held.  The xHCI IRQ event pump does not take either lock, so the
path does not introduce VFS or poll wakeup work in hard-IRQ context.  `STREAMOFF`
may wait for the in-flight USB transfer, then stops the UVC stream and clears the
queue.

The UVC descriptor maximum frame size is carried through `VideoFormat`; this is
the source of `v4l2_pix_format.sizeimage` and the allocation bound.  A camera
whose negotiated frame exceeds the explicit driver limit fails allocation with
`ENOMEM`, never silently truncates a frame while reporting a larger `sizeimage`.
An unexpected payload beyond an allocated frame is returned with the truncated
byte count and `V4L2_BUF_FLAG_ERROR`, so userspace can discard it.

## Compatibility and validation

The node uses Linux's V4L2 video major (`81`) and minor zero.  Ioctl numbers are
provided by target-specific `linux-raw-sys` UAPI constants, rather than being
hard-coded.  Kernel compile-time assertions tie the C layouts to those ioctl
encodings, and host-side unit tests cover format selection without hardware.

Board validation requires a physical UVC camera attached to the K3 COM260Kit,
so it cannot run in QEMU. The board acceptance sequence is: enumerate formats,
select an advertised format, allocate and map MMAP buffers, queue all buffers,
stream on, dequeue and validate a non-empty frame, requeue it, then dequeue and
validate a second frame before requeuing and streaming off. A failed K3 `open`
without a USB host or matching camera must return `ENODEV`; unsupported ioctls
must return an error and never report success.
