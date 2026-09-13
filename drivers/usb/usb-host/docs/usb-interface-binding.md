# USB interface driver binding

## Problem and scope

`crab-usb` enumerates USB devices and exposes their descriptors, but previously
left every consumer to rediscover a device class and choose a driver.  The
kernel USB serial path is one example of this duplicated matching.  This design
adds the smallest shared binding point: after enumeration of the active
configuration, USB interface descriptors are matched against registered driver
IDs.

The direct path is:

```text
port connection -> address/enumerate -> GET_DESCRIPTOR -> UsbInterface
    -> UsbId::matches -> UsbDriver::probe -> interface binding
```

The scope is descriptor-based, interface-level binding.  It deliberately does
not add a userspace ABI, hotplug daemon, endpoint runtime, or a mass-storage,
HID, UVC, or serial function driver.

## Prior art and alternatives

Linux USB core matches an interface against a driver's ID table before calling
its probe callback, and calls the driver's disconnect callback during unbind:
<https://github.com/torvalds/linux/blob/master/drivers/usb/core/driver.c>.
The implementation here retains that ordering but does not copy Linux's device
model, PM, authorization, dynamic IDs, or multi-interface claim machinery.

Three options were considered:

| Option | Result |
| --- | --- |
| Keep per-consumer descriptor scans | Preserves duplication and cannot report one binding owner. |
| Global IRQ-locked registry | Couples the reusable host crate to a runtime lock and risks holding a lock while calling driver code. |
| Host-owned flat ID registry | Selected: enumeration already has exclusive `&mut USBHost` access, so matching is direct and driver callbacks run without a registry lock. |

## Ownership and lifecycle

`USBHost` owns the `UsbDriverRegistry` and, while a device is present, a map of
the interface objects that it bound.  `DeviceInfo` exposes the same
`Arc<UsbInterface>` objects to callers.  `UsbInterface` owns its descriptor,
endpoint descriptors, and an optional static driver binding; it references its
immutable `UsbDevice` descriptor through `Arc`.

On a successful `probe`, the core records the driver in the interface.  On a
disconnect transition, it removes the host-owned interface set, clears each
binding, and calls that driver's `disconnect` once.  An ID match whose `probe`
returns an error is not a claim; the next matching entry is tried.

Callbacks run from normal enumeration/refresh context, never from the xHCI IRQ
handler.  There is no registry lock held across `probe` or `disconnect`.

## Current limitation

This first step establishes descriptor ownership only.  It does not yet pass
the live `DeviceOp` to a class driver or create endpoint workers.  Consequently
no built-in driver is registered by default, preserving existing `usbfs` and
userspace interface-claim behavior.  A future functional class driver must
define its transfer ownership and endpoint lifecycle before it is registered.

## Validation

Unit tests cover wildcard and VID/PID/interface-class matching, continuing
after a rejecting probe, successful binding, and disconnect.  A host test
covers the integration point from `USBHost::probe_changes` to driver binding.
