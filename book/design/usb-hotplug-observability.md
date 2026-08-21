# USB hotplug topology and observability

## Problem and success criteria

The current kernel USB path can enumerate devices during the initial probe and
can defer an xHCI root-port event from hard IRQ context to the usbfs refresh
task. It does not model a disconnect as a probe result, however. A port remains
in `Probed`, the USB core retains the old device and child-hub topology, and
usbfs cannot mark the device absent. External USB 2.0 hub ports also have no
continuously armed status-change endpoint, so they need an explicit task-context
status poll until that endpoint is implemented.

This change targets users debugging USB on physical boards, initially the
SpacemiT K3 COM260. It succeeds when:

- a connection or disconnection after boot wakes or is found by the deferred
  topology worker without doing slow work in hard IRQ context;
- root-hub and external-hub ports emit explicit connected/disconnected events;
- disconnecting a hub removes its complete descendant topology;
- a later connection on the same port can be enumerated again;
- usbfs marks removed devices absent and prints one stable state line containing
  bus/device number, logical id, VID, PID, class, and hub/device kind;
- K3 can repeatedly disconnect and reconnect the tested hub/device without
  exhausting xHCI slots.

Opening and driving every USB class is not a goal. Hotplug reports discovery
state; functional support still depends on the relevant class driver. A fully
armed external-hub interrupt endpoint is also a separate optimization: the
initial implementation polls hub port status in task context at a bounded
interval.

## Sources and existing implementations

- USB 2.0, chapter 11 defines hub port status/change reporting and the class
  requests used to read and clear port changes. The current specification
  bundle is published by the
  [USB-IF document library](https://www.usb.org/documents?search=usb+2.0+specification).
- Intel xHCI 1.2c defines root-port status change events and `Disable Slot`; the
  current public specification is the
  [xHCI requirements specification](https://www.intel.com/content/www/us/en/content-details/868295/extensible-host-controller-interface-for-universal-serial-bus-xhci-requirements-specification-r1-2c.html).
- Repository commit `106bede3070da3c44fd1cc61190aa95e96149ca6`
  already introduced the intended `PortEvent`, monotonic logical device ids,
  recursive topology removal, and usbfs absent-state propagation on the newer
  development line. This backport keeps those semantics while adapting them to
  the older endpoint API on the K3 branch.

## Alternatives

### Log only the xHCI IRQ

This is small but incorrect as a hotplug implementation. It observes a root
port interrupt without changing the port state, removing topology, updating
usbfs, or allowing re-enumeration.

### Treat an empty probe result as disconnect

The existing probe result is incremental: no returned device normally means
there was no change. Reinterpreting it as a full snapshot would remove healthy
devices on every quiet pass.

### Arm the external hub interrupt endpoint immediately

This is the lowest-latency long-term implementation, but the current K3 branch
does not retain a hub interrupt-endpoint session or distinguish its completion
from ordinary transfer activity. Adding that ownership and re-arm lifecycle
would couple this focused topology fix to the larger endpoint-lifecycle
refactor. It remains a later optimization.

### Deferred bounded polling plus explicit events (selected)

Root-port IRQs keep their immediate deferred path. The usbfs task additionally
polls topology at a bounded interval so downstream ports of an external hub are
observed. Both sources feed the same `PortEvent` and topology owner, avoiding a
second state model. The cost is a small number of hub `GET_STATUS` control
requests while a hub is present.

## Ownership and flow

Hard IRQ processing only consumes controller events, marks the host dirty, and
wakes `usbfs-refresh`. The task owns all slow control transfers and topology
mutation:

```text
xHCI IRQ or poll deadline
        -> usbfs-refresh
        -> USB core probes each hub
        -> PortEvent::Connected / Disconnected
        -> topology mutation and HCD detach
        -> ProbeChanges
        -> usbfs present/absent record + [usb-hotplug] log
```

The USB core is the only topology owner. It maps `(parent hub, port)` to a
monotonic logical device id and optional child hub. Disconnect walks descendants
leaf-first before removing the parent. Logical ids are never derived from xHCI
slot ids, because the controller may reuse a slot after `Disable Slot`.

usbfs remains the owner of bus-visible device numbers and open-file state. A
disconnect marks a record absent and removes unopened publication data. The
current branch continues to defer topology refresh while a host has open usbfs
leases; changing active-transfer disconnect semantics belongs to the endpoint
lifecycle work and is not silently approximated here.

## Failure handling and rollback

- Port status/control errors are returned from the probe and logged by usbfs;
  no connected/disconnected state line is emitted for an uncommitted change.
- xHCI `Disable Slot` failure is reported and the software object is retained
  rather than declaring a clean detach.
- A repeated disconnect for an already empty port is idempotent.
- Removing the bounded poll restores the old IRQ-only behavior; removing the
  explicit topology/event API requires reverting the usbfs absent-state
  consumer at the same time.

## Validation

- A regression test proves a `Probed` port emits one disconnect and returns to
  the re-enumerable state.
- A host API test proves connected and disconnected probe changes cross the
  backend boundary without being collapsed into a device list.
- `cargo test --locked -p crab-usb` and targeted clippy validate the shared
  driver.
- `cargo xtask clippy --package starry-kernel` (or the applicable K3-target
  subset if an unrelated feature matrix fails) and the K3 Starry build validate
  the OS integration.
- Physical-board completion requires: boot, unplug, observe one
  `state=disconnected`, reconnect, observe one `state=connected` and a new
  successful `device descriptor ok`, repeated at least three times.
