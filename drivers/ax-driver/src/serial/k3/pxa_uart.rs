//! SpacemiT K3 PXA UART serial probe
//!
//! Translated from Linux drivers/tty/serial/serial_spacemit.c probe path.
//! Matches FDT compatible "spacemit,k1-uart".

use alloc::format;

use log::info;
use rdrive::{probe::OnProbeError, register::ProbeFdt};
use some_serial::pxa_uart::PxaUart;

use super::super::{PlatformSerialDevice, prop_u32, serial_device_info, serial_runtime};

/// Default UART functional clock for PXA UART (14.7 MHz).
/// From Linux: the default `rate = 14700000` in pxa_set_baudrate_clk().
const PXA_DEFAULT_CLOCK: u32 = 14_700_000;

model_register!(
    name: "K3 PXA UART serial",
    level: ProbeLevel::PreKernel,
    priority: ProbePriority::DEFAULT,
    probe_kinds: &[
        ProbeKind::Fdt {
            compatibles: &["spacemit,k1-uart"],
            on_probe: probe
        },
    ],
);

fn probe(probe: ProbeFdt<'_>) -> Result<(), OnProbeError> {
    let (info, plat_dev) = probe.into_parts();

    info!("Probing K3 PXA UART serial device: {}", info.node.name());

    let base_reg = info
        .node
        .regs()
        .into_iter()
        .next()
        .ok_or_else(|| OnProbeError::other(format!("[{}] has no reg", info.node.name())))?;

    let mmio_size = base_reg.size.unwrap_or(0x1000);
    let mmio_base = crate::mmio::iomap(base_reg.address as usize, mmio_size as usize)?;

    let node = info.node.as_node();

    // Read clock-frequency from DTS, or use PXA default
    let clock_freq = prop_u32(node, "clock-frequency").unwrap_or(PXA_DEFAULT_CLOCK);

    info!(
        "K3 PXA UART at {:#x} (size {:#x}), clock={} Hz",
        base_reg.address, mmio_size, clock_freq
    );

    let raw = PxaUart::new(mmio_base, clock_freq);
    let serial = serial_runtime(raw);

    info!(
        "K3 PXA UART serial@{:#x} registered successfully",
        serial.base_addr
    );

    let device_info = serial_device_info(&info, &base_reg, serial.base_addr, serial.baudrate);

    plat_dev.register(PlatformSerialDevice::new(
        serial.name.into(),
        device_info,
        serial.runtime,
    ));
    Ok(())
}
