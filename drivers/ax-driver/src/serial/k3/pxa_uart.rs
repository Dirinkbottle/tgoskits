//! SpacemiT K3 PXA/XScale UART serial probe
//!
//! Translated from Linux drivers/tty/serial/serial_spacemit.c probe path.
//! Matches FDT compatible "spacemit,k1-uart".

use alloc::format;

use log::info;
use rdrive::{probe::OnProbeError, register::ProbeFdt};
use some_serial::pxa_uart::PxaUart;

use super::super::{PlatformSerialDevice, erase_uart, prop_u32, serial_device_info};

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
    let clock_freq = prop_u32(node, "clock-frequency").unwrap_or(PXA_DEFAULT_CLOCK);

    info!(
        "K3 PXA UART at {:#x} (size {:#x}), clock={} Hz",
        base_reg.address, mmio_size, clock_freq
    );

    let raw = PxaUart::new(mmio_base, clock_freq);
    let serial = erase_uart(raw);
    let device_info = serial_device_info(&info, &base_reg);

    info!(
        "K3 PXA UART serial@{:#x} registered successfully",
        serial.hardware.register_base
    );

    plat_dev.register(PlatformSerialDevice::new(
        serial,
        device_info.path,
        device_info.alias_index,
        device_info.paddr,
        device_info.irq,
    ));
    Ok(())
}
