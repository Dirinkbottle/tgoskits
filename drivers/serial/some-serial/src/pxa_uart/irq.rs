//! Hard-IRQ endpoint for a PXA/XScale UART.
//!
//! Owns a disjoint view of the same MMIO window as the paired [`PxaUart`]
//! plus a private LSR error latch used while draining the RX FIFO inside the
//! IRQ handler.

use rdif_serial::{IrqRxSink, SerialEventSet, SerialIrqEvent, UartIrq};

use super::{
    ier_for_events, read_reg_base,
    regs::{UART_IER, UART_IIR, UART_LSR, UART_MSR, ier, iir, lsr, msr},
    rx::{read_rx_sample, rx_errors_from_sample},
    write_reg_base,
};

/// Maximum samples drained from the RX FIFO in one IRQ pass.
const RX_SAMPLE_BUDGET: usize = 256;

/// Hard-IRQ endpoint for a PXA/XScale UART.
///
/// Owns a disjoint view of the same MMIO window as [`PxaUart`] plus a private
/// LSR error latch used while draining the RX FIFO inside the IRQ handler.
pub struct PxaUartIrq {
    base: usize,
    saved_lsr: lsr::Flags,
}

impl PxaUartIrq {
    /// Create an IRQ endpoint sharing `base` with its paired port.
    ///
    /// The port and IRQ endpoints must run on disjoint contexts under the
    /// device-serialization contract of `rdif_serial::UartPort`.
    pub(super) fn new(base: usize) -> Self {
        Self {
            base,
            saved_lsr: lsr::Flags::empty(),
        }
    }

    fn read_reg(&self, reg: u8) -> u8 {
        read_reg_base(self.base, reg)
    }

    fn write_reg(&self, reg: u8, val: u8) {
        write_reg_base(self.base, reg, val)
    }

    fn read_flags<F: bitflags::Flags<Bits = u8>>(&self, reg: u8) -> F {
        F::from_bits_retain(self.read_reg(reg))
    }

    fn write_flags<F: bitflags::Flags<Bits = u8>>(&self, reg: u8, val: F) {
        self.write_reg(reg, val.bits())
    }

    /// Map the current IIR pending interrupt to a stable event class.
    fn next_event(&self) -> Option<SerialEventSet> {
        let iir: iir::Flags = self.read_flags(UART_IIR);
        iir_to_event(iir)
    }

    fn ack_modem_status(&self) {
        // Reading MSR clears the delta bits, which de-asserts the modem IRQ.
        let _: msr::Flags = self.read_flags(UART_MSR);
    }

    /// Disable the given event sources in IER (keeps UUE untouched).
    fn mask(&self, events: SerialEventSet) {
        let mut ier: ier::Flags = self.read_flags(UART_IER);
        ier.remove(ier_for_events(events));
        self.write_flags(UART_IER, ier);
    }

    /// Drain RX samples from the FIFO, applying the PXA Errata #20 RTOIE
    /// toggle around the drain (mirror of Linux `serial_spacemit.c
    /// receive_chars()`: disable IER[RTOIE] before reading, re-enable it once
    /// the FIFO is empty). Returns the number of samples read.
    fn drain_rx_samples(
        &mut self,
        rx: &mut dyn IrqRxSink,
        event: &mut SerialIrqEvent,
        rx_samples: &mut usize,
    ) -> usize {
        let mut ier = self.read_flags::<ier::Flags>(UART_IER);
        ier.remove(ier::Flags::RTOIE);
        self.write_flags(UART_IER, ier);

        let start = *rx_samples;
        while *rx_samples < RX_SAMPLE_BUDGET {
            let Some(sample) = read_rx_sample(self.base, &mut self.saved_lsr) else {
                break;
            };
            event.rx_errors |= rx_errors_from_sample(sample);
            rx.push(sample);
            *rx_samples += 1;
        }

        let mut ier = self.read_flags::<ier::Flags>(UART_IER);
        ier.insert(ier::Flags::RTOIE);
        self.write_flags(UART_IER, ier);

        *rx_samples - start
    }
}

impl UartIrq for PxaUartIrq {
    fn handle(&mut self, rx: &mut dyn IrqRxSink) -> Option<SerialIrqEvent> {
        const IRQ_PASS_BUDGET: usize = 32;

        let mut event = SerialIrqEvent::default();
        let mut rx_samples = 0;
        for _ in 0..IRQ_PASS_BUDGET {
            let Some(current) = self.next_event() else {
                // IIR reports no pending interrupt, yet RX data or errors are
                // still in the FIFO. This is the Errata #20 signature: the
                // receiver-timeout condition (RTOIE/CTI) was lost while bytes
                // sat below the FIFO trigger level. Drain them here so input
                // cannot freeze with data stranded, and re-arm RTOIE for the
                // next batch.
                let lsr = self.read_flags::<lsr::Flags>(UART_LSR);
                // This LSR read clears sticky error bits; fold them into the
                // saved latch before the drain re-reads LSR, or the error
                // flag for a stranded byte is lost (mirror of
                // `read_lsr_preserving`).
                self.saved_lsr
                    .insert(lsr & (lsr::Flags::ERROR_MASK | lsr::Flags::FIFOE));
                if lsr.intersects(lsr::Flags::DR | lsr::Flags::ERROR_MASK) {
                    self.drain_rx_samples(rx, &mut event, &mut rx_samples);
                    event.events |= SerialEventSet::RX_DATA;
                }
                break;
            };
            event.events |= current;

            if current.intersects(SerialEventSet::RX) {
                let drained = self.drain_rx_samples(rx, &mut event, &mut rx_samples);
                if drained == 0 {
                    break;
                }
            }
            if current.contains(SerialEventSet::MODEM_STATUS) {
                self.ack_modem_status();
            }
            if current.contains(SerialEventSet::FAULT) {
                // Unknown interrupt source: mask everything (keep unit on) and stop.
                self.write_flags(UART_IER, ier::Flags::UUE);
                break;
            }

            // TX_SPACE is level-triggered on THRE; mask it and hand the source
            // back to the port so the maintenance task re-enables it on demand.
            let rearm = current & SerialEventSet::TX_SPACE;
            if !rearm.is_empty() {
                self.mask(rearm);
                event.rearm |= rearm;
            }
        }

        (!event.events.is_empty()).then_some(event)
    }
}

/// Map an IIR value to the stable event class it reports.
///
/// Returns `None` when `NO_INT` (IIR bit 0) is set, meaning the shared
/// interrupt was not raised by this UART. Otherwise the pending interrupt id
/// in IIR bits [3:1] selects the event; the PXA `MSI` id is `0x00`.
fn iir_to_event(iir: iir::Flags) -> Option<SerialEventSet> {
    if iir.contains(iir::Flags::NO_INT) {
        return None;
    }

    let id = iir & iir::Flags::ID_MASK;
    let event = if id == iir::Flags::RLSI {
        SerialEventSet::RX_STATUS
    } else if id == iir::Flags::RDI {
        SerialEventSet::RX_DATA
    } else if id == iir::Flags::CTI {
        SerialEventSet::RX_TIMEOUT
    } else if id == iir::Flags::THRI {
        SerialEventSet::TX_SPACE
    } else if id == iir::Flags::MSI {
        SerialEventSet::MODEM_STATUS
    } else {
        SerialEventSet::FAULT
    };
    Some(event)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reports_none_when_shared_irq_is_not_pending() {
        assert_eq!(iir_to_event(iir::Flags::NO_INT), None);
    }

    #[test]
    fn maps_each_pending_interrupt_id_to_its_event_class() {
        assert_eq!(
            iir_to_event(iir::Flags::RLSI),
            Some(SerialEventSet::RX_STATUS)
        );
        assert_eq!(iir_to_event(iir::Flags::RDI), Some(SerialEventSet::RX_DATA));
        assert_eq!(
            iir_to_event(iir::Flags::CTI),
            Some(SerialEventSet::RX_TIMEOUT)
        );
        assert_eq!(
            iir_to_event(iir::Flags::THRI),
            Some(SerialEventSet::TX_SPACE)
        );
    }

    #[test]
    fn maps_msi_zero_id_to_modem_status_when_no_int_is_clear() {
        // MSI has id 0x00; with NO_INT clear this is a real modem-status IRQ.
        assert_eq!(
            iir_to_event(iir::Flags::MSI),
            Some(SerialEventSet::MODEM_STATUS)
        );
    }

    #[test]
    fn maps_unknown_interrupt_id_to_fault() {
        let unknown = iir::Flags::from_bits_retain(0x08);
        assert_eq!(iir_to_event(unknown), Some(SerialEventSet::FAULT));
    }
}
