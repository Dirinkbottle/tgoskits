//! SpacemiT K3 PXA/XScale UART driver
//!
//! Translated from Linux drivers/tty/serial/serial_spacemit.c
//!
//! This UART is an extended 16550-compatible controller found on SpacemiT K1/K3 SoCs.
//! Key differences from standard 16550:
//! - 32-bit MMIO access with reg-shift=2 (offset * 4)
//! - Extended FIFO (64 bytes per direction)
//! - PXA-specific IER bits: UUE (Unit Enable), RTOIE (RX Timeout), DMAE (DMA Enable)
//! - PXA-specific FCR trigger levels: PXAR1, PXAR8, PXAR32
//! - PXA-specific MCR bit: AFE (Auto Flow Control)

extern crate alloc;

mod regs;

use core::ptr::{NonNull, read_volatile, write_volatile};

use bitflags::Flags;
use rdif_serial::{
    Config, ConfigError, DataBits, IrqRxSink, Parity, RxErrorFlags, RxFlag, RxSample,
    SerialEventSet, SerialIrqEvent, SplitUart, StopBits, UartInfo, UartIrq, UartParts, UartPort,
};
use regs::{
    FIFO_SIZE, REG_WIDTH, UART_DLH, UART_DLL, UART_FCR, UART_IER, UART_IIR, UART_LCR, UART_LSR,
    UART_MCR, UART_MSR, UART_RBR, UART_THR, fcr, ier, iir, lcr, lsr, mcr, msr,
};

use crate::{PollingEvent, PollingUart, TransferError};

// ============================================================================
// PXA UART driver struct
// ============================================================================

/// SpacemiT K3 PXA/XScale UART driver.
///
/// Translated from Linux `drivers/tty/serial/serial_spacemit.c`.
/// Implements the non-DMA, interrupt-driven path.
pub struct PxaUart {
    /// MMIO base address
    base: usize,
    /// UART functional clock frequency in Hz
    clock_freq: u32,
    /// Saved LSR error bits (cleared on LSR read, preserved across IRQ handling)
    saved_lsr: lsr::Flags,
    /// Current baud rate
    current_baud: u32,
    /// Currently configured IER value (cached)
    ier: ier::Flags,
    /// Currently configured MCR value (cached)
    mcr: mcr::Flags,
    /// Currently configured LCR value (cached, without DLAB)
    lcr: lcr::Flags,
}

impl PxaUart {
    /// Create a new PXA UART instance from an MMIO base address and clock frequency.
    ///
    /// `base` is a non-null pointer to the MMIO region.
    /// `clock_freq` is the UART functional clock in Hz (e.g. 14_700_000).
    pub fn new(base: NonNull<u8>, clock_freq: u32) -> Self {
        Self {
            base: base.as_ptr() as usize,
            clock_freq,
            saved_lsr: lsr::Flags::empty(),
            current_baud: 0,
            ier: ier::Flags::empty(),
            mcr: mcr::Flags::empty(),
            lcr: lcr::Flags::WLEN8,
        }
    }

    // --- MMIO access helpers ---
    // Register offset is multiplied by 4 (reg-shift=2)

    fn read_reg(&self, reg: u8) -> u8 {
        let addr = self.base + (reg as usize) * REG_WIDTH;
        unsafe { read_volatile(addr as *const u8) }
    }

    fn write_reg(&self, reg: u8, val: u8) {
        let addr = self.base + (reg as usize) * REG_WIDTH;
        unsafe { write_volatile(addr as *mut u8, val) };
    }

    fn read_flags<F: Flags<Bits = u8>>(&self, reg: u8) -> F {
        F::from_bits_retain(self.read_reg(reg))
    }

    fn write_flags<F: Flags<Bits = u8>>(&self, reg: u8, val: F) {
        self.write_reg(reg, val.bits())
    }

    // --- Baud rate ---

    /// Calculate divisor from clock frequency and baud rate.
    /// Standard 16550 formula, rounded to the closest usable integral divisor.
    fn divisor(&self, baudrate: u32) -> Option<u16> {
        if baudrate == 0 || self.clock_freq == 0 {
            return None;
        }
        let baudrate = baudrate as u64;
        let div = (self.clock_freq as u64 + 8 * baudrate) / (16 * baudrate);
        if div == 0 || div > 0xFFFF {
            return None;
        }
        Some(div as u16)
    }

    /// Set the baud rate divisor.
    /// Follows the PXA Errata #75 sequence: write DLH → read DLH → write DLL.
    fn set_divisor(&self, divisor: u16) {
        // Enter DLAB mode
        let lcr = self.read_flags::<lcr::Flags>(UART_LCR);
        self.write_flags(UART_LCR, lcr | lcr::Flags::DLAB);

        // PXA-specific sequence: write DLH, then read DLH, then write DLL
        self.write_reg(UART_DLH, ((divisor >> 8) & 0xFF) as u8);
        self.read_reg(UART_DLH);
        self.write_reg(UART_DLL, (divisor & 0xFF) as u8);

        // PXA Errata #75: read DLL twice
        self.read_reg(UART_DLL);
        let dll = self.read_reg(UART_DLL);

        if (dll as u16) != (divisor & 0xFF) {
            log::warn!(
                "PXA UART baud divisor mismatch: wrote {:04x}, read DLL={:02x}",
                divisor,
                dll
            );
        }

        // Exit DLAB mode
        self.write_flags(UART_LCR, lcr);
    }

    // --- FIFO trigger level selection ---

    /// Select FCR flags based on effective baud rate.
    /// From Linux serial_pxa_set_termios():
    /// - uartclk/quot < 2400*16  → PXAR1 (1-byte trigger)
    /// - uartclk/quot < 230400*16 → PXAR8 (8-byte trigger)
    /// - otherwise                → PXAR32 (32-byte trigger)
    fn select_fcr_flags(&self, divisor: u32) -> fcr::Flags {
        let rate = self.clock_freq / divisor;
        if rate < 2400 * 16 {
            fcr::Flags::ENABLE_FIFO | fcr::Flags::PXAR1
        } else if rate < 230400 * 16 {
            fcr::Flags::ENABLE_FIFO | fcr::Flags::PXAR8
        } else {
            fcr::Flags::ENABLE_FIFO | fcr::Flags::PXAR32
        }
    }

    // --- LSR with error latch ---

    /// Read LSR, merging saved error bits so they survive across IRQ handling.
    fn read_lsr_preserving(&mut self) -> lsr::Flags {
        let lsr: lsr::Flags = self.read_flags(UART_LSR);
        self.saved_lsr
            .insert(lsr & (lsr::Flags::ERROR_MASK | lsr::Flags::FIFOE));
        lsr | self.saved_lsr
    }

    /// Drain any remaining RX data on shutdown.
    fn drain_rx(&self) {
        while self
            .read_flags::<lsr::Flags>(UART_LSR)
            .contains(lsr::Flags::DR)
        {
            self.read_reg(UART_RBR);
        }
    }
}

// ============================================================================
// ============================================================================
// IRQ-owned endpoint
// ============================================================================

/// IRQ endpoint for the PXA UART.
///
/// Owned by the registered IRQ callback; drains RX samples into the runtime
/// sink and never touches TX state.
pub struct PxaUartIrq {
    base: usize,
    saved_lsr: lsr::Flags,
}

impl PxaUartIrq {
    /// Creates an IRQ endpoint for the given MMIO base address.
    pub fn new(base: NonNull<u8>) -> Self {
        Self {
            base: base.as_ptr() as usize,
            saved_lsr: lsr::Flags::empty(),
        }
    }

    fn read_reg(&self, reg: u8) -> u8 {
        let addr = self.base + (reg as usize) * REG_WIDTH;
        unsafe { read_volatile(addr as *const u8) }
    }

    fn read_flags<F: Flags<Bits = u8>>(&self, reg: u8) -> F {
        F::from_bits_retain(self.read_reg(reg))
    }

    fn write_flags<F: Flags<Bits = u8>>(&self, reg: u8, val: F) {
        let addr = self.base + (reg as usize) * REG_WIDTH;
        unsafe { write_volatile(addr as *mut u8, val.bits()) };
    }

    /// Read LSR, merging saved error bits so they survive across IRQ handling.
    fn read_lsr_preserving(&mut self) -> lsr::Flags {
        let lsr: lsr::Flags = self.read_flags(UART_LSR);
        self.saved_lsr
            .insert(lsr & (lsr::Flags::ERROR_MASK | lsr::Flags::FIFOE));
        lsr | self.saved_lsr
    }

    fn read_rx_sample(&mut self) -> Option<RxSample> {
        let lsr = self.read_lsr_preserving();

        if !lsr.intersects(lsr::Flags::DR | lsr::Flags::ERROR_MASK) {
            return None;
        }

        let byte = lsr
            .contains(lsr::Flags::DR)
            .then(|| self.read_reg(UART_RBR));

        let flag = if lsr.contains(lsr::Flags::BI) {
            RxFlag::Break
        } else if lsr.contains(lsr::Flags::PE) {
            RxFlag::Parity
        } else if lsr.contains(lsr::Flags::FE) {
            RxFlag::Framing
        } else {
            RxFlag::Normal
        };

        let overrun = lsr.contains(lsr::Flags::OE);

        // Clear consumed error flags from saved latch
        self.saved_lsr
            .remove(lsr::Flags::ERROR_MASK | lsr::Flags::FIFOE);

        Some(RxSample {
            byte,
            flag,
            overrun,
        })
    }
}

impl UartIrq for PxaUartIrq {
    fn handle(&mut self, rx: &mut dyn IrqRxSink) -> Option<SerialIrqEvent> {
        const IRQ_PASS_BUDGET: usize = 32;
        const RX_SAMPLE_BUDGET: usize = 256;

        let mut event = SerialIrqEvent::default();
        let mut rx_samples = 0;
        for _ in 0..IRQ_PASS_BUDGET {
            let iir: iir::Flags = self.read_flags(UART_IIR);

            // No interrupt pending
            if iir.contains(iir::Flags::NO_INT) {
                break;
            }

            let id = iir & iir::Flags::ID_MASK;
            let current = if id == iir::Flags::RLSI {
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
            event.events |= current;

            if current.intersects(SerialEventSet::RX) {
                let before = rx_samples;
                while rx_samples < RX_SAMPLE_BUDGET {
                    let Some(sample) = self.read_rx_sample() else {
                        break;
                    };
                    event.rx_errors |= rx_errors_from_sample(sample);
                    rx.push(sample);
                    rx_samples += 1;
                }
                if rx_samples == RX_SAMPLE_BUDGET || rx_samples == before {
                    break;
                }
            }

            if current.contains(SerialEventSet::MODEM_STATUS) {
                let _: msr::Flags = self.read_flags(UART_MSR);
            }
            if current.contains(SerialEventSet::FAULT) {
                // Mask every interrupt source until the maintenance task
                // recovers the port.
                self.write_flags(UART_IER, ier::Flags::UUE);
                break;
            }

            let rearm = current & SerialEventSet::TX_SPACE;
            if !rearm.is_empty() {
                let ier: ier::Flags = self.read_flags(UART_IER);
                self.write_flags(UART_IER, ier & !ier::Flags::THRI);
                event.rearm |= rearm;
            }
        }

        (!event.events.is_empty()).then_some(event)
    }
}

fn rx_errors_from_sample(sample: RxSample) -> RxErrorFlags {
    let mut errors = match sample.flag {
        RxFlag::Normal => RxErrorFlags::empty(),
        RxFlag::Break => RxErrorFlags::BREAK,
        RxFlag::Parity => RxErrorFlags::PARITY,
        RxFlag::Framing => RxErrorFlags::FRAMING,
    };
    if sample.overrun {
        errors |= RxErrorFlags::OVERRUN;
    }
    errors
}

fn ier_bits_for_events(events: SerialEventSet) -> ier::Flags {
    let mut bits = ier::Flags::empty();
    if events.intersects(SerialEventSet::RX) {
        bits.insert(ier::Flags::RDI | ier::Flags::RLSI | ier::Flags::RTOIE);
    }
    if events.contains(SerialEventSet::TX_SPACE) {
        bits.insert(ier::Flags::THRI);
    }
    bits
}

// ============================================================================
// UartPort trait implementation
// ============================================================================

impl UartPort for PxaUart {
    // --- startup: serial_pxa_startup() ---
    fn startup(&mut self, config: &Config) -> Result<(), ConfigError> {
        // Line 1101-1103: Clear and disable FIFOs
        self.write_flags(UART_FCR, fcr::Flags::ENABLE_FIFO);
        self.write_flags(
            UART_FCR,
            fcr::Flags::ENABLE_FIFO | fcr::Flags::CLEAR_RX | fcr::Flags::CLEAR_TX,
        );
        self.write_flags(UART_FCR, fcr::Flags::empty());

        // Line 1106-1109: Clear interrupt registers by reading them
        self.read_reg(UART_LSR);
        self.read_reg(UART_RBR);
        self.read_reg(UART_IIR);
        self.read_reg(UART_MSR);

        // Line 1112: Initialize with 8-bit word
        self.write_flags(UART_LCR, lcr::Flags::WLEN8);
        self.lcr = lcr::Flags::WLEN8;

        // Line 1115-1118: Set MCR OUT2 (interrupt gate)
        self.mcr = mcr::Flags::OUT2;
        self.write_flags(UART_MCR, self.mcr);

        // Apply configuration (baud rate, data bits, stop bits, parity)
        self.set_config(config)?;

        // Line 1136-1138: Enable the unit while leaving every interrupt
        // source masked; the runtime enables RX/TX events through rearm().
        self.ier = ier::Flags::UUE;
        self.write_flags(UART_IER, self.ier);

        // Line 1142-1145: Clear interrupt registers again
        self.read_reg(UART_LSR);
        self.read_reg(UART_RBR);
        self.read_reg(UART_IIR);
        self.read_reg(UART_MSR);

        self.saved_lsr = lsr::Flags::empty();
        Ok(())
    }

    // --- shutdown: serial_pxa_shutdown() ---
    fn shutdown(&mut self) {
        // Disable all interrupts
        self.ier = ier::Flags::empty();
        self.write_flags(UART_IER, ier::Flags::empty());

        // Clear MCR OUT2
        self.mcr.remove(mcr::Flags::OUT2);
        self.write_flags(UART_MCR, self.mcr);

        // Disable break condition and clear FIFOs
        let lcr = self.read_flags::<lcr::Flags>(UART_LCR) & !lcr::Flags::SBRK;
        self.write_flags(UART_LCR, lcr);
        self.write_flags(
            UART_FCR,
            fcr::Flags::ENABLE_FIFO | fcr::Flags::CLEAR_RX | fcr::Flags::CLEAR_TX,
        );
        self.write_flags(UART_FCR, fcr::Flags::empty());

        self.drain_rx();
    }

    // --- set_config: serial_pxa_set_termios() (simplified) ---
    fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        // --- Baud rate ---
        if let Some(baudrate) = config.baudrate {
            let divisor = self.divisor(baudrate).ok_or(ConfigError::InvalidBaudrate)?;
            self.set_divisor(divisor);
            self.current_baud = baudrate;
        }

        // --- Word length ---
        if let Some(data_bits) = config.data_bits {
            let wlen = match data_bits {
                DataBits::Five => lcr::Flags::WLEN5,
                DataBits::Six => lcr::Flags::WLEN6,
                DataBits::Seven => lcr::Flags::WLEN7,
                DataBits::Eight => lcr::Flags::WLEN8,
            };
            self.lcr.remove(lcr::Flags::WLEN_MASK);
            self.lcr.insert(wlen);
        }

        // --- Stop bits ---
        if let Some(stop_bits) = config.stop_bits {
            match stop_bits {
                StopBits::One => self.lcr.remove(lcr::Flags::STOP),
                StopBits::Two => self.lcr.insert(lcr::Flags::STOP),
            }
        }

        // --- Parity ---
        if let Some(parity) = config.parity {
            self.lcr
                .remove(lcr::Flags::PARITY | lcr::Flags::EPAR | lcr::Flags::SPAR);
            match parity {
                Parity::None => {}
                Parity::Odd => self.lcr.insert(lcr::Flags::PARITY),
                Parity::Even => self.lcr.insert(lcr::Flags::PARITY | lcr::Flags::EPAR),
                Parity::Mark => self.lcr.insert(lcr::Flags::PARITY | lcr::Flags::SPAR),
                Parity::Space => {
                    self.lcr
                        .insert(lcr::Flags::PARITY | lcr::Flags::EPAR | lcr::Flags::SPAR);
                }
            }
        }

        // Write LCR (without DLAB)
        self.write_flags(UART_LCR, self.lcr);

        // --- FIFO control ---
        let divisor = self.divisor(self.current_baud).unwrap_or(1) as u32;
        let fcr = self.select_fcr_flags(divisor);
        self.write_flags(UART_FCR, fcr);

        // Ensure UUE is set in IER
        self.ier.insert(ier::Flags::UUE);
        self.write_flags(UART_IER, self.ier);

        Ok(())
    }

    // --- read_rx: receive_chars() ---
    fn read_rx(&mut self) -> Option<RxSample> {
        let lsr = self.read_lsr_preserving();

        if !lsr.intersects(lsr::Flags::DR | lsr::Flags::ERROR_MASK) {
            return None;
        }

        let byte = lsr
            .contains(lsr::Flags::DR)
            .then(|| self.read_reg(UART_RBR));

        let flag = if lsr.contains(lsr::Flags::BI) {
            RxFlag::Break
        } else if lsr.contains(lsr::Flags::PE) {
            RxFlag::Parity
        } else if lsr.contains(lsr::Flags::FE) {
            RxFlag::Framing
        } else {
            RxFlag::Normal
        };

        let overrun = lsr.contains(lsr::Flags::OE);

        // Clear consumed error flags from saved latch
        self.saved_lsr
            .remove(lsr::Flags::ERROR_MASK | lsr::Flags::FIFOE);

        Some(RxSample {
            byte,
            flag,
            overrun,
        })
    }

    // --- write_tx: drain the caller buffer while the FIFO accepts bytes ---
    fn write_tx(&mut self, bytes: &[u8]) -> usize {
        let mut written = 0;
        while written < bytes.len() && written < FIFO_SIZE {
            if !self
                .read_flags::<lsr::Flags>(UART_LSR)
                .contains(lsr::Flags::THRE)
            {
                break;
            }
            self.write_reg(UART_THR, bytes[written]);
            written += 1;
        }
        written
    }

    // --- tx_idle ---
    fn tx_idle(&mut self) -> bool {
        let lsr: lsr::Flags = self.read_flags(UART_LSR);
        lsr.contains(lsr::Flags::THRE | lsr::Flags::TEMT)
    }

    // --- mask_all: leave the unit enabled with every IRQ source masked ---
    fn mask_all(&mut self) {
        self.ier = ier::Flags::UUE;
        self.write_flags(UART_IER, self.ier);
    }

    // --- rearm: enable requested events and close the enable/readiness race ---
    fn rearm(&mut self, sources: SerialEventSet) -> SerialEventSet {
        let mut ier = self.ier;
        ier.insert(ier_bits_for_events(sources));
        self.ier = ier;
        self.write_flags(UART_IER, ier);

        let lsr = self.read_lsr_preserving();
        let mut ready = SerialEventSet::empty();
        if sources.intersects(SerialEventSet::RX)
            && lsr.intersects(lsr::Flags::DR | lsr::Flags::ERROR_MASK)
        {
            ready |= if lsr.contains(lsr::Flags::DR) {
                SerialEventSet::RX_DATA
            } else {
                SerialEventSet::RX_STATUS
            };
        }
        if sources.contains(SerialEventSet::TX_SPACE) && lsr.contains(lsr::Flags::THRE) {
            ready |= SerialEventSet::TX_SPACE;
        }
        if !ready.is_empty() {
            self.ier.remove(ier_bits_for_events(ready));
            self.write_flags(UART_IER, self.ier);
        }
        ready
    }
}

// ============================================================================
// SplitUart trait implementation
// ============================================================================

impl SplitUart for PxaUart {
    type Port = Self;
    type Irq = PxaUartIrq;

    fn runtime_info(&self) -> UartInfo {
        UartInfo {
            name: "PXA UART",
            register_base: self.base,
            initial_baudrate: self.current_baud,
        }
    }

    fn split(self) -> UartParts<Self::Port, Self::Irq> {
        let irq = PxaUartIrq {
            base: self.base,
            saved_lsr: lsr::Flags::empty(),
        };
        UartParts::new(self, irq)
    }
}

// ============================================================================
// PollingUart implementation (early console)
// ============================================================================

impl PollingUart for PxaUart {
    fn poll_status(&mut self) -> PollingEvent {
        let lsr = self.read_lsr_preserving();
        let mut event = PollingEvent::empty();

        if lsr.contains(lsr::Flags::DR) {
            event |= PollingEvent::RX_READY;
        }
        if lsr.intersects(lsr::Flags::PE | lsr::Flags::FE | lsr::Flags::BI) {
            event |= PollingEvent::RX_ERROR;
        }
        if lsr.contains(lsr::Flags::OE) {
            event |= PollingEvent::RX_ERROR | PollingEvent::OVERRUN;
        }
        if lsr.contains(lsr::Flags::THRE) {
            event |= PollingEvent::TX_READY;
        }
        event
    }

    fn write_byte(&mut self, byte: u8) {
        // Wait for TX holding register to be empty
        while !self
            .read_flags::<lsr::Flags>(UART_LSR)
            .contains(lsr::Flags::THRE)
        {
            core::hint::spin_loop();
        }
        self.write_reg(UART_THR, byte);
    }

    fn read_byte(&mut self, status: PollingEvent) -> Option<Result<u8, TransferError>> {
        if !status.rx_ready() && !status.rx_error() {
            return None;
        }

        if self.saved_lsr.contains(lsr::Flags::OE) {
            let b = self.read_reg(UART_RBR);
            self.saved_lsr.remove(lsr::Flags::OE);
            return Some(Err(TransferError::Overrun(b)));
        }
        if self.saved_lsr.contains(lsr::Flags::PE) {
            let _ = self.read_reg(UART_RBR);
            self.saved_lsr.remove(lsr::Flags::PE);
            return Some(Err(TransferError::Parity));
        }
        if self.saved_lsr.contains(lsr::Flags::FE) {
            let _ = self.read_reg(UART_RBR);
            self.saved_lsr.remove(lsr::Flags::FE);
            return Some(Err(TransferError::Framing));
        }
        if self.saved_lsr.contains(lsr::Flags::BI) {
            let _ = self.read_reg(UART_RBR);
            self.saved_lsr.remove(lsr::Flags::BI);
            return Some(Err(TransferError::Break));
        }
        if status.rx_ready() {
            return Some(Ok(self.read_reg(UART_RBR)));
        }
        None
    }
}

// Safety: PxaUart only accesses its own MMIO range via raw pointers.
// The base address is provided by the platform and must be valid.
unsafe impl Send for PxaUart {}

// Safety: PxaUartIrq only accesses its own MMIO range via raw pointers.
// The base address is provided by the platform and must be valid.
unsafe impl Send for PxaUartIrq {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn divisor_rounds_to_nearest_value() {
        let uart = PxaUart::new(NonNull::dangling(), 14_700_000);

        assert_eq!(uart.divisor(115_200), Some(8));
    }
}
