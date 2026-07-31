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

use crate::{PollingUart, SerialEvent, TransferError};

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
        let base_addr = base.as_ptr() as usize;
        let current_baud = probe_current_baud(base_addr, clock_freq);
        Self {
            base: base_addr,
            clock_freq,
            saved_lsr: lsr::Flags::empty(),
            current_baud,
            ier: ier::Flags::empty(),
            mcr: mcr::Flags::empty(),
            lcr: lcr::Flags::WLEN8,
        }
    }

    // --- MMIO access helpers ---
    // Register offset is multiplied by 4 (reg-shift=2)

    fn read_reg(&self, reg: u8) -> u8 {
        read_reg_base(self.base, reg)
    }

    fn write_reg(&self, reg: u8, val: u8) {
        write_reg_base(self.base, reg, val)
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

    // --- Data-plane helpers shared by the trait impls ---

    pub fn read_rx(&mut self) -> Option<RxSample> {
        read_rx_sample(self.base, &mut self.saved_lsr)
    }

    pub fn tx_idle(&mut self) -> bool {
        let lsr: lsr::Flags = self.read_flags(UART_LSR);
        lsr.contains(lsr::Flags::THRE | lsr::Flags::TEMT)
    }

    pub fn poll_status(&mut self) -> SerialEvent {
        let lsr = self.read_lsr_preserving();
        let mut event = SerialEvent::empty();

        if lsr.contains(lsr::Flags::DR) {
            event |= SerialEvent::RX_READY;
        }
        if lsr.intersects(lsr::Flags::PE | lsr::Flags::FE | lsr::Flags::BI) {
            event |= SerialEvent::RX_ERROR;
        }
        if lsr.contains(lsr::Flags::OE) {
            event |= SerialEvent::RX_ERROR | SerialEvent::OVERRUN;
        }
        if lsr.contains(lsr::Flags::THRE) {
            event |= SerialEvent::TX_READY;
        }
        event
    }

    pub fn write_byte(&mut self, byte: u8) {
        while !self
            .read_flags::<lsr::Flags>(UART_LSR)
            .contains(lsr::Flags::THRE)
        {
            core::hint::spin_loop();
        }
        self.write_reg(UART_THR, byte);
    }

    pub fn read_byte(&mut self, status: SerialEvent) -> Option<Result<u8, TransferError>> {
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

    // --- Configuration queries (inherent helpers) ---

    pub fn baudrate(&self) -> u32 {
        self.current_baud
    }

    pub fn data_bits(&self) -> DataBits {
        match self.lcr & lcr::Flags::WLEN_MASK {
            lcr::Flags::WLEN5 => DataBits::Five,
            lcr::Flags::WLEN6 => DataBits::Six,
            lcr::Flags::WLEN7 => DataBits::Seven,
            _ => DataBits::Eight,
        }
    }

    pub fn stop_bits(&self) -> StopBits {
        if self.lcr.contains(lcr::Flags::STOP) {
            StopBits::Two
        } else {
            StopBits::One
        }
    }

    pub fn parity(&self) -> Parity {
        if !self.lcr.contains(lcr::Flags::PARITY) {
            Parity::None
        } else if self.lcr.contains(lcr::Flags::SPAR) {
            if self.lcr.contains(lcr::Flags::EPAR) {
                Parity::Space
            } else {
                Parity::Mark
            }
        } else if self.lcr.contains(lcr::Flags::EPAR) {
            Parity::Even
        } else {
            Parity::Odd
        }
    }

    pub fn enable_loopback(&mut self) {
        self.mcr.insert(mcr::Flags::LOOP);
        self.write_flags(UART_MCR, self.mcr);
    }

    pub fn disable_loopback(&mut self) {
        self.mcr.remove(mcr::Flags::LOOP);
        self.write_flags(UART_MCR, self.mcr);
    }

    pub fn is_loopback_enabled(&self) -> bool {
        self.mcr.contains(mcr::Flags::LOOP)
    }
}

// ============================================================================
// Configuration / lifecycle (inherent, shared by startup and set_config)
// ============================================================================

impl PxaUart {
    // --- startup: serial_pxa_startup() ---
    pub fn startup(&mut self, config: &Config) -> Result<(), ConfigError> {
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

        // Line 1142-1145: Clear interrupt registers again
        self.read_reg(UART_LSR);
        self.read_reg(UART_RBR);
        self.read_reg(UART_IIR);
        self.read_reg(UART_MSR);

        // Keep the UART unit enabled but leave every interrupt source masked; the
        // runtime maintenance task rearms the sources it wants to service.
        self.ier = ier::Flags::UUE;
        self.write_flags(UART_IER, self.ier);

        self.saved_lsr = lsr::Flags::empty();
        Ok(())
    }

    // --- shutdown: serial_pxa_shutdown() ---
    pub fn shutdown(&mut self) {
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
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
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

        // Ensure UUE is set in IER (the UART unit must stay enabled)
        self.ier.insert(ier::Flags::UUE);
        self.write_flags(UART_IER, self.ier);

        Ok(())
    }
}

// ============================================================================
// rdif-serial capability boundary implementations
// ============================================================================

impl PollingUart for PxaUart {
    fn poll_status(&mut self) -> SerialEvent {
        PxaUart::poll_status(self)
    }

    fn write_byte(&mut self, byte: u8) {
        PxaUart::write_byte(self, byte);
    }

    fn read_byte(&mut self, status: SerialEvent) -> Option<Result<u8, TransferError>> {
        PxaUart::read_byte(self, status)
    }
}

impl UartPort for PxaUart {
    fn startup(&mut self, config: &Config) -> Result<(), ConfigError> {
        PxaUart::startup(self, config)
    }

    fn shutdown(&mut self) {
        PxaUart::shutdown(self)
    }

    fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        PxaUart::set_config(self, config)
    }

    fn read_rx(&mut self) -> Option<RxSample> {
        PxaUart::read_rx(self)
    }

    fn write_tx(&mut self, bytes: &[u8]) -> usize {
        let mut written = 0;
        let max_burst = bytes.len().min(FIFO_SIZE);
        while written < max_burst {
            let lsr: lsr::Flags = self.read_flags(UART_LSR);
            if !lsr.contains(lsr::Flags::THRE) {
                break;
            }
            self.write_reg(UART_THR, bytes[written]);
            written += 1;
        }
        written
    }

    fn tx_idle(&mut self) -> bool {
        PxaUart::tx_idle(self)
    }

    fn mask_all(&mut self) {
        // Disable every interrupt source while keeping the UART unit enabled.
        self.ier = self.read_flags::<ier::Flags>(UART_IER) & ier::Flags::UUE;
        self.write_flags(UART_IER, self.ier);
    }

    fn rearm(&mut self, sources: SerialEventSet) -> SerialEventSet {
        let mut ier: ier::Flags = self.read_flags(UART_IER);
        ier.insert(ier_for_events(sources));
        self.write_flags(UART_IER, ier);
        self.ier = ier;

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
            ier.remove(ier_for_events(ready));
            self.write_flags(UART_IER, ier);
            self.ier = ier;
        }
        ready
    }
}

impl SplitUart for PxaUart {
    type Port = Self;
    type Irq = PxaUartIrq;

    fn runtime_info(&self) -> UartInfo {
        UartInfo {
            name: "PXA UART",
            register_base: self.base,
            initial_baudrate: if self.current_baud != 0 {
                self.current_baud
            } else if self.clock_freq != 0 {
                self.clock_freq / 16
            } else {
                0
            },
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
// Hard-IRQ endpoint
// ============================================================================

/// Hard-IRQ endpoint for a PXA/XScale UART.
///
/// Owns a disjoint view of the same MMIO window as [`PxaUart`] plus a private
/// LSR error latch used while draining the RX FIFO inside the IRQ handler.
pub struct PxaUartIrq {
    base: usize,
    saved_lsr: lsr::Flags,
}

impl PxaUartIrq {
    fn read_reg(&self, reg: u8) -> u8 {
        read_reg_base(self.base, reg)
    }

    fn write_reg(&self, reg: u8, val: u8) {
        write_reg_base(self.base, reg, val)
    }

    fn read_flags<F: Flags<Bits = u8>>(&self, reg: u8) -> F {
        F::from_bits_retain(self.read_reg(reg))
    }

    fn write_flags<F: Flags<Bits = u8>>(&self, reg: u8, val: F) {
        self.write_reg(reg, val.bits())
    }

    /// Map the current IIR pending interrupt to a stable event class.
    fn next_event(&self) -> Option<SerialEventSet> {
        let iir: iir::Flags = self.read_flags(UART_IIR);
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

    fn ack_modem_status(&self) {
        let _: msr::Flags = self.read_flags(UART_MSR);
    }

    /// Disable the given event sources in IER (keeps UUE untouched).
    fn mask(&self, events: SerialEventSet) {
        let mut ier: ier::Flags = self.read_flags(UART_IER);
        ier.remove(ier_for_events(events));
        self.write_flags(UART_IER, ier);
    }
}

impl UartIrq for PxaUartIrq {
    fn handle(&mut self, rx: &mut dyn IrqRxSink) -> Option<SerialIrqEvent> {
        const IRQ_PASS_BUDGET: usize = 32;
        const RX_SAMPLE_BUDGET: usize = 256;

        let mut event = SerialIrqEvent::default();
        let mut rx_samples = 0;
        for _ in 0..IRQ_PASS_BUDGET {
            let Some(current) = self.next_event() else {
                break;
            };
            event.events |= current;

            if current.intersects(SerialEventSet::RX) {
                let before = rx_samples;
                while rx_samples < RX_SAMPLE_BUDGET {
                    let Some(sample) = read_rx_sample(self.base, &mut self.saved_lsr) else {
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

// ============================================================================
// Module-level helpers (shared by the port and IRQ endpoint)
// ============================================================================

fn read_reg_base(base: usize, reg: u8) -> u8 {
    let addr = base + (reg as usize) * REG_WIDTH;
    // The PXA/K3 UART requires 32-bit MMIO access (reg-io-width=4); the
    // register value occupies the low byte of the 32-bit word.
    (unsafe { read_volatile(addr as *const u32) } & 0xFF) as u8
}

fn write_reg_base(base: usize, reg: u8, val: u8) {
    let addr = base + (reg as usize) * REG_WIDTH;
    // Use a 32-bit write so the hardware sees a complete bus transaction on
    // its 32-bit peripheral bus.
    unsafe { write_volatile(addr as *mut u32, val as u32) };
}

/// Read the current baud rate from the hardware divisor latch registers.
///
/// Temporarily enables DLAB, reads DLL/DLH, restores the original LCR,
/// and computes `clock_freq / (16 * divisor)`. Returns 0 when the
/// divisor is zero or the clock frequency is unknown.
fn probe_current_baud(base: usize, clock_freq: u32) -> u32 {
    let lcr = read_reg_base(base, UART_LCR);
    write_reg_base(base, UART_LCR, lcr | lcr::Flags::DLAB.bits());
    let dll = read_reg_base(base, UART_DLL) as u32;
    let dlh = read_reg_base(base, UART_DLH) as u32;
    write_reg_base(base, UART_LCR, lcr);
    let divisor = (dlh << 8) | dll;
    if divisor == 0 || clock_freq == 0 {
        0
    } else {
        clock_freq / (16 * divisor)
    }
}

/// Read one RX sample from `base`, folding sticky LSR error bits into `saved_lsr`.
fn read_rx_sample(base: usize, saved_lsr: &mut lsr::Flags) -> Option<RxSample> {
    let current = lsr::Flags::from_bits_retain(read_reg_base(base, UART_LSR));
    saved_lsr.insert(current & (lsr::Flags::ERROR_MASK | lsr::Flags::FIFOE));
    let lsr = current | *saved_lsr;

    if !lsr.intersects(lsr::Flags::DR | lsr::Flags::ERROR_MASK) {
        return None;
    }

    let byte = lsr
        .contains(lsr::Flags::DR)
        .then(|| read_reg_base(base, UART_RBR));
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

    saved_lsr.remove(lsr::Flags::ERROR_MASK | lsr::Flags::FIFOE);

    Some(RxSample {
        byte,
        flag,
        overrun,
    })
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

/// Translate stable event classes to PXA IER interrupt-source bits.
///
/// All RX classes share the same hardware sources (`RDI | RLSI | RTOIE`); TX
/// space maps to the transmitter-holding-empty source (`THRI`).
fn ier_for_events(events: SerialEventSet) -> ier::Flags {
    let mut ier = ier::Flags::empty();
    if events.intersects(SerialEventSet::RX) {
        ier.insert(ier::Flags::RDI | ier::Flags::RLSI | ier::Flags::RTOIE);
    }
    if events.contains(SerialEventSet::TX_SPACE) {
        ier.insert(ier::Flags::THRI);
    }
    ier
}

// Safety: PxaUart only accesses its own MMIO range via raw pointers.
// The base address is provided by the platform and must be valid.
unsafe impl Send for PxaUart {}

// Safety: PxaUartIrq only accesses its own MMIO range via raw pointers, the
// same range as the originating PxaUart. The port and IRQ endpoints are used
// from disjoint contexts under the device-serialization contract of UartPort.
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
