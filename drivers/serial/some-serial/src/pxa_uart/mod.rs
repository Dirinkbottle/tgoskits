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

use core::{
    num::NonZeroU32,
    ptr::{NonNull, read_volatile, write_volatile},
};

use bitflags::Flags;
use rdif_serial::{
    Config, ConfigError, DataBits, InterruptMask, IrqSnapshot, IrqSource, Parity, RawUart, RxFlag,
    RxSample, SerialEvent, StopBits, TransferError,
};
use regs::{
    FIFO_SIZE, REG_WIDTH, UART_DLH, UART_DLL, UART_FCR, UART_IER, UART_IIR, UART_LCR, UART_LSR,
    UART_MCR, UART_MSR, UART_RBR, UART_THR, fcr, ier, iir, lcr, lsr, mcr, msr,
};

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
// RawUart trait implementation
// ============================================================================

impl RawUart for PxaUart {
    fn name(&self) -> &'static str {
        "PXA UART"
    }

    fn base_addr(&self) -> usize {
        self.base
    }

    fn clock_freq(&self) -> Option<NonZeroU32> {
        self.clock_freq.try_into().ok()
    }

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

        // Line 1136-1138: Enable interrupts (non-DMA path)
        // IER = RLSI | RDI | RTOIE | UUE
        self.ier = ier::Flags::RLSI | ier::Flags::RDI | ier::Flags::RTOIE | ier::Flags::UUE;
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

    fn baudrate(&self) -> u32 {
        self.current_baud
    }

    fn data_bits(&self) -> DataBits {
        match self.lcr & lcr::Flags::WLEN_MASK {
            lcr::Flags::WLEN5 => DataBits::Five,
            lcr::Flags::WLEN6 => DataBits::Six,
            lcr::Flags::WLEN7 => DataBits::Seven,
            _ => DataBits::Eight,
        }
    }

    fn stop_bits(&self) -> StopBits {
        if self.lcr.contains(lcr::Flags::STOP) {
            StopBits::Two
        } else {
            StopBits::One
        }
    }

    fn parity(&self) -> Parity {
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

    fn enable_loopback(&mut self) {
        self.mcr.insert(mcr::Flags::LOOP);
        self.write_flags(UART_MCR, self.mcr);
    }

    fn disable_loopback(&mut self) {
        self.mcr.remove(mcr::Flags::LOOP);
        self.write_flags(UART_MCR, self.mcr);
    }

    fn is_loopback_enabled(&self) -> bool {
        self.mcr.contains(mcr::Flags::LOOP)
    }

    // --- set_irq_mask ---
    fn set_irq_mask(&mut self, mask: InterruptMask) {
        // Start with UUE (always enabled when port is active)
        let mut ier = ier::Flags::UUE;

        if mask.intersects(InterruptMask::RX) {
            ier.insert(ier::Flags::RDI | ier::Flags::RLSI | ier::Flags::RTOIE);
        }
        if mask.contains(InterruptMask::TX_SPACE) {
            ier.insert(ier::Flags::THRI);
        }

        self.ier = ier;
        self.write_flags(UART_IER, ier);
    }

    // --- take_irq_snapshot: serial_pxa_irq() ---
    fn take_irq_snapshot(&mut self) -> IrqSnapshot {
        let iir: iir::Flags = self.read_flags(UART_IIR);

        // No interrupt pending
        if iir.contains(iir::Flags::NO_INT) {
            return IrqSnapshot::default();
        }

        let id = iir & iir::Flags::ID_MASK;
        let sources = if id == iir::Flags::RLSI {
            IrqSource::RX_STATUS
        } else if id == iir::Flags::RDI {
            IrqSource::RX_DATA
        } else if id == iir::Flags::CTI {
            IrqSource::RX_TIMEOUT
        } else if id == iir::Flags::THRI {
            IrqSource::TX_SPACE
        } else if id == iir::Flags::MSI {
            IrqSource::MODEM_STATUS
        } else {
            IrqSource::OTHER_ACK
        };

        // For RX-related IRQs, read LSR to latch error state and clear the condition
        if sources.intersects(IrqSource::RX_DATA | IrqSource::RX_TIMEOUT | IrqSource::RX_STATUS) {
            let _ = self.read_lsr_preserving();
        }

        IrqSnapshot {
            claimed: true,
            sources,
        }
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

    // --- tx_ready ---
    fn tx_ready(&mut self) -> bool {
        self.read_flags::<lsr::Flags>(UART_LSR)
            .contains(lsr::Flags::THRE)
    }

    // --- write_tx ---
    fn write_tx(&mut self, byte: u8) {
        self.write_reg(UART_THR, byte)
    }

    // --- tx_load_size: use FIFO depth ---
    fn tx_load_size(&self) -> usize {
        FIFO_SIZE
    }

    // --- tx_idle ---
    fn tx_idle(&mut self) -> bool {
        let lsr: lsr::Flags = self.read_flags(UART_LSR);
        lsr.contains(lsr::Flags::THRE | lsr::Flags::TEMT)
    }

    // --- ack_modem_status ---
    fn ack_modem_status(&mut self) {
        let _: msr::Flags = self.read_flags(UART_MSR);
    }

    // --- poll_status: for early console / polling users ---
    fn poll_status(&mut self) -> SerialEvent {
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

    // --- write_byte: for console output ---
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

    // --- read_byte ---
    fn read_byte(&mut self, status: SerialEvent) -> Option<Result<u8, TransferError>> {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn divisor_rounds_to_nearest_value() {
        let uart = PxaUart::new(NonNull::dangling(), 14_700_000);

        assert_eq!(uart.divisor(115_200), Some(8));
    }
}
