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

mod irq;
mod regs;
mod rx;

use core::ptr::{NonNull, read_volatile, write_volatile};

use bitflags::Flags;
use irq::PxaUartIrq;
use rdif_serial::{
    Config, ConfigError, DataBits, Parity, RxSample, SerialEventSet, SplitUart, StopBits, UartInfo,
    UartParts, UartPort,
};
use regs::{
    FIFO_SIZE, REG_WIDTH, UART_DLH, UART_DLL, UART_FCR, UART_IER, UART_IIR, UART_LCR, UART_LSR,
    UART_MCR, UART_MSR, UART_RBR, UART_THR, fcr, ier, lcr, lsr, mcr,
};
use rx::read_rx_sample;

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
        calculate_divisor(self.clock_freq, baudrate)
    }

    /// Set the baud rate divisor.
    /// Follows the PXA Errata #75 sequence: write DLH → read DLH → write DLL.
    fn set_divisor(&self, divisor: u16) {
        // The divisor latch shares offsets with RBR/THR/IER and is only
        // reachable while DLAB is set in LCR.
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

        // Restore the non-DLAB value so RBR/THR are reachable again.
        self.write_flags(UART_LCR, lcr);
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
        // serial_spacemit.c lines 1101-1103: clear and disable FIFOs.
        self.write_flags(UART_FCR, fcr::Flags::ENABLE_FIFO);
        self.write_flags(
            UART_FCR,
            fcr::Flags::ENABLE_FIFO | fcr::Flags::CLEAR_RX | fcr::Flags::CLEAR_TX,
        );
        self.write_flags(UART_FCR, fcr::Flags::empty());

        // serial_spacemit.c lines 1106-1109: reading the status registers
        // clears any pending sticky flags and pending interrupt state.
        self.read_reg(UART_LSR);
        self.read_reg(UART_RBR);
        self.read_reg(UART_IIR);
        self.read_reg(UART_MSR);

        // serial_spacemit.c line 1112: initialize with an 8-bit word.
        self.write_flags(UART_LCR, lcr::Flags::WLEN8);
        self.lcr = lcr::Flags::WLEN8;

        // serial_spacemit.c lines 1115-1118: OUT2 gates 16550-class IRQ
        // delivery, so it must be asserted before any interrupt source is
        // enabled.
        self.mcr = mcr::Flags::OUT2;
        self.write_flags(UART_MCR, self.mcr);

        // Apply configuration (baud rate, data bits, stop bits, parity)
        self.set_config(config)?;

        // serial_spacemit.c lines 1142-1145: clear any flags latched by the
        // configuration writes above before enabling interrupts.
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
        // Mask every interrupt source so no IRQ can fire while the port is
        // shut down.
        self.ier = ier::Flags::empty();
        self.write_flags(UART_IER, ier::Flags::empty());

        // Drop the OUT2 IRQ gate so the interrupt line de-asserts.
        self.mcr.remove(mcr::Flags::OUT2);
        self.write_flags(UART_MCR, self.mcr);

        // Clear any looped-back break condition, then reset both FIFOs.
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

        // Commit the cached line control. DLAB is deliberately excluded: the
        // divisor writes in `set_divisor` manage it themselves.
        self.write_flags(UART_LCR, self.lcr);

        // --- FIFO control ---
        // The RX FIFO trigger level depends on the effective baud rate, so a
        // valid divisor must be available even when `config.baudrate` is
        // `None`; failing that, the configuration is invalid.
        let divisor = self
            .divisor(self.current_baud)
            .ok_or(ConfigError::InvalidBaudrate)?;
        let fcr = select_fcr_flags_for(self.clock_freq / u32::from(divisor));
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
        let irq = PxaUartIrq::new(self.base);
        UartParts::new(self, irq)
    }
}

// ============================================================================
// Module-level helpers (shared by the port and IRQ endpoint)
// ============================================================================

fn read_reg_base(base: usize, reg: u8) -> u8 {
    let addr = base + (reg as usize) * REG_WIDTH;
    // The PXA/K3 UART requires 32-bit MMIO access (reg-io-width=4); the
    // register value occupies the low byte of the 32-bit word.
    //
    // SAFETY: `base` must point to a mapped, 4-byte-aligned MMIO window owned
    // exclusively by this driver, and `base + reg*REG_WIDTH` must stay inside
    // that window for every register offset used. The port and IRQ endpoints
    // access this window under the device-serialization contract of `UartPort`.
    (unsafe { read_volatile(addr as *const u32) } & 0xFF) as u8
}

fn write_reg_base(base: usize, reg: u8, val: u8) {
    let addr = base + (reg as usize) * REG_WIDTH;
    // Use a 32-bit write so the hardware sees a complete bus transaction on
    // its 32-bit peripheral bus.
    //
    // SAFETY: same MMIO window and alignment guarantees as `read_reg_base`.
    unsafe { write_volatile(addr as *mut u32, val as u32) };
}

/// Calculate the 16550 baud rate divisor, rounded to the closest usable
/// integral divisor. Returns `None` for a zero baud rate or clock frequency,
/// or when the divisor would overflow the 16-bit divisor latch.
fn calculate_divisor(clock_freq: u32, baudrate: u32) -> Option<u16> {
    if baudrate == 0 || clock_freq == 0 {
        return None;
    }
    let baudrate = baudrate as u64;
    let div = (clock_freq as u64 + 8 * baudrate) / (16 * baudrate);
    if div == 0 || div > 0xFFFF {
        return None;
    }
    Some(div as u16)
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

/// Select FCR RX-FIFO trigger flags for an effective RX baud rate.
///
/// From Linux `serial_pxa_set_termios()`: below 2400 baud use the 1-byte
/// trigger, below 230400 baud use the 8-byte trigger, and above that use the
/// 32-byte trigger. The thresholds keep the per-byte interrupt rate bounded on
/// the slow side and prevent the high-rate path from draining too often.
fn select_fcr_flags_for(rate: u32) -> fcr::Flags {
    if rate < 2400 * 16 {
        fcr::Flags::ENABLE_FIFO | fcr::Flags::PXAR1
    } else if rate < 230400 * 16 {
        fcr::Flags::ENABLE_FIFO | fcr::Flags::PXAR8
    } else {
        fcr::Flags::ENABLE_FIFO | fcr::Flags::PXAR32
    }
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

// SAFETY: PxaUart only accesses its own MMIO range via raw pointers.
// The base address is provided by the platform and must be valid.
unsafe impl Send for PxaUart {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn divisor_rounds_to_nearest_value() {
        assert_eq!(calculate_divisor(14_700_000, 115_200), Some(8));
    }

    #[test]
    fn divisor_rejects_zero_or_overflow() {
        assert_eq!(calculate_divisor(14_700_000, 0), None);
        assert_eq!(calculate_divisor(0, 115_200), None);
        // A baud rate far below the clock frequency would overflow the
        // 16-bit divisor latch.
        assert_eq!(calculate_divisor(14_700_000, 1), None);
    }

    #[test]
    fn selects_pxar1_trigger_below_2400_baud() {
        // PXAR1 is a zero-valued bit, so compare the masked trigger field
        // rather than using `contains`.
        let flags = select_fcr_flags_for(2400 * 16 - 1);
        assert_eq!(flags & fcr::Flags::TRIGGER_MASK, fcr::Flags::PXAR1);
        assert!(flags.contains(fcr::Flags::ENABLE_FIFO));
    }

    #[test]
    fn selects_pxar8_trigger_between_2400_and_230400_baud() {
        assert_eq!(
            select_fcr_flags_for(2400 * 16) & fcr::Flags::TRIGGER_MASK,
            fcr::Flags::PXAR8
        );
        assert_eq!(
            select_fcr_flags_for(230400 * 16 - 1) & fcr::Flags::TRIGGER_MASK,
            fcr::Flags::PXAR8
        );
    }

    #[test]
    fn selects_pxar32_trigger_at_or_above_230400_baud() {
        assert_eq!(
            select_fcr_flags_for(230400 * 16) & fcr::Flags::TRIGGER_MASK,
            fcr::Flags::PXAR32
        );
    }

    #[test]
    fn enables_fifo_in_every_trigger_band() {
        for rate in [1, 2400 * 16, 230400 * 16] {
            assert!(select_fcr_flags_for(rate).contains(fcr::Flags::ENABLE_FIFO));
        }
    }

    #[test]
    fn ier_rx_sources_enable_all_rx_classes() {
        let events = SerialEventSet::RX_DATA | SerialEventSet::RX_STATUS;
        let ier = ier_for_events(events);
        assert!(ier.contains(ier::Flags::RDI | ier::Flags::RLSI | ier::Flags::RTOIE));
        assert!(!ier.contains(ier::Flags::THRI));
    }

    #[test]
    fn ier_tx_space_maps_to_thri_only() {
        let ier = ier_for_events(SerialEventSet::TX_SPACE);
        assert!(ier.contains(ier::Flags::THRI));
        assert!(!ier.intersects(ier::Flags::RDI | ier::Flags::RLSI | ier::Flags::RTOIE));
    }

    #[test]
    fn ier_empty_events_enable_nothing() {
        assert!(ier_for_events(SerialEventSet::empty()).is_empty());
    }
}
