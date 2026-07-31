//! RX sample decoding shared by the polling port and the IRQ endpoint.
//!
//! LSR error bits are sticky: reading LSR clears them, so a byte that arrives
//! together with an error must still report that error. All decoding here is
//! pure over a raw LSR value so the folding/clear semantics can be tested
//! without touching MMIO.

use rdif_serial::{RxErrorFlags, RxFlag, RxSample};

use super::{
    read_reg_base,
    regs::{UART_LSR, UART_RBR, lsr},
};

/// Classify one raw LSR snapshot into the pending RX sample state, folding
/// sticky error bits into `saved_lsr`.
///
/// Returns `(flag, overrun, has_data)` when the receiver has a byte or a
/// sticky error to report, and `None` when it is idle. The caller is
/// responsible for reading the RX buffer register when `has_data` is set.
///
/// The folded error bits are consumed by the returned sample: the latch is
/// cleared so the next LSR read starts from a clean state.
fn classify_rx(raw_lsr: lsr::Flags, saved_lsr: &mut lsr::Flags) -> Option<(RxFlag, bool, bool)> {
    saved_lsr.insert(raw_lsr & (lsr::Flags::ERROR_MASK | lsr::Flags::FIFOE));
    let merged = raw_lsr | *saved_lsr;

    if !merged.intersects(lsr::Flags::DR | lsr::Flags::ERROR_MASK) {
        return None;
    }

    let flag = if merged.contains(lsr::Flags::BI) {
        RxFlag::Break
    } else if merged.contains(lsr::Flags::PE) {
        RxFlag::Parity
    } else if merged.contains(lsr::Flags::FE) {
        RxFlag::Framing
    } else {
        RxFlag::Normal
    };
    let overrun = merged.contains(lsr::Flags::OE);

    saved_lsr.remove(lsr::Flags::ERROR_MASK | lsr::Flags::FIFOE);

    Some((flag, overrun, merged.contains(lsr::Flags::DR)))
}

/// Read one RX sample from `base`, folding sticky LSR error bits into `saved_lsr`.
pub(super) fn read_rx_sample(base: usize, saved_lsr: &mut lsr::Flags) -> Option<RxSample> {
    let raw = lsr::Flags::from_bits_retain(read_reg_base(base, UART_LSR));
    let (flag, overrun, has_data) = classify_rx(raw, saved_lsr)?;
    let byte = has_data.then(|| read_reg_base(base, UART_RBR));
    Some(RxSample {
        byte,
        flag,
        overrun,
    })
}

/// Translate one decoded RX sample into the IRQ endpoint's error summary.
pub(super) fn rx_errors_from_sample(sample: RxSample) -> RxErrorFlags {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classifies_normal_data_sample_with_byte_read() {
        let mut saved = lsr::Flags::empty();
        assert_eq!(
            classify_rx(lsr::Flags::DR, &mut saved),
            Some((RxFlag::Normal, false, true))
        );
        assert!(saved.is_empty());
    }

    #[test]
    fn classifies_parity_error_sample_without_data() {
        let mut saved = lsr::Flags::empty();
        assert_eq!(
            classify_rx(lsr::Flags::PE, &mut saved),
            Some((RxFlag::Parity, false, false))
        );
        assert!(saved.is_empty());
    }

    #[test]
    fn error_priority_is_break_over_parity_over_framing() {
        let mut saved = lsr::Flags::empty();
        assert_eq!(
            classify_rx(lsr::Flags::FE | lsr::Flags::PE, &mut saved),
            Some((RxFlag::Parity, false, false))
        );

        let mut saved = lsr::Flags::empty();
        assert_eq!(
            classify_rx(lsr::Flags::BI | lsr::Flags::PE, &mut saved),
            Some((RxFlag::Break, false, false))
        );
    }

    #[test]
    fn folds_sticky_overrun_into_following_sample_then_clears_latch() {
        // OE was latched by an earlier LSR read; the next data byte still
        // reports the overrun, and the latch is cleared once consumed.
        let mut saved = lsr::Flags::OE;
        assert_eq!(
            classify_rx(lsr::Flags::DR, &mut saved),
            Some((RxFlag::Normal, true, true))
        );
        assert!(saved.is_empty());
    }

    #[test]
    fn returns_none_when_receiver_is_idle() {
        let mut saved = lsr::Flags::empty();
        assert_eq!(classify_rx(lsr::Flags::THRE, &mut saved), None);
        assert!(saved.is_empty());
    }

    #[test]
    fn maps_sample_to_irq_error_summary() {
        let sample = RxSample {
            byte: None,
            flag: RxFlag::Break,
            overrun: true,
        };
        assert_eq!(
            rx_errors_from_sample(sample),
            RxErrorFlags::BREAK | RxErrorFlags::OVERRUN
        );
    }
}
