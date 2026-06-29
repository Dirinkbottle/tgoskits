use bitflags::bitflags;

pub const UART_RBR: u8 = 0x00; // RX Buffer Register (read, DLAB=0)
pub const UART_THR: u8 = 0x00; // TX Holding Register (write, DLAB=0)
pub const UART_DLL: u8 = 0x00; // Divisor Latch LSB (DLAB=1)
pub const UART_IER: u8 = 0x01; // Interrupt Enable Register
pub const UART_DLH: u8 = 0x01; // Divisor Latch MSB (DLAB=1)
pub const UART_IIR: u8 = 0x02; // Interrupt Identification Register (read)
pub const UART_FCR: u8 = 0x02; // FIFO Control Register (write)
pub const UART_LCR: u8 = 0x03; // Line Control Register
pub const UART_MCR: u8 = 0x04; // Modem Control Register
pub const UART_LSR: u8 = 0x05; // Line Status Register
pub const UART_MSR: u8 = 0x06; // Modem Status Register

pub const REG_WIDTH: usize = 4;
pub const FIFO_SIZE: usize = 64;

pub mod ier {
    use super::bitflags;

    bitflags! {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        pub struct Flags: u8 {
            /// Received Data Available.
            const RDI = 0x01;
            /// Transmitter Holding Register Empty.
            const THRI = 0x02;
            /// Receiver Line Status.
            const RLSI = 0x04;
            /// Modem Status.
            const MSI = 0x08;
            /// PXA: Receiver Time-Out Interrupt Enable.
            const RTOIE = 0x10;
            /// PXA: UART Unit Enable.
            const UUE = 0x40;
            /// PXA: DMA Requests Enable.
            const DMAE = 0x80;
        }
    }
}

pub mod iir {
    use super::bitflags;

    bitflags! {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        pub struct Flags: u8 {
            const NO_INT = 0x01;
            const ID_MASK = 0x0E;
            const RLSI = 0x06;
            const RDI = 0x04;
            const CTI = 0x0C;
            const THRI = 0x02;
            const MSI = 0x00;
            const FIFO_ENABLE = 0xC0;
        }
    }
}

pub mod fcr {
    use super::bitflags;

    bitflags! {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        pub struct Flags: u8 {
            const ENABLE_FIFO = 0x01;
            const CLEAR_RX = 0x02;
            const CLEAR_TX = 0x04;
            const DMA_SELECT = 0x08;
            /// PXA: trailing bytes interrupt in DMA mode.
            const TRAIL = 0x10;
            /// PXA: 32-bit peripheral bus mode.
            const BUS32 = 0x20;
            /// PXA: RX FIFO trigger at 1 byte.
            const PXAR1 = 0x00;
            /// PXA: RX FIFO trigger at 8 bytes.
            const PXAR8 = 0x40;
            /// PXA: RX FIFO trigger at 32 bytes.
            const PXAR32 = 0x80;
            const TRIGGER_MASK = 0xC0;
        }
    }
}

pub mod lcr {
    use super::bitflags;

    bitflags! {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        pub struct Flags: u8 {
            const WLEN5 = 0x00;
            const WLEN6 = 0x01;
            const WLEN7 = 0x02;
            const WLEN8 = 0x03;
            const WLEN_MASK = 0x03;
            const STOP = 0x04;
            const PARITY = 0x08;
            const EPAR = 0x10;
            const SPAR = 0x20;
            const SBRK = 0x40;
            const DLAB = 0x80;
        }
    }
}

pub mod mcr {
    use super::bitflags;

    bitflags! {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        pub struct Flags: u8 {
            const DTR = 0x01;
            const RTS = 0x02;
            const OUT1 = 0x04;
            const OUT2 = 0x08;
            const LOOP = 0x10;
            /// PXA: Auto Flow Control Enable.
            const AFE = 0x20;
        }
    }
}

pub mod lsr {
    use super::bitflags;

    bitflags! {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        pub struct Flags: u8 {
            const DR = 0x01;
            const OE = 0x02;
            const PE = 0x04;
            const FE = 0x08;
            const BI = 0x10;
            const THRE = 0x20;
            const TEMT = 0x40;
            const FIFOE = 0x80;
            const ERROR_MASK = 0x1E;
        }
    }
}

pub mod msr {
    use super::bitflags;

    bitflags! {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        pub struct Flags: u8 {
            const DCTS = 0x01;
            const DDSR = 0x02;
            const TERI = 0x04;
            const DDCD = 0x08;
            const CTS = 0x10;
            const DSR = 0x20;
            const RI = 0x40;
            const DCD = 0x80;
            const DELTA_MASK = 0x0F;
        }
    }
}
