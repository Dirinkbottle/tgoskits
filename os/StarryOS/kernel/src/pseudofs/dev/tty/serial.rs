use alloc::{boxed::Box, format, string::String, sync::Arc, vec, vec::Vec};
use core::{
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
    time::Duration,
};

use ax_driver::serial::{
    self as ax_serial, Config, ConfigError, OwnerId, RxFlag, RxItem, RxQueue, SerialDevice,
    SerialIrqHandler, SerialIrqOutcome, SerialPort, SerialSoftWork, TxQueue,
};
use ax_errno::{AxError, AxResult};
use ax_kspin::SpinNoIrq;
use ax_runtime::hal::{
    console::{ConsoleDeviceIdError, ConsoleDeviceIdResult},
    irq::{AutoEnable, CpuId, IrqAffinity, IrqHandle, IrqId, IrqRequest, ShareMode},
};
use ax_sync::Mutex;
use ax_task::IrqNotify;
use axpoll::{IoEvents, PollSet};
use bitflags::bitflags;
use rdrive::DeviceId as RDriveDeviceId;
use spin::LazyLock;
use starry_process::Process;

use super::{
    Tty,
    terminal::{
        Terminal,
        ldisc::{ProcessMode, TtyConfig, TtyRead, TtyWrite},
        termios::Termios2,
    },
};
use crate::pseudofs::DeviceOps;

pub type SerialTtyDriver = Tty<SerialReader, SerialWriter>;

const SERIAL_RX_DRAIN_CHUNK: usize = 256;
const SERIAL_SYNC_ECHO_LIMIT: usize = 256;
const SERIAL_DEFAULT_BAUDRATE: u32 = 115_200;
const SERIAL_POLL_INTERVAL_MS: u64 = 1;

bitflags! {
    #[derive(Clone, Copy, Debug, Default)]
    struct SerialEventBits: u32 {
        const RX_READY = 1 << 0;
        const TX_SPACE = 1 << 1;
        const HANGUP   = 1 << 2;
    }
}

pub struct SerialTtyEntry {
    number: usize,
    tty: Arc<SerialTtyDriver>,
    backend: Arc<SerialBackend>,
}

impl SerialTtyEntry {
    pub fn number(&self) -> usize {
        self.number
    }

    pub fn tty(&self) -> Arc<SerialTtyDriver> {
        self.tty.clone()
    }
}

struct SerialRegistry {
    entries: Vec<SerialTtyEntry>,
    console_index: Option<usize>,
}

struct SerialBackend {
    name: String,
    tty_name: String,
    rdrive_device_id: RDriveDeviceId,
    number: usize,
    mode: SerialBackendMode,
    owner: OwnerId,
    port: Arc<SerialPort>,
    tx: SpinNoIrq<TxQueue>,
    rx: SpinNoIrq<RxQueue>,
    irq_handle: SpinNoIrq<Option<IrqHandle>>,
    started: AtomicBool,
    start_lock: Mutex<()>,
    events: SerialEvents,
    input_source: Arc<PollSet>,
    output_source: Arc<PollSet>,
    tx_notify: IrqNotify,
    output_lock: Mutex<()>,
}

#[derive(Clone, Copy, Debug)]
enum SerialBackendMode {
    Interrupt { irq: IrqId },
    Polling,
}

struct SerialEvents {
    pending: AtomicU32,
    notify: IrqNotify,
}

impl SerialEvents {
    const fn new() -> Self {
        Self {
            pending: AtomicU32::new(0),
            notify: IrqNotify::new(),
        }
    }

    fn publish_irq(&self, events: SerialEventBits) {
        if events.is_empty() {
            return;
        }
        self.pending.fetch_or(events.bits(), Ordering::Release);
        self.notify.notify_irq();
    }

    fn publish(&self, events: SerialEventBits) {
        if events.is_empty() {
            return;
        }
        self.pending.fetch_or(events.bits(), Ordering::Release);
        self.notify.notify();
    }

    fn wait(&self) {
        self.notify.wait();
    }

    fn take(&self) -> SerialEventBits {
        SerialEventBits::from_bits_retain(self.pending.swap(0, Ordering::AcqRel))
    }
}

struct NoConsole;

impl DeviceOps for NoConsole {
    fn read_at(&self, _buf: &mut [u8], _offset: u64) -> AxResult<usize> {
        Err(AxError::NoSuchDevice)
    }

    fn write_at(&self, _buf: &[u8], _offset: u64) -> AxResult<usize> {
        Err(AxError::NoSuchDevice)
    }

    fn ioctl(&self, _cmd: u32, _arg: usize) -> AxResult<usize> {
        Err(AxError::NoSuchDevice)
    }

    fn open(&self, _exclusive: bool) -> AxResult<()> {
        Err(AxError::NoSuchDevice)
    }

    fn as_any(&self) -> &dyn core::any::Any {
        self
    }
}

#[derive(Clone, Copy, Debug)]
struct ConsoleCandidate {
    number: usize,
    device_id: RDriveDeviceId,
}

#[derive(Debug)]
#[cfg_attr(test, derive(PartialEq, Eq))]
enum ConsoleSelection {
    SelectedDevice(usize),
    TtyS0Fallback(usize),
}

impl ConsoleSelection {
    fn index(&self) -> usize {
        match self {
            Self::SelectedDevice(index) | Self::TtyS0Fallback(index) => *index,
        }
    }
}

#[derive(Clone)]
pub struct SerialReader {
    backend: Arc<SerialBackend>,
}

#[derive(Clone)]
pub struct SerialWriter {
    backend: Arc<SerialBackend>,
}

static SERIAL_REGISTRY: LazyLock<SerialRegistry> = LazyLock::new(SerialRegistry::discover);

pub fn serial_tty_entries() -> &'static [SerialTtyEntry] {
    &SERIAL_REGISTRY.entries
}

impl SerialTtyDriver {
    pub fn serial_number(&self) -> usize {
        self.writer.backend.number
    }
}

pub fn console_device() -> Arc<dyn DeviceOps> {
    SERIAL_REGISTRY
        .console_index
        .and_then(|index| SERIAL_REGISTRY.entries.get(index))
        .map(|entry| entry.tty() as Arc<dyn DeviceOps>)
        .unwrap_or_else(|| Arc::new(NoConsole))
}

pub fn bind_console_to(proc: &Process) -> AxResult<()> {
    if let Some(index) = SERIAL_REGISTRY.console_index
        && let Some(entry) = SERIAL_REGISTRY.entries.get(index)
    {
        entry.backend.ensure_started()?;
        // ax_runtime::hal::console::claim_runtime_output();
        return entry.tty.bind_to(proc);
    }

    Err(AxError::NoSuchDevice)
}

pub fn arm_console_irq() {
    if let Some(index) = SERIAL_REGISTRY.console_index
        && let Some(entry) = SERIAL_REGISTRY.entries.get(index)
    {
        entry.backend.start_port();
    }
}

impl SerialRegistry {
    fn discover() -> Self {
        let serials = ax_serial::take_serial_devices();
        warn!(
            "SerialRegistry::discover: take_serial_devices returned {} device(s)",
            serials.len()
        );
        for (i, s) in serials.iter().enumerate() {
            warn!(
                "  serial[{i}]: name={}, path={}, alias={:?}, irq={:?}",
                s.name, s.info.fdt_path, s.info.alias_index, s.info.irq
            );
        }
        let numbers = assign_tty_numbers(
            serials
                .iter()
                .map(|serial| serial.info.alias_index)
                .collect::<Vec<_>>()
                .as_slice(),
        );
        warn!("SerialRegistry::discover: assigned ttyS numbers = {numbers:?}");

        let mut entries = Vec::new();
        for (serial, number) in serials.into_iter().zip(numbers) {
            let Some(number) = number else {
                warn!(
                    "Skipping serial device {} at {} because ttyS number could not be assigned",
                    serial.name, serial.info.fdt_path
                );
                continue;
            };
            match new_serial_tty(number, serial) {
                Ok(entry) => {
                    warn!(
                        "SerialRegistry::discover: ttyS{number} created, device_id={:?}",
                        entry.backend.rdrive_device_id
                    );
                    entries.push(entry);
                }
                Err(err) => warn!("Skipping ttyS{number}: {err:?}"),
            }
        }
        entries.sort_by_key(|entry| entry.number);

        let candidates = entries
            .iter()
            .map(|entry| ConsoleCandidate {
                number: entry.number,
                device_id: entry.backend.rdrive_device_id,
            })
            .collect::<Vec<_>>();
        warn!(
            "SerialRegistry::discover: candidates = {:?}",
            candidates
                .iter()
                .map(|c| (c.number, c.device_id))
                .collect::<Vec<_>>()
        );
        let device_id_result = ax_runtime::hal::console::device_id();
        warn!("SerialRegistry::discover: hal::console::device_id() = {device_id_result:?}");
        let console_selection = select_console_candidate(&candidates, device_id_result);
        warn!("SerialRegistry::discover: select_console_candidate = {console_selection:?}");
        let console_index = console_selection.as_ref().map(ConsoleSelection::index);
        if let Some(index) = console_index {
            let number = entries[index].number;
            match console_selection {
                Some(ConsoleSelection::SelectedDevice(_)) => {
                    info!("/dev/console bound to ttyS{number}");
                }
                Some(ConsoleSelection::TtyS0Fallback(_)) => {
                    info!("/dev/console bound to ttyS0");
                }
                None => {}
            }
        } else {
            warn!("/dev/console has no serial TTY binding");
        }

        Self {
            entries,
            console_index,
        }
    }
}

fn new_serial_tty(number: usize, serial: SerialDevice) -> AxResult<SerialTtyEntry> {
    let tty_name = format!("ttyS{number}");
    let SerialDevice {
        name,
        rdrive_device_id,
        info,
        runtime,
    } = serial;
    let mode = serial_backend_mode(&tty_name, &info);
    let port = runtime.port;
    let tx = runtime.tx;
    let rx = runtime.rx;
    let irq = runtime.irq;
    let owner = port.owner();
    let backend = Arc::new(SerialBackend {
        name,
        tty_name: tty_name.clone(),
        rdrive_device_id,
        number,
        mode,
        owner,
        port,
        tx: SpinNoIrq::new(tx),
        rx: SpinNoIrq::new(rx),
        irq_handle: SpinNoIrq::new(None),
        started: AtomicBool::new(false),
        start_lock: Mutex::new(()),
        events: SerialEvents::new(),
        input_source: Arc::new(PollSet::new()),
        output_source: Arc::new(PollSet::new()),
        tx_notify: IrqNotify::new(),
        output_lock: Mutex::new(()),
    });

    if let SerialBackendMode::Interrupt { irq: irq_id } = mode {
        backend.register_irq(irq_id, irq)?;
    } else {
        spawn_serial_poll_worker(backend.clone());
    }
    spawn_serial_event_worker(backend.clone());

    let terminal = Arc::new(Terminal::default());
    let entry_backend = backend.clone();
    let tty = Tty::new(
        terminal,
        TtyConfig {
            reader: SerialReader {
                backend: backend.clone(),
            },
            writer: SerialWriter { backend },
            process_mode: ProcessMode::InterruptDriven {
                input: entry_backend.input_source.clone(),
                output: Some(entry_backend.output_source.clone()),
            },
        },
    );
    info!(
        "{} registered: path={}, alias={:?}, paddr={:#x}, mapped={:#x}, mode={}",
        tty_name,
        info.fdt_path,
        info.alias_index,
        info.paddr,
        info.mapped_base,
        mode.label()
    );
    Ok(SerialTtyEntry {
        number,
        tty,
        backend: entry_backend,
    })
}

fn serial_backend_mode(tty_name: &str, info: &ax_serial::SerialDeviceInfo) -> SerialBackendMode {
    let Some(irq_binding) = info.irq.clone() else {
        warn!(
            "{} at {} has no IRQ binding; using polling mode",
            tty_name, info.fdt_path
        );
        return SerialBackendMode::Polling;
    };

    match ax_runtime::irq::resolve_binding_irq(irq_binding) {
        Ok(irq) => SerialBackendMode::Interrupt { irq },
        Err(err) => {
            warn!(
                "Failed to resolve {} IRQ binding for {}: {err:?}; using polling mode",
                tty_name, info.fdt_path
            );
            SerialBackendMode::Polling
        }
    }
}

impl SerialBackendMode {
    fn label(self) -> &'static str {
        match self {
            Self::Interrupt { .. } => "interrupt",
            Self::Polling => "polling",
        }
    }
}

impl SerialBackend {
    fn register_irq(self: &Arc<Self>, irq_id: IrqId, mut irq: SerialIrqHandler) -> AxResult<()> {
        let backend = self.clone();
        let request = IrqRequest::new_boxed(Box::new(move |ctx| {
            let outcome = backend.handle_irq_on_owner(ctx.cpu, &mut irq);
            if !outcome.claimed {
                return ax_runtime::hal::irq::IrqReturn::Unhandled;
            }
            let events = publish_serial_outcome(&backend, outcome, true);
            if events.is_empty() {
                ax_runtime::hal::irq::IrqReturn::Handled
            } else {
                ax_runtime::hal::irq::IrqReturn::Wake
            }
        }))
        .share_mode(ShareMode::Shared)
        .affinity(IrqAffinity::Fixed(CpuId(self.owner.0)))
        .auto_enable(AutoEnable::No);
        match ax_runtime::hal::irq::request_irq(irq_id, request) {
            Ok(handle) => {
                *self.irq_handle.lock() = Some(handle);
                Ok(())
            }
            Err(err) => {
                warn!(
                    "Failed to register {} IRQ handler for irq {:?}: {err:?}",
                    self.tty_name, irq_id
                );
                Err(AxError::Unsupported)
            }
        }
    }

    fn start_port(&self) -> bool {
        if self.started.load(Ordering::Acquire) {
            return true;
        }
        let _guard = self.start_lock.lock();
        if self.started.load(Ordering::Acquire) {
            return true;
        }

        let config = Config::new().baudrate(startup_baudrate(self.baudrate()));
        let startup_result = match self.mode {
            SerialBackendMode::Interrupt { .. } => self.startup_port(&config),
            SerialBackendMode::Polling => self.startup_polling_port(&config),
        };
        if let Err(err) = startup_result {
            warn!(
                "{} failed to start serial port {}: {:?}",
                self.tty_name, self.name, err
            );
            return false;
        }

        if let SerialBackendMode::Interrupt { irq } = self.mode {
            let Some(handle) = *self.irq_handle.lock() else {
                self.shutdown_port();
                return false;
            };
            if let Err(err) = ax_runtime::hal::irq::enable_irq(handle) {
                self.shutdown_port();
                warn!(
                    "Failed to enable {} IRQ handler for irq {:?}: {err:?}",
                    self.tty_name, irq
                );
                return false;
            }
        }

        self.started.store(true, Ordering::Release);
        publish_serial_outcome(
            self,
            self.service_on_owner(SerialSoftWork::RESERVICE),
            false,
        );
        self.events.publish(SerialEventBits::RX_READY);
        true
    }

    fn ensure_started(&self) -> AxResult<()> {
        if self.start_port() {
            Ok(())
        } else {
            Err(AxError::Unsupported)
        }
    }

    fn startup_port(&self, config: &Config) -> Result<SerialIrqOutcome, ConfigError> {
        ax_serial::run_on_owner(self.owner, |lease| self.port.startup(lease, config))
            .map_err(|_| ConfigError::RegisterError)?
    }

    fn startup_polling_port(&self, config: &Config) -> Result<SerialIrqOutcome, ConfigError> {
        ax_serial::run_on_owner(self.owner, |lease| self.port.startup_polling(lease, config))
            .map_err(|_| ConfigError::RegisterError)?
    }

    fn shutdown_port(&self) {
        let _ = ax_serial::run_on_owner(self.owner, |lease| self.port.shutdown(lease));
    }

    fn set_port_config(&self, config: &Config) -> Result<(), ConfigError> {
        ax_serial::run_on_owner(self.owner, |lease| self.port.set_config(lease, config))
            .map_err(|_| ConfigError::RegisterError)?
    }

    fn baudrate(&self) -> u32 {
        ax_serial::run_on_owner(self.owner, |lease| self.port.baudrate(lease)).unwrap_or(0)
    }

    fn service_on_owner(&self, work: SerialSoftWork) -> SerialIrqOutcome {
        ax_serial::run_on_owner(self.owner, |lease| self.port.service(lease, work))
            .unwrap_or_default()
    }

    fn handle_irq_on_owner(&self, cpu: CpuId, irq: &mut SerialIrqHandler) -> SerialIrqOutcome {
        let Some(lease) = ax_serial::owner_lease_for_cpu(self.owner, cpu) else {
            return SerialIrqOutcome::default();
        };
        irq.handle(lease)
    }

    fn submit_tx(&self, bytes: &[u8]) -> (usize, SerialIrqOutcome) {
        let submit = self.tx.lock().submit(bytes);
        let outcome = if submit.needs_kick {
            self.service_on_owner(SerialSoftWork::TX_KICK)
        } else {
            SerialIrqOutcome::default()
        };
        (submit.accepted, outcome)
    }

    fn drain_rx(&self, out: &mut [RxItem]) -> usize {
        self.rx.lock().drain(out)
    }
}

fn startup_baudrate(current: u32) -> u32 {
    if current == 0 {
        SERIAL_DEFAULT_BAUDRATE
    } else {
        current
    }
}

fn spawn_serial_event_worker(backend: Arc<SerialBackend>) {
    let task_name = format!("{}-event", backend.tty_name);
    ax_task::spawn_with_name(
        move || loop {
            backend.events.wait();
            loop {
                let pending = backend.events.take();
                if pending.is_empty() {
                    break;
                }
                if pending.contains(SerialEventBits::RX_READY) {
                    unsafe { backend.input_source.wake(IoEvents::IN) };
                }
                if pending.contains(SerialEventBits::TX_SPACE) {
                    backend.tx_notify.notify();
                    unsafe { backend.output_source.wake(IoEvents::OUT) };
                    let outcome = backend.service_on_owner(SerialSoftWork::TX_KICK);
                    publish_serial_outcome(&backend, outcome, false);
                }
            }
        },
        task_name,
    );
}

fn spawn_serial_poll_worker(backend: Arc<SerialBackend>) {
    let task_name = format!("{}-poll", backend.tty_name);
    ax_task::spawn_with_name(
        move || {
            let interval = Duration::from_millis(SERIAL_POLL_INTERVAL_MS);
            loop {
                if backend.started.load(Ordering::Acquire) {
                    let outcome = backend.service_on_owner(SerialSoftWork::RESERVICE);
                    publish_serial_outcome(&backend, outcome, false);
                }
                ax_task::sleep(interval);
            }
        },
        task_name,
    );
}

fn publish_serial_outcome(
    backend: &SerialBackend,
    outcome: SerialIrqOutcome,
    from_irq: bool,
) -> SerialEventBits {
    let mut events = SerialEventBits::empty();
    if outcome.rx_pushed > 0 {
        events |= SerialEventBits::RX_READY;
    }
    if outcome.tx_wakeup {
        events |= SerialEventBits::TX_SPACE;
    }

    if from_irq {
        backend.events.publish_irq(events);
    } else {
        backend.events.publish(events);
    }
    events
}

impl TtyRead for SerialReader {
    fn read(&mut self, buf: &mut [u8]) -> usize {
        if !self.backend.started.load(Ordering::Acquire) {
            return 0;
        }

        let mut total = 0;
        let mut temp = [RxItem::default(); SERIAL_RX_DRAIN_CHUNK];

        while total < buf.len() {
            let limit = (buf.len() - total).min(temp.len());
            let read = self.backend.drain_rx(&mut temp[..limit]);
            if read == 0 {
                break;
            }
            for item in &temp[..read] {
                match *item {
                    RxItem::Byte {
                        byte,
                        flag: RxFlag::Normal,
                    } => {
                        buf[total] = byte;
                        total += 1;
                    }
                    RxItem::Byte { byte, flag } => {
                        warn!(
                            "{} RX error {:?} while preserving byte {byte:#x}",
                            self.backend.tty_name, flag
                        );
                        buf[total] = byte;
                        total += 1;
                    }
                    RxItem::Overrun => {
                        warn!("{} RX overrun", self.backend.tty_name);
                    }
                }
            }
        }

        total
    }
}

impl TtyWrite for SerialWriter {
    fn open(&self) -> AxResult<()> {
        self.backend.ensure_started()
    }

    fn write(&self, buf: &[u8]) {
        if buf.is_empty() {
            return;
        }
        if self.backend.ensure_started().is_err() {
            return;
        }
        let _guard = self.backend.output_lock.lock();
        let mut written = 0;
        while written < buf.len() {
            let (count, outcome) = self.backend.submit_tx(&buf[written..]);
            publish_serial_outcome(&self.backend, outcome, false);
            if count == 0 {
                self.backend.tx_notify.wait();
                continue;
            }
            written += count;
        }
    }

    fn try_write(&self, buf: &[u8]) -> usize {
        if buf.is_empty() {
            return 0;
        }
        if self.backend.ensure_started().is_err() {
            return 0;
        }
        let Some(_guard) = self.backend.output_lock.try_lock() else {
            return 0;
        };
        let (count, outcome) = self.backend.submit_tx(buf);
        publish_serial_outcome(&self.backend, outcome, false);
        count
    }

    fn flush_echo_before_input(&self) -> bool {
        true
    }

    fn max_sync_echo_bytes(&self) -> usize {
        SERIAL_SYNC_ECHO_LIMIT
    }

    fn termios_changed(&self, old: &Termios2, new: &Termios2) {
        let Some(new_baud) = new.baudrate() else {
            return;
        };
        if old.baudrate() == Some(new_baud) {
            return;
        }
        if self.backend.ensure_started().is_err() {
            return;
        }
        if let Err(err) = self
            .backend
            .set_port_config(&Config::new().baudrate(new_baud))
        {
            warn!(
                "{} failed to set baudrate {new_baud} on {}: {:?}",
                self.backend.tty_name, self.backend.name, err
            );
        }
    }
}

fn assign_tty_numbers(alias_indices: &[Option<usize>]) -> Vec<Option<usize>> {
    let mut assigned = vec![None; alias_indices.len()];
    let mut used = Vec::new();

    for (device_index, alias) in alias_indices.iter().copied().enumerate() {
        let Some(number) = alias else {
            continue;
        };
        if used.contains(&number) {
            warn!("Duplicate FDT serial{number} alias ignored for later serial device");
            continue;
        }
        assigned[device_index] = Some(number);
        used.push(number);
    }

    let mut next = 0usize;
    for number in &mut assigned {
        if number.is_some() {
            continue;
        }
        while used.contains(&next) {
            next += 1;
        }
        *number = Some(next);
        used.push(next);
    }

    assigned
}

fn select_console_candidate(
    candidates: &[ConsoleCandidate],
    selected_device_id: ConsoleDeviceIdResult,
) -> Option<ConsoleSelection> {
    match selected_device_id {
        Ok(device_id) => {
            warn!(
                "select_console_candidate: searching for device_id={device_id:?} among {} \
                 candidate(s)",
                candidates.len()
            );
            if let Some(index) = candidates
                .iter()
                .position(|candidate| candidate.device_id == device_id)
            {
                warn!("select_console_candidate: matched candidate[{index}] -> SelectedDevice");
                return Some(ConsoleSelection::SelectedDevice(index));
            }
            warn!("selected console device {device_id:?} did not match a discovered serial TTY");
            None
        }
        Err(ConsoleDeviceIdError::NotSpecified) => {
            warn!(
                "select_console_candidate: NotSpecified, falling back to ttyS0 (candidates={})",
                candidates.len()
            );
            let result = candidates
                .iter()
                .position(|candidate| candidate.number == 0)
                .map(ConsoleSelection::TtyS0Fallback);
            warn!("select_console_candidate: ttyS0 fallback = {result:?}");
            result
        }
        Err(
            err @ (ConsoleDeviceIdError::NoHardwareDevice | ConsoleDeviceIdError::DeviceNotFound),
        ) => {
            warn!("select_console_candidate: no hardware console ({err:?})");
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use rdrive::DeviceId as RDriveDeviceId;

    use super::{
        ConsoleCandidate, ConsoleDeviceIdError, ConsoleSelection, assign_tty_numbers,
        select_console_candidate,
    };

    #[test]
    fn aliases_keep_linux_ttys_numbering() {
        assert_eq!(assign_tty_numbers(&[Some(0), Some(2)]), [Some(0), Some(2)]);
    }

    #[test]
    fn unaliased_serials_take_first_free_ttys_numbers() {
        assert_eq!(
            assign_tty_numbers(&[Some(0), None, Some(2), None]),
            [Some(0), Some(1), Some(2), Some(3)]
        );
    }

    #[test]
    fn duplicate_alias_keeps_first_device_and_reassigns_later_one() {
        assert_eq!(
            assign_tty_numbers(&[Some(1), Some(1), None]),
            [Some(1), Some(0), Some(2)]
        );
    }

    #[test]
    fn matching_device_id_wins_over_ttys0_fallback() {
        let tty_s0 = RDriveDeviceId::from(10);
        let tty_s1 = RDriveDeviceId::from(11);
        let candidates = [
            ConsoleCandidate {
                number: 0,
                device_id: tty_s0,
            },
            ConsoleCandidate {
                number: 1,
                device_id: tty_s1,
            },
        ];

        assert_eq!(
            select_console_candidate(&candidates, Ok(tty_s1)),
            Some(ConsoleSelection::SelectedDevice(1))
        );
    }

    #[test]
    fn unmatched_device_id_keeps_dev_console_unbound() {
        let tty_s0 = RDriveDeviceId::from(10);
        let missing = RDriveDeviceId::from(99);
        let candidates = [ConsoleCandidate {
            number: 0,
            device_id: tty_s0,
        }];

        assert_eq!(select_console_candidate(&candidates, Ok(missing)), None);
    }

    #[test]
    fn missing_device_id_falls_back_to_ttys0() {
        let tty_s0 = RDriveDeviceId::from(10);
        let candidates = [ConsoleCandidate {
            number: 0,
            device_id: tty_s0,
        }];

        assert_eq!(
            select_console_candidate(&candidates, Err(ConsoleDeviceIdError::NotSpecified)),
            Some(ConsoleSelection::TtyS0Fallback(0))
        );
    }

    #[test]
    fn no_ttys0_keeps_dev_console_unbound() {
        let tty_s1 = RDriveDeviceId::from(11);
        let candidates = [ConsoleCandidate {
            number: 1,
            device_id: tty_s1,
        }];

        assert_eq!(
            select_console_candidate(&candidates, Err(ConsoleDeviceIdError::NotSpecified)),
            None
        );
    }

    #[test]
    fn non_hardware_console_keeps_dev_console_unbound() {
        let tty_s0 = RDriveDeviceId::from(10);
        let candidates = [ConsoleCandidate {
            number: 0,
            device_id: tty_s0,
        }];

        assert_eq!(
            select_console_candidate(&candidates, Err(ConsoleDeviceIdError::NoHardwareDevice)),
            None
        );
    }

    #[test]
    fn zero_hardware_baudrate_uses_runtime_default() {
        assert_eq!(super::startup_baudrate(0), super::SERIAL_DEFAULT_BAUDRATE);
        assert_eq!(super::startup_baudrate(1_500_000), 1_500_000);
    }
}
