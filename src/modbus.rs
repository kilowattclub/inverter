//! Modbus transports.
//!
//! [`ModbusBus`] abstracts serial RTU and Modbus TCP connections. Concrete
//! transports require the `serial` or `tcp` feature.

use std::time::Duration;

use crate::register::{RegKind, RegisterDef};
use crate::Error;

const RETRIES: u32 = 3;
const BACKOFF: Duration = Duration::from_millis(500);

/// A Modbus connection to one unit.
pub trait ModbusBus: Send {
    /// Read `words` input registers starting at `address`.
    fn read_input(&mut self, address: u16, words: u8) -> Result<Vec<u16>, Error>;
    /// Read `words` holding registers starting at `address`.
    fn read_holding(&mut self, address: u16, words: u8) -> Result<Vec<u16>, Error>;
    /// Write one holding register.
    fn write_holding(&mut self, address: u16, value: u16) -> Result<(), Error>;
}

/// Read the exact number of words in a register definition.
pub fn read_words(bus: &mut dyn ModbusBus, reg: &RegisterDef) -> Result<Vec<u16>, Error> {
    let words = match reg.kind {
        RegKind::Input => bus.read_input(reg.address, reg.words)?,
        RegKind::Holding => bus.read_holding(reg.address, reg.words)?,
    };
    if words.len() != reg.words as usize {
        return Err(Error::Comm(format!(
            "short modbus reply for {}: got {} words, expected {}",
            reg.name,
            words.len(),
            reg.words
        )));
    }
    Ok(words)
}

/// Run up to three attempts, with 500 ms and 1 s delays between failures.
/// A lost connection returns immediately; callers must start a fresh operation.
pub fn with_retries<B: ModbusBus + ?Sized, R>(
    bus: &mut B,
    log_target: &str,
    operation: &str,
    mut run: impl FnMut(&mut B) -> Result<R, Error>,
) -> Result<R, Error> {
    let mut delay = BACKOFF;
    let mut last_error = None;
    for attempt in 1..=RETRIES {
        match run(bus) {
            Ok(value) => return Ok(value),
            Err(error) => {
                log::warn!(
                    target: log_target,
                    "modbus operation failed: operation={operation} attempt={attempt} error={error}"
                );
                // A retry must not reopen a connection in the middle of a
                // telemetry sample or replay part of an old control command.
                if matches!(error, Error::Disconnected(_)) {
                    return Err(error);
                }
                last_error = Some(error);
                if attempt < RETRIES {
                    std::thread::sleep(delay);
                    delay *= 2;
                }
            }
        }
    }
    Err(last_error.unwrap_or_else(|| Error::Comm("modbus operation failed".into())))
}

#[cfg(any(feature = "serial", feature = "tcp"))]
mod stream {
    use std::io::{Read, Write};
    use std::time::{Duration, Instant};

    use rmodbus::client::ModbusRequest;
    use rmodbus::ModbusProto;

    use super::ModbusBus;
    use crate::register::RegKind;
    use crate::Error;

    const READ_DEADLINE: Duration = Duration::from_secs(3);

    /// A byte stream carrying Modbus frames.
    pub(super) trait ModbusStream: Read + Write + Send {
        /// Drop buffered input left over from an abandoned transaction.
        fn discard_input(&mut self) {}
    }

    /// Modbus over any byte stream, framed by `proto`.
    pub(super) struct StreamBus<S: ModbusStream> {
        pub(super) stream: S,
        pub(super) unit: u8,
        pub(super) proto: ModbusProto,
        pub(super) transaction_id: u16,
    }

    impl<S: ModbusStream> StreamBus<S> {
        fn request(&mut self) -> ModbusRequest {
            self.transaction_id = self.transaction_id.wrapping_add(1);
            let mut request = ModbusRequest::new(self.unit, self.proto);
            request.tr_id = self.transaction_id;
            request
        }

        fn transact<T>(
            &mut self,
            request: &[u8],
            parse: impl FnOnce(&[u8]) -> Result<T, Error>,
        ) -> Result<T, Error> {
            // A timed-out transaction may leave bytes belonging to an earlier reply.
            self.stream.discard_input();
            self.stream
                .write_all(request)
                .map_err(|e| Error::Comm(format!("modbus write failed: {e}")))?;
            self.stream
                .flush()
                .map_err(|e| Error::Comm(format!("modbus flush failed: {e}")))?;

            // Replies arrive in pieces; accumulate until the frame is complete.
            let deadline = Instant::now() + READ_DEADLINE;
            let mut buf = Vec::with_capacity(260);
            let mut chunk = [0u8; 260];
            let header_len = if self.proto == ModbusProto::TcpUdp {
                6
            } else {
                3
            };
            let mut frame_len = header_len;
            loop {
                if Instant::now() >= deadline {
                    return Err(Error::Comm(format!(
                        "incomplete modbus response ({} bytes) before deadline",
                        buf.len()
                    )));
                }
                let n = self
                    .stream
                    .read(&mut chunk[..frame_len - buf.len()])
                    .map_err(|e| Error::Comm(format!("modbus read failed: {e}")))?;
                if n == 0 {
                    return Err(Error::Comm("empty modbus response".into()));
                }
                buf.extend_from_slice(&chunk[..n]);
                if buf.len() < frame_len {
                    continue;
                }
                if frame_len != header_len {
                    break;
                }
                frame_len = if self.proto == ModbusProto::TcpUdp {
                    let length = usize::from(u16::from_be_bytes([buf[4], buf[5]]));
                    if buf[2..4] != [0, 0] || !(3..=254).contains(&length) {
                        return Err(Error::Comm("invalid modbus TCP header".into()));
                    }
                    6 + length
                } else {
                    match buf[1] {
                        3 | 4 => 5 + usize::from(buf[2]),
                        6 => 8,
                        0x80..=0xff => 5,
                        _ => return Err(Error::Comm("invalid modbus response function".into())),
                    }
                };
            }
            parse(&buf)
        }

        fn read_registers(
            &mut self,
            kind: RegKind,
            address: u16,
            words: u8,
        ) -> Result<Vec<u16>, Error> {
            if !(1..=125).contains(&words) || address.checked_add(u16::from(words) - 1).is_none() {
                return Err(Error::Range("invalid modbus register range".into()));
            }
            let mut builder = self.request();
            let mut request = Vec::new();
            let built = match kind {
                RegKind::Input => builder.generate_get_inputs(address, words.into(), &mut request),
                RegKind::Holding => {
                    builder.generate_get_holdings(address, words.into(), &mut request)
                }
            };
            built.map_err(|e| Error::Comm(format!("frame build failed: {e:?}")))?;
            self.transact(&request, move |buf| {
                // Validate length before parse_u16: an empty payload can panic
                // in rmodbus, and extra words would otherwise be truncated.
                let payload = builder
                    .parse_slice(buf)
                    .map_err(|e| Error::Comm(format!("modbus read error at {address}: {e:?}")))?;
                let offset = if builder.proto == ModbusProto::TcpUdp {
                    6
                } else {
                    0
                };
                let expected = usize::from(words) * 2;
                if payload.len() != expected || usize::from(buf[offset + 2]) != expected {
                    return Err(Error::Comm(format!(
                        "invalid modbus read length at {address}: expected {expected} bytes"
                    )));
                }
                let mut out = Vec::new();
                builder
                    .parse_u16(buf, &mut out)
                    .map_err(|e| Error::Comm(format!("modbus read error at {address}: {e:?}")))?;
                Ok(out)
            })
        }
    }

    impl<S: ModbusStream> ModbusBus for StreamBus<S> {
        fn read_input(&mut self, address: u16, words: u8) -> Result<Vec<u16>, Error> {
            self.read_registers(RegKind::Input, address, words)
        }

        fn read_holding(&mut self, address: u16, words: u8) -> Result<Vec<u16>, Error> {
            self.read_registers(RegKind::Holding, address, words)
        }

        fn write_holding(&mut self, address: u16, value: u16) -> Result<(), Error> {
            let mut builder = self.request();
            let mut request = Vec::new();
            builder
                .generate_set_holding(address, value, &mut request)
                .map_err(|e| Error::Comm(format!("frame build failed: {e:?}")))?;
            self.transact(&request, move |buf| {
                let response = builder
                    .parse_slice(buf)
                    .map_err(|e| Error::Comm(format!("modbus write error at {address}: {e:?}")))?;
                let [address_hi, address_lo] = address.to_be_bytes();
                let [value_hi, value_lo] = value.to_be_bytes();
                if response != [address_hi, address_lo, value_hi, value_lo] {
                    return Err(Error::Readback(format!(
                        "modbus write acknowledgement does not match register {address} value {value}"
                    )));
                }
                Ok(())
            })
        }
    }
}

#[cfg(feature = "serial")]
mod serial {
    use std::io::{Read, Write};
    use std::time::{Duration, Instant};

    use rmodbus::ModbusProto;

    use super::stream::{ModbusStream, StreamBus};
    use super::ModbusBus;
    use crate::Error;

    struct SerialStream(Box<dyn serialport::SerialPort>);

    impl Read for SerialStream {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            self.0.read(buf)
        }
    }

    impl Write for SerialStream {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.0.write(buf)
        }
        fn flush(&mut self) -> std::io::Result<()> {
            self.0.flush()
        }
    }

    impl ModbusStream for SerialStream {
        fn discard_input(&mut self) {
            let _ = self.0.clear(serialport::ClearBuffer::Input);
        }
    }

    /// Modbus RTU over a serial port.
    ///
    /// Use a stable `/dev/serial/by-id/...` path rather than `/dev/ttyUSB0`,
    /// whose number changes across boots and when another adapter is present.
    /// A failed transaction closes the handle and returns `Error::Disconnected`.
    /// Subsequent reads reopen the configured path with 0.5–30 second backoff.
    /// Writes never reopen or replay an interrupted operation.
    pub struct SerialBus {
        port: String,
        baud_rate: u32,
        unit_id: u8,
        bus: Option<StreamBus<SerialStream>>,
        next_open: Instant,
        backoff: Duration,
    }

    const MIN_BACKOFF: Duration = Duration::from_millis(500);
    const MAX_BACKOFF: Duration = Duration::from_secs(30);

    impl SerialBus {
        /// Open `port` at `baud_rate`, addressing unit `unit_id`.
        pub fn open(port: &str, baud_rate: u32, unit_id: u8) -> Result<Self, Error> {
            Ok(Self {
                port: port.into(),
                baud_rate,
                unit_id,
                bus: Some(Self::connect(port, baud_rate, unit_id)?),
                next_open: Instant::now(),
                backoff: MIN_BACKOFF,
            })
        }

        fn connect(
            port: &str,
            baud_rate: u32,
            unit_id: u8,
        ) -> Result<StreamBus<SerialStream>, Error> {
            let handle = serialport::new(port, baud_rate)
                .data_bits(serialport::DataBits::Eight)
                .parity(serialport::Parity::None)
                .stop_bits(serialport::StopBits::One)
                .timeout(Duration::from_secs(2))
                .open()
                .map_err(|e| Error::Comm(format!("could not open serial port {port}: {e}")))?;
            Ok(StreamBus {
                stream: SerialStream(handle),
                unit: unit_id,
                proto: ModbusProto::Rtu,
                transaction_id: 0,
            })
        }

        fn reopen_for_read(&mut self) -> Result<(), Error> {
            if self.bus.is_some() {
                return Ok(());
            }
            if Instant::now() < self.next_open {
                return Err(Error::Disconnected("waiting to reopen serial port".into()));
            }
            match Self::connect(&self.port, self.baud_rate, self.unit_id) {
                Ok(bus) => {
                    self.bus = Some(bus);
                    log::info!(target: "inverter.serial", "reopened {}; reading fresh telemetry", self.port);
                    Ok(())
                }
                Err(error) => {
                    self.defer_open();
                    Err(Error::Disconnected(error.to_string()))
                }
            }
        }

        fn defer_open(&mut self) {
            self.next_open = Instant::now() + self.backoff;
            self.backoff = (self.backoff * 2).min(MAX_BACKOFF);
        }

        fn finish<T>(&mut self, result: Result<T, Error>) -> Result<T, Error> {
            match result {
                Err(error @ Error::Comm(_)) => {
                    // Dropping the handle matters: the same by-id symlink can
                    // now point at a new tty after a USB disconnect/reconnect.
                    self.bus = None;
                    self.defer_open();
                    Err(Error::Disconnected(error.to_string()))
                }
                Ok(value) => {
                    self.backoff = MIN_BACKOFF;
                    Ok(value)
                }
                Err(error) => Err(error),
            }
        }
    }

    impl ModbusBus for SerialBus {
        fn read_input(&mut self, address: u16, words: u8) -> Result<Vec<u16>, Error> {
            self.reopen_for_read()?;
            let result = self
                .bus
                .as_mut()
                .expect("open serial bus")
                .read_input(address, words);
            self.finish(result)
        }
        fn read_holding(&mut self, address: u16, words: u8) -> Result<Vec<u16>, Error> {
            self.reopen_for_read()?;
            let result = self
                .bus
                .as_mut()
                .expect("open serial bus")
                .read_holding(address, words);
            self.finish(result)
        }
        fn write_holding(&mut self, address: u16, value: u16) -> Result<(), Error> {
            let bus = self.bus.as_mut().ok_or_else(|| {
                Error::Disconnected("read fresh telemetry before writing after a disconnect".into())
            })?;
            let result = bus.write_holding(address, value);
            self.finish(result)
        }
    }

    // macOS's serial driver requires modem ioctls that pseudo-terminals do
    // not support. Exercise USB replacement with Linux PTYs (the Pi's OS).
    #[cfg(all(test, target_os = "linux"))]
    mod tests {
        use super::*;
        use serialport::SerialPort;
        use std::sync::atomic::{AtomicUsize, Ordering};

        struct Adapter {
            path: std::path::PathBuf,
        }

        impl Adapter {
            fn new() -> Self {
                static ID: AtomicUsize = AtomicUsize::new(0);
                let path = std::env::temp_dir().join(format!(
                    "inverter-serial-{}-{}",
                    std::process::id(),
                    ID.fetch_add(1, Ordering::Relaxed)
                ));
                Self { path }
            }

            fn attach(&self) -> serialport::TTYPort {
                let (mut master, mut slave) = serialport::TTYPort::pair().unwrap();
                master.set_timeout(Duration::from_secs(3)).unwrap();
                slave.set_exclusive(false).unwrap();
                let _ = std::fs::remove_file(&self.path);
                std::os::unix::fs::symlink(slave.name().unwrap(), &self.path).unwrap();
                master
            }

            fn open(&self) -> SerialBus {
                SerialBus::open(self.path.to_str().unwrap(), 9600, 1).unwrap()
            }
        }

        impl Drop for Adapter {
            fn drop(&mut self) {
                let _ = std::fs::remove_file(&self.path);
            }
        }

        fn reply(master: &mut serialport::TTYPort) {
            let mut request = [0u8; 8];
            master.read_exact(&mut request).unwrap();
            // The first operation on a reopened connection must be a read,
            // never the interrupted write to register 44002.
            assert_eq!(request[1], 3);
            let mut response = vec![request[0], 3, 2, 0, 42];
            let mut crc = 0xffff_u16;
            for byte in &response {
                crc ^= u16::from(*byte);
                for _ in 0..8 {
                    crc = (crc >> 1) ^ if crc & 1 != 0 { 0xa001 } else { 0 };
                }
            }
            response.extend_from_slice(&crc.to_le_bytes());
            master.write_all(&response).unwrap();
        }

        #[test]
        fn usb_replacement_reopens_stable_path_without_replaying_a_failed_write() {
            let adapter = Adapter::new();
            let old = adapter.attach();
            let original_path = std::fs::read_link(&adapter.path).unwrap();
            let mut bus = adapter.open();
            drop(old); // USB removed, leaving the existing handle unusable.
            assert!(matches!(
                super::super::with_retries(&mut bus, "test", "write", |bus| bus
                    .write_holding(44002, 2000)),
                Err(Error::Disconnected(_))
            ));
            assert!(bus.bus.is_none());

            // Device remains absent: reads fail, with bounded reopen attempts.
            bus.next_open = Instant::now();
            assert!(matches!(
                bus.read_holding(31000, 1),
                Err(Error::Disconnected(_))
            ));
            let scheduled = bus.next_open;
            assert!(matches!(
                bus.read_holding(31000, 1),
                Err(Error::Disconnected(_))
            ));
            assert_eq!(
                bus.next_open, scheduled,
                "backoff must prevent an immediate reopen"
            );

            let _reserve_old_tty = serialport::TTYPort::pair().unwrap();
            let mut new = adapter.attach();
            assert_ne!(std::fs::read_link(&adapter.path).unwrap(), original_path);
            bus.next_open = Instant::now();
            assert!(matches!(
                bus.write_holding(44002, 2000),
                Err(Error::Disconnected(_))
            ));
            assert!(bus.bus.is_none(), "writes must never reopen the adapter");
            let (done, wait) = std::sync::mpsc::channel();
            let device = std::thread::spawn(move || {
                reply(&mut new);
                wait.recv_timeout(Duration::from_secs(3)).unwrap();
            });
            assert_eq!(bus.read_holding(31000, 1).unwrap(), vec![42]);
            done.send(()).unwrap();
            device.join().unwrap();
        }

        #[test]
        fn disconnected_read_is_returned_to_the_caller_without_hidden_reconnection() {
            let adapter = Adapter::new();
            let master = adapter.attach();
            let mut bus = adapter.open();
            drop(master);
            let mut calls = 0;
            let result = super::super::with_retries(&mut bus, "test", "read", |bus| {
                calls += 1;
                bus.read_input(100, 1)
            });
            assert!(matches!(result, Err(Error::Disconnected(_))));
            assert_eq!(
                calls, 1,
                "a partial sample must not continue on a new connection"
            );
        }
    }
}

#[cfg(feature = "serial")]
pub use serial::SerialBus;

#[cfg(feature = "tcp")]
mod tcp {
    use std::io::{Read, Write};
    use std::net::TcpStream;
    use std::time::Duration;

    use rmodbus::ModbusProto;

    use super::stream::{ModbusStream, StreamBus};
    use super::ModbusBus;
    use crate::Error;

    struct TcpModbusStream(TcpStream);

    impl Read for TcpModbusStream {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            self.0.read(buf)
        }
    }

    impl Write for TcpModbusStream {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.0.write(buf)
        }
        fn flush(&mut self) -> std::io::Result<()> {
            self.0.flush()
        }
    }

    // A TCP socket cannot be flushed of stale input the way a serial buffer
    // can; rmodbus rejects a reply whose transaction id does not match, so a
    // desynchronised socket surfaces as an error rather than a bad decode.
    impl ModbusStream for TcpModbusStream {}

    /// Modbus TCP, for a serial bridge beside the inverter.
    ///
    /// The bridge must be configured for Modbus TCP, not raw RTU over TCP.
    pub struct TcpBus(StreamBus<TcpModbusStream>);

    impl TcpBus {
        /// Connect to `addr` (typically `host:502`), addressing unit `unit_id`.
        pub fn connect(addr: &str, unit_id: u8) -> Result<Self, Error> {
            let stream = TcpStream::connect(addr)
                .map_err(|e| Error::Comm(format!("could not connect to {addr}: {e}")))?;
            stream
                .set_read_timeout(Some(Duration::from_secs(2)))
                .and_then(|()| stream.set_write_timeout(Some(Duration::from_secs(2))))
                .map_err(|e| Error::Comm(format!("could not set timeouts on {addr}: {e}")))?;
            // Modbus frames are small and latency-sensitive.
            let _ = stream.set_nodelay(true);
            Ok(Self(StreamBus {
                stream: TcpModbusStream(stream),
                unit: unit_id,
                proto: ModbusProto::TcpUdp,
                transaction_id: 0,
            }))
        }
    }

    impl ModbusBus for TcpBus {
        fn read_input(&mut self, address: u16, words: u8) -> Result<Vec<u16>, Error> {
            self.0.read_input(address, words)
        }
        fn read_holding(&mut self, address: u16, words: u8) -> Result<Vec<u16>, Error> {
            self.0.read_holding(address, words)
        }
        fn write_holding(&mut self, address: u16, value: u16) -> Result<(), Error> {
            self.0.write_holding(address, value)
        }
    }
}

#[cfg(feature = "tcp")]
pub use tcp::TcpBus;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::register::RegisterDef;

    /// Fails the first `failures` reads, then serves `value`.
    struct FlakyBus {
        failures: u32,
        calls: u32,
        value: u16,
    }

    impl FlakyBus {
        fn new(failures: u32, value: u16) -> Self {
            Self {
                failures,
                calls: 0,
                value,
            }
        }

        fn serve(&mut self, words: u8) -> Result<Vec<u16>, Error> {
            self.calls += 1;
            if self.failures > 0 {
                self.failures -= 1;
                return Err(Error::Comm("injected failure".into()));
            }
            Ok(vec![self.value; words as usize])
        }
    }

    impl ModbusBus for FlakyBus {
        fn read_input(&mut self, _address: u16, words: u8) -> Result<Vec<u16>, Error> {
            self.serve(words)
        }
        fn read_holding(&mut self, _address: u16, words: u8) -> Result<Vec<u16>, Error> {
            self.serve(words)
        }
        fn write_holding(&mut self, _address: u16, _value: u16) -> Result<(), Error> {
            Err(Error::Comm("read-only".into()))
        }
    }

    /// Always returns `reply` regardless of what was asked for.
    struct CannedBus(Vec<u16>);

    impl ModbusBus for CannedBus {
        fn read_input(&mut self, _address: u16, _words: u8) -> Result<Vec<u16>, Error> {
            Ok(self.0.clone())
        }
        fn read_holding(&mut self, _address: u16, _words: u8) -> Result<Vec<u16>, Error> {
            Ok(self.0.clone())
        }
        fn write_holding(&mut self, _address: u16, _value: u16) -> Result<(), Error> {
            Ok(())
        }
    }

    #[test]
    fn read_words_rejects_a_short_reply_instead_of_decoding_zeroes() {
        let two_words = RegisterDef::input("wide", 100).words(2);
        let err = read_words(&mut CannedBus(vec![7]), &two_words).unwrap_err();
        assert!(err.to_string().contains("short modbus reply"), "{err}");
    }

    #[test]
    fn read_words_accepts_an_exact_reply_for_both_register_kinds() {
        assert_eq!(
            read_words(&mut CannedBus(vec![7]), &RegisterDef::input("in", 1)).unwrap(),
            vec![7]
        );
        assert_eq!(
            read_words(&mut CannedBus(vec![9]), &RegisterDef::holding("hold", 1)).unwrap(),
            vec![9]
        );
    }

    #[test]
    fn with_retries_recovers_from_a_transient_failure() {
        let mut bus = FlakyBus::new(1, 42);
        let words = with_retries(&mut bus, "test", "read", |bus| bus.read_input(1, 1)).unwrap();
        assert_eq!(words, vec![42]);
        assert_eq!(bus.calls, 2, "one failure, one success");
    }

    #[test]
    fn with_retries_gives_up_after_three_attempts_with_the_last_error() {
        let mut bus = FlakyBus::new(u32::MAX, 0);
        let err = with_retries(&mut bus, "test", "read", |bus| bus.read_input(1, 1)).unwrap_err();
        assert_eq!(bus.calls, 3, "exactly three attempts");
        assert!(err.to_string().contains("injected failure"), "{err}");
    }

    #[cfg(any(feature = "serial", feature = "tcp"))]
    mod framing {
        use super::super::stream::{ModbusStream, StreamBus};
        use super::super::ModbusBus;
        use rmodbus::ModbusProto;
        use std::collections::VecDeque;
        use std::io::{Read, Write};

        /// Replays canned chunks, one per read call; empty means EOF.
        struct ScriptedStream {
            chunks: VecDeque<Vec<u8>>,
            written: Vec<u8>,
        }

        impl ScriptedStream {
            fn replying(chunks: &[&[u8]]) -> Self {
                Self {
                    chunks: chunks.iter().map(|c| c.to_vec()).collect(),
                    written: Vec::new(),
                }
            }
        }

        impl Read for ScriptedStream {
            fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
                match self.chunks.pop_front() {
                    Some(chunk) => {
                        let n = buf.len().min(chunk.len());
                        buf[..n].copy_from_slice(&chunk[..n]);
                        if n < chunk.len() {
                            self.chunks.push_front(chunk[n..].to_vec());
                        }
                        Ok(n)
                    }
                    None => Ok(0),
                }
            }
        }

        impl Write for ScriptedStream {
            fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
                self.written.extend_from_slice(buf);
                Ok(buf.len())
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }

        impl ModbusStream for ScriptedStream {}

        fn rtu_bus(chunks: &[&[u8]]) -> StreamBus<ScriptedStream> {
            StreamBus {
                stream: ScriptedStream::replying(chunks),
                unit: 1,
                proto: ModbusProto::Rtu,
                transaction_id: 0,
            }
        }

        /// Standard Modbus CRC16 (poly 0xA001), low byte first on the wire.
        fn crc16(data: &[u8]) -> [u8; 2] {
            let mut crc: u16 = 0xFFFF;
            for &byte in data {
                crc ^= byte as u16;
                for _ in 0..8 {
                    crc = if crc & 1 != 0 {
                        (crc >> 1) ^ 0xA001
                    } else {
                        crc >> 1
                    };
                }
            }
            crc.to_le_bytes()
        }

        fn rtu_read_input_response(unit: u8, value: u16) -> Vec<u8> {
            let mut frame = vec![unit, 0x04, 0x02];
            frame.extend_from_slice(&value.to_be_bytes());
            let crc = crc16(&frame);
            frame.extend_from_slice(&crc);
            frame
        }

        fn frame(proto: ModbusProto, transaction: u16, pdu: &[u8]) -> Vec<u8> {
            if proto == ModbusProto::TcpUdp {
                let mut frame = transaction.to_be_bytes().to_vec();
                frame.extend_from_slice(&[0, 0]);
                frame.extend_from_slice(&((pdu.len() + 1) as u16).to_be_bytes());
                frame.push(1);
                frame.extend_from_slice(pdu);
                frame
            } else {
                let mut frame = vec![1];
                frame.extend_from_slice(pdu);
                frame.extend_from_slice(&crc16(&frame));
                frame
            }
        }

        #[test]
        fn write_acknowledgements_must_echo_the_register_and_value() {
            for proto in [ModbusProto::Rtu, ModbusProto::TcpUdp] {
                for (pdu, accepted) in [
                    ([6, 0, 100, 0, 42], true),
                    ([6, 0, 101, 0, 42], false),
                    ([6, 0, 100, 0, 43], false),
                ] {
                    let response = frame(proto, 1, &pdu);
                    let mut bus = rtu_bus(&[&response]);
                    bus.proto = proto;
                    assert_eq!(bus.write_holding(100, 42).is_ok(), accepted);
                }
            }
        }

        #[test]
        fn reads_reject_empty_short_odd_and_extra_payloads() {
            for proto in [ModbusProto::Rtu, ModbusProto::TcpUdp] {
                for pdu in [
                    vec![4, 0],
                    vec![4, 1, 42],
                    vec![4, 2, 0, 42],
                    vec![4, 3, 0, 42, 0],
                    vec![4, 6, 0, 42, 0, 43, 0, 44],
                ] {
                    let response = frame(proto, 1, &pdu);
                    let mut bus = rtu_bus(&[&response]);
                    bus.proto = proto;
                    assert!(bus.read_input(100, 2).is_err(), "{pdu:?}");
                }
            }
        }

        #[test]
        fn invalid_register_ranges_do_not_write_to_the_stream() {
            for (address, words) in [(0, 0), (0, 126), (u16::MAX, 2)] {
                let mut bus = rtu_bus(&[]);
                assert!(matches!(
                    bus.read_input(address, words),
                    Err(crate::Error::Range(_))
                ));
                assert!(bus.stream.written.is_empty());
            }
        }

        #[test]
        fn tcp_requests_use_distinct_transaction_ids_and_reject_stale_replies() {
            let response = frame(ModbusProto::TcpUdp, 1, &[4, 2, 0, 42]);
            let mut bus = rtu_bus(&[&response, &response]);
            bus.proto = ModbusProto::TcpUdp;
            assert_eq!(bus.read_input(100, 1).unwrap(), vec![42]);
            assert!(bus.read_input(101, 1).is_err());
            assert_eq!(&bus.stream.written[..2], &[0, 1]);
            assert_eq!(&bus.stream.written[12..14], &[0, 2]);
        }

        #[test]
        fn coalesced_tcp_frames_are_read_separately() {
            let mut responses = frame(ModbusProto::TcpUdp, 1, &[4, 2, 0, 42]);
            responses.extend(frame(ModbusProto::TcpUdp, 2, &[4, 2, 0, 43]));
            let mut bus = rtu_bus(&[&responses]);
            bus.proto = ModbusProto::TcpUdp;
            assert_eq!(bus.read_input(100, 1).unwrap(), vec![42]);
            assert_eq!(bus.read_input(101, 1).unwrap(), vec![43]);
        }

        #[test]
        fn malformed_tcp_lengths_return_errors_without_panicking() {
            for length in [0u16, 2, 255, u16::MAX] {
                let [hi, lo] = length.to_be_bytes();
                let mut bus = rtu_bus(&[&[0, 1, 0, 0, hi, lo]]);
                bus.proto = ModbusProto::TcpUdp;
                assert!(bus.read_input(100, 1).is_err());
            }
        }

        #[test]
        fn a_reply_arriving_in_pieces_is_reassembled() {
            let frame = rtu_read_input_response(1, 0x1234);
            let (head, tail) = frame.split_at(2);
            let mut bus = rtu_bus(&[head, tail]);
            assert_eq!(bus.read_input(100, 1).unwrap(), vec![0x1234]);
        }

        #[test]
        fn a_reply_arriving_byte_by_byte_is_reassembled() {
            // Regression: probing an incomplete buffer for the frame length
            // must not panic, however few bytes have arrived.
            let frame = rtu_read_input_response(1, 0x1234);
            let chunks: Vec<&[u8]> = frame.chunks(1).collect();
            let mut bus = rtu_bus(&chunks);
            assert_eq!(bus.read_input(100, 1).unwrap(), vec![0x1234]);
        }

        #[test]
        fn a_complete_reply_in_one_chunk_decodes() {
            let frame = rtu_read_input_response(1, 0xBEEF);
            let mut bus = rtu_bus(&[&frame]);
            assert_eq!(bus.read_input(7, 1).unwrap(), vec![0xBEEF]);
        }

        #[test]
        fn a_closed_stream_is_an_error_not_a_zero() {
            let mut bus = rtu_bus(&[]);
            let err = bus.read_input(100, 1).unwrap_err();
            assert!(err.to_string().contains("empty modbus response"), "{err}");
        }

        #[test]
        fn a_modbus_exception_reply_surfaces_as_an_error() {
            // Function 0x84 = exception response to a read-input request.
            let mut frame = vec![1u8, 0x84, 0x02];
            let crc = crc16(&frame);
            frame.extend_from_slice(&crc);
            let mut bus = rtu_bus(&[&frame]);
            assert!(bus.read_input(100, 1).is_err());
        }

        #[test]
        fn a_corrupt_crc_surfaces_as_an_error() {
            let mut frame = rtu_read_input_response(1, 0x1234);
            let last = frame.len() - 1;
            frame[last] ^= 0xFF;
            let mut bus = rtu_bus(&[&frame]);
            assert!(bus.read_input(100, 1).is_err());
        }
    }

    #[cfg(feature = "serial")]
    #[test]
    fn opening_a_missing_serial_port_fails_with_the_port_in_the_error() {
        let err = SerialBus::open("/definitely/not/a/port", 9600, 1)
            .err()
            .expect("opening a missing port must fail");
        assert!(err.to_string().contains("/definitely/not/a/port"), "{err}");
    }

    #[cfg(feature = "tcp")]
    #[test]
    fn tcp_bus_round_trips_a_read_against_a_real_socket() {
        use std::io::{Read as _, Write as _};
        use std::net::TcpListener;

        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let server = std::thread::spawn(move || {
            let (mut sock, _) = listener.accept().unwrap();
            // Read-input request: 7-byte MBAP header + 5-byte PDU.
            let mut request = [0u8; 12];
            sock.read_exact(&mut request).unwrap();
            assert_eq!(request[7], 0x04, "expected a read-input request");
            // Echo the transaction id and unit; reply with one register, 0x2A.
            let response = [
                request[0], request[1], 0, 0, 0, 5, request[6], 0x04, 0x02, 0x00, 0x2A,
            ];
            sock.write_all(&response).unwrap();
        });

        let mut bus = TcpBus::connect(&addr.to_string(), 1).unwrap();
        assert_eq!(bus.read_input(100, 1).unwrap(), vec![0x2A]);
        server.join().unwrap();
    }
}
