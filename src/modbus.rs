//! Modbus transports.
//!
//! Two ways to reach the same registers: RTU over a serial adapter wired to
//! the inverter, or TCP to a serial bridge sitting beside it. Drivers are
//! written against [`ModbusBus`] and do not care which is in use. The trait
//! and its helpers are always compiled; the concrete transports sit behind
//! the `serial` and `tcp` features.

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

/// Read a register definition's words, rejecting short replies.
///
/// A truncated reply decodes to plausible zeroes, which is worse than an error.
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

/// Run a bus operation, retrying with exponential backoff.
///
/// Serial lines drop frames; a single failure is not evidence of a broken
/// inverter. Persistent failure is, and is reported.
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
    }

    impl<S: ModbusStream> StreamBus<S> {
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
            let mut buf: Vec<u8> = Vec::with_capacity(256);
            let mut chunk = [0u8; 256];
            loop {
                let n = self
                    .stream
                    .read(&mut chunk)
                    .map_err(|e| Error::Comm(format!("modbus read failed: {e}")))?;
                if n == 0 {
                    return Err(Error::Comm("empty modbus response".into()));
                }
                buf.extend_from_slice(&chunk[..n]);
                // guess_response_frame_len panics below these lengths; a slow
                // line can legitimately deliver fewer bytes in the first read.
                let min_guess = match self.proto {
                    ModbusProto::TcpUdp => 6,
                    _ => 3,
                };
                if buf.len() >= min_guess {
                    if let Ok(frame_len) = rmodbus::guess_response_frame_len(&buf, self.proto) {
                        if buf.len() >= frame_len as usize {
                            buf.truncate(frame_len as usize);
                            break;
                        }
                    }
                }
                if Instant::now() >= deadline {
                    return Err(Error::Comm(format!(
                        "incomplete modbus response ({} bytes) before deadline",
                        buf.len()
                    )));
                }
            }
            parse(&buf)
        }

        fn read_registers(
            &mut self,
            kind: RegKind,
            address: u16,
            words: u8,
        ) -> Result<Vec<u16>, Error> {
            let mut builder = ModbusRequest::new(self.unit, self.proto);
            let mut request = Vec::new();
            let built = match kind {
                RegKind::Input => builder.generate_get_inputs(address, words.into(), &mut request),
                RegKind::Holding => {
                    builder.generate_get_holdings(address, words.into(), &mut request)
                }
            };
            built.map_err(|e| Error::Comm(format!("frame build failed: {e:?}")))?;
            self.transact(&request, move |buf| {
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
            let mut builder = ModbusRequest::new(self.unit, self.proto);
            let mut request = Vec::new();
            builder
                .generate_set_holding(address, value, &mut request)
                .map_err(|e| Error::Comm(format!("frame build failed: {e:?}")))?;
            self.transact(&request, move |buf| {
                builder
                    .parse_ok(buf)
                    .map_err(|e| Error::Comm(format!("modbus write error at {address}: {e:?}")))?;
                Ok(())
            })
        }
    }
}

#[cfg(feature = "serial")]
mod serial {
    use std::io::{Read, Write};
    use std::time::Duration;

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
    pub struct SerialBus(StreamBus<SerialStream>);

    impl SerialBus {
        /// Open `port` at `baud_rate`, addressing unit `unit_id`.
        pub fn open(port: &str, baud_rate: u32, unit_id: u8) -> Result<Self, Error> {
            let handle = serialport::new(port, baud_rate)
                .data_bits(serialport::DataBits::Eight)
                .parity(serialport::Parity::None)
                .stop_bits(serialport::StopBits::One)
                .timeout(Duration::from_secs(2))
                .open()
                .map_err(|e| Error::Comm(format!("could not open serial port {port}: {e}")))?;
            Ok(Self(StreamBus {
                stream: SerialStream(handle),
                unit: unit_id,
                proto: ModbusProto::Rtu,
            }))
        }
    }

    impl ModbusBus for SerialBus {
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
    /// This is the route for an RS485-to-network adapter (Elfin EW11 and
    /// similar), which lets the controller live somewhere other than next to
    /// the inverter. It puts the home network in the control path: treat a
    /// dropped connection as lost telemetry, not as a silent success.
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
                        buf[..chunk.len()].copy_from_slice(&chunk);
                        Ok(chunk.len())
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
