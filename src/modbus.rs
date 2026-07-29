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
                if let Ok(frame_len) = rmodbus::guess_response_frame_len(&buf, self.proto) {
                    if buf.len() >= frame_len as usize {
                        buf.truncate(frame_len as usize);
                        break;
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
