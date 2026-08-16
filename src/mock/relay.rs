//! A mock inverter whose current mode is shown on a Waveshare 4-channel relay.

use std::io::{Read, Write};
use std::time::Duration;

use super::MockInverter;
use crate::{Applied, Capabilities, Command, DischargeTarget, Error, Inverter, Mode, Telemetry};

const PASSIVE_MASK: u8 = 1 << 0;
const CHARGE_MASK: u8 = 1 << 1;
const DISCHARGE_MASK: u8 = 1 << 2;
const EXPORT_MASK: u8 = 1 << 3;
const ALL_CHANNELS_MASK: u8 = 0x0f;
const SERIAL_TIMEOUT: Duration = Duration::from_millis(750);

trait RelayTransport: Send {
    fn exchange(&mut self, request: &[u8], response_len: usize) -> Result<Vec<u8>, Error>;
}

struct SerialRelayTransport {
    port: Box<dyn serialport::SerialPort>,
}

impl SerialRelayTransport {
    fn open(port: &str, baud_rate: u32) -> Result<Self, Error> {
        let port = serialport::new(port, baud_rate)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .timeout(SERIAL_TIMEOUT)
            .open()
            .map_err(|error| {
                Error::Comm(format!(
                    "could not open mock relay serial port {port}: {error}"
                ))
            })?;
        Ok(Self { port })
    }
}

impl RelayTransport for SerialRelayTransport {
    fn exchange(&mut self, request: &[u8], response_len: usize) -> Result<Vec<u8>, Error> {
        let _ = self.port.clear(serialport::ClearBuffer::Input);
        self.port
            .write_all(request)
            .and_then(|()| self.port.flush())
            .map_err(|error| Error::Comm(format!("mock relay write failed: {error}")))?;
        let mut response = vec![0; response_len];
        self.port
            .read_exact(&mut response)
            .map_err(|error| Error::Comm(format!("mock relay response failed: {error}")))?;
        Ok(response)
    }
}

struct WaveshareRelay {
    transport: Box<dyn RelayTransport>,
    slave: u8,
}

impl WaveshareRelay {
    fn open(port: &str, baud_rate: u32, slave: u8) -> Result<Self, Error> {
        if slave == 0 {
            return Err(Error::Range(
                "mock relay slave must be in the range 1..=255".into(),
            ));
        }
        Ok(Self {
            transport: Box::new(SerialRelayTransport::open(port, baud_rate)?),
            slave,
        })
    }

    #[cfg(test)]
    fn with_transport(transport: Box<dyn RelayTransport>, slave: u8) -> Self {
        Self { transport, slave }
    }

    fn set_mask(&mut self, mask: u8) -> Result<(), Error> {
        if mask & !ALL_CHANNELS_MASK != 0 {
            return Err(Error::Range(format!(
                "mock relay mask must fit four channels, got 0x{mask:02x}"
            )));
        }
        let request = with_crc(vec![self.slave, 0x0f, 0x00, 0x00, 0x00, 0x04, 0x01, mask]);
        let response = self.transport.exchange(&request, 8)?;
        check_response(&response, self.slave, 0x0f, 8)?;
        let expected = with_crc(vec![self.slave, 0x0f, 0x00, 0x00, 0x00, 0x04]);
        if response != expected {
            return Err(Error::Readback(
                "mock relay did not acknowledge coils 0..3".into(),
            ));
        }
        let actual = self.read_mask()?;
        if actual != mask {
            return Err(Error::Readback(format!(
                "mock relay mask is 0x{actual:x}, expected 0x{mask:x}"
            )));
        }
        Ok(())
    }

    fn read_mask(&mut self) -> Result<u8, Error> {
        let request = with_crc(vec![self.slave, 0x01, 0x00, 0x00, 0x00, 0x04]);
        let response = self.transport.exchange(&request, 6)?;
        check_response(&response, self.slave, 0x01, 6)?;
        if response[2] != 1 || response[3] & !ALL_CHANNELS_MASK != 0 {
            return Err(Error::Readback(format!(
                "invalid mock relay coil response: {:02x?}",
                response
            )));
        }
        Ok(response[3] & ALL_CHANNELS_MASK)
    }

    fn show(&mut self, command: Command) -> Result<(), Error> {
        self.set_mask(mask_for(command))
    }
}

/// A [`MockInverter`] that displays its current command on a Waveshare
/// Modbus RTU Relay 4CH.
///
/// Channel 1 is passive, channel 2 force charge, channel 3 house-only
/// discharge, and channel 4 grid export. All channels off means hold. The
/// relay is an indicator only: all battery telemetry and command-expiry
/// behavior still come from the mock.
pub struct RelayMockInverter {
    mock: MockInverter,
    relay: WaveshareRelay,
}

impl MockInverter {
    /// Attach a Waveshare Modbus RTU Relay 4CH mode indicator.
    ///
    /// The serial port opens immediately and channel 1 is selected to show the
    /// mock's initial passive state. Each write is verified by reading all four
    /// coils back from the relay.
    pub fn with_waveshare_relay(
        self,
        port: &str,
        baud_rate: u32,
        slave: u8,
    ) -> Result<RelayMockInverter, Error> {
        let mut relay = WaveshareRelay::open(port, baud_rate, slave)?;
        relay.show(Command::passive())?;
        Ok(RelayMockInverter { mock: self, relay })
    }
}

impl RelayMockInverter {
    #[cfg(test)]
    fn with_transport(mock: MockInverter, transport: Box<dyn RelayTransport>, slave: u8) -> Self {
        Self {
            mock,
            relay: WaveshareRelay::with_transport(transport, slave),
        }
    }

    fn refresh_indicator(&mut self) -> Result<(), Error> {
        self.relay.show(self.mock.active_command())
    }

    /// Advance simulated time and refresh the relay, without sleeping.
    pub fn advance(&mut self, elapsed: Duration) -> Result<(), Error> {
        self.mock.advance(elapsed);
        self.refresh_indicator()
    }

    /// Return the mock command currently in force, including TTL expiry.
    #[must_use]
    pub fn active_command(&self) -> Command {
        self.mock.active_command()
    }
}

impl Inverter for RelayMockInverter {
    fn capabilities(&self) -> Capabilities {
        let mut capabilities = self.mock.capabilities();
        capabilities.model = "mock + Waveshare relay";
        capabilities
    }

    fn read_telemetry(&mut self) -> Result<Telemetry, Error> {
        let telemetry = self.mock.read_telemetry()?;
        self.refresh_indicator()?;
        Ok(telemetry)
    }

    fn apply(&mut self, command: Command) -> Result<Applied, Error> {
        let applied = self.mock.apply(command)?;
        self.relay.show(command)?;
        Ok(applied)
    }

    fn mode(&mut self) -> Result<Mode, Error> {
        let mode = self.mock.mode()?;
        self.refresh_indicator()?;
        Ok(mode)
    }

    fn close(&mut self) {
        if let Err(error) = self.relay.set_mask(0) {
            log::warn!(target: "inverter.mock", "could not switch mock relay off: {error}");
        }
        self.mock.close();
    }
}

fn mask_for(command: Command) -> u8 {
    match (command.mode, command.target) {
        (Mode::Passive, _) => PASSIVE_MASK,
        (Mode::Hold, _) => 0,
        (Mode::ForceCharge, _) => CHARGE_MASK,
        (Mode::ForceDischarge, DischargeTarget::HouseOnly) => DISCHARGE_MASK,
        (Mode::ForceDischarge, DischargeTarget::GridExport) => EXPORT_MASK,
    }
}

fn crc16_modbus(bytes: &[u8]) -> u16 {
    let mut crc = 0xffffu16;
    for byte in bytes {
        crc ^= u16::from(*byte);
        for _ in 0..8 {
            crc = if crc & 1 == 1 {
                (crc >> 1) ^ 0xa001
            } else {
                crc >> 1
            };
        }
    }
    crc
}

fn with_crc(mut frame: Vec<u8>) -> Vec<u8> {
    frame.extend_from_slice(&crc16_modbus(&frame).to_le_bytes());
    frame
}

fn check_response(frame: &[u8], slave: u8, function: u8, expected_len: usize) -> Result<(), Error> {
    if frame.len() != expected_len {
        return Err(Error::Comm(format!(
            "mock relay response has {} bytes, expected {expected_len}",
            frame.len()
        )));
    }
    let data_len = frame.len() - 2;
    let received_crc = u16::from_le_bytes([frame[data_len], frame[data_len + 1]]);
    let expected_crc = crc16_modbus(&frame[..data_len]);
    if received_crc != expected_crc {
        return Err(Error::Comm(format!(
            "mock relay response has CRC 0x{received_crc:04x}, expected 0x{expected_crc:04x}"
        )));
    }
    if frame[0] != slave {
        return Err(Error::Comm(format!(
            "mock relay response came from slave {}, expected {slave}",
            frame[0]
        )));
    }
    if frame[1] == function | 0x80 {
        return Err(Error::Comm(format!(
            "mock relay returned Modbus exception 0x{:02x}",
            frame[2]
        )));
    }
    if frame[1] != function {
        return Err(Error::Comm(format!(
            "mock relay response used function 0x{:02x}, expected 0x{function:02x}",
            frame[1]
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use super::*;

    struct RecordingTransport {
        mask: Arc<Mutex<u8>>,
    }

    impl RelayTransport for RecordingTransport {
        fn exchange(&mut self, request: &[u8], response_len: usize) -> Result<Vec<u8>, Error> {
            check_response(request, 1, request[1], request.len())?;
            match request[1] {
                0x0f => {
                    *self.mask.lock().unwrap() = request[7];
                    assert_eq!(response_len, 8);
                    Ok(with_crc(vec![1, 0x0f, 0, 0, 0, 4]))
                }
                0x01 => {
                    assert_eq!(response_len, 6);
                    Ok(with_crc(vec![1, 0x01, 1, *self.mask.lock().unwrap()]))
                }
                function => panic!("unexpected function 0x{function:02x}"),
            }
        }
    }

    fn indicated_mock() -> (RelayMockInverter, Arc<Mutex<u8>>) {
        let mask = Arc::new(Mutex::new(0));
        let transport = RecordingTransport {
            mask: Arc::clone(&mask),
        };
        let mock = RelayMockInverter::with_transport(MockInverter::new(), Box::new(transport), 1);
        (mock, mask)
    }

    #[test]
    fn command_modes_select_the_four_relay_channels() {
        let ttl = Duration::from_secs(60);
        let (mut mock, mask) = indicated_mock();
        for (command, expected) in [
            (Command::passive(), PASSIVE_MASK),
            (Command::hold(ttl), 0),
            (Command::charge(2, ttl), CHARGE_MASK),
            (Command::discharge(2, ttl), DISCHARGE_MASK),
            (Command::export(2, ttl), EXPORT_MASK),
        ] {
            mock.apply(command).unwrap();
            assert_eq!(*mask.lock().unwrap(), expected);
        }
    }

    #[test]
    fn ttl_expiry_returns_the_indicator_to_passive() {
        let (mut mock, mask) = indicated_mock();
        mock.apply(Command::charge(2, Duration::from_secs(5)))
            .unwrap();
        mock.advance(Duration::from_secs(5)).unwrap();
        assert_eq!(*mask.lock().unwrap(), PASSIVE_MASK);
        assert_eq!(mock.active_command().mode, Mode::Passive);
    }

    #[test]
    fn close_switches_every_channel_off() {
        let (mut mock, mask) = indicated_mock();
        mock.apply(Command::passive()).unwrap();
        mock.close();
        assert_eq!(*mask.lock().unwrap(), 0);
    }

    #[test]
    fn crc_matches_the_waveshare_protocol_example() {
        assert_eq!(
            with_crc(vec![1, 0x01, 0, 0, 0, 4]),
            vec![1, 0x01, 0, 0, 0, 4, 0x3d, 0xc9]
        );
    }
}
