//! FoxESS H1-series driver.
//!
//! **Reads only.** The register map here is compiled from community
//! documentation and has not been verified against hardware, so this driver
//! reports [`Capabilities::can_write`] as `false` and refuses commands. Verify
//! the map against the inverter's own display before that changes — see the
//! crate README for what verification means.

use crate::modbus::{read_words, with_retries, ModbusBus};
use crate::register::{decode, RegisterDef};
use crate::{Applied, Capabilities, Command, Error, Expiry, Inverter, Telemetry};
use std::time::{Instant, SystemTime};

const LOG_TARGET: &str = "inverter.foxess";

const WRITE_BLOCKED: &str = "FoxESS register map is unverified on hardware; \
     reads are trusted, writes are not implemented";

/// FoxESS H1-shaped Modbus map. Every address is unverified.
///
/// Addresses live here and nowhere else, so the map can be checked against
/// hardware without reading driver code.
pub mod registers {
    use crate::register::RegisterDef as R;

    /// Battery state of charge, percent.
    pub const BATTERY_SOC: R = R::input("battery_soc", 11036);
    /// Battery power. Positive is charging.
    pub const BATTERY_POWER: R = R::input("battery_power", 11008).signed();
    /// Grid power. Positive is import.
    pub const GRID_ACTIVE_POWER: R = R::input("grid_active_power", 11021).signed();
    /// Household consumption.
    pub const HOUSE_LOAD_POWER: R = R::input("house_load_power", 11023);
}

/// A FoxESS H1-series inverter on any Modbus transport.
pub struct FoxEss<B: ModbusBus> {
    bus: B,
}

impl<B: ModbusBus> FoxEss<B> {
    /// Wrap an already-open bus.
    pub fn new(bus: B) -> Self {
        log::warn!(
            target: LOG_TARGET,
            "FoxESS driver started with an UNVERIFIED register map - reads only"
        );
        Self { bus }
    }

    fn read(&mut self, reg: &RegisterDef) -> Result<f64, Error> {
        with_retries(
            &mut self.bus,
            LOG_TARGET,
            &format!("read {}", reg.name),
            |bus| read_words(bus, reg).map(|words| decode(reg, &words)),
        )
    }
}

#[cfg(feature = "serial")]
impl FoxEss<crate::modbus::SerialBus> {
    /// Open a FoxESS inverter over a serial RS485 adapter.
    pub fn open_serial(port: &str, baud_rate: u32, unit_id: u8) -> Result<Self, Error> {
        Ok(Self::new(crate::modbus::SerialBus::open(
            port, baud_rate, unit_id,
        )?))
    }
}

#[cfg(feature = "tcp")]
impl FoxEss<crate::modbus::TcpBus> {
    /// Open a FoxESS inverter through a Modbus TCP bridge, e.g. `"10.0.0.5:502"`.
    pub fn open_tcp(addr: &str, unit_id: u8) -> Result<Self, Error> {
        Ok(Self::new(crate::modbus::TcpBus::connect(addr, unit_id)?))
    }
}

impl<B: ModbusBus> Inverter for FoxEss<B> {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            model: "FoxESS H1 (community map, unverified)",
            can_write: false,
            modes: &[],
            // Nothing is commanded, so nothing expires. Revisit alongside the
            // write path: the answer depends on whether the H1's remote-control
            // registers carry a duration, which is the open question.
            expiry: Expiry::UntilChanged,
            reports_solar: false,
            write_blocked_reason: Some(WRITE_BLOCKED),
        }
    }

    fn read_telemetry(&mut self) -> Result<Telemetry, Error> {
        let soc_pct = self.read(&registers::BATTERY_SOC)?;
        let battery_w = self.read(&registers::BATTERY_POWER)?;
        let grid_w = self.read(&registers::GRID_ACTIVE_POWER)?;
        let load_w = self.read(&registers::HOUSE_LOAD_POWER)?;
        Ok(Telemetry {
            soc_pct,
            battery_w,
            grid_w,
            load_w,
            solar_w: 0.0,
            at: SystemTime::now(),
            read_at: Instant::now(),
        })
    }

    fn apply(&mut self, command: Command) -> Result<Applied, Error> {
        Err(Error::Unsupported(format!(
            "{WRITE_BLOCKED} (refused: {})",
            command.describe()
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Mode;
    use std::collections::HashMap;

    /// What a verified write path would eventually expose. Asserted against so
    /// that enabling writes without updating `capabilities` fails a test.
    const INTENDED_MODES: &[Mode] = &[Mode::Passive, Mode::ForceCharge, Mode::ForceDischarge];

    /// A bus that replays canned register values.
    struct FakeBus {
        input: HashMap<u16, Vec<u16>>,
    }

    impl FakeBus {
        fn new(pairs: &[(u16, u16)]) -> Self {
            Self {
                input: pairs.iter().map(|&(a, v)| (a, vec![v])).collect(),
            }
        }
    }

    impl ModbusBus for FakeBus {
        fn read_input(&mut self, address: u16, _words: u8) -> Result<Vec<u16>, Error> {
            self.input
                .get(&address)
                .cloned()
                .ok_or_else(|| Error::Comm(format!("no fixture for input {address}")))
        }
        fn read_holding(&mut self, address: u16, _words: u8) -> Result<Vec<u16>, Error> {
            Err(Error::Comm(format!("no fixture for holding {address}")))
        }
        fn write_holding(&mut self, _address: u16, _value: u16) -> Result<(), Error> {
            Err(Error::Comm("fake bus is read-only".into()))
        }
    }

    fn fixture() -> FakeBus {
        FakeBus::new(&[
            (registers::BATTERY_SOC.address, 64),
            // -500 W: discharging, in two's complement.
            (registers::BATTERY_POWER.address, (-500i16) as u16),
            (registers::GRID_ACTIVE_POWER.address, 250),
            (registers::HOUSE_LOAD_POWER.address, 750),
        ])
    }

    #[test]
    fn decodes_telemetry_with_crate_sign_conventions() {
        let mut inv = FoxEss::new(fixture());
        let t = inv.read_telemetry().unwrap();
        assert_eq!(t.soc_pct, 64.0);
        assert_eq!(t.battery_w, -500.0, "negative means discharging");
        assert_eq!(t.grid_w, 250.0, "positive means importing");
        assert_eq!(t.load_w, 750.0);
        assert_eq!(t.export_w(), 0.0, "importing, so nothing is exported");
    }

    #[test]
    fn reports_that_it_cannot_write_and_says_why() {
        let inv = FoxEss::new(fixture());
        let caps = inv.capabilities();
        assert!(!caps.can_write);
        assert!(caps.write_blocked_reason.is_some());
        for mode in INTENDED_MODES {
            assert!(!caps.supports(*mode), "{mode:?} must not be advertised");
        }
    }

    #[test]
    fn refuses_every_command_while_the_map_is_unverified() {
        let mut inv = FoxEss::new(fixture());
        for command in [
            Command::passive(),
            Command::charge(2_000.0),
            Command::export(3_000.0),
        ] {
            assert!(matches!(inv.apply(command), Err(Error::Unsupported(_)),));
        }
    }

    #[test]
    fn a_missing_register_is_an_error_not_a_plausible_zero() {
        let mut inv = FoxEss::new(FakeBus::new(&[(registers::BATTERY_SOC.address, 50)]));
        assert!(inv.read_telemetry().is_err());
    }
}
