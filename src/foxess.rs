//! FoxESS H1-series driver.
//!
//! **Reads only.** The register maps here are compiled from community
//! documentation — principally the
//! [`nathanmarlor/foxess_modbus`](https://github.com/nathanmarlor/foxess_modbus)
//! Home Assistant integration (MIT) — and have not been verified against
//! hardware, so this driver reports [`Capabilities::can_write`] as `false` and
//! refuses commands. Verify a map against the inverter's own display before
//! that changes — see the crate README for what verification means.
//!
//! # Which map?
//!
//! The H1 generations differ over the same RS485 wire: a G1 serves telemetry
//! as *input* registers in the 11000 range, while a G2 (H1-\*-G2, AC1-G2, P1)
//! serves *holding* registers in the 31000 range. Pick [`registers::H1_G1`]
//! or [`registers::H1_G2`] to match the unit. An H1 connected through its own
//! LAN module speaks a third, reduced map that this driver does not cover.
//!
//! # The write path that is not implemented yet
//!
//! The H1's remote-control block (see [`registers::remote_control`]) carries
//! a genuine watchdog: a timeout register the inverter counts down on its own
//! and, on expiry, reverts to its programmed work mode. A verified write path
//! can therefore offer [`Expiry::InverterTimeout`] — a dead controller leaves
//! the inverter reverting by itself. That answer comes from reading
//! `foxess_modbus`; it still needs proving on hardware before writes open.

use crate::modbus::{read_words, with_retries, ModbusBus};
use crate::register::{decode, RegisterDef};
use crate::{Applied, Capabilities, Command, Error, Expiry, Inverter, Telemetry};
use std::time::{Instant, SystemTime};

const LOG_TARGET: &str = "inverter.foxess";

const WRITE_BLOCKED: &str = "FoxESS register map is unverified on hardware; \
     reads are trusted, writes are not implemented";

/// The registers a FoxESS telemetry read needs, for one model generation.
///
/// Raw FoxESS values use the opposite sign to this crate for battery and grid
/// power — positive raw means discharging and exporting respectively — so
/// those registers carry a `-1` scale to land on crate conventions.
pub struct RegisterMap {
    /// Human-readable model identifier, surfaced in [`Capabilities::model`].
    pub model: &'static str,
    /// Battery state of charge, percent.
    pub battery_soc: RegisterDef,
    /// Battery power, normalised so positive means charging.
    pub battery_power: RegisterDef,
    /// Grid power, normalised so positive means importing.
    pub grid_power: RegisterDef,
    /// Household consumption.
    pub load_power: RegisterDef,
    /// Per-string PV generation; summed into [`Telemetry::solar_w`].
    pub pv_powers: &'static [RegisterDef],
}

/// FoxESS H1-shaped Modbus maps. Every address is unverified.
///
/// Addresses live here and nowhere else, so a map can be checked against
/// hardware without reading driver code. Sources: `foxess_modbus`
/// `entity_descriptions.py` and `remote_control_description.py` (MIT).
pub mod registers {
    use super::RegisterMap;
    use crate::register::RegisterDef as R;

    /// H1/AC1/AIO-H1 first generation over RS485: input registers.
    pub const H1_G1: RegisterMap = RegisterMap {
        model: "FoxESS H1 G1 (RS485, community map, unverified)",
        battery_soc: R::input("battery_soc", 11036),
        // Raw: positive = discharging. Negated to crate convention.
        battery_power: R::input("battery_power", 11008).signed().scale(-1.0),
        // Raw: positive = exporting. Negated to crate convention.
        grid_power: R::input("grid_ct", 11021).signed().scale(-1.0),
        load_power: R::input("load_power", 11023).signed(),
        pv_powers: &[R::input("pv1_power", 11002), R::input("pv2_power", 11005)],
    };

    /// H1-G2, AC1-G2 and P1 over RS485: holding registers.
    ///
    /// The G2 does not serve the G1's 11000-range input registers; reading
    /// the wrong generation's map fails rather than returning wrong numbers.
    pub const H1_G2: RegisterMap = RegisterMap {
        model: "FoxESS H1 G2 (RS485, community map, unverified)",
        battery_soc: R::holding("battery_soc", 31024),
        // Raw: positive = discharging. Negated to crate convention.
        battery_power: R::holding("battery_power", 31022).signed().scale(-1.0),
        // Raw: positive = exporting. Negated to crate convention.
        grid_power: R::holding("grid_ct", 31014).signed().scale(-1.0),
        load_power: R::holding("load_power", 31016).signed(),
        pv_powers: &[
            R::holding("pv1_power", 39280),
            R::holding("pv2_power", 39282),
        ],
    };

    /// The H1 family's remote-control block: the future write path.
    ///
    /// Not used by the driver yet — recorded so verification against hardware
    /// can start from data, not from a Home Assistant code dive. Semantics
    /// observed in `foxess_modbus`'s `remote_control_manager.py`:
    ///
    /// * Enabling: write [`remote_control::TIMEOUT_SET`], then `1` to
    ///   [`remote_control::REMOTE_ENABLE`]. These registers reject
    ///   multi-register writes; use function 6.
    /// * While enabled, [`remote_control::ACTIVE_POWER`] sets inverter power:
    ///   positive exports/discharges, negative imports/charges (opposite sign
    ///   to [`crate::Telemetry::battery_w`] — a write path must negate).
    /// * [`remote_control::TIMEOUT_SET`] is a watchdog reload in seconds. If
    ///   the controller stops writing, the inverter reverts *by itself* to
    ///   the work mode in [`remote_control::WORK_MODE`] — `foxess_modbus`
    ///   re-writes power every poll and sets the timeout to twice its poll
    ///   rate. This is what makes [`crate::Expiry::InverterTimeout`] honest
    ///   for this hardware, and it also means the fallback must be programmed
    ///   to self-use *before* enabling remote control if expiry is to mean
    ///   "passive".
    /// * The inverter does **not** respect [`remote_control::MAX_SOC`] while
    ///   remote-control charging (`foxess_modbus` enforces it in software);
    ///   it does respect [`remote_control::MIN_SOC`] and the max discharge
    ///   current while discharging.
    /// * The FoxESS app's "strategy periods" drive the same registers, so a
    ///   phone app is a competing writer, not a passive observer.
    ///
    /// Addresses are common to H1 G1 and G2; the H3 family differs.
    pub mod remote_control {
        use crate::register::RegisterDef as R;

        /// Remote control on/off: write 1 to enable, 0 to disable.
        pub const REMOTE_ENABLE: R = R::holding("remote_enable", 44000);
        /// Watchdog reload value, seconds.
        pub const TIMEOUT_SET: R = R::holding("timeout_set", 44001);
        /// Power command, watts. Positive = export, negative = import.
        pub const ACTIVE_POWER: R = R::holding("active_power", 44002).signed();
        /// Work mode the inverter reverts to when the watchdog expires:
        /// 0 self-use, 1 feed-in first, 2 back-up.
        pub const WORK_MODE: R = R::holding("work_mode", 41000);
        /// Discharge floor, percent. Respected during remote control.
        pub const MIN_SOC: R = R::holding("min_soc", 41009);
        /// Charge ceiling, percent. NOT respected during remote control.
        pub const MAX_SOC: R = R::holding("max_soc", 41010);
    }
}

/// A FoxESS H1-series inverter on any Modbus transport.
pub struct FoxEss<B: ModbusBus> {
    bus: B,
    map: &'static RegisterMap,
}

impl<B: ModbusBus> FoxEss<B> {
    /// Wrap an already-open bus, reading via `map`.
    pub fn new(bus: B, map: &'static RegisterMap) -> Self {
        log::warn!(
            target: LOG_TARGET,
            "FoxESS driver started with an UNVERIFIED register map ({}) - reads only",
            map.model
        );
        Self { bus, map }
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
    pub fn open_serial(
        port: &str,
        baud_rate: u32,
        unit_id: u8,
        map: &'static RegisterMap,
    ) -> Result<Self, Error> {
        Ok(Self::new(
            crate::modbus::SerialBus::open(port, baud_rate, unit_id)?,
            map,
        ))
    }
}

#[cfg(feature = "tcp")]
impl FoxEss<crate::modbus::TcpBus> {
    /// Open a FoxESS inverter through a Modbus TCP bridge, e.g. `"10.0.0.5:502"`.
    pub fn open_tcp(addr: &str, unit_id: u8, map: &'static RegisterMap) -> Result<Self, Error> {
        Ok(Self::new(
            crate::modbus::TcpBus::connect(addr, unit_id)?,
            map,
        ))
    }
}

impl<B: ModbusBus> Inverter for FoxEss<B> {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            model: self.map.model,
            can_write: false,
            modes: &[],
            // Nothing is commanded, so nothing expires. A verified write path
            // built on the remote-control watchdog would report
            // InverterTimeout - see the registers::remote_control docs.
            expiry: Expiry::UntilChanged,
            reports_solar: !self.map.pv_powers.is_empty(),
            write_blocked_reason: Some(WRITE_BLOCKED),
        }
    }

    fn read_telemetry(&mut self) -> Result<Telemetry, Error> {
        let map = self.map;
        let soc_pct = self.read(&map.battery_soc)?;
        let battery_w = self.read(&map.battery_power)?;
        let grid_w = self.read(&map.grid_power)?;
        // FoxESS reports load signed and it can dip slightly negative from
        // metering noise; the crate convention is load_w >= 0.
        let load_w = self.read(&map.load_power)?.max(0.0);
        let mut solar_w = 0.0;
        for pv in map.pv_powers {
            solar_w += self.read(pv)?;
        }
        Ok(Telemetry {
            soc_pct,
            battery_w,
            grid_w,
            load_w,
            solar_w,
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
        holding: HashMap<u16, Vec<u16>>,
    }

    impl FakeBus {
        fn with_input(pairs: &[(u16, u16)]) -> Self {
            Self {
                input: pairs.iter().map(|&(a, v)| (a, vec![v])).collect(),
                holding: HashMap::new(),
            }
        }

        fn with_holding(pairs: &[(u16, u16)]) -> Self {
            Self {
                input: HashMap::new(),
                holding: pairs.iter().map(|&(a, v)| (a, vec![v])).collect(),
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
            self.holding
                .get(&address)
                .cloned()
                .ok_or_else(|| Error::Comm(format!("no fixture for holding {address}")))
        }
        fn write_holding(&mut self, _address: u16, _value: u16) -> Result<(), Error> {
            Err(Error::Comm("fake bus is read-only".into()))
        }
    }

    /// A G1 charging from the grid: raw battery negative, raw grid negative.
    fn g1_fixture() -> FakeBus {
        FakeBus::with_input(&[
            (registers::H1_G1.battery_soc.address, 64),
            // Raw -1500: charging, in FoxESS's discharge-positive convention.
            (registers::H1_G1.battery_power.address, (-1500i16) as u16),
            // Raw -2000: importing, in FoxESS's export-positive convention.
            (registers::H1_G1.grid_power.address, (-2000i16) as u16),
            (registers::H1_G1.load_power.address, 500),
            (registers::H1_G1.pv_powers[0].address, 0),
            (registers::H1_G1.pv_powers[1].address, 0),
        ])
    }

    /// A G2 discharging to cover load and exporting the rest, with some sun.
    fn g2_fixture() -> FakeBus {
        FakeBus::with_holding(&[
            (registers::H1_G2.battery_soc.address, 55),
            // Raw +400: discharging.
            (registers::H1_G2.battery_power.address, 400),
            // Raw +250: exporting.
            (registers::H1_G2.grid_power.address, 250),
            (registers::H1_G2.load_power.address, 750),
            (registers::H1_G2.pv_powers[0].address, 300),
            (registers::H1_G2.pv_powers[1].address, 300),
        ])
    }

    #[test]
    fn g1_normalises_foxess_signs_to_crate_conventions() {
        let mut inv = FoxEss::new(g1_fixture(), &registers::H1_G1);
        let t = inv.read_telemetry().unwrap();
        assert_eq!(t.soc_pct, 64.0);
        assert_eq!(t.battery_w, 1500.0, "raw negative means charging");
        assert_eq!(t.grid_w, 2000.0, "raw negative means importing");
        assert_eq!(t.load_w, 500.0);
        assert_eq!(t.solar_w, 0.0);
        assert_eq!(t.export_w(), 0.0, "importing, so nothing is exported");
    }

    #[test]
    fn g2_reads_holding_registers_and_sums_pv_strings() {
        let mut inv = FoxEss::new(g2_fixture(), &registers::H1_G2);
        let t = inv.read_telemetry().unwrap();
        assert_eq!(t.soc_pct, 55.0);
        assert_eq!(t.battery_w, -400.0, "raw positive means discharging");
        assert_eq!(t.grid_w, -250.0, "raw positive means exporting");
        assert_eq!(t.export_w(), 250.0);
        assert_eq!(t.load_w, 750.0);
        assert_eq!(t.solar_w, 600.0, "both PV strings summed");
    }

    #[test]
    fn a_slightly_negative_load_reading_clamps_to_zero() {
        let mut bus = g2_fixture();
        bus.holding
            .insert(registers::H1_G2.load_power.address, vec![(-5i16) as u16]);
        let mut inv = FoxEss::new(bus, &registers::H1_G2);
        assert_eq!(inv.read_telemetry().unwrap().load_w, 0.0);
    }

    #[test]
    fn reports_that_it_cannot_write_and_says_why() {
        for map in [&registers::H1_G1, &registers::H1_G2] {
            let inv = FoxEss::new(FakeBus::with_input(&[]), map);
            let caps = inv.capabilities();
            assert!(!caps.can_write);
            assert!(caps.write_blocked_reason.is_some());
            assert!(caps.reports_solar);
            for mode in INTENDED_MODES {
                assert!(!caps.supports(*mode), "{mode:?} must not be advertised");
            }
        }
    }

    #[test]
    fn refuses_every_command_while_the_map_is_unverified() {
        let mut inv = FoxEss::new(g2_fixture(), &registers::H1_G2);
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
        let mut inv = FoxEss::new(
            FakeBus::with_input(&[(registers::H1_G1.battery_soc.address, 50)]),
            &registers::H1_G1,
        );
        assert!(inv.read_telemetry().is_err());
    }

    #[test]
    fn a_g2_wired_up_with_the_g1_map_fails_rather_than_lying() {
        // The maps live in different register tables, so the mismatch is an
        // error instead of plausible zeroes.
        let mut inv = FoxEss::new(g2_fixture(), &registers::H1_G1);
        assert!(inv.read_telemetry().is_err());
    }

    #[test]
    fn the_maps_pin_the_community_documented_addresses() {
        use crate::register::RegKind;

        let g1 = &registers::H1_G1;
        assert_eq!(g1.battery_soc.address, 11036);
        assert_eq!(g1.battery_power.address, 11008);
        assert_eq!(g1.grid_power.address, 11021);
        assert_eq!(g1.load_power.address, 11023);
        assert!(g1.pv_powers.iter().all(|reg| reg.kind == RegKind::Input));

        let g2 = &registers::H1_G2;
        assert_eq!(g2.battery_soc.address, 31024);
        assert_eq!(g2.battery_power.address, 31022);
        assert_eq!(g2.grid_power.address, 31014);
        assert_eq!(g2.load_power.address, 31016);
        assert!(g2.pv_powers.iter().all(|reg| reg.kind == RegKind::Holding));

        for map in [g1, g2] {
            assert_eq!(
                map.battery_power.scale, -1.0,
                "raw battery power is discharge-positive and must be negated"
            );
            assert_eq!(
                map.grid_power.scale, -1.0,
                "raw grid power is export-positive and must be negated"
            );
            assert!(map.battery_power.signed && map.grid_power.signed && map.load_power.signed);
        }
    }

    #[test]
    fn a_missing_pv_register_is_an_error_like_any_other() {
        let mut bus = g2_fixture();
        bus.holding.remove(&registers::H1_G2.pv_powers[1].address);
        let mut inv = FoxEss::new(bus, &registers::H1_G2);
        assert!(inv.read_telemetry().is_err());
    }

    #[test]
    fn remote_control_registers_are_recorded_but_unused() {
        // The write path is not implemented; the block is data for hardware
        // verification. The watchdog register is the whole point: it is what
        // will let a verified write path report Expiry::InverterTimeout.
        assert_eq!(registers::remote_control::TIMEOUT_SET.address, 44001);
        assert_eq!(registers::remote_control::REMOTE_ENABLE.address, 44000);
        assert_eq!(registers::remote_control::ACTIVE_POWER.address, 44002);
    }
}
