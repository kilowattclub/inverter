//! FoxESS H1-series driver.
//!
//! The register maps here are compiled from community documentation — principally the
//! [`nathanmarlor/foxess_modbus`](https://github.com/nathanmarlor/foxess_modbus)
//! Home Assistant integration (MIT). Telemetry and the H1 remote-control block
//! have been exercised by that project across H1-family hardware, but remain
//! model- and firmware-sensitive; see the crate README before enabling a real
//! installation.
//!
//! # Which map?
//!
//! The H1 generations differ over the same RS485 wire: a G1 serves telemetry
//! as *input* registers in the 11000 range, while a G2 (H1-\*-G2, AC1-G2, P1)
//! serves *holding* registers in the 31000 range. Pick [`registers::H1_G1`]
//! or [`registers::H1_G2`] to match the unit. An H1 connected through its own
//! LAN module speaks a third, reduced map that this driver does not cover.
//!
//! # Native command timeout
//!
//! The H1's remote-control block (see [`registers::remote_control`]) carries
//! a genuine watchdog: a timeout register the inverter counts down on its own
//! and, on expiry, reverts to its programmed work mode. This driver programs
//! self-use as that fallback, replaces the timeout before every command, and
//! disables remote control for passive. The countdown lives in the inverter,
//! so a dead controller still leaves the command expiring by itself.

use crate::modbus::{read_words, with_retries, ModbusBus};
use crate::register::{decode, encode, RegisterDef};
use crate::{
    Applied, Capabilities, Command, DischargeTarget, Error, Expiry, Inverter, Mode, Telemetry,
};
use std::time::{Duration, Instant, SystemTime};

const LOG_TARGET: &str = "inverter.foxess";

const MODES: &[Mode] = &[
    Mode::Passive,
    Mode::Hold,
    Mode::ForceCharge,
    Mode::ForceDischarge,
];
const MAX_TIMEOUT: Duration = Duration::from_secs(u16::MAX as u64);

/// The registers a FoxESS telemetry read needs, for one model generation.
///
/// Raw FoxESS power registers are watts, and battery/grid use the opposite
/// sign to this crate — positive raw means discharging and exporting — so
/// power registers carry a `0.001` scale (negated where the sign flips) to
/// land on crate kilowatts.
pub struct RegisterMap {
    /// Human-readable model identifier, surfaced in [`Capabilities::model`].
    pub model: &'static str,
    /// Battery state of charge, percent.
    pub battery_soc: RegisterDef,
    /// Battery power, kilowatts, normalised so positive means charging.
    pub battery_power: RegisterDef,
    /// Grid power, kilowatts, normalised so positive means importing.
    pub grid_power: RegisterDef,
    /// Household consumption, kilowatts.
    pub load_power: RegisterDef,
    /// Per-string PV generation; summed into [`Telemetry::solar_kw`].
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
        // Raw: watts, positive = discharging. Scaled to charge-positive kW.
        battery_power: R::input("battery_power", 11008).signed().scale(-0.001),
        // Raw: watts, positive = exporting. Scaled to import-positive kW.
        grid_power: R::input("grid_ct", 11021).signed().scale(-0.001),
        load_power: R::input("load_power", 11023).signed().scale(0.001),
        pv_powers: &[
            R::input("pv1_power", 11002).scale(0.001),
            R::input("pv2_power", 11005).scale(0.001),
        ],
    };

    /// H1-G2, AC1-G2 and P1 over RS485: holding registers.
    ///
    /// The G2 does not serve the G1's 11000-range input registers; reading
    /// the wrong generation's map fails rather than returning wrong numbers.
    pub const H1_G2: RegisterMap = RegisterMap {
        model: "FoxESS H1 G2 (RS485, community map, unverified)",
        battery_soc: R::holding("battery_soc", 31024),
        // Raw: watts, positive = discharging. Scaled to charge-positive kW.
        battery_power: R::holding("battery_power", 31022).signed().scale(-0.001),
        // Raw: watts, positive = exporting. Scaled to import-positive kW.
        grid_power: R::holding("grid_ct", 31014).signed().scale(-0.001),
        load_power: R::holding("load_power", 31016).signed().scale(0.001),
        pv_powers: &[
            R::holding("pv1_power", 39280).scale(0.001),
            R::holding("pv2_power", 39282).scale(0.001),
        ],
    };

    /// The H1 family's remote-control block.
    ///
    /// Semantics observed in `foxess_modbus`'s
    /// `remote_control_manager.py` and its hardware reports:
    ///
    /// * Enabling: write [`remote_control::TIMEOUT_SET`], then `1` to
    ///   [`remote_control::REMOTE_ENABLE`]. These registers reject
    ///   multi-register writes; use function 6.
    /// * While enabled, [`remote_control::ACTIVE_POWER`] sets inverter power:
    ///   positive exports/discharges, negative imports/charges (opposite sign
    ///   to [`crate::Telemetry::battery_kw`] — a write path must negate).
    /// * [`remote_control::TIMEOUT_SET`] sets the watchdog period in seconds;
    ///   writing [`remote_control::ACTIVE_POWER`] loads/reloads its countdown.
    ///   If the controller stops writing, the inverter reverts *by itself* to
    ///   the work mode in [`remote_control::WORK_MODE`]. This driver sets that
    ///   fallback to self-use before enabling remote control, so expiry means
    ///   [`Mode::Passive`](crate::Mode::Passive).
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
        /// Power command, kilowatts (the raw register is watts).
        /// Positive = export, negative = import.
        pub const ACTIVE_POWER: R = R::holding("active_power", 44002).signed().scale(0.001);
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
    /// Wrap an already-open bus, reading and commanding via `map`.
    pub fn new(bus: B, map: &'static RegisterMap) -> Self {
        log::warn!(
            target: LOG_TARGET,
            "FoxESS driver started with a community register map ({})",
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

    fn write(&mut self, reg: &RegisterDef, value: u16) -> Result<(), Error> {
        with_retries(
            &mut self.bus,
            LOG_TARGET,
            &format!("write {}", reg.name),
            |bus| bus.write_holding(reg.address, value),
        )
    }

    fn command_values(command: Command) -> Result<(u16, u16, f64), Error> {
        let ttl = command
            .ttl()
            .ok_or_else(|| Error::Range(format!("{} requires a non-zero TTL", command.mode)))?;
        if ttl < Duration::from_secs(1) || ttl > MAX_TIMEOUT {
            return Err(Error::Range(format!(
                "FoxESS command TTL must be 1..={} seconds, got {ttl:?}",
                u16::MAX
            )));
        }

        let remote_power_kw = match (command.mode, command.target) {
            (Mode::Hold, _) => 0.0,
            (Mode::ForceCharge, _) => -command.power_kw,
            (Mode::ForceDischarge, DischargeTarget::GridExport) => command.power_kw,
            (Mode::ForceDischarge, DischargeTarget::HouseOnly) => {
                return Err(Error::Unsupported(
                    "FoxESS active-power control cannot guarantee house-only discharge; \
                     use export() when grid export is intended"
                        .into(),
                ));
            }
            (Mode::Passive, _) => {
                return Err(Error::Range("passive has no command values".into()));
            }
        };
        if command.mode != Mode::Hold && command.power_kw <= 0.0 {
            return Err(Error::Range(format!(
                "command power must be positive, got {} kW",
                command.power_kw
            )));
        }

        let raw_power = encode(&registers::remote_control::ACTIVE_POWER, remote_power_kw)?;
        let applied_power_kw = decode(&registers::remote_control::ACTIVE_POWER, &[raw_power]).abs();
        Ok((ttl.as_secs() as u16, raw_power, applied_power_kw))
    }

    fn return_to_passive(&mut self) -> Result<(), Error> {
        use registers::remote_control as remote;

        self.write(&remote::REMOTE_ENABLE, 0)?;
        self.write(&remote::WORK_MODE, 0)
    }

    fn program_command(&mut self, timeout: u16, raw_power: u16) -> Result<(), Error> {
        use registers::remote_control as remote;

        // Cancel the old remote state first. A partially programmed replacement
        // therefore fails passive instead of leaving the previous power active.
        self.return_to_passive()?;
        self.write(&remote::TIMEOUT_SET, timeout)?;
        self.write(&remote::REMOTE_ENABLE, 1)?;
        // FoxESS loads/reloads the hardware countdown on this write.
        self.write(&remote::ACTIVE_POWER, raw_power)
    }

    fn disable_after_error(&mut self, error: Error) -> Error {
        match self.return_to_passive() {
            Ok(()) => error,
            Err(disable_error) => Error::Comm(format!(
                "command failed ({error}); also failed to return FoxESS to passive ({disable_error})"
            )),
        }
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
        let mut caps =
            Capabilities::writable(self.map.model, MODES, Expiry::InverterTimeout(MAX_TIMEOUT));
        caps.reports_solar = !self.map.pv_powers.is_empty();
        caps
    }

    fn read_telemetry(&mut self) -> Result<Telemetry, Error> {
        let map = self.map;
        let soc_pct = self.read(&map.battery_soc)?;
        let battery_kw = self.read(&map.battery_power)?;
        let grid_kw = self.read(&map.grid_power)?;
        // FoxESS reports load signed and it can dip slightly negative from
        // metering noise; the crate convention is load_kw >= 0.
        let load_kw = self.read(&map.load_power)?.max(0.0);
        let mut solar_kw = 0.0;
        for pv in map.pv_powers {
            solar_kw += self.read(pv)?;
        }
        Ok(Telemetry {
            soc_pct,
            battery_kw,
            grid_kw,
            load_kw,
            solar_kw,
            at: SystemTime::now(),
            read_at: Instant::now(),
        })
    }

    fn apply(&mut self, command: Command) -> Result<Applied, Error> {
        if command.mode == Mode::Passive {
            self.return_to_passive()?;
            return Ok(Applied {
                expiry: Expiry::UntilChanged,
                power_kw: 0.0,
            });
        }

        // Validate every value before touching Modbus. Once programming starts,
        // any failure is followed by a best-effort return to passive.
        let (timeout, raw_power, applied_power_kw) = Self::command_values(command)?;
        if let Err(error) = self.program_command(timeout, raw_power) {
            return Err(self.disable_after_error(error));
        }
        Ok(Applied {
            expiry: Expiry::InverterTimeout(Duration::from_secs(timeout.into())),
            power_kw: applied_power_kw,
        })
    }

    fn mode(&mut self) -> Result<Mode, Error> {
        // Remote-enable and active-power read-back varies by connection route;
        // repeating the last command would hide an expiry or app-driven change.
        Err(Error::Unsupported(
            "FoxESS mode read-back is not reliable across supported connection routes".into(),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    /// A bus that replays reads and records function-6 writes.
    struct FakeBus {
        input: HashMap<u16, Vec<u16>>,
        holding: HashMap<u16, Vec<u16>>,
        writes: Vec<(u16, u16)>,
    }

    impl FakeBus {
        fn with_input(pairs: &[(u16, u16)]) -> Self {
            Self {
                input: pairs.iter().map(|&(a, v)| (a, vec![v])).collect(),
                holding: HashMap::new(),
                writes: Vec::new(),
            }
        }

        fn with_holding(pairs: &[(u16, u16)]) -> Self {
            Self {
                input: HashMap::new(),
                holding: pairs.iter().map(|&(a, v)| (a, vec![v])).collect(),
                writes: Vec::new(),
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
        fn write_holding(&mut self, address: u16, value: u16) -> Result<(), Error> {
            self.writes.push((address, value));
            Ok(())
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
    fn g1_normalises_foxess_signs_and_watts_to_crate_conventions() {
        let mut inv = FoxEss::new(g1_fixture(), &registers::H1_G1);
        let t = inv.read_telemetry().unwrap();
        assert_eq!(t.soc_pct, 64.0);
        assert_eq!(t.battery_kw, 1.5, "raw negative watts mean charging");
        assert_eq!(t.grid_kw, 2.0, "raw negative watts mean importing");
        assert_eq!(t.load_kw, 0.5);
        assert_eq!(t.solar_kw, 0.0);
        assert_eq!(t.export_kw(), 0.0, "importing, so nothing is exported");
    }

    #[test]
    fn g2_reads_holding_registers_and_sums_pv_strings() {
        let mut inv = FoxEss::new(g2_fixture(), &registers::H1_G2);
        let t = inv.read_telemetry().unwrap();
        assert_eq!(t.soc_pct, 55.0);
        assert_eq!(t.battery_kw, -0.4, "raw positive means discharging");
        assert_eq!(t.grid_kw, -0.25, "raw positive means exporting");
        assert_eq!(t.export_kw(), 0.25);
        assert_eq!(t.load_kw, 0.75);
        assert_eq!(t.solar_kw, 0.6, "both PV strings summed");
    }

    #[test]
    fn a_slightly_negative_load_reading_clamps_to_zero() {
        let mut bus = g2_fixture();
        bus.holding
            .insert(registers::H1_G2.load_power.address, vec![(-5i16) as u16]);
        let mut inv = FoxEss::new(bus, &registers::H1_G2);
        assert_eq!(inv.read_telemetry().unwrap().load_kw, 0.0);
    }

    #[test]
    fn reports_native_timeout_write_capability() {
        for map in [&registers::H1_G1, &registers::H1_G2] {
            let inv = FoxEss::new(FakeBus::with_input(&[]), map);
            let caps = inv.capabilities();
            assert!(caps.can_write);
            assert_eq!(caps.write_blocked_reason, None);
            assert_eq!(caps.expiry, Expiry::InverterTimeout(MAX_TIMEOUT));
            assert!(caps.expiry.is_dead_controller_safe());
            assert!(caps.reports_solar);
            assert!(!caps.reports_mode, "mode read-back varies by connection");
            for mode in MODES {
                assert!(caps.supports(*mode), "{mode:?} must be advertised");
            }
        }
    }

    #[test]
    fn mode_read_back_is_honestly_unsupported() {
        let mut inv = FoxEss::new(g2_fixture(), &registers::H1_G2);
        assert!(matches!(inv.mode(), Err(Error::Unsupported(_))));
    }

    #[test]
    fn a_charge_uses_the_native_watchdog_and_foxess_power_sign() {
        let mut inv = FoxEss::new(g2_fixture(), &registers::H1_G2);
        let ttl = Duration::from_secs(60);

        let applied = inv.apply(Command::charge(2.0, ttl)).unwrap();

        assert_eq!(applied.expiry, Expiry::InverterTimeout(ttl));
        assert_eq!(applied.power_kw, 2.0);
        assert_eq!(
            inv.bus.writes,
            [
                (registers::remote_control::REMOTE_ENABLE.address, 0),
                (registers::remote_control::WORK_MODE.address, 0),
                (registers::remote_control::TIMEOUT_SET.address, 60),
                (registers::remote_control::REMOTE_ENABLE.address, 1),
                (
                    registers::remote_control::ACTIVE_POWER.address,
                    (-2000i16) as u16
                ),
            ]
        );
    }

    #[test]
    fn a_new_command_cancels_and_replaces_the_native_watchdog() {
        let mut inv = FoxEss::new(g2_fixture(), &registers::H1_G2);
        inv.apply(Command::charge(2.0, Duration::from_secs(60)))
            .unwrap();
        inv.bus.writes.clear();

        let applied = inv
            .apply(Command::export(3.0, Duration::from_secs(15)))
            .unwrap();

        assert_eq!(
            applied.expiry,
            Expiry::InverterTimeout(Duration::from_secs(15))
        );
        assert_eq!(
            inv.bus.writes,
            [
                (registers::remote_control::REMOTE_ENABLE.address, 0),
                (registers::remote_control::WORK_MODE.address, 0),
                (registers::remote_control::TIMEOUT_SET.address, 15),
                (registers::remote_control::REMOTE_ENABLE.address, 1),
                (registers::remote_control::ACTIVE_POWER.address, 3000),
            ]
        );
    }

    #[test]
    fn hold_uses_zero_active_power_with_the_native_watchdog() {
        let mut inv = FoxEss::new(g2_fixture(), &registers::H1_G2);
        let ttl = Duration::from_secs(90);

        let applied = inv.apply(Command::hold(ttl)).unwrap();

        assert_eq!(applied.expiry, Expiry::InverterTimeout(ttl));
        assert_eq!(applied.power_kw, 0.0);
        assert_eq!(
            inv.bus.writes,
            [
                (registers::remote_control::REMOTE_ENABLE.address, 0),
                (registers::remote_control::WORK_MODE.address, 0),
                (registers::remote_control::TIMEOUT_SET.address, 90),
                (registers::remote_control::REMOTE_ENABLE.address, 1),
                (registers::remote_control::ACTIVE_POWER.address, 0),
            ]
        );
    }

    #[test]
    fn passive_disables_remote_control_without_arming_another_timeout() {
        let mut inv = FoxEss::new(g2_fixture(), &registers::H1_G2);

        let applied = inv.apply(Command::passive()).unwrap();

        assert_eq!(applied.expiry, Expiry::UntilChanged);
        assert_eq!(applied.power_kw, 0.0);
        assert_eq!(
            inv.bus.writes,
            [
                (registers::remote_control::REMOTE_ENABLE.address, 0),
                (registers::remote_control::WORK_MODE.address, 0),
            ]
        );
    }

    #[test]
    fn invalid_commands_are_rejected_before_modbus_is_touched() {
        let commands = [
            Command::charge(2.0, Duration::from_millis(999)),
            Command::charge(2.0, MAX_TIMEOUT + Duration::from_secs(1)),
            Command::charge(0.0, Duration::from_secs(60)),
            Command::discharge(2.0, Duration::from_secs(60)),
        ];

        for command in commands {
            let mut inv = FoxEss::new(g2_fixture(), &registers::H1_G2);
            assert!(inv.apply(command).is_err());
            assert!(inv.bus.writes.is_empty());
        }
    }

    #[test]
    fn fractional_ttls_round_down_so_the_hardware_never_outlives_the_command() {
        let mut inv = FoxEss::new(g2_fixture(), &registers::H1_G2);
        let applied = inv
            .apply(Command::export(1.0, Duration::from_millis(1_999)))
            .unwrap();

        assert_eq!(
            applied.expiry,
            Expiry::InverterTimeout(Duration::from_secs(1))
        );
        assert_eq!(
            inv.bus.writes[2],
            (registers::remote_control::TIMEOUT_SET.address, 1)
        );
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
                map.battery_power.scale, -0.001,
                "raw battery watts are discharge-positive; scaled to charge-positive kW"
            );
            assert_eq!(
                map.grid_power.scale, -0.001,
                "raw grid watts are export-positive; scaled to import-positive kW"
            );
            assert_eq!(map.load_power.scale, 0.001);
            for pv in map.pv_powers {
                assert_eq!(pv.scale, 0.001);
            }
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
    fn remote_control_registers_pin_the_h1_function_6_addresses() {
        assert_eq!(registers::remote_control::TIMEOUT_SET.address, 44001);
        assert_eq!(registers::remote_control::REMOTE_ENABLE.address, 44000);
        assert_eq!(registers::remote_control::ACTIVE_POWER.address, 44002);
    }
}
