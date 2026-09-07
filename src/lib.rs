//! Control hybrid solar/battery inverters over Modbus.
//!
//! Read telemetry, check driver capabilities and apply commands with explicit
//! timeouts. [`Applied`] reports the accepted power and [`Expiry`].
//!
//! # Sign conventions
//!
//! All drivers use these signs:
//!
//! * `battery_kw > 0` — charging (power into the cells)
//! * `grid_kw > 0` — importing; `< 0` — exporting
//! * `load_kw >= 0` — household consumption
//! * `solar_kw >= 0` — PV generation, `0.0` when the model cannot report it
//!
//! Powers are kilowatts and energies are kilowatt-hours. Power arguments accept
//! numeric types convertible to `f64`.
//!
//! # Example
//!
//! ```
//! # #[cfg(feature = "mock")]
//! # fn main() -> Result<(), inverter::Error> {
//! use inverter::{Inverter, InverterExt, Mode, mock::MockInverter};
//! use std::time::Duration;
//!
//! let mut inv = MockInverter::new();
//! let caps = inv.capabilities();
//! assert!(caps.can_write);
//!
//! let telemetry = inv.read_telemetry()?;
//! println!("battery at {}%", telemetry.soc_pct);
//!
//! // Single-value reads:
//! let soc = inv.get_soc_pct()?;
//! assert_eq!(inv.get_mode()?, Mode::Passive);
//!
//! if caps.supports(Mode::ForceCharge) {
//!     let applied = inv.charge(2, Duration::from_secs(60))?;
//!     println!("expires: {:?}", applied.expiry);
//! }
//! # Ok(())
//! # }
//! # #[cfg(not(feature = "mock"))]
//! # fn main() {}
//! ```

#![forbid(unsafe_code)]
#![warn(missing_docs)]
// Feature banners on docs.rs.
#![cfg_attr(docsrs, feature(doc_cfg))]

use std::time::{Duration, Instant, SystemTime};

mod factory;

pub use factory::{open, MockOptions, OpenOptions};

pub mod register;

pub mod modbus;

#[cfg(feature = "foxess")]
pub mod foxess;

#[cfg(feature = "mock")]
pub mod mock;

/// Inverter communication, validation and capability errors.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    /// The transport failed, or the inverter's reply was unusable.
    #[error("communication error: {0}")]
    Comm(String),

    /// A write succeeded but reading the register back returned another value.
    #[error("read-back mismatch: {0}")]
    Readback(String),

    /// A value could not be represented in the target register.
    #[error("value out of range: {0}")]
    Range(String),

    /// The driver does not implement this operation for this model.
    #[error("unsupported: {0}")]
    Unsupported(String),
}

/// Inverter operating mode. Overrides require a timeout and return to passive.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum Mode {
    /// Self-use: solar supplies the house, surplus charges the battery, and
    /// the battery covers demand down to its configured minimum SoC.
    Passive,
    /// Keep battery power at zero until the command expires.
    Hold,
    /// Charge the battery, importing from the grid as needed.
    ForceCharge,
    /// Discharge towards the command's [`DischargeTarget`].
    ForceDischarge,
}

impl Mode {
    /// Stable lowercase identifier, for logs and configuration.
    ///
    /// [`Display`](std::fmt::Display) prints the same identifier; parse it
    /// back with [`str::parse`].
    #[must_use]
    pub fn as_str(&self) -> &'static str {
        match self {
            Mode::Passive => "passive",
            Mode::Hold => "hold",
            Mode::ForceCharge => "force_charge",
            Mode::ForceDischarge => "force_discharge",
        }
    }
}

impl std::fmt::Display for Mode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// The error from parsing a string that names no [`Mode`].
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error(
    r#"unrecognised mode {0:?}: expected "passive", "hold", "force_charge" or "force_discharge""#
)]
pub struct ParseModeError(String);

impl std::str::FromStr for Mode {
    type Err = ParseModeError;

    /// Parse the identifier produced by [`Mode::as_str`].
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "passive" => Ok(Mode::Passive),
            "hold" => Ok(Mode::Hold),
            "force_charge" => Ok(Mode::ForceCharge),
            "force_discharge" => Ok(Mode::ForceDischarge),
            _ => Err(ParseModeError(s.to_string())),
        }
    }
}

/// Discharge target. Support depends on the driver.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum DischargeTarget {
    /// Cover household load only; do not push power out to the grid.
    HouseOnly,
    /// Allow discharge to export to the grid.
    GridExport,
}

/// A single instruction to the inverter.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Command {
    /// What the inverter should do.
    pub mode: Mode,
    /// Requested power in kilowatts. Ignored for [`Mode::Passive`] and
    /// [`Mode::Hold`].
    pub power_kw: f64,
    /// Where discharged energy should go. Ignored unless discharging.
    pub target: DischargeTarget,
    /// `None` for passive; always `Some` for an override.
    ttl: Option<Duration>,
}

impl Command {
    /// Return to the inverter's own self-use behaviour ([`Mode::Passive`]).
    #[must_use]
    pub fn passive() -> Self {
        Command {
            mode: Mode::Passive,
            power_kw: 0.0,
            target: DischargeTarget::HouseOnly,
            ttl: None,
        }
    }

    /// Keep battery power at zero for at most `ttl`, then return to passive.
    #[must_use]
    pub fn hold(ttl: Duration) -> Self {
        Command {
            mode: Mode::Hold,
            power_kw: 0.0,
            target: DischargeTarget::HouseOnly,
            ttl: Some(ttl),
        }
    }

    /// Charge at `power_kw`, importing if necessary, for at most `ttl`.
    #[must_use]
    pub fn charge(power_kw: impl Into<f64>, ttl: Duration) -> Self {
        Command {
            mode: Mode::ForceCharge,
            power_kw: power_kw.into(),
            target: DischargeTarget::HouseOnly,
            ttl: Some(ttl),
        }
    }

    /// Discharge at `power_kw` to cover household load for at most `ttl`,
    /// without exporting.
    #[must_use]
    pub fn discharge(power_kw: impl Into<f64>, ttl: Duration) -> Self {
        Command {
            mode: Mode::ForceDischarge,
            power_kw: power_kw.into(),
            target: DischargeTarget::HouseOnly,
            ttl: Some(ttl),
        }
    }

    /// Discharge at `power_kw` for at most `ttl`, deliberately exporting to
    /// the grid.
    #[must_use]
    pub fn export(power_kw: impl Into<f64>, ttl: Duration) -> Self {
        Command {
            mode: Mode::ForceDischarge,
            power_kw: power_kw.into(),
            target: DischargeTarget::GridExport,
            ttl: Some(ttl),
        }
    }

    /// The override's time to live, or `None` for [`Mode::Passive`].
    #[must_use]
    pub fn ttl(&self) -> Option<Duration> {
        self.ttl
    }
}

impl std::fmt::Display for Command {
    /// Log-friendly form: `passive`, `force_charge@2kW`,
    /// `force_discharge@3kW(grid-export)`.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match (self.mode, self.target) {
            (Mode::Passive, _) => f.write_str("passive"),
            (Mode::Hold, _) => f.write_str("hold"),
            (Mode::ForceDischarge, DischargeTarget::GridExport) => {
                write!(f, "force_discharge@{}kW(grid-export)", self.power_kw)
            }
            _ => write!(f, "{}@{}kW", self.mode, self.power_kw),
        }
    }
}

/// How a command ends.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum Expiry {
    /// One-shot inverter timeout. Returns to passive without controller action.
    InverterTimeout(Duration),

    /// Reverts on an inverter condition, such as target SoC; not time-bounded.
    InverterCondition(&'static str),

    /// Repeating schedule that remains active without a controller.
    RecurringWindow,

    /// Remains active until overwritten.
    UntilChanged,
}

impl Expiry {
    /// Whether this is a non-zero, one-shot inverter timeout.
    #[must_use]
    pub fn is_dead_controller_safe(&self) -> bool {
        matches!(self, Expiry::InverterTimeout(timeout) if !timeout.is_zero())
    }
}

/// Driver support for the connected model and transport.
///
/// Construct with [`Capabilities::read_only`] or [`Capabilities::writable`],
/// then set the public reporting fields.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[non_exhaustive]
pub struct Capabilities {
    /// Human-readable model or map identifier, for logs and diagnostics.
    pub model: &'static str,
    /// Whether this driver writes to the inverter at all.
    pub can_write: bool,
    /// Modes this driver can command. Always contains [`Mode::Passive`] when
    /// `can_write` is true.
    pub modes: &'static [Mode],
    /// How commands issued by this driver end.
    ///
    /// For [`Expiry::InverterTimeout`], the duration is the largest timeout
    /// the driver can accept. [`Applied::expiry`] reports the exact timeout
    /// armed for an individual command.
    pub expiry: Expiry,
    /// Whether the driver can report PV generation.
    pub reports_solar: bool,
    /// Whether [`Inverter::mode`] can answer, rather than returning
    /// [`Error::Unsupported`].
    pub reports_mode: bool,
    /// Why writes are unavailable, when `can_write` is false.
    pub write_blocked_reason: Option<&'static str>,
}

impl Capabilities {
    /// A driver that reports telemetry but refuses every command.
    ///
    /// Reporting flags default to `false`. Set them to match the driver:
    ///
    /// ```
    /// use inverter::Capabilities;
    ///
    /// let mut caps = Capabilities::read_only("Acme X1 (RS485)", "map unverified on hardware");
    /// caps.reports_solar = true;
    /// assert!(!caps.can_write);
    /// ```
    #[must_use]
    pub fn read_only(model: &'static str, reason: &'static str) -> Self {
        Capabilities {
            model,
            can_write: false,
            modes: &[],
            // Nothing can be commanded, so nothing this driver does expires.
            expiry: Expiry::UntilChanged,
            reports_solar: false,
            reports_mode: false,
            write_blocked_reason: Some(reason),
        }
    }

    /// A driver that can command `modes`, each ending the way `expiry` says.
    ///
    /// The reporting flags start `false`; set the public fields for whatever
    /// the driver can do.
    ///
    /// # Panics
    ///
    /// Panics unless `modes` contains [`Mode::Passive`].
    #[must_use]
    pub fn writable(model: &'static str, modes: &'static [Mode], expiry: Expiry) -> Self {
        assert!(
            modes.contains(&Mode::Passive),
            "a writable driver must support Mode::Passive"
        );
        Capabilities {
            model,
            can_write: true,
            modes,
            expiry,
            reports_solar: false,
            reports_mode: false,
            write_blocked_reason: None,
        }
    }

    /// Whether `mode` can be commanded.
    #[must_use]
    pub fn supports(&self, mode: Mode) -> bool {
        self.can_write && self.modes.contains(&mode)
    }
}

/// A reading from the inverter.
///
/// See the [crate] docs for sign conventions.
#[derive(Clone, Copy, Debug)]
pub struct Telemetry {
    /// Battery state of charge, percent.
    pub soc_pct: f64,
    /// Battery power, kilowatts. Positive means charging.
    pub battery_kw: f64,
    /// Grid power, kilowatts. Positive means importing.
    pub grid_kw: f64,
    /// Household consumption, kilowatts.
    pub load_kw: f64,
    /// PV generation, kilowatts. `0.0` when the model cannot report it.
    pub solar_kw: f64,
    /// Wall-clock time of the reading, for display and storage.
    pub at: SystemTime,
    /// Monotonic start time of the reading, used for staleness checks.
    pub read_at: Instant,
}

impl Telemetry {
    /// Power flowing out to the grid, kilowatts. Zero while importing.
    #[must_use]
    pub fn export_kw(&self) -> f64 {
        (-self.grid_kw).max(0.0)
    }

    /// How long ago this reading was taken.
    #[must_use]
    pub fn age(&self) -> Duration {
        self.read_at.elapsed()
    }
}

/// Accepted power setpoint and expiry.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Applied {
    /// How this command will actually end.
    ///
    /// For a non-passive command this must be a non-zero
    /// [`Expiry::InverterTimeout`] no longer than [`Command::ttl`]. A driver
    /// that cannot provide that guarantee must refuse the command.
    pub expiry: Expiry,
    /// Power the driver actually commanded, kilowatts, after any
    /// model-specific clamping.
    pub power_kw: f64,
}

/// Inverter driver. Implementations must use the crate's sign conventions
/// and return errors for failed reads and writes.
pub trait Inverter: Send {
    /// What this driver can do with the connected hardware.
    ///
    /// Cheap and side-effect free; callers may call it on every tick.
    fn capabilities(&self) -> Capabilities;

    /// Read the current state of the system.
    fn read_telemetry(&mut self) -> Result<Telemetry, Error>;

    /// Command the inverter.
    ///
    /// Returns [`Error::Unsupported`] when [`Capabilities`] says the mode is
    /// unavailable. Implementors should verify writes by reading them back.
    /// A non-passive command must replace any previous inverter-side watchdog
    /// with a one-shot timeout no longer than [`Command::ttl`]; passive must
    /// cancel that watchdog. A driver that cannot do this must refuse the
    /// non-passive command without changing inverter state.
    fn apply(&mut self, command: Command) -> Result<Applied, Error>;

    /// Read the current mode. Returns [`Error::Unsupported`] when the driver
    /// cannot read it back; see [`Capabilities::reports_mode`].
    fn mode(&mut self) -> Result<Mode, Error>;

    /// Release the transport. Called once, on shutdown.
    fn close(&mut self) {}
}

/// Command and single-value read methods for every [`Inverter`].
///
/// Command methods call [`Inverter::apply`]. Each telemetry getter performs
/// a full [`Inverter::read_telemetry`]; read once when several values are needed.
/// [`InverterExt::get_mode`] calls [`Inverter::mode`].
pub trait InverterExt: Inverter {
    /// Return to the inverter's own self-use behaviour ([`Mode::Passive`]).
    fn passive(&mut self) -> Result<Applied, Error> {
        self.apply(Command::passive())
    }

    /// Keep battery power at zero for at most `ttl`, then return to passive.
    fn hold(&mut self, ttl: Duration) -> Result<Applied, Error> {
        self.apply(Command::hold(ttl))
    }

    /// Charge at `power_kw`, importing if necessary, for at most `ttl`.
    fn charge(&mut self, power_kw: impl Into<f64>, ttl: Duration) -> Result<Applied, Error> {
        self.apply(Command::charge(power_kw, ttl))
    }

    /// Discharge at `power_kw` to cover household load for at most `ttl`,
    /// without exporting.
    fn discharge(&mut self, power_kw: impl Into<f64>, ttl: Duration) -> Result<Applied, Error> {
        self.apply(Command::discharge(power_kw, ttl))
    }

    /// Discharge at `power_kw` for at most `ttl`, deliberately exporting to
    /// the grid.
    fn export(&mut self, power_kw: impl Into<f64>, ttl: Duration) -> Result<Applied, Error> {
        self.apply(Command::export(power_kw, ttl))
    }

    /// Battery state of charge, percent. Performs a full telemetry read.
    fn get_soc_pct(&mut self) -> Result<f64, Error> {
        Ok(self.read_telemetry()?.soc_pct)
    }

    /// Battery power, kilowatts; positive means charging. Performs a full
    /// telemetry read.
    fn get_battery_kw(&mut self) -> Result<f64, Error> {
        Ok(self.read_telemetry()?.battery_kw)
    }

    /// Grid power, kilowatts; positive means importing. Performs a full
    /// telemetry read.
    fn get_grid_kw(&mut self) -> Result<f64, Error> {
        Ok(self.read_telemetry()?.grid_kw)
    }

    /// Household consumption, kilowatts. Performs a full telemetry read.
    fn get_load_kw(&mut self) -> Result<f64, Error> {
        Ok(self.read_telemetry()?.load_kw)
    }

    /// PV generation, kilowatts. Performs a full telemetry read.
    fn get_solar_kw(&mut self) -> Result<f64, Error> {
        Ok(self.read_telemetry()?.solar_kw)
    }

    /// Grid export, kilowatts; zero while importing. Performs a full
    /// telemetry read.
    fn get_export_kw(&mut self) -> Result<f64, Error> {
        Ok(self.read_telemetry()?.export_kw())
    }

    /// Read the current mode; see [`Inverter::mode`].
    fn get_mode(&mut self) -> Result<Mode, Error> {
        self.mode()
    }
}

impl<I: Inverter + ?Sized> InverterExt for I {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_round_trips_through_its_identifier() {
        for mode in [
            Mode::Passive,
            Mode::Hold,
            Mode::ForceCharge,
            Mode::ForceDischarge,
        ] {
            assert_eq!(mode.as_str().parse(), Ok(mode));
            assert_eq!(mode.to_string(), mode.as_str(), "Display matches as_str");
        }
        let err = "nonsense".parse::<Mode>().unwrap_err();
        assert!(err.to_string().contains("nonsense"), "{err}");
    }

    #[test]
    fn only_a_one_shot_inverter_timeout_survives_a_dead_controller() {
        assert!(Expiry::InverterTimeout(Duration::from_secs(60)).is_dead_controller_safe());
        assert!(!Expiry::InverterTimeout(Duration::ZERO).is_dead_controller_safe());
        assert!(!Expiry::InverterCondition("target soc").is_dead_controller_safe());
        assert!(!Expiry::RecurringWindow.is_dead_controller_safe());
        assert!(!Expiry::UntilChanged.is_dead_controller_safe());
    }

    #[test]
    fn export_is_distinguishable_from_house_only_discharge() {
        let ttl = Duration::from_secs(60);
        assert_eq!(
            Command::discharge(1.5, ttl).target,
            DischargeTarget::HouseOnly
        );
        assert_eq!(Command::export(3, ttl).target, DischargeTarget::GridExport);
        assert!(Command::export(3, ttl).to_string().contains("grid-export"));
    }

    #[test]
    fn display_names_the_mode_power_and_export_intent() {
        let ttl = Duration::from_secs(60);
        assert_eq!(Command::passive().to_string(), "passive");
        assert_eq!(Command::hold(ttl).to_string(), "hold");
        assert_eq!(Command::charge(2, ttl).to_string(), "force_charge@2kW");
        assert_eq!(
            Command::discharge(1.5, ttl).to_string(),
            "force_discharge@1.5kW"
        );
        assert_eq!(
            Command::export(3, ttl).to_string(),
            "force_discharge@3kW(grid-export)"
        );
    }

    #[test]
    fn non_passive_commands_have_explicit_ttls_but_passive_does_not() {
        let ttl = Duration::from_secs(60);
        assert_eq!(Command::hold(ttl).ttl(), Some(ttl));
        assert_eq!(Command::charge(1.0, ttl).ttl(), Some(ttl));
        assert_eq!(Command::discharge(1.0, ttl).ttl(), Some(ttl));
        assert_eq!(Command::export(1.0, ttl).ttl(), Some(ttl));
        assert_eq!(Command::passive().ttl(), None);
    }

    #[test]
    fn export_kw_is_the_positive_part_of_negative_grid_flow() {
        let mut t = Telemetry {
            soc_pct: 50.0,
            battery_kw: 0.0,
            grid_kw: -0.3,
            load_kw: 0.0,
            solar_kw: 0.0,
            at: SystemTime::now(),
            read_at: Instant::now(),
        };
        assert_eq!(t.export_kw(), 0.3);
        t.grid_kw = 0.2;
        assert_eq!(t.export_kw(), 0.0);
    }

    #[test]
    fn integer_and_float_powers_build_the_same_command() {
        let ttl = Duration::from_secs(60);
        assert_eq!(Command::charge(2, ttl), Command::charge(2.0, ttl));
        assert_eq!(Command::discharge(1, ttl), Command::discharge(1.0, ttl));
        assert_eq!(Command::export(3, ttl), Command::export(3.0, ttl));
    }

    #[test]
    fn a_writable_driver_supports_only_its_listed_modes() {
        let caps = Capabilities::writable(
            "test",
            &[Mode::Passive, Mode::ForceCharge],
            Expiry::UntilChanged,
        );
        assert!(caps.supports(Mode::Passive));
        assert!(caps.supports(Mode::ForceCharge));
        assert!(!caps.supports(Mode::ForceDischarge));
        assert_eq!(caps.write_blocked_reason, None);
    }

    #[test]
    #[should_panic(expected = "must support Mode::Passive")]
    fn a_writable_driver_without_passive_is_rejected_outright() {
        let _ = Capabilities::writable("test", &[Mode::ForceCharge], Expiry::UntilChanged);
    }

    #[test]
    fn capabilities_refuse_every_mode_when_the_driver_cannot_write() {
        let caps = Capabilities {
            model: "test",
            can_write: false,
            // Listed modes must not leak through while writes are off.
            modes: &[Mode::Passive, Mode::ForceCharge],
            expiry: Expiry::UntilChanged,
            reports_solar: false,
            reports_mode: false,
            write_blocked_reason: Some("unverified map"),
        };
        assert!(!caps.supports(Mode::Passive));
        assert!(!caps.supports(Mode::ForceCharge));
    }

    #[test]
    fn a_read_only_driver_carries_its_reason_and_reports_nothing_extra() {
        let caps = Capabilities::read_only("test", "map unverified");
        assert!(!caps.can_write);
        assert_eq!(caps.write_blocked_reason, Some("map unverified"));
        assert!(!caps.reports_solar && !caps.reports_mode);
        assert!(!caps.supports(Mode::Passive));
    }
}
