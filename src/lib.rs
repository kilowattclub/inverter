//! Control hybrid solar/battery inverters over Modbus.
//!
//! The crate exists to make one distinction impossible to ignore: **how a
//! command ends**. Telling an inverter to charge for twenty minutes and
//! programming a daily window that starts at the same moment look identical
//! at most APIs, and they are not the same thing. The first stops on its own
//! if the controller dies; the second repeats tomorrow, and every day after,
//! with nobody left to cancel it. See [`Expiry`].
//!
//! # Sign conventions
//!
//! Every driver reports telemetry with these signs, whatever the inverter's
//! own convention is:
//!
//! * `battery_kw > 0` — charging (power into the cells)
//! * `grid_kw > 0` — importing; `< 0` — exporting
//! * `load_kw >= 0` — household consumption
//! * `solar_kw >= 0` — PV generation, `0.0` when the model cannot report it
//!
//! All powers are kilowatts and energies kilowatt-hours. Anywhere a power is
//! taken, any numeric type convertible to `f64` is accepted: `charge(2)` and
//! `charge(2.5)` both work.
//!
//! # Example
//!
//! ```
//! # #[cfg(feature = "mock")] {
//! use inverter::{Inverter, InverterExt, Mode, mock::MockInverter};
//!
//! let mut inv = MockInverter::new();
//! let caps = inv.capabilities();
//! assert!(caps.can_write);
//!
//! let telemetry = inv.read_telemetry().unwrap();
//! println!("battery at {}%", telemetry.soc_pct);
//!
//! // Or single values, and the mode currently in force:
//! let soc = inv.soc_pct().unwrap();
//! assert_eq!(inv.mode().unwrap(), Mode::Passive);
//!
//! if caps.supports(Mode::ForceCharge) {
//!     // Sugar for inv.apply(Command::charge(2)). Powers are kilowatts.
//!     let applied = inv.charge(2).unwrap();
//!     // How this command ends is data, not an assumption.
//!     println!("expires: {:?}", applied.expiry);
//! }
//! # }
//! ```

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use std::time::{Duration, Instant, SystemTime};

pub mod register;

pub mod modbus;

#[cfg(feature = "foxess")]
pub mod foxess;

#[cfg(feature = "mock")]
pub mod mock;

/// Everything that can go wrong talking to an inverter.
///
/// Drivers report failures rather than returning plausible-looking data: a
/// zero that came from a dropped frame is far more dangerous than an error.
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
    ///
    /// Prefer checking [`Capabilities`] first — this exists for the case where
    /// a caller commands something the inverter turned out not to accept.
    #[error("unsupported: {0}")]
    Unsupported(String),
}

/// What the inverter should be doing.
///
/// [`Passive`](Mode::Passive) is the inverter's own behaviour; the other two
/// override it. The overrides are what need [`Expiry`] semantics — passive
/// has no power level and nothing to expire, which is what makes it the safe
/// fallback.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum Mode {
    /// The controller steps out of the way: the inverter runs its own
    /// self-use logic, exactly as it would with no controller attached.
    ///
    /// Concretely: solar powers the house; surplus charges the battery, then
    /// exports once the battery is full; after dark the battery covers the
    /// house down to the inverter's configured minimum state of charge, then
    /// the grid takes over. "Passive" describes the *controller's* stance —
    /// the hardware is busy. Vendors call this "self-use",
    /// "self-consumption" or "general" mode.
    ///
    /// This is the state every writable driver must be able to return to,
    /// the state a caller should fall back to when unsure, and the state a
    /// dead controller's hardware should decay to.
    Passive,
    /// Force energy into the battery now, importing from the grid when solar
    /// cannot cover the requested power.
    ///
    /// Overrides the self-use economics — this is how a controller buys a
    /// cheap tariff window.
    ForceCharge,
    /// Force energy out of the battery now.
    ///
    /// Where the energy goes — household load only, or deliberately out past
    /// the meter — is the command's [`DischargeTarget`].
    ForceDischarge,
}

impl Mode {
    /// Stable lowercase identifier, for logs and configuration.
    pub fn as_str(&self) -> &'static str {
        match self {
            Mode::Passive => "passive",
            Mode::ForceCharge => "force_charge",
            Mode::ForceDischarge => "force_discharge",
        }
    }

    /// Parse the identifier produced by [`Mode::as_str`].
    #[allow(clippy::should_implement_trait)]
    pub fn from_str(s: &str) -> Option<Mode> {
        match s {
            "passive" => Some(Mode::Passive),
            "force_charge" => Some(Mode::ForceCharge),
            "force_discharge" => Some(Mode::ForceDischarge),
            _ => None,
        }
    }
}

/// Where discharged energy is meant to go.
///
/// This is a target, not a permission: the caller decides policy. It is part
/// of the command because some inverters reach the two behaviours through
/// different work modes rather than through a power limit.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DischargeTarget {
    /// Cover household load only; do not push power out to the grid.
    HouseOnly,
    /// Deliberately export to the grid, for a grid-services event.
    GridExport,
}

/// A single instruction to the inverter.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Command {
    /// What the inverter should do.
    pub mode: Mode,
    /// Requested power in kilowatts. Ignored for [`Mode::Passive`].
    pub power_kw: f64,
    /// Where discharged energy should go. Ignored unless discharging.
    pub target: DischargeTarget,
    /// How long the caller wants the command to last.
    ///
    /// This is a *request*. What the inverter actually commits to comes back
    /// in [`Applied::expiry`], and it may be weaker than what you asked for.
    pub hold: Duration,
}

/// Default hold for the convenience constructors: long enough to survive a
/// missed control tick, short enough that a dead controller stops mattering.
pub const DEFAULT_HOLD: Duration = Duration::from_secs(300);

impl Command {
    /// Return to the inverter's own self-use behaviour ([`Mode::Passive`]).
    pub fn passive() -> Self {
        Command {
            mode: Mode::Passive,
            power_kw: 0.0,
            target: DischargeTarget::HouseOnly,
            hold: DEFAULT_HOLD,
        }
    }

    /// Charge at `power_kw`, importing if necessary.
    pub fn charge(power_kw: impl Into<f64>) -> Self {
        Command {
            mode: Mode::ForceCharge,
            power_kw: power_kw.into(),
            target: DischargeTarget::HouseOnly,
            hold: DEFAULT_HOLD,
        }
    }

    /// Discharge at `power_kw` to cover household load, without exporting.
    pub fn discharge(power_kw: impl Into<f64>) -> Self {
        Command {
            mode: Mode::ForceDischarge,
            power_kw: power_kw.into(),
            target: DischargeTarget::HouseOnly,
            hold: DEFAULT_HOLD,
        }
    }

    /// Discharge at `power_kw`, deliberately exporting to the grid.
    pub fn export(power_kw: impl Into<f64>) -> Self {
        Command {
            mode: Mode::ForceDischarge,
            power_kw: power_kw.into(),
            target: DischargeTarget::GridExport,
            hold: DEFAULT_HOLD,
        }
    }

    /// Ask the inverter to hold this command for `hold` rather than the default.
    pub fn holding_for(mut self, hold: Duration) -> Self {
        self.hold = hold;
        self
    }

    /// Human-readable form for logs.
    pub fn describe(&self) -> String {
        match (self.mode, self.target) {
            (Mode::Passive, _) => "passive".to_string(),
            (Mode::ForceDischarge, DischargeTarget::GridExport) => {
                format!("force_discharge@{}kW(grid-export)", self.power_kw)
            }
            _ => format!("{}@{}kW", self.mode.as_str(), self.power_kw),
        }
    }
}

/// How a non-passive command stops.
///
/// The reason this crate exists. A caller that treats
/// [`Expiry::RecurringWindow`] as though it were
/// [`Expiry::InverterTimeout`] has built a system that keeps forcing a
/// battery after the controller is gone — every day, at the same time, until
/// someone notices.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Expiry {
    /// The inverter reverts by itself after this long, once, without repeating.
    ///
    /// The only variant that is a true fail-safe against a dead controller.
    InverterTimeout(Duration),

    /// The inverter reverts when a condition it evaluates is met — a target
    /// state of charge, for instance.
    ///
    /// Bounded, but not bounded in *time*: a battery that never reaches the
    /// threshold never reverts.
    InverterCondition(&'static str),

    /// A schedule that repeats on the inverter's own clock.
    ///
    /// **Not a fail-safe.** It outlives the controller and fires again
    /// tomorrow. Anything relying on it must have another way to revert.
    RecurringWindow,

    /// Applies until something changes it.
    ///
    /// **Not a fail-safe.** If the controller stops, the command stands.
    UntilChanged,
}

impl Expiry {
    /// Whether a dead controller leaves the inverter safely reverting on its own.
    ///
    /// Callers that can only tolerate a genuine dead-man's handle should refuse
    /// to issue non-passive commands when this is `false`.
    pub fn is_dead_controller_safe(&self) -> bool {
        matches!(self, Expiry::InverterTimeout(_))
    }
}

/// What a driver can actually do with the connected hardware.
///
/// Ask before you command. Feature support varies by model *and* by how the
/// inverter is connected — the same unit over RS485 and over its own network
/// module does not expose the same registers.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct Capabilities {
    /// Human-readable model or map identifier, for logs and diagnostics.
    pub model: &'static str,
    /// Whether this driver writes to the inverter at all.
    ///
    /// A read-only driver still reports telemetry; it just refuses commands.
    pub can_write: bool,
    /// Modes this driver can command. Always contains [`Mode::Passive`] when
    /// `can_write` is true.
    pub modes: &'static [Mode],
    /// How commands issued by this driver end.
    pub expiry: Expiry,
    /// Whether the driver can report PV generation.
    pub reports_solar: bool,
    /// Why writes are unavailable, when `can_write` is false.
    pub write_blocked_reason: Option<&'static str>,
}

impl Capabilities {
    /// Whether `mode` can be commanded.
    pub fn supports(&self, mode: Mode) -> bool {
        self.can_write && self.modes.contains(&mode)
    }
}

/// A reading from the inverter.
///
/// See the [crate] docs for sign conventions.
#[derive(Clone, Debug)]
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
    /// Monotonic time of the reading.
    ///
    /// Use this for staleness checks: unlike [`Telemetry::at`] it cannot be
    /// dragged backwards by an NTP step or a daylight-saving change.
    pub read_at: Instant,
}

impl Telemetry {
    /// Power flowing out to the grid, kilowatts. Zero while importing.
    pub fn export_kw(&self) -> f64 {
        (-self.grid_kw).max(0.0)
    }

    /// How long ago this reading was taken.
    pub fn age(&self) -> Duration {
        self.read_at.elapsed()
    }
}

/// What the inverter accepted, which may be less than what was asked for.
#[derive(Clone, Debug)]
pub struct Applied {
    /// How this command will actually end.
    ///
    /// Compare against what the caller needs. A driver is allowed to return a
    /// weaker guarantee than requested; silently assuming otherwise is the
    /// mistake this type exists to prevent.
    pub expiry: Expiry,
    /// Power the driver actually commanded, kilowatts, after any
    /// model-specific clamping.
    pub power_kw: f64,
}

/// An inverter this crate can talk to.
///
/// Implementors must report failures rather than returning plausible data,
/// and must honour the crate's sign conventions.
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
    fn apply(&mut self, command: Command) -> Result<Applied, Error>;

    /// The [`Mode`] currently in force, as far as this driver can know it.
    ///
    /// Drivers must not guess. A driver that cannot read the imposed state
    /// back from the hardware returns [`Error::Unsupported`] rather than
    /// repeating what it last commanded — a stale belief is exactly the
    /// mistake that hides an expired or externally-changed command.
    fn mode(&mut self) -> Result<Mode, Error>;

    /// Release the transport. Called once, on shutdown.
    fn close(&mut self) {}
}

/// Partial applications of the [`Inverter`] operations.
///
/// Sugar only, in two groups. The command methods each build the matching
/// [`Command`] with [`DEFAULT_HOLD`] and call [`Inverter::apply`]; for a
/// non-default hold, build the [`Command`] and call `apply` directly. The
/// telemetry methods each perform a **full** [`Inverter::read_telemetry`]
/// and return one field — convenient for a one-off check, wasteful in a
/// loop; when you need several values, read once and use the fields.
///
/// The blanket implementation is the only one the coherence rules allow, so
/// no driver can override these — every spelling reaches hardware through
/// `apply` and `read_telemetry`.
pub trait InverterExt: Inverter {
    /// Return to the inverter's own self-use behaviour ([`Mode::Passive`]).
    fn passive(&mut self) -> Result<Applied, Error> {
        self.apply(Command::passive())
    }

    /// Charge at `power_kw`, importing if necessary.
    fn charge(&mut self, power_kw: impl Into<f64>) -> Result<Applied, Error> {
        self.apply(Command::charge(power_kw))
    }

    /// Discharge at `power_kw` to cover household load, without exporting.
    fn discharge(&mut self, power_kw: impl Into<f64>) -> Result<Applied, Error> {
        self.apply(Command::discharge(power_kw))
    }

    /// Discharge at `power_kw`, deliberately exporting to the grid.
    fn export(&mut self, power_kw: impl Into<f64>) -> Result<Applied, Error> {
        self.apply(Command::export(power_kw))
    }

    /// Battery state of charge, percent. Performs a full telemetry read.
    fn soc_pct(&mut self) -> Result<f64, Error> {
        Ok(self.read_telemetry()?.soc_pct)
    }

    /// Battery power, kilowatts; positive means charging. Performs a full
    /// telemetry read.
    fn battery_kw(&mut self) -> Result<f64, Error> {
        Ok(self.read_telemetry()?.battery_kw)
    }

    /// Grid power, kilowatts; positive means importing. Performs a full
    /// telemetry read.
    fn grid_kw(&mut self) -> Result<f64, Error> {
        Ok(self.read_telemetry()?.grid_kw)
    }

    /// Household consumption, kilowatts. Performs a full telemetry read.
    fn load_kw(&mut self) -> Result<f64, Error> {
        Ok(self.read_telemetry()?.load_kw)
    }

    /// PV generation, kilowatts. Performs a full telemetry read.
    fn solar_kw(&mut self) -> Result<f64, Error> {
        Ok(self.read_telemetry()?.solar_kw)
    }

    /// Grid export, kilowatts; zero while importing. Performs a full
    /// telemetry read.
    fn export_kw(&mut self) -> Result<f64, Error> {
        Ok(self.read_telemetry()?.export_kw())
    }
}

impl<I: Inverter + ?Sized> InverterExt for I {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_round_trips_through_its_identifier() {
        for mode in [Mode::Passive, Mode::ForceCharge, Mode::ForceDischarge] {
            assert_eq!(Mode::from_str(mode.as_str()), Some(mode));
        }
        assert_eq!(Mode::from_str("nonsense"), None);
    }

    #[test]
    fn only_a_one_shot_inverter_timeout_survives_a_dead_controller() {
        assert!(Expiry::InverterTimeout(Duration::from_secs(60)).is_dead_controller_safe());
        assert!(!Expiry::InverterCondition("target soc").is_dead_controller_safe());
        assert!(!Expiry::RecurringWindow.is_dead_controller_safe());
        assert!(!Expiry::UntilChanged.is_dead_controller_safe());
    }

    #[test]
    fn export_is_distinguishable_from_house_only_discharge() {
        assert_eq!(Command::discharge(1.5).target, DischargeTarget::HouseOnly);
        assert_eq!(Command::export(3).target, DischargeTarget::GridExport);
        assert!(Command::export(3).describe().contains("grid-export"));
    }

    #[test]
    fn describe_names_the_mode_power_and_export_intent() {
        assert_eq!(Command::passive().describe(), "passive");
        assert_eq!(Command::charge(2).describe(), "force_charge@2kW");
        assert_eq!(Command::discharge(1.5).describe(), "force_discharge@1.5kW");
        assert_eq!(
            Command::export(3).describe(),
            "force_discharge@3kW(grid-export)"
        );
    }

    #[test]
    fn constructors_request_the_default_hold_unless_overridden() {
        assert_eq!(Command::charge(1.0).hold, DEFAULT_HOLD);
        let short = Command::charge(1.0).holding_for(Duration::from_secs(60));
        assert_eq!(short.hold, Duration::from_secs(60));
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
        assert_eq!(Command::charge(2), Command::charge(2.0));
        assert_eq!(Command::discharge(1), Command::discharge(1.0));
        assert_eq!(Command::export(3), Command::export(3.0));
    }

    #[test]
    fn a_writable_driver_supports_only_its_listed_modes() {
        let caps = Capabilities {
            model: "test",
            can_write: true,
            modes: &[Mode::Passive, Mode::ForceCharge],
            expiry: Expiry::UntilChanged,
            reports_solar: false,
            write_blocked_reason: None,
        };
        assert!(caps.supports(Mode::Passive));
        assert!(caps.supports(Mode::ForceCharge));
        assert!(!caps.supports(Mode::ForceDischarge));
    }

    #[test]
    fn capabilities_refuse_every_mode_when_the_driver_cannot_write() {
        let caps = Capabilities {
            model: "test",
            can_write: false,
            modes: &[Mode::Passive, Mode::ForceCharge],
            expiry: Expiry::UntilChanged,
            reports_solar: false,
            write_blocked_reason: Some("unverified map"),
        };
        assert!(!caps.supports(Mode::Passive));
        assert!(!caps.supports(Mode::ForceCharge));
    }
}
