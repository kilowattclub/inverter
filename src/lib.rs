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
//! // Or single values, and the mode currently in force:
//! let soc = inv.get_soc_pct()?;
//! assert_eq!(inv.get_mode()?, Mode::Passive);
//!
//! if caps.supports(Mode::ForceCharge) {
//!     // Every override has an explicit TTL. Powers are kilowatts.
//!     let applied = inv.charge(2, Duration::from_secs(60))?;
//!     // How this command ends is data, not an assumption.
//!     println!("expires: {:?}", applied.expiry);
//! }
//! # Ok(())
//! # }
//! # #[cfg(not(feature = "mock"))]
//! # fn main() {}
//! ```

#![forbid(unsafe_code)]
#![warn(missing_docs)]
// `doc_auto_cfg` was merged into `doc_cfg` in Rust 1.92; the automatic
// feature-requirement banners on docs.rs come from this single gate now.
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
/// [`Passive`](Mode::Passive) is the inverter's own behaviour; the other three
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
    /// Keep battery power at zero: the house and battery neither charge nor
    /// discharge one another until the command expires.
    ///
    /// Unlike [`Passive`](Mode::Passive), this reserves stored energy instead
    /// of letting the inverter's self-use logic spend it on the house.
    Hold,
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

/// Where discharged energy is meant to go.
///
/// This is a target, not a permission: the caller decides policy. It is part
/// of the command because some inverters reach the two behaviours through
/// different work modes rather than through a power limit.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
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

/// How a non-passive command stops.
///
/// The reason this crate exists. A caller that treats
/// [`Expiry::RecurringWindow`] as though it were
/// [`Expiry::InverterTimeout`] has built a system that keeps forcing a
/// battery after the controller is gone — every day, at the same time, until
/// someone notices.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
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
    #[must_use]
    pub fn is_dead_controller_safe(&self) -> bool {
        matches!(self, Expiry::InverterTimeout(timeout) if !timeout.is_zero())
    }
}

/// What a driver can actually do with the connected hardware.
///
/// Ask before you command. Feature support varies by model *and* by how the
/// inverter is connected — the same unit over RS485 and over its own network
/// module does not expose the same registers.
///
/// The struct is `#[non_exhaustive]` so capabilities can grow without
/// breaking callers. Drivers outside this crate therefore build it through
/// [`Capabilities::read_only`] or [`Capabilities::writable`] and then set the
/// public reporting fields directly.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
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
    /// `reason` is surfaced through [`Capabilities::write_blocked_reason`] so
    /// a caller can log *why* writes are unavailable instead of a bare
    /// "unsupported". The reporting flags start `false`; set the public
    /// fields for whatever the driver can do:
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
    /// Panics unless `modes` contains [`Mode::Passive`]: a writable driver
    /// that cannot step out of the way leaves callers with no safe fallback.
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
    /// Monotonic time of the reading.
    ///
    /// Use this for staleness checks: unlike [`Telemetry::at`] it cannot be
    /// dragged backwards by an NTP step or a daylight-saving change.
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

/// What the inverter accepted, which may be less than what was asked for.
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
    /// A non-passive command must replace any previous inverter-side watchdog
    /// with a one-shot timeout no longer than [`Command::ttl`]; passive must
    /// cancel that watchdog. A driver that cannot do this must refuse the
    /// non-passive command without changing inverter state.
    fn apply(&mut self, command: Command) -> Result<Applied, Error>;

    /// The [`Mode`] currently in force, as far as this driver can know it.
    ///
    /// Drivers must not guess. A driver that cannot read the imposed state
    /// back from the hardware returns [`Error::Unsupported`] rather than
    /// repeating what it last commanded — a stale belief is exactly the
    /// mistake that hides an expired or externally-changed command.
    /// [`Capabilities::reports_mode`] says up front whether this can answer.
    fn mode(&mut self) -> Result<Mode, Error>;

    /// Release the transport. Called once, on shutdown.
    fn close(&mut self) {}
}

/// Partial applications of the [`Inverter`] operations.
///
/// Sugar only, in two groups. Every non-passive command method requires its
/// TTL at the call site, builds the matching [`Command`], and calls
/// [`Inverter::apply`]. The `get_*` methods each perform a **full**
/// [`Inverter::read_telemetry`] (or [`Inverter::mode`]) and return one value —
/// convenient for a one-off check, wasteful in a loop; when you need several
/// values, read once and use the fields. The prefix marks the cost: `get_*`
/// talks to hardware, while same-named accessors on [`Telemetry`] are free
/// field reads.
///
/// The blanket implementation is the only one the coherence rules allow, so
/// no driver can override these — every spelling reaches hardware through
/// `apply` and `read_telemetry`.
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

    /// The [`Mode`] currently in force — the `get_*` spelling of
    /// [`Inverter::mode`], with the same contract: drivers that cannot read
    /// it back honestly error rather than guessing.
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
