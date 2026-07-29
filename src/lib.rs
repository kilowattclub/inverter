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
//! * `battery_w > 0` — charging (power into the cells)
//! * `grid_w > 0` — importing; `< 0` — exporting
//! * `load_w >= 0` — household consumption
//! * `solar_w >= 0` — PV generation, `0.0` when the model cannot report it
//!
//! # Example
//!
//! ```
//! use inverter::{Command, Inverter, mock::MockInverter};
//!
//! let mut inv = MockInverter::new();
//! let caps = inv.capabilities();
//! assert!(caps.can_write);
//!
//! let telemetry = inv.read_telemetry().unwrap();
//! println!("battery at {}%", telemetry.soc_pct);
//!
//! if caps.supports(Command::charge(2_000.0).mode) {
//!     let applied = inv.apply(Command::charge(2_000.0)).unwrap();
//!     // How this command ends is data, not an assumption.
//!     println!("expires: {:?}", applied.expiry);
//! }
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
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum Mode {
    /// The inverter's own self-consumption behaviour, with no imposed schedule.
    ///
    /// This is the state every driver must be able to return to, and the state
    /// a caller should fall back to when it is unsure.
    Passive,
    /// Charge the battery, importing from the grid if needed.
    ForceCharge,
    /// Discharge the battery.
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
    /// Requested power in watts. Ignored for [`Mode::Passive`].
    pub power_w: f64,
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
    /// Return to the inverter's own self-use behaviour.
    pub fn passive() -> Self {
        Command {
            mode: Mode::Passive,
            power_w: 0.0,
            target: DischargeTarget::HouseOnly,
            hold: DEFAULT_HOLD,
        }
    }

    /// Charge at `power_w`, importing if necessary.
    pub fn charge(power_w: f64) -> Self {
        Command {
            mode: Mode::ForceCharge,
            power_w,
            target: DischargeTarget::HouseOnly,
            hold: DEFAULT_HOLD,
        }
    }

    /// Discharge at `power_w` to cover household load, without exporting.
    pub fn discharge(power_w: f64) -> Self {
        Command {
            mode: Mode::ForceDischarge,
            power_w,
            target: DischargeTarget::HouseOnly,
            hold: DEFAULT_HOLD,
        }
    }

    /// Discharge at `power_w`, deliberately exporting to the grid.
    pub fn export(power_w: f64) -> Self {
        Command {
            mode: Mode::ForceDischarge,
            power_w,
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
                format!("force_discharge@{:.0}W(grid-export)", self.power_w)
            }
            _ => format!("{}@{:.0}W", self.mode.as_str(), self.power_w),
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
    /// Battery power, watts. Positive means charging.
    pub battery_w: f64,
    /// Grid power, watts. Positive means importing.
    pub grid_w: f64,
    /// Household consumption, watts.
    pub load_w: f64,
    /// PV generation, watts. `0.0` when the model cannot report it.
    pub solar_w: f64,
    /// Wall-clock time of the reading, for display and storage.
    pub at: SystemTime,
    /// Monotonic time of the reading.
    ///
    /// Use this for staleness checks: unlike [`Telemetry::at`] it cannot be
    /// dragged backwards by an NTP step or a daylight-saving change.
    pub read_at: Instant,
}

impl Telemetry {
    /// Power flowing out to the grid, watts. Zero while importing.
    pub fn export_w(&self) -> f64 {
        (-self.grid_w).max(0.0)
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
    /// Power the driver actually commanded, after any model-specific clamping.
    pub power_w: f64,
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

    /// Release the transport. Called once, on shutdown.
    fn close(&mut self) {}
}

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
        assert_eq!(
            Command::discharge(1_000.0).target,
            DischargeTarget::HouseOnly
        );
        assert_eq!(Command::export(1_000.0).target, DischargeTarget::GridExport);
        assert!(Command::export(1_000.0).describe().contains("grid-export"));
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
