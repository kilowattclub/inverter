//! A simulated inverter.
//!
//! Useful for tests and for running a controller with no hardware attached.
//! It deliberately models the *well-behaved* case: commands carry a real
//! one-shot timeout and the inverter reverts to passive on its own when that
//! elapses. Code that works against the mock and breaks against a real
//! inverter has usually assumed a fail-safe the hardware does not provide —
//! check [`Capabilities::expiry`] rather than trusting the mock's guarantee.

use std::time::{Duration, Instant, SystemTime};

use crate::{
    Applied, Capabilities, Command, DischargeTarget, Error, Expiry, Inverter, Mode, Telemetry,
};

const MODES: &[Mode] = &[Mode::Passive, Mode::ForceCharge, Mode::ForceDischarge];

/// A simulated battery and inverter.
///
/// The simulation runs on one clock: real time flows into it on every
/// interaction, and [`MockInverter::advance`] adds simulated time on top.
/// A command's hold elapses on that same clock, so a test can watch the
/// one-shot timeout revert without sleeping.
///
/// Builder inputs are clamped to physically meaningful values rather than
/// poisoning the arithmetic — a non-positive capacity becomes 1 Wh, negative
/// powers become zero, and `NaN` falls to the nearest bound.
pub struct MockInverter {
    capacity_kwh: f64,
    max_power_kw: f64,
    soc_pct: f64,
    baseline_load_kw: f64,
    solar_kw: f64,
    command: Command,
    /// Simulated time since `command` was applied.
    since_command: Duration,
    /// Real-time watermark; everything before it is already in the simulation.
    last_sync: Instant,
}

impl Default for MockInverter {
    fn default() -> Self {
        Self::new()
    }
}

impl MockInverter {
    /// A 10 kWh battery on a 5 kW inverter at 50%, with a 0.4 kW household load.
    pub fn new() -> Self {
        Self {
            capacity_kwh: 10.0,
            max_power_kw: 5.0,
            soc_pct: 50.0,
            baseline_load_kw: 0.4,
            solar_kw: 0.0,
            command: Command::passive(),
            since_command: Duration::ZERO,
            last_sync: Instant::now(),
        }
    }

    /// Set the usable capacity in kilowatt-hours. Clamped to at least 1 Wh.
    #[must_use]
    pub fn with_capacity_kwh(mut self, capacity_kwh: impl Into<f64>) -> Self {
        self.capacity_kwh = capacity_kwh.into().max(0.001);
        self
    }

    /// Set the inverter's power limit in kilowatts. Clamped to at least zero.
    #[must_use]
    pub fn with_max_power_kw(mut self, max_power_kw: impl Into<f64>) -> Self {
        self.max_power_kw = max_power_kw.into().max(0.0);
        self
    }

    /// Set the starting state of charge. Clamped to 0–100; `NaN` becomes 0.
    #[must_use]
    pub fn with_soc_pct(mut self, soc_pct: impl Into<f64>) -> Self {
        let soc_pct = soc_pct.into();
        self.soc_pct = if soc_pct.is_nan() {
            0.0
        } else {
            soc_pct.clamp(0.0, 100.0)
        };
        self
    }

    /// Set the simulated household load in kilowatts. Clamped to at least zero.
    #[must_use]
    pub fn with_load_kw(mut self, load_kw: impl Into<f64>) -> Self {
        self.baseline_load_kw = load_kw.into().max(0.0);
        self
    }

    /// Set simulated PV generation in kilowatts. Clamped to at least zero.
    #[must_use]
    pub fn with_solar_kw(mut self, solar_kw: impl Into<f64>) -> Self {
        self.solar_kw = solar_kw.into().max(0.0);
        self
    }

    /// Advance the simulation by `elapsed` without waiting for real time.
    ///
    /// The command's hold elapses in simulated time too: advancing past it
    /// integrates the battery up to the expiry boundary, reverts to passive,
    /// and runs the remainder passively — exactly what the real hardware the
    /// mock stands in for would have done. Tests should drive the model with
    /// this rather than sleeping.
    pub fn advance(&mut self, elapsed: Duration) {
        self.sync();
        self.step(elapsed);
    }

    /// The command currently in force, accounting for an elapsed timeout.
    #[must_use]
    pub fn active_command(&self) -> Command {
        let since = self.since_command + self.last_sync.elapsed();
        if self.command.mode != Mode::Passive && since >= self.command.hold {
            Command::passive()
        } else {
            self.command
        }
    }

    /// Fold real elapsed time into the simulation.
    fn sync(&mut self) {
        let elapsed = self.last_sync.elapsed();
        self.last_sync = Instant::now();
        self.step(elapsed);
    }

    fn battery_flow_kw(&self) -> f64 {
        let requested = self.command.power_kw.abs().min(self.max_power_kw);
        match self.command.mode {
            Mode::Passive => {
                // Self-use: soak surplus PV, otherwise cover the load.
                let surplus = self.solar_kw - self.baseline_load_kw;
                surplus.clamp(-self.max_power_kw, self.max_power_kw)
            }
            Mode::ForceCharge => requested,
            Mode::ForceDischarge => -requested,
        }
    }

    /// Run `elapsed` of simulated time, honouring the one-shot timeout: if
    /// the hold runs out mid-step, integration splits at that boundary and
    /// the rest of the step runs passive.
    fn step(&mut self, elapsed: Duration) {
        let mut remaining = elapsed;
        if self.command.mode != Mode::Passive {
            let until_expiry = self.command.hold.saturating_sub(self.since_command);
            if remaining >= until_expiry {
                self.integrate(until_expiry);
                remaining -= until_expiry;
                // A real one-shot timeout: the inverter reverts by itself.
                self.command = Command::passive();
                self.since_command = Duration::ZERO;
            }
        }
        self.integrate(remaining);
        self.since_command = self.since_command.saturating_add(remaining);
    }

    fn integrate(&mut self, elapsed: Duration) {
        let hours = elapsed.as_secs_f64() / 3600.0;
        let delta_kwh = self.battery_flow_kw() * hours;
        let stored = self.capacity_kwh * self.soc_pct / 100.0 + delta_kwh;
        self.soc_pct = (stored / self.capacity_kwh * 100.0).clamp(0.0, 100.0);
    }
}

impl Inverter for MockInverter {
    fn capabilities(&self) -> Capabilities {
        let mut caps =
            Capabilities::writable("mock", MODES, Expiry::InverterTimeout(self.command.hold));
        caps.reports_solar = true;
        caps.reports_mode = true;
        caps
    }

    fn read_telemetry(&mut self) -> Result<Telemetry, Error> {
        self.sync();

        let battery_kw = self.battery_flow_kw();
        let load_kw = self.baseline_load_kw;
        // AC balance: what the house and battery need beyond PV comes from the grid.
        let grid_kw = load_kw + battery_kw - self.solar_kw;
        let grid_kw = if self.command.mode == Mode::ForceDischarge
            && self.command.target == DischargeTarget::HouseOnly
        {
            // Without an export path, discharge cannot push past the load.
            grid_kw.max(0.0)
        } else {
            grid_kw
        };

        Ok(Telemetry {
            soc_pct: self.soc_pct,
            battery_kw,
            grid_kw,
            load_kw,
            solar_kw: self.solar_kw,
            at: SystemTime::now(),
            read_at: Instant::now(),
        })
    }

    fn apply(&mut self, command: Command) -> Result<Applied, Error> {
        if !self.capabilities().supports(command.mode) {
            return Err(Error::Unsupported(format!(
                "mock does not support {}",
                command.mode.as_str()
            )));
        }
        if !command.power_kw.is_finite() || command.power_kw < 0.0 {
            return Err(Error::Range(format!(
                "power must be finite and non-negative, got {}",
                command.power_kw
            )));
        }
        self.sync();

        self.command = command;
        self.since_command = Duration::ZERO;
        Ok(Applied {
            expiry: Expiry::InverterTimeout(command.hold),
            power_kw: command.power_kw.min(self.max_power_kw),
        })
    }

    // The mock genuinely knows its mode: it is the hardware, timeout included.
    fn mode(&mut self) -> Result<Mode, Error> {
        self.sync();
        Ok(self.command.mode)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::InverterExt;

    #[test]
    fn ext_methods_are_partial_applications_of_apply() {
        let mut sugared = MockInverter::new();
        let mut explicit = MockInverter::new();
        for (via_ext, command) in [
            (sugared.charge(2), Command::charge(2)),
            (sugared.discharge(1.5), Command::discharge(1.5)),
            (sugared.export(3), Command::export(3)),
            (sugared.passive(), Command::passive()),
        ] {
            let via_ext = via_ext.unwrap();
            let via_apply = explicit.apply(command).unwrap();
            assert_eq!(via_ext.expiry, via_apply.expiry);
            assert_eq!(via_ext.power_kw, via_apply.power_kw);
        }
    }

    #[test]
    fn telemetry_getters_are_partial_applications_of_read_telemetry() {
        let mut inv = MockInverter::new().with_load_kw(0.3).with_solar_kw(2.3);
        inv.apply(Command::passive()).unwrap();
        let t = inv.read_telemetry().unwrap();
        // The simulation steps by real elapsed time, so SoC drifts a hair
        // between reads; the powers are functions of the command and config.
        assert!((inv.get_soc_pct().unwrap() - t.soc_pct).abs() < 1e-6);
        assert_eq!(inv.get_battery_kw().unwrap(), t.battery_kw);
        assert_eq!(inv.get_grid_kw().unwrap(), t.grid_kw);
        assert_eq!(inv.get_load_kw().unwrap(), t.load_kw);
        assert_eq!(inv.get_solar_kw().unwrap(), t.solar_kw);
        assert_eq!(inv.get_export_kw().unwrap(), t.export_kw());
    }

    #[test]
    fn mode_reports_the_live_state_including_the_timeout_revert() {
        let mut inv = MockInverter::new();
        assert_eq!(inv.mode().unwrap(), Mode::Passive);
        inv.apply(Command::charge(1).holding_for(Duration::from_millis(1)))
            .unwrap();
        assert_eq!(inv.mode().unwrap(), Mode::ForceCharge);
        std::thread::sleep(Duration::from_millis(5));
        assert_eq!(
            inv.mode().unwrap(),
            Mode::Passive,
            "mode must report the hardware's revert, not the last command"
        );
    }

    #[test]
    fn applied_power_is_clamped_to_the_inverter_limit() {
        let mut inv = MockInverter::new().with_max_power_kw(5);
        let applied = inv.apply(Command::charge(50)).unwrap();
        assert_eq!(applied.power_kw, 5.0, "the clamp must be reported back");
    }

    #[test]
    fn the_mock_advertises_a_dead_controller_safe_expiry() {
        let inv = MockInverter::new();
        let caps = inv.capabilities();
        assert!(caps.can_write);
        assert!(caps.expiry.is_dead_controller_safe());
        assert!(caps.reports_mode, "the mock can always answer mode()");
    }

    #[test]
    fn charging_raises_the_state_of_charge() {
        let mut inv = MockInverter::new().with_soc_pct(50.0);
        inv.apply(Command::charge(1).holding_for(Duration::from_secs(3600)))
            .unwrap();
        inv.advance(Duration::from_secs(3600));
        // 1 kWh into a 10 kWh battery is ten percentage points.
        assert!((inv.soc_pct - 60.0).abs() < 0.5, "soc was {}", inv.soc_pct);
    }

    #[test]
    fn a_command_reverts_to_passive_once_its_hold_elapses() {
        let mut inv = MockInverter::new();
        inv.apply(Command::charge(1).holding_for(Duration::from_secs(60)))
            .unwrap();
        assert_eq!(inv.active_command().mode, Mode::ForceCharge);
        inv.advance(Duration::from_secs(61));
        assert_eq!(
            inv.active_command().mode,
            Mode::Passive,
            "a one-shot timeout must revert without the controller"
        );
    }

    #[test]
    fn the_hold_expires_in_simulated_time_and_charging_stops_at_the_boundary() {
        let mut inv = MockInverter::new()
            .with_soc_pct(50)
            .with_load_kw(0)
            .with_solar_kw(0);
        inv.apply(Command::charge(3.6).holding_for(Duration::from_secs(1800)))
            .unwrap();
        inv.advance(Duration::from_secs(3600));
        // 3.6 kW for the 30-minute hold is 1.8 kWh (18 points); the second
        // half hour must run passive, with nothing flowing.
        assert!((inv.soc_pct - 68.0).abs() < 0.1, "soc was {}", inv.soc_pct);
        assert_eq!(inv.active_command().mode, Mode::Passive);
    }

    #[test]
    fn active_command_reports_an_elapsed_timeout_without_a_prior_read() {
        let mut inv = MockInverter::new();
        inv.apply(Command::charge(1).holding_for(Duration::from_millis(1)))
            .unwrap();
        std::thread::sleep(Duration::from_millis(5));
        // No advance() or read in between: the answer must still be honest.
        assert_eq!(inv.active_command().mode, Mode::Passive);
    }

    #[test]
    fn state_of_charge_is_clamped_at_both_ends() {
        let hold = Duration::from_secs(4 * 3600);
        let mut inv = MockInverter::new().with_soc_pct(99.0);
        inv.apply(Command::charge(5).holding_for(hold)).unwrap();
        inv.advance(Duration::from_secs(4 * 3600));
        assert!(inv.soc_pct <= 100.0);

        let mut inv = MockInverter::new().with_soc_pct(1.0);
        inv.apply(Command::discharge(5).holding_for(hold)).unwrap();
        inv.advance(Duration::from_secs(4 * 3600));
        assert!(inv.soc_pct >= 0.0);
    }

    #[test]
    fn nonsense_builder_values_are_clamped_not_propagated() {
        // A zero capacity must not poison the SoC arithmetic with NaN, and a
        // negative power limit must not panic the passive flow clamp.
        let mut inv = MockInverter::new()
            .with_capacity_kwh(0)
            .with_max_power_kw(-5)
            .with_soc_pct(f64::NAN);
        inv.apply(Command::charge(1)).unwrap();
        inv.advance(Duration::from_secs(60));
        let t = inv.read_telemetry().unwrap();
        assert!(t.soc_pct.is_finite(), "soc was {}", t.soc_pct);
    }

    #[test]
    fn house_only_discharge_does_not_export() {
        let mut inv = MockInverter::new().with_load_kw(0.2);
        inv.apply(Command::discharge(3)).unwrap();
        let t = inv.read_telemetry().unwrap();
        assert_eq!(t.export_kw(), 0.0, "house-only discharge must not export");
    }

    #[test]
    fn a_grid_export_command_does_export() {
        let mut inv = MockInverter::new().with_load_kw(0.2);
        inv.apply(Command::export(3)).unwrap();
        let t = inv.read_telemetry().unwrap();
        assert!(t.export_kw() > 0.0, "export command must reach the grid");
    }

    #[test]
    fn rejects_a_nonsensical_power_value() {
        let mut inv = MockInverter::new();
        assert!(inv.apply(Command::charge(f64::NAN)).is_err());
        assert!(inv.apply(Command::charge(-1.0)).is_err());
    }

    #[test]
    fn passive_soaks_surplus_solar() {
        let mut inv = MockInverter::new().with_load_kw(0.3).with_solar_kw(2.3);
        inv.apply(Command::passive()).unwrap();
        let t = inv.read_telemetry().unwrap();
        assert!((t.battery_kw - 2.0).abs() < 1e-9, "got {}", t.battery_kw);
    }
}
