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
pub struct MockInverter {
    capacity_wh: f64,
    max_power_w: f64,
    soc_pct: f64,
    baseline_load_w: f64,
    solar_w: f64,
    command: Command,
    commanded_at: Instant,
    last_step: Instant,
}

impl Default for MockInverter {
    fn default() -> Self {
        Self::new()
    }
}

impl MockInverter {
    /// A 10 kWh battery on a 5 kW inverter at 50%, with a 400 W household load.
    pub fn new() -> Self {
        let now = Instant::now();
        Self {
            capacity_wh: 10_000.0,
            max_power_w: 5_000.0,
            soc_pct: 50.0,
            baseline_load_w: 400.0,
            solar_w: 0.0,
            command: Command::passive(),
            commanded_at: now,
            last_step: now,
        }
    }

    /// Set the usable capacity in watt-hours.
    pub fn with_capacity_wh(mut self, capacity_wh: f64) -> Self {
        self.capacity_wh = capacity_wh;
        self
    }

    /// Set the inverter's power limit in watts.
    pub fn with_max_power_w(mut self, max_power_w: f64) -> Self {
        self.max_power_w = max_power_w;
        self
    }

    /// Set the starting state of charge.
    pub fn with_soc_pct(mut self, soc_pct: f64) -> Self {
        self.soc_pct = soc_pct.clamp(0.0, 100.0);
        self
    }

    /// Set the simulated household load in watts.
    pub fn with_load_w(mut self, load_w: f64) -> Self {
        self.baseline_load_w = load_w.max(0.0);
        self
    }

    /// Set simulated PV generation in watts.
    pub fn with_solar_w(mut self, solar_w: f64) -> Self {
        self.solar_w = solar_w.max(0.0);
        self
    }

    /// Advance the simulation by `elapsed` without waiting for real time.
    ///
    /// Tests should drive the model with this rather than sleeping.
    pub fn advance(&mut self, elapsed: Duration) {
        self.step(elapsed);
        self.last_step = Instant::now();
    }

    /// The command currently in force, after any timeout has been applied.
    pub fn active_command(&self) -> Command {
        self.command
    }

    fn expire_if_due(&mut self) {
        if self.command.mode != Mode::Passive && self.commanded_at.elapsed() >= self.command.hold {
            // A real one-shot timeout: the inverter reverts by itself.
            self.command = Command::passive();
        }
    }

    fn battery_w(&self) -> f64 {
        let requested = self.command.power_w.abs().min(self.max_power_w);
        match self.command.mode {
            Mode::Passive => {
                // Self-use: soak surplus PV, otherwise cover the load.
                let surplus = self.solar_w - self.baseline_load_w;
                surplus.clamp(-self.max_power_w, self.max_power_w)
            }
            Mode::ForceCharge => requested,
            Mode::ForceDischarge => -requested,
        }
    }

    fn step(&mut self, elapsed: Duration) {
        self.expire_if_due();
        let hours = elapsed.as_secs_f64() / 3600.0;
        let delta_wh = self.battery_w() * hours;
        let stored = self.capacity_wh * self.soc_pct / 100.0 + delta_wh;
        self.soc_pct = (stored / self.capacity_wh * 100.0).clamp(0.0, 100.0);
    }
}

impl Inverter for MockInverter {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            model: "mock",
            can_write: true,
            modes: MODES,
            expiry: Expiry::InverterTimeout(self.command.hold),
            reports_solar: true,
            write_blocked_reason: None,
        }
    }

    fn read_telemetry(&mut self) -> Result<Telemetry, Error> {
        let elapsed = self.last_step.elapsed();
        self.step(elapsed);
        self.last_step = Instant::now();

        let battery_w = self.battery_w();
        let load_w = self.baseline_load_w;
        // AC balance: what the house and battery need beyond PV comes from the grid.
        let grid_w = load_w + battery_w - self.solar_w;
        let grid_w = if self.command.mode == Mode::ForceDischarge
            && self.command.target == DischargeTarget::HouseOnly
        {
            // Without an export path, discharge cannot push past the load.
            grid_w.max(0.0)
        } else {
            grid_w
        };

        Ok(Telemetry {
            soc_pct: self.soc_pct,
            battery_w,
            grid_w,
            load_w,
            solar_w: self.solar_w,
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
        if !command.power_w.is_finite() || command.power_w < 0.0 {
            return Err(Error::Range(format!(
                "power must be finite and non-negative, got {}",
                command.power_w
            )));
        }
        let elapsed = self.last_step.elapsed();
        self.step(elapsed);
        self.last_step = Instant::now();

        self.command = command;
        self.commanded_at = Instant::now();
        Ok(Applied {
            expiry: Expiry::InverterTimeout(command.hold),
            power_w: command.power_w.min(self.max_power_w),
        })
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
            (sugared.charge(2_000.0), Command::charge(2_000.0)),
            (sugared.discharge(1_500.0), Command::discharge(1_500.0)),
            (sugared.export(3_000.0), Command::export(3_000.0)),
            (sugared.passive(), Command::passive()),
        ] {
            let via_ext = via_ext.unwrap();
            let via_apply = explicit.apply(command).unwrap();
            assert_eq!(via_ext.expiry, via_apply.expiry);
            assert_eq!(via_ext.power_w, via_apply.power_w);
        }
    }

    #[test]
    fn charging_raises_the_state_of_charge() {
        let mut inv = MockInverter::new().with_soc_pct(50.0);
        inv.apply(Command::charge(1_000.0)).unwrap();
        inv.advance(Duration::from_secs(3600));
        // 1 kWh into a 10 kWh battery is ten percentage points.
        assert!((inv.soc_pct - 60.0).abs() < 0.5, "soc was {}", inv.soc_pct);
    }

    #[test]
    fn a_command_reverts_to_passive_once_its_hold_elapses() {
        let mut inv = MockInverter::new();
        inv.apply(Command::charge(1_000.0).holding_for(Duration::from_millis(1)))
            .unwrap();
        assert_eq!(inv.active_command().mode, Mode::ForceCharge);
        std::thread::sleep(Duration::from_millis(5));
        inv.advance(Duration::from_secs(1));
        assert_eq!(
            inv.active_command().mode,
            Mode::Passive,
            "a one-shot timeout must revert without the controller"
        );
    }

    #[test]
    fn state_of_charge_is_clamped_at_both_ends() {
        let mut inv = MockInverter::new().with_soc_pct(99.0);
        inv.apply(Command::charge(5_000.0)).unwrap();
        inv.advance(Duration::from_secs(4 * 3600));
        assert!(inv.soc_pct <= 100.0);

        let mut inv = MockInverter::new().with_soc_pct(1.0);
        inv.apply(Command::discharge(5_000.0)).unwrap();
        inv.advance(Duration::from_secs(4 * 3600));
        assert!(inv.soc_pct >= 0.0);
    }

    #[test]
    fn house_only_discharge_does_not_export() {
        let mut inv = MockInverter::new().with_load_w(200.0);
        inv.apply(Command::discharge(3_000.0)).unwrap();
        let t = inv.read_telemetry().unwrap();
        assert_eq!(t.export_w(), 0.0, "house-only discharge must not export");
    }

    #[test]
    fn a_grid_export_command_does_export() {
        let mut inv = MockInverter::new().with_load_w(200.0);
        inv.apply(Command::export(3_000.0)).unwrap();
        let t = inv.read_telemetry().unwrap();
        assert!(t.export_w() > 0.0, "export command must reach the grid");
    }

    #[test]
    fn rejects_a_nonsensical_power_value() {
        let mut inv = MockInverter::new();
        assert!(inv.apply(Command::charge(f64::NAN)).is_err());
        assert!(inv.apply(Command::charge(-1.0)).is_err());
    }

    #[test]
    fn passive_soaks_surplus_solar() {
        let mut inv = MockInverter::new().with_load_w(300.0).with_solar_w(2_300.0);
        inv.apply(Command::passive()).unwrap();
        let t = inv.read_telemetry().unwrap();
        assert!((t.battery_w - 2_000.0).abs() < 1.0, "got {}", t.battery_w);
    }
}
