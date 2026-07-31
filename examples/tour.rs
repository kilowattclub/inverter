//! A tour of the whole model on the simulated inverter: read telemetry,
//! check capabilities, command a charge, and watch the one-shot timeout
//! revert to passive. No hardware required.
//!
//! Run with: `cargo run --example tour`

use std::time::Duration;

use inverter::{mock::MockInverter, Command, Inverter, InverterExt, Mode};

fn main() -> Result<(), inverter::Error> {
    // A 10 kWh battery at 40%, with some afternoon sun on the roof.
    let mut inverter = MockInverter::new().with_soc_pct(40).with_solar_kw(1.2);

    let caps = inverter.capabilities();
    println!("model:    {}", caps.model);

    let t = inverter.read_telemetry()?;
    println!(
        "reading:  {:.0}%  battery {:+.2} kW  grid {:+.2} kW  load {:.2} kW  solar {:.2} kW",
        t.soc_pct, t.battery_kw, t.grid_kw, t.load_kw, t.solar_kw
    );

    // Ask before commanding: support varies by model and connection route.
    if !caps.supports(Mode::ForceCharge) {
        println!(
            "no writes: {}",
            caps.write_blocked_reason.unwrap_or("unsupported")
        );
        return Ok(());
    }

    // Charge at 2 kW. What the inverter committed to comes back: the power
    // (possibly clamped) and — the crate's reason to exist — how the
    // command will end.
    let applied = inverter.charge(2)?;
    println!(
        "charging: {} kW, ends by {:?}",
        applied.power_kw, applied.expiry
    );
    assert!(applied.expiry.is_dead_controller_safe());

    // Six simulated minutes later the five-minute hold has lapsed and the
    // inverter has reverted by itself — no controller involved. That is
    // what a real fail-safe looks like.
    inverter.advance(Duration::from_secs(360));
    println!("later:    mode is {} again", inverter.mode()?);

    // Commands are also plain data, for planners and safety layers; build
    // one explicitly to ask for a non-default hold.
    let command = Command::charge(1.5).holding_for(Duration::from_secs(3600));
    println!("planned:  {command}");
    inverter.apply(command)?;

    // And passive is the safe floor: no power, nothing to expire, always
    // safe to hand back early.
    inverter.passive()?;
    Ok(())
}
