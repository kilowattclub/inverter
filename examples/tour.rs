//! Read telemetry, apply a charge and advance past its timeout on the mock.
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

    // Apply a 2 kW charge with a five-minute timeout.
    let applied = inverter.charge(2, Duration::from_secs(300))?;
    println!(
        "charging: {} kW, ends by {:?}",
        applied.power_kw, applied.expiry
    );
    assert!(applied.expiry.is_dead_controller_safe());

    // Advance past the timeout.
    inverter.advance(Duration::from_secs(360));
    println!("later:    mode is {} again", inverter.get_mode()?);

    // Commands are also plain data, for planners and safety layers; build
    // one explicitly with its required TTL.
    let command = Command::charge(1.5, Duration::from_secs(3600));
    println!("planned:  {command}");
    inverter.apply(command)?;

    // Cancel the override.
    inverter.passive()?;
    Ok(())
}
