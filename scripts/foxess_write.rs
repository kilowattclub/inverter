//! One bounded FoxESS H1 G2 commissioning command over RS485.
use inverter::foxess::{registers, FoxEss};
use inverter::modbus::{read_words, SerialBus};
use inverter::{Command, Expiry, Inverter, Mode, Telemetry};
use std::error::Error;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const USAGE: &str = "usage: scripts/foxess-write PORT passive\n       scripts/foxess-write PORT <charge|export> KW SECONDS\nH1 G2, 9600 baud, unit 247; power > 0 and <= 5 kW; duration 1..=300 seconds.";

fn parse(args: &[String]) -> Result<(String, Command)> {
    let command = match args {
        [_, mode] if mode == "passive" => Command::passive(),
        [_, mode, power, seconds] if mode == "charge" || mode == "export" => {
            let power: f64 = power.parse()?;
            let seconds: u64 = seconds.parse()?;
            if !power.is_finite() || power <= 0.0 || power > 5.0 || !(1..=300).contains(&seconds) {
                return Err(USAGE.into());
            }
            let ttl = Duration::from_secs(seconds);
            if mode == "charge" {
                Command::charge(power, ttl)
            } else {
                Command::export(power, ttl)
            }
        }
        _ => return Err(USAGE.into()),
    };
    if args[0].is_empty() {
        return Err("serial port must not be empty".into());
    }
    Ok((args[0].clone(), command))
}

fn check_reading(t: &Telemetry, command: Command, min_soc: f64, max_soc: f64) -> Result<()> {
    if !t.soc_pct.is_finite()
        || !(0.0..=100.0).contains(&t.soc_pct)
        || t.age() > Duration::from_secs(5)
    {
        return Err("invalid or stale battery charge reading".into());
    }
    if command.mode == Mode::ForceCharge && t.soc_pct >= max_soc {
        return Err(format!(
            "charge level {}% has reached the {}% test ceiling",
            t.soc_pct, max_soc
        )
        .into());
    }
    if command.mode == Mode::ForceDischarge && t.soc_pct <= min_soc {
        return Err(format!(
            "charge level {}% has reached the {}% test floor",
            t.soc_pct, min_soc
        )
        .into());
    }
    Ok(())
}

fn report(label: &str, t: Telemetry) {
    let timestamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    println!(
        "{timestamp} {label}: soc={:.1}% battery={:+.3}kW grid={:+.3}kW load={:.3}kW solar={:.3}kW",
        t.soc_pct, t.battery_kw, t.grid_kw, t.load_kw, t.solar_kw
    );
}

fn run(
    inv: &mut dyn Inverter,
    command: Command,
    min_soc: f64,
    max_soc: f64,
    stop: &AtomicBool,
) -> Result<()> {
    if command.mode == Mode::Passive {
        inv.apply(command)?;
        println!("Passive/self-use acknowledged.");
        return Ok(());
    }
    let before = inv.read_telemetry()?;
    report("before", before);
    check_reading(&before, command, min_soc, max_soc)?;
    if stop.load(Ordering::Relaxed) {
        return Err("interrupted before command".into());
    }
    let ttl = command.ttl().ok_or("override requires a timeout")?;
    // All exits after attempting an override try to cancel it, including write errors.
    let result = (|| -> Result<()> {
        let started = Instant::now();
        let applied = inv.apply(command)?;
        match applied.expiry {
            Expiry::InverterTimeout(duration) if !duration.is_zero() && duration <= ttl => {}
            _ => return Err("driver did not report a bounded hardware timeout".into()),
        }
        println!("Command acknowledged: {command}; hardware timeout {ttl:?}. This is the requested setpoint, not measured battery power.");
        while started.elapsed() < ttl && !stop.load(Ordering::Relaxed) {
            let t = inv.read_telemetry()?;
            report("during", t);
            check_reading(&t, command, min_soc, max_soc)?;
            std::thread::sleep(Duration::from_secs(2).min(ttl.saturating_sub(started.elapsed())));
        }
        Ok(())
    })();
    let cleanup = inv.apply(Command::passive());
    match (result, cleanup) {
        (Ok(()), Ok(_)) => {
            println!("Test ended; passive/self-use acknowledged.");
            report("after", inv.read_telemetry()?);
            Ok(())
        }
        (Err(error), Ok(_)) => Err(format!("{error}; passive/self-use acknowledged").into()),
        (result, Err(error)) => Err(format!(
            "test result: {result:?}; FAILED to restore passive: {error}; check inverter locally"
        )
        .into()),
    }
}

fn main() -> Result<()> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    if args == ["--help"] {
        println!("{USAGE}");
        return Ok(());
    }
    let (port, command) = parse(&args)?;
    let stop = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, stop.clone())?;
    signal_hook::flag::register(signal_hook::consts::SIGTERM, stop.clone())?;
    let mut bus = SerialBus::open(&port, 9600, 247)?;
    let (min_soc, max_soc) = if command.mode == Mode::Passive {
        (15.0, 95.0)
    } else {
        let min_soc = read_words(&mut bus, &registers::remote_control::MIN_SOC)?[0] as f64;
        let max_soc = read_words(&mut bus, &registers::remote_control::MAX_SOC)?[0] as f64;
        if !(0.0..=100.0).contains(&min_soc)
            || !(0.0..=100.0).contains(&max_soc)
            || min_soc >= max_soc
        {
            return Err("invalid inverter charge limits; no write attempted".into());
        }
        (min_soc.max(15.0), max_soc.min(95.0))
    };
    if min_soc >= max_soc {
        return Err("inverter limits leave no commissioning charge range".into());
    }
    println!("H1 G2 on {port}, 9600 baud, unit 247. Test charge range: {min_soc}%..{max_soc}%. Battery + means charging; grid + means importing.");
    let mut inv = FoxEss::new(bus, &registers::H1_G2);
    let result = run(&mut inv, command, min_soc, max_soc, &stop);
    inv.close();
    result
}

#[cfg(test)]
#[path = "foxess_write_tests.rs"]
mod tests;
