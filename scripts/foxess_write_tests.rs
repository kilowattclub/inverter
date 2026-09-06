use super::*;
use inverter::{Applied, Capabilities};

fn args(values: &[&str]) -> Vec<String> {
    values.iter().map(|v| (*v).to_owned()).collect()
}

#[test]
fn rejects_unbounded_invalid_and_ambiguous_commands() {
    for values in [
        vec![],
        vec!["port", "charge"],
        vec!["port", "charge", "5", "0"],
        vec!["port", "charge", "5", "301"],
        vec!["port", "charge", "NaN", "60"],
        vec!["port", "charge", "inf", "60"],
        vec!["port", "charge", "-1", "60"],
        vec!["port", "charge", "5.1", "60"],
        vec!["port", "discharge", "5", "60"],
        vec!["port", "passive", "extra"],
        vec!["", "passive"],
    ] {
        assert!(parse(&args(&values)).is_err(), "{values:?}");
    }
    assert_eq!(
        parse(&args(&["port", "charge", "5", "60"])).unwrap().1,
        Command::charge(5.0, Duration::from_secs(60))
    );
    assert_eq!(
        parse(&args(&["port", "export", "1", "30"])).unwrap().1,
        Command::export(1.0, Duration::from_secs(30))
    );
}

struct Fake {
    commands: Vec<Command>,
    reads: usize,
    soc: f64,
    fail_apply: bool,
    fail_reads: bool,
}
impl Inverter for Fake {
    fn capabilities(&self) -> Capabilities {
        unreachable!()
    }
    fn read_telemetry(&mut self) -> std::result::Result<Telemetry, inverter::Error> {
        self.reads += 1;
        if self.fail_reads && self.reads > 1 {
            return Err(inverter::Error::Comm("lost telemetry".into()));
        }
        Ok(Telemetry {
            soc_pct: self.soc,
            battery_kw: 0.0,
            grid_kw: 0.0,
            load_kw: 0.0,
            solar_kw: 0.0,
            at: SystemTime::now(),
            read_at: Instant::now(),
        })
    }
    fn apply(&mut self, command: Command) -> std::result::Result<Applied, inverter::Error> {
        self.commands.push(command);
        if self.fail_apply && command.mode != Mode::Passive {
            return Err(inverter::Error::Comm("write reply lost".into()));
        }
        Ok(Applied {
            power_kw: command.power_kw,
            expiry: command
                .ttl()
                .map_or(Expiry::UntilChanged, Expiry::InverterTimeout),
        })
    }
    fn mode(&mut self) -> std::result::Result<Mode, inverter::Error> {
        unreachable!()
    }
}
fn fake(soc: f64) -> Fake {
    Fake {
        commands: vec![],
        reads: 0,
        soc,
        fail_apply: false,
        fail_reads: true,
    }
}

#[test]
fn soc_limits_prevent_writes() {
    for (soc, command) in [
        (95.0, Command::charge(5.0, Duration::from_secs(60))),
        (15.0, Command::export(1.0, Duration::from_secs(60))),
        (f64::NAN, Command::charge(1.0, Duration::from_secs(60))),
    ] {
        let mut inv = fake(soc);
        assert!(run(&mut inv, command, 15.0, 95.0, &AtomicBool::new(false)).is_err());
        assert!(inv.commands.is_empty());
    }
}

#[test]
fn lost_telemetry_or_write_reply_triggers_passive_cleanup() {
    for fail_apply in [false, true] {
        let mut inv = fake(50.0);
        inv.fail_apply = fail_apply;
        let command = Command::charge(5.0, Duration::from_secs(60));
        assert!(run(&mut inv, command, 15.0, 95.0, &AtomicBool::new(false)).is_err());
        assert_eq!(inv.commands, vec![command, Command::passive()]);
    }
}

#[test]
fn passive_does_not_require_working_telemetry() {
    let mut inv = fake(f64::NAN);
    run(
        &mut inv,
        Command::passive(),
        15.0,
        95.0,
        &AtomicBool::new(false),
    )
    .unwrap();
    assert_eq!(inv.reads, 0);
    assert_eq!(inv.commands, vec![Command::passive()]);
}

#[test]
fn successful_test_sends_one_override_then_returns_to_passive() {
    let mut inv = fake(50.0);
    inv.fail_reads = false;
    let command = Command::charge(5.0, Duration::from_secs(1));
    run(&mut inv, command, 15.0, 95.0, &AtomicBool::new(false)).unwrap();
    assert_eq!(inv.commands, vec![command, Command::passive()]);
    assert!(inv.reads >= 3);
}
