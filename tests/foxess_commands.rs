#![cfg(feature = "foxess")]

use inverter::foxess::{registers, FoxEss};
use inverter::modbus::ModbusBus;
use inverter::{Command, Error, Expiry, Inverter};
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[derive(Default)]
struct State {
    enabled: bool,
    power: u16,
    timeout: u16,
    ignore_timeout: bool,
    fail_power: bool,
    writes: Vec<(u16, u16)>,
}

struct Hardware(Arc<Mutex<State>>);

impl ModbusBus for Hardware {
    fn read_input(&mut self, address: u16, words: u8) -> Result<Vec<u16>, Error> {
        self.read_holding(address, words)
    }

    fn read_holding(&mut self, address: u16, words: u8) -> Result<Vec<u16>, Error> {
        assert_eq!((address, words), (44001, 1));
        Ok(vec![self.0.lock().unwrap().timeout])
    }

    fn write_holding(&mut self, address: u16, value: u16) -> Result<(), Error> {
        let mut state = self.0.lock().unwrap();
        state.writes.push((address, value));
        match address {
            44000 => {
                if value == 1 {
                    assert_eq!(state.power, 0, "enabling must not resume an old setpoint");
                    state.timeout = 60; // Observed H1 G2 behaviour.
                }
                state.enabled = value == 1;
            }
            44001 if !state.ignore_timeout => state.timeout = value,
            44002 => {
                if state.fail_power && value != 0 {
                    return Err(Error::Comm("injected power write failure".into()));
                }
                state.power = value;
            }
            _ => {}
        }
        Ok(())
    }
}

#[test]
fn enabling_cannot_replace_the_requested_timeout_or_resume_old_power() {
    for map in [&registers::H1_G1, &registers::H1_G2] {
        let state = Arc::new(Mutex::new(State {
            enabled: true,
            power: 5000,
            timeout: 60,
            ..State::default()
        }));
        let mut inverter = FoxEss::new(Hardware(state.clone()), map);
        let applied = inverter
            .apply(Command::export(1.0, Duration::from_secs(80)))
            .unwrap();
        assert_eq!(
            applied.expiry,
            Expiry::InverterTimeout(Duration::from_secs(80))
        );
        let state = state.lock().unwrap();
        assert!(state.enabled);
        assert_eq!(state.timeout, 80);
        assert_eq!(state.power, 1000);
    }
}

#[test]
fn an_acknowledged_but_ignored_timeout_never_applies_requested_power() {
    let state = Arc::new(Mutex::new(State {
        ignore_timeout: true,
        ..State::default()
    }));
    let mut inverter = FoxEss::new(Hardware(state.clone()), &registers::H1_G2);
    let error = inverter
        .apply(Command::charge(1.0, Duration::from_secs(20)))
        .unwrap_err();
    assert!(error.to_string().contains("timeout read-back"));
    let state = state.lock().unwrap();
    assert!(!state.enabled);
    assert_eq!(state.power, 0);
    assert!(!state.writes.iter().any(|&(a, v)| a == 44002 && v != 0));
    assert_eq!(state.writes.last(), Some(&(41000, 0)));
}

#[test]
fn a_failed_power_write_disables_the_partially_programmed_command() {
    let state = Arc::new(Mutex::new(State {
        fail_power: true,
        ..State::default()
    }));
    let mut inverter = FoxEss::new(Hardware(state.clone()), &registers::H1_G2);
    assert!(inverter
        .apply(Command::export(1.0, Duration::from_secs(20)))
        .is_err());
    let state = state.lock().unwrap();
    assert!(!state.enabled);
    assert_eq!(state.power, 0);
    assert_eq!(state.writes.last(), Some(&(41000, 0)));
}
