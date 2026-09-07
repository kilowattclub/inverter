# inverter

[![CI](https://github.com/kilowattclub/inverter/actions/workflows/ci.yml/badge.svg)](https://github.com/kilowattclub/inverter/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/inverter.svg)](https://crates.io/crates/inverter)
[![docs.rs](https://img.shields.io/docsrs/inverter)](https://docs.rs/inverter)

Rust drivers for hybrid solar/battery inverters over Modbus. Supports FoxESS
H1 G1/G2 and a simulated inverter.

## Usage

```sh
cargo add inverter
```

```rust
use inverter::{mock::MockInverter, Inverter, InverterExt};
use std::time::Duration;

let mut inverter = MockInverter::new();
let t = inverter.read_telemetry()?;
println!("{:.0}%  battery {:+.2} kW  grid {:+.2} kW", t.soc_pct, t.battery_kw, t.grid_kw);

let ttl = Duration::from_secs(60);
let applied = inverter.charge(2, ttl)?;
println!("{} kW, {:?}", applied.power_kw, applied.expiry);
inverter.passive()?;
```

FoxESS connections:

```rust
use inverter::foxess::{registers, FoxEss};

let mut inverter = FoxEss::open_serial("/dev/serial/by-id/usb-...", 9600, 247, &registers::H1_G2)?;
// Or a bridge configured for Modbus TCP:
let mut inverter = FoxEss::open_tcp("10.0.0.5:502", 247, &registers::H1_G2)?;
```

Use `open(OpenOptions { ... })` to select `mock`, `mock-relay` or `foxess`
from configuration. The factory uses H1 G2 for FoxESS; `unit_id: 0` selects
247. Unknown drivers, missing features and connection failures return errors.

[API reference](https://docs.rs/inverter) · `cargo run --example tour`

## Commands

| Method | Behaviour |
|---|---|
| `passive()` | Return to the inverter's self-use mode. |
| `hold(ttl)` | Keep battery power at zero. |
| `charge(kw, ttl)` | Charge, importing from the grid as needed. |
| `discharge(kw, ttl)` | Discharge to cover net household load. |
| `export(kw, ttl)` | Discharge with grid export permitted. |

Commands return `Applied`: the accepted power setpoint and expiry. They can
also be built as `Command` values and passed to `Inverter::apply`.

Every override requires a non-zero TTL. Drivers must provide a one-shot
inverter timeout, replace it on a new command, and cancel it on passive.
Unsupported commands return an error. `capabilities()` lists supported modes,
maximum timeout and telemetry support; discharge-target support varies by driver.

`Expiry` distinguishes an inverter timeout from a condition, recurring schedule
or persistent setting. Only a non-zero `InverterTimeout` passes
`is_dead_controller_safe()`. Passive returns `UntilChanged`.

The hardware timeout survives controller failure. Applications should also
request passive on SIGINT/SIGTERM. Power limits, SoC limits and export limits
are the controller's responsibility.

## Telemetry

| Field | Units and sign |
|---|---|
| `soc_pct` | Battery state of charge, percent |
| `battery_kw` | Positive when charging, negative when discharging |
| `grid_kw` | Positive when importing, negative when exporting |
| `load_kw` | Household consumption, non-negative |
| `solar_kw` | PV generation, non-negative; zero when unavailable |

`export_kw()` returns grid export as a positive value. `age()` uses a monotonic
clock. FoxESS timestamps the start of a telemetry read so retries count towards
its age; the fields are read sequentially.

Each `get_*` telemetry method performs a full read. Use `read_telemetry()` once
when several values are needed. `get_mode()` reads the current mode or returns
`Unsupported`; FoxESS does not currently support mode read-back.

## FoxESS

Select `registers::H1_G1` for first-generation H1/AC1/AIO-H1 input registers,
or `registers::H1_G2` for H1-G2/AC1-G2/P1 holding registers. Both use RS485,
directly or through a Modbus TCP bridge. The inverter's built-in LAN map is
unsupported.

The maps come from [foxess_modbus](https://github.com/nathanmarlor/foxess_modbus).
Check telemetry and timeout behaviour on the installed model and firmware.
The H1 G2 watchdog was tested on Master 1.53 / Manager 1.39.

Commands use function 6, with 30 ms between writes:

1. Disable remote control, select self-use and clear the old power setpoint.
2. Enable remote control, then set and read back the TTL at `44001`.
3. Write power to `44002`, loading the watchdog.

H1 G2 resets the timeout to 60 seconds when enabled, so the TTL must be written
afterwards. Programming failures trigger a best-effort return to passive.
Requested TTLs range from 1 to 65,535 seconds; fractions round down. Firmware
countdown, power ramp and telemetry cadence affect the observed stopping time.

Power commands set **inverter AC power**. Battery losses, household load and
separate solar generation affect measured battery and grid power.
`Applied.power_kw` is a setpoint, not a measured grid-flow guarantee.
`solar_kw` includes only PV connected directly to FoxESS.

`hold()` uses zero remote active power. `discharge()` is unsupported because
this control cannot guarantee house-only discharge as load changes; use
`export()` for export-capable discharge. Expiry returns to self-use.

Remote charging does not respect the inverter's maximum SoC setting; the
controller must enforce it. Remote discharge respects minimum SoC and maximum
discharge current. FoxESS app strategy periods can overwrite these commands.

## Mock and relay indicator

`MockInverter` simulates battery limits, household load, solar and command expiry.
It advances with real time; `advance(Duration)` adds simulated time for tests.

With `mock` and `serial`, `with_waveshare_relay` attaches a Waveshare Modbus RTU
Relay 4CH indicator: CH1 passive, CH2 charge, CH3 house-only discharge, CH4 export,
all off for hold. The indicator refreshes on driver calls; it has no independent
hardware timeout. `close()` switches all channels off.

## Build and test

Requires Rust 1.85 or newer. Features `serial`, `tcp`, `foxess` and `mock` are
enabled by default. The API is synchronous.

```sh
cargo test --all-features
cargo test --all-features --example foxess_write
cargo fmt --check
cargo clippy --all-targets --all-features -- -D warnings
```

Tests use simulated devices and a loopback TCP socket. For hardware tests, see
[scripts/README.md](scripts/README.md).

## Publishing

Configure a crates.io trusted publisher for `kilowattclub/inverter`, workflow
`publish.yml`, with no environment. Bump the version and changelog, merge to
`main`, then run **Publish crate** in GitHub Actions. The workflow tests,
checks the package and publishes with a temporary crates.io token.

## Licence

[MIT](LICENSE).
