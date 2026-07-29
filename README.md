# inverter

Control hybrid solar/battery inverters over Modbus, from Rust.

One trait covers every driver: ask what the hardware can do, read
telemetry, issue commands, and — the crate's central idea — learn **how each
command ends**. A command that the inverter reverts by itself is a fail-safe;
a daily window that happens to end at the same time is not, because it fires
again tomorrow whether or not your controller is still alive. The API makes
that difference impossible to ignore: every accepted command returns its
[`Expiry`](#expiry--how-a-command-ends).

```rust
use inverter::{Inverter, InverterExt, Mode, mock::MockInverter};

let mut inv = MockInverter::new();

let t = inv.read_telemetry()?;
println!("{:.0}% battery {:+.0} W grid {:+.0} W", t.soc_pct, t.battery_w, t.grid_w);

if inv.capabilities().supports(Mode::ForceCharge) {
    let applied = inv.charge(2_000.0)?;
    assert!(applied.expiry.is_dead_controller_safe());
}
```

## What the crate offers

| | |
|---|---|
| [`Inverter`](#the-inverter-trait) | the driver trait: `capabilities`, `read_telemetry`, `apply`, `close` |
| [`InverterExt`](#command-methods-inverterext) | per-command methods: `passive`, `charge`, `discharge`, `export` |
| [`Command`](#building-commands) | commands as data, for planners and safety layers |
| [`Telemetry`](#reading-telemetry), [`Applied`](#what-the-inverter-committed-to), [`Expiry`](#expiry--how-a-command-ends) | what came back, and what it means |
| [`modbus`](#transports) | Modbus RTU over serial, Modbus TCP to a bridge |
| [`foxess`](#foxess-h1-driver) | FoxESS H1 G1/G2 driver — reads only until the map is hardware-verified |
| [`mock`](#the-mock) | a full simulation for tests and hardware-free development |
| [`register`](#writing-a-new-driver) | register definitions and value coding, for writing drivers |

The crate is synchronous by design: Modbus over a serial line does not
benefit from async, and a blocking API keeps a runtime out of your
dependency tree.

## The `Inverter` trait

Every driver implements these four methods.

### `capabilities() -> Capabilities`

What this driver can do with the connected hardware. Cheap and
side-effect-free; call it every tick if you like. Support varies by model
*and* connection route — the same inverter over RS485 and over its own LAN
module does not expose the same registers — so ask before you command.

```rust
let caps = inv.capabilities();
println!("driver: {}", caps.model);

if !caps.can_write {
    // A read-only driver says why, instead of failing later.
    println!("writes unavailable: {}", caps.write_blocked_reason.unwrap_or("unknown"));
}

if caps.supports(Mode::ForceDischarge) && caps.expiry.is_dead_controller_safe() {
    // Safe to command: this hardware reverts by itself if we die.
}
```

`Capabilities` fields: `model`, `can_write`, `modes` (commandable modes),
`expiry` (how this driver's commands end), `reports_solar`,
`write_blocked_reason`.

### `read_telemetry() -> Result<Telemetry, Error>`

Read the current state of the system. Drivers report failures instead of
returning plausible-looking data — a zero from a dropped frame is more
dangerous than an error.

```rust
let t = inv.read_telemetry()?;
println!(
    "soc {:.0}%  battery {:+.0} W  grid {:+.0} W  load {:.0} W  solar {:.0} W",
    t.soc_pct, t.battery_w, t.grid_w, t.load_w, t.solar_w
);
```

### `apply(Command) -> Result<Applied, Error>`

Command the inverter. Takes a [`Command`](#building-commands) value and
returns what the inverter [actually committed to](#what-the-inverter-committed-to),
which may be less than you asked for. Returns `Error::Unsupported` for a
mode the capabilities don't advertise.

```rust
use inverter::Command;
use std::time::Duration;

let applied = inv.apply(Command::charge(3_000.0).holding_for(Duration::from_secs(120)))?;
println!("commanded {} W, ends: {:?}", applied.power_w, applied.expiry);
```

### `close()`

Release the transport. Called once, on shutdown. The default implementation
does nothing.

```rust
inv.close();
```

## Command methods (`InverterExt`)

Partial applications of `apply`, one per command shape, using the default
hold. They come from an extension trait whose blanket implementation is the
only one the coherence rules allow — no driver can override them, so every
spelling reaches hardware through `apply`.

```rust
use inverter::InverterExt;

inv.passive()?;             // the inverter's own self-use behaviour
inv.charge(2_000.0)?;       // charge at 2 kW, importing if needed
inv.discharge(1_500.0)?;    // cover household load only; no export
inv.export(3_000.0)?;       // deliberately export, e.g. a grid-services event
```

Each returns `Result<Applied, Error>`, exactly as `apply` does. For a
non-default hold, build the `Command` yourself and call `apply`.

## Building commands

`Command` is plain data — construct it, store it, compare it, log it, then
apply it. This is the type to use when commands come from a planner or pass
through a safety layer rather than being issued imperatively.

```rust
use inverter::Command;
use std::time::Duration;

let cmd = Command::charge(2_000.0);            // ForceCharge @ 2 kW
let cmd = Command::discharge(1_500.0);         // ForceDischarge, house only
let cmd = Command::export(3_000.0);            // ForceDischarge, grid export
let cmd = Command::passive();                  // back to self-use

// The hold is a request for how long the command should stand
// (default 300 s); what the inverter commits to comes back in Applied.
let cmd = Command::charge(2_000.0).holding_for(Duration::from_secs(60));

assert_eq!(cmd.describe(), "force_charge@2000W");  // stable form for logs
```

`Mode` round-trips through stable identifiers for configuration and logs:

```rust
use inverter::Mode;

assert_eq!(Mode::ForceCharge.as_str(), "force_charge");
assert_eq!(Mode::from_str("passive"), Some(Mode::Passive));
```

## Reading telemetry

`Telemetry` uses one sign convention across every driver, whatever the
inverter reports natively:

| Field | Meaning |
|---|---|
| `soc_pct` | battery state of charge, percent |
| `battery_w > 0` | charging (power into the cells) |
| `grid_w > 0` | importing; `< 0` exporting |
| `load_w >= 0` | household consumption |
| `solar_w >= 0` | PV generation, `0.0` when the model cannot report it |
| `at` | wall-clock time of the reading, for display and storage |
| `read_at` | monotonic time of the reading, for staleness checks |

Two helpers:

```rust
let t = inv.read_telemetry()?;

// Power flowing out to the grid; zero while importing.
let export = t.export_w();

// How long ago the reading was taken. Monotonic: an NTP step or DST
// change cannot drag it backwards, so it is safe to gate commands on.
if t.age() > std::time::Duration::from_secs(30) {
    // stale — don't act on it
}
```

## What the inverter committed to

`apply` (and the `InverterExt` methods) return `Applied`:

```rust
let applied = inv.charge(50_000.0)?;
applied.power_w;  // after model-specific clamping — may be less than asked
applied.expiry;   // how this command will actually end
```

A driver is allowed to return a weaker guarantee than requested; silently
assuming otherwise is the mistake this type exists to prevent.

## `Expiry` — how a command ends

```rust
pub enum Expiry {
    InverterTimeout(Duration),       // reverts by itself, once. A real dead-man's handle.
    InverterCondition(&'static str), // reverts on a condition, e.g. target SoC. Not time-bounded.
    RecurringWindow,                 // repeats daily. NOT a fail-safe.
    UntilChanged,                    // stands until overwritten. NOT a fail-safe.
}
```

Exactly one variant survives a dead controller, and there is a method for
asking:

```rust
let applied = inv.charge(2_000.0)?;
if !applied.expiry.is_dead_controller_safe() {
    // If this process dies now, the command outlives it.
    // Refuse, or arrange an external watchdog before continuing.
}
```

## Errors

```rust
pub enum Error {
    Comm(String),         // transport failed, or the reply was unusable
    Readback(String),     // a write read back a different value
    Range(String),        // value does not fit the target register
    Unsupported(String),  // this driver cannot do that on this hardware
}
```

Match on the variant when the reaction differs; all variants format into
log-ready messages via `Display`.

## Transports

Both transports implement the `modbus::ModbusBus` trait; drivers are written
against the trait and do not care which is in use.

```rust
use inverter::modbus::{SerialBus, TcpBus};

// Modbus RTU over a serial RS485 adapter. Use a stable by-id path -
// /dev/ttyUSB0 changes number across boots and when another adapter is
// present. Arguments: port, baud rate, unit (slave) id.
let bus = SerialBus::open("/dev/serial/by-id/usb-...", 9600, 247)?;

// Modbus TCP, for an RS485-to-network bridge (Elfin EW11 and similar)
// sitting beside the inverter. This puts the home network in the control
// path: a dropped connection is lost telemetry, not silent success.
let bus = TcpBus::connect("10.0.0.5:502", 247)?;
```

Reads retry with exponential backoff before reporting failure; short and
corrupt replies are errors, never zeroes.

## FoxESS H1 driver

Reads only, deliberately: the register maps are compiled from community
documentation and **have not been verified against hardware**, so
`capabilities()` reports `can_write: false` and every command returns
`Error::Unsupported`. Reads are exposed so the map *can* be checked.

The two H1 generations answer differently over the same RS485 wire, so the
driver takes a register map:

```rust
use inverter::foxess::{registers, FoxEss};

// Open over serial (port, baud, unit id, map)...
let inv = FoxEss::open_serial("/dev/serial/by-id/usb-...", 9600, 247, &registers::H1_G2)?;

// ...or through a TCP bridge...
let inv = FoxEss::open_tcp("10.0.0.5:502", 247, &registers::H1_G2)?;

// ...or wrap any ModbusBus you already have.
let inv = FoxEss::new(bus, &registers::H1_G1);
```

`registers::H1_G1` is the first generation (input registers);
`registers::H1_G2` covers H1-G2, AC1-G2 and P1 (holding registers). The
wrong map fails with an error rather than returning wrong numbers.

`registers::remote_control` documents the H1's write path as data — the
remote-enable, watchdog-timeout and power-setpoint registers a verified
write path will use. The watchdog is what will let this driver report
`Expiry::InverterTimeout`; see the module docs for the semantics and
sources.

Verifying a map means running read-only against a real inverter and
confirming every value against the inverter's own display across a range of
states — charging, discharging, idle, and at both ends of the
state-of-charge range. Contributions of verified maps are very welcome;
please say which model, firmware version and connection route you verified
against.

## The mock

A full simulation implementing `Inverter`, for tests and for running a
controller with no hardware attached. It models the *well-behaved* case:
commands carry a real one-shot timeout and the mock reverts to passive on
its own when it elapses.

```rust
use inverter::{Command, Inverter, InverterExt, Mode, mock::MockInverter};
use std::time::Duration;

let mut inv = MockInverter::new()        // 10 kWh, 5 kW, 50% SoC, 400 W load
    .with_capacity_wh(13_500.0)
    .with_max_power_w(3_680.0)
    .with_soc_pct(25.0)
    .with_load_w(600.0)
    .with_solar_w(1_200.0);

inv.charge(2_000.0)?;

// Drive simulated time from tests instead of sleeping.
inv.advance(Duration::from_secs(1800));
assert!(inv.read_telemetry()?.soc_pct > 25.0);

// The command currently in force, after any timeout has been applied.
assert_eq!(inv.active_command().mode, Mode::ForceCharge);
```

## Writing a new driver

Implement `Inverter` over any `ModbusBus`, keeping addresses as data in one
place so a map can be checked against hardware without reading driver code:

```rust
use inverter::modbus::{read_words, with_retries, ModbusBus};
use inverter::register::{decode, encode, RegisterDef};

// Addresses are data. Builders: .words(n), .scale(x), .signed().
const BATTERY_SOC: RegisterDef = RegisterDef::input("battery_soc", 33139);
const BATTERY_POWER: RegisterDef = RegisterDef::input("battery_power", 33149).words(2).signed();
const CHARGE_CURRENT: RegisterDef = RegisterDef::holding("charge_current", 43141).scale(0.1);

fn read(bus: &mut impl ModbusBus, reg: &RegisterDef) -> Result<f64, inverter::Error> {
    // Retries with backoff; a short reply is an error, not zeroes.
    with_retries(bus, "my.driver", &format!("read {}", reg.name), |bus| {
        read_words(bus, reg).map(|words| decode(reg, &words))
    })
}

fn write(bus: &mut impl ModbusBus, reg: &RegisterDef, value: f64) -> Result<(), inverter::Error> {
    // encode rejects NaN, infinities and anything that would wrap.
    bus.write_holding(reg.address, encode(reg, value)?)
}
```

A negative `scale` normalises an inverted sign convention at the data
level: `.signed().scale(-1.0)` turns a discharge-positive register into
this crate's charge-positive convention.

Implementors must report failures rather than plausible data, honour the
sign conventions above, and — for the write path — report the weakest
`Expiry` that is actually true of the hardware.

## Features

| Feature | Default | Effect |
|---|---|---|
| `serial` | yes | Modbus RTU over a serial port |
| `tcp` | yes | Modbus TCP, for a network bridge |
| `foxess` | yes | FoxESS H1-series driver |
| `mock` | yes | Simulated inverter |

The `ModbusBus` trait, register toolkit and core types are always
available; every feature combination builds and is tested in CI.

## Safety

This crate is a protocol layer. It does not implement command leases, power
limits, state-of-charge floors, export caps or watchdogs — those are policy,
they belong to the system that decides what to command, and they should not
be silently applied by a library. What this crate does guarantee is that it
will never quietly report a plausible number in place of a failed read.

Controlling grid-connected storage carries real risk. Verify your register
map before enabling writes.

## Licence

MIT. See [LICENSE](LICENSE).
