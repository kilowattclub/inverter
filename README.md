# inverter

Control hybrid solar/battery inverters over Modbus, from Rust.

Home battery control in the open-source world lives almost entirely in Python,
inside Home Assistant. This crate is the Rust equivalent of the transport and
protocol layer — telemetry in, commands out — with one idea the existing
integrations do not model.

## The idea: how a command ends is part of its type

Ask an inverter to charge for twenty minutes. Two things can happen:

* it charges for twenty minutes and stops on its own, or
* you program a window that starts now and ends in twenty minutes — **and
  fires again tomorrow**, and every day after, whether or not your controller
  is still alive.

Most APIs make those the same call. They are not the same safety property.
The second is not a fail-safe: if the controller dies mid-command, a recurring
window keeps forcing someone's battery indefinitely.

So `apply()` returns what the inverter actually committed to:

```rust
pub enum Expiry {
    InverterTimeout(Duration),      // reverts by itself, once. A real dead-man's handle.
    InverterCondition(&'static str),// reverts on a condition, e.g. target SoC. Not time-bounded.
    RecurringWindow,                // repeats daily. NOT a fail-safe.
    UntilChanged,                   // stands until overwritten. NOT a fail-safe.
}
```

A caller that needs a genuine dead-controller guarantee checks
`expiry.is_dead_controller_safe()` and refuses to issue non-passive commands
when it is false, rather than discovering the difference in someone's garage.

## Capabilities before commands

Support varies by model *and* by how the inverter is connected — the same unit
over RS485 and over its own network module do not expose the same registers.
Ask first:

```rust
let caps = inverter.capabilities();
if caps.supports(Mode::ForceCharge) {
    inverter.apply(Command::charge(2_000.0))?;
}
```

A driver that cannot write says so, and says why, instead of failing at the
moment you needed it to work.

## Usage

```rust
use inverter::{Command, Inverter, mock::MockInverter};

let mut inv = MockInverter::new();
let telemetry = inv.read_telemetry()?;
println!("{}%, battery {:+.0} W", telemetry.soc_pct, telemetry.battery_w);

let applied = inv.apply(Command::charge(2_000.0))?;
assert!(applied.expiry.is_dead_controller_safe());
```

Transports:

```rust
use inverter::foxess::{registers, FoxEss};

// RS485 adapter wired to the inverter. Use a stable by-id path — /dev/ttyUSB0
// changes number across boots and when another adapter is present. The map
// must match the generation: a G2 serves different registers than a G1.
let inv = FoxEss::open_serial("/dev/serial/by-id/usb-...", 9600, 247, &registers::H1_G2)?;

// Or a network bridge (Elfin EW11 and similar) sitting beside the inverter,
// which lets the controller live somewhere with better signal.
let inv = FoxEss::open_tcp("10.0.0.5:502", 247, &registers::H1_G2)?;
```

## Sign conventions

Normalised across every driver, whatever the inverter reports natively:

| Field | Meaning |
|---|---|
| `battery_w > 0` | charging (power into the cells) |
| `grid_w > 0` | importing; `< 0` exporting |
| `load_w >= 0` | household consumption |
| `solar_w >= 0` | PV generation, `0.0` when unavailable |

## Supported hardware

| Driver | Reads | Writes | Notes |
|---|---|---|---|
| `mock` | ✅ | ✅ | Full simulation with a real one-shot timeout |
| `foxess` | ⚠️ unverified | ❌ | H1 G1 and G2 maps (RS485) from community documentation |

**FoxESS writes are deliberately not implemented.** The register map is
compiled from community documentation and has not been checked against
hardware. Reads are exposed so the map *can* be checked; writes stay closed
until it has been.

Verifying a map means running read-only against a real inverter and confirming
every value against the inverter's own display across a range of states —
charging, discharging, idle, and at both ends of the state-of-charge range.
Contributions of verified maps are very welcome; please say which model,
firmware version and connection route you verified against.

The H1's remote-control register block carries a genuine **watchdog**: a
timeout register the inverter counts down on its own, reverting to its
programmed work mode when it expires. That means a verified write path can
offer `Expiry::InverterTimeout` — a dead controller is safe by construction.
The block's addresses and semantics are recorded as data in
`foxess::registers::remote_control`, sourced from the
[`nathanmarlor/foxess_modbus`](https://github.com/nathanmarlor/foxess_modbus)
integration (MIT); they remain unverified on hardware, so writes stay closed.

## Features

| Feature | Default | Effect |
|---|---|---|
| `serial` | yes | Modbus RTU over a serial port |
| `tcp` | yes | Modbus TCP, for a network bridge |
| `foxess` | yes | FoxESS H1-series driver |
| `mock` | yes | Simulated inverter |

The crate is synchronous by design: Modbus over a serial line does not benefit
from async, and a blocking API keeps a runtime out of your dependency tree.

## Safety

This crate is a protocol layer. It does not implement command leases, power
limits, state-of-charge floors, export caps or watchdogs — those are policy,
they belong to the system that decides what to command, and they should not be
silently applied by a library. What this crate does guarantee is that it will
never quietly report a plausible number in place of a failed read.

Controlling grid-connected storage carries real risk. Verify your register map
before enabling writes.

## Licence

MIT. See [LICENSE](LICENSE).
