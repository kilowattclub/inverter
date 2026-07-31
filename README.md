# inverter

[![CI](https://github.com/kilowattclub/inverter/actions/workflows/ci.yml/badge.svg)](https://github.com/kilowattclub/inverter/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/inverter.svg)](https://crates.io/crates/inverter)
[![docs.rs](https://img.shields.io/docsrs/inverter)](https://docs.rs/inverter)

Control hybrid solar/battery inverters over Modbus, from Rust.

You open a driver, read telemetry from it, and call command methods on it.
Every accepted command tells you **how it will end** — because a command the
inverter reverts by itself is a fail-safe, and a daily schedule that happens
to end at the same time is not.

## Usage

```sh
cargo add inverter
```

```rust
use inverter::{Inverter, InverterExt, mock::MockInverter};

// 1. Open a driver. The mock needs no hardware:
let mut inverter = MockInverter::new();

// 2. Read telemetry — everything at once, or single values:
let t = inverter.read_telemetry()?;
println!("{:.0}%  battery {:+.2} kW  grid {:+.2} kW", t.soc_pct, t.battery_kw, t.grid_kw);

let soc = inverter.soc_pct()?;    // one field, one call
let mode = inverter.mode()?;      // the Mode currently in force

// 3. Command it. Powers are kilowatts; int or float both work:
inverter.charge(2)?;        // charge at 2 kW, importing if needed
inverter.discharge(1.5)?;   // cover household load only; no export
inverter.export(3)?;        // deliberately export to the grid
inverter.passive()?;        // back to the inverter's own self-use
```

Real hardware instead of the mock:

```rust
use inverter::foxess::{registers, FoxEss};

// RS485 adapter (use a stable by-id path, not /dev/ttyUSB0):
let mut inverter = FoxEss::open_serial("/dev/serial/by-id/usb-...", 9600, 247, &registers::H1_G2)?;

// or through an RS485-to-network bridge (Elfin EW11 and similar):
let mut inverter = FoxEss::open_tcp("10.0.0.5:502", 247, &registers::H1_G2)?;
```

That's the whole model. Full API reference: [docs.rs/inverter](https://docs.rs/inverter).
Runnable walkthrough: `cargo run --example tour`.

## What each command does

| Command | The inverter... |
|---|---|
| `passive()` | runs its **own self-use logic**, exactly as if no controller were attached: solar powers the house, surplus charges the battery then exports, and after dark the battery covers the house down to its minimum SoC. Vendors call this "self-use" or "self-consumption". |
| `charge(kw)` | **forces energy into the battery** at `kw`, importing from the grid when solar can't cover it — how a controller buys a cheap tariff window. |
| `discharge(kw)` | **forces energy out of the battery** at `kw`, but only to cover the household load — nothing is pushed past the meter. |
| `export(kw)` | **forces energy out of the battery and past the meter** at `kw`, deliberately exporting — for things like grid-services events. |

Passive is the safe floor: it has no power level and nothing to expire, so
it is always safe to command, and it is what a dead controller's hardware
should decay to. The three overrides are the commands that need the expiry
semantics below.

## How a command ends

Every command method returns what the inverter actually committed to:

```rust
let applied = inverter.charge(2)?;

applied.power_kw;  // possibly clamped by the hardware
applied.expiry;    // how this command ends — the crate's reason to exist
```

```rust
pub enum Expiry {
    InverterTimeout(Duration),       // reverts by itself, once. A real dead-man's handle.
    InverterCondition(&'static str), // reverts on a condition, e.g. target SoC. Not time-bounded.
    RecurringWindow,                 // repeats daily. NOT a fail-safe.
    UntilChanged,                    // stands until overwritten. NOT a fail-safe.
}
```

If your controller could die mid-command, check before commanding:

```rust
if !inverter.capabilities().expiry.is_dead_controller_safe() {
    // A dead controller would leave this command standing. Don't issue it,
    // or arrange an external watchdog first.
}
```

## Check capabilities before commanding

Support varies by model *and* connection route. A driver that cannot write
says so up front, with a reason, instead of failing when you needed it:

```rust
use inverter::Mode;

let caps = inverter.capabilities();
if caps.supports(Mode::ForceCharge) {
    inverter.charge(2)?;
} else {
    println!("no writes: {}", caps.write_blocked_reason.unwrap_or("unsupported"));
}
```

## Units and sign conventions

All powers are **kilowatts** (energies kilowatt-hours), and the same signs
come from every driver, whatever the inverter's native convention:

| Field | Meaning |
|---|---|
| `battery_kw > 0` | charging (power into the cells) |
| `grid_kw > 0` | importing; `< 0` exporting |
| `load_kw >= 0` | household consumption |
| `solar_kw >= 0` | PV generation, `0.0` when unavailable |

`t.export_kw()` gives grid export as a positive number; `t.age()` is a
monotonic staleness check that NTP steps cannot corrupt.

For a one-off value there are single-field methods — sugar over
`read_telemetry`, so each call performs a full read; batch with
`read_telemetry` when you need several:

```rust
let soc = inverter.soc_pct()?;
let export = inverter.export_kw()?;
```

`inverter.mode()?` asks which `Mode` is currently in force. A driver that
cannot read that back from the hardware errors instead of repeating its last
command — a stale answer would hide an expired or externally-changed
command — and `capabilities().reports_mode` says up front whether it can
answer. The mock answers exactly (it simulates the timeout); FoxESS is
`Unsupported` until its remote-control registers are verified readable.

## Commands as data

The methods above are sugar over one underlying operation,
`apply(Command)`. Build `Command` values directly when commands come from a
planner or pass through a safety layer before reaching hardware — they can
be stored, compared, logged and applied later:

```rust
use inverter::Command;
use std::time::Duration;

let cmd = Command::charge(2).holding_for(Duration::from_secs(60));
inverter.apply(cmd)?;
```

Both spellings reach hardware through `apply`; drivers cannot make them
diverge.

## Drivers

| Driver | Reads | Writes | Notes |
|---|---|---|---|
| `mock` | ✅ | ✅ | Full simulation with a real one-shot timeout; builders for capacity, SoC, load, solar |
| `foxess` | ⚠️ unverified | ❌ | H1 G1 (`registers::H1_G1`) and G2 (`registers::H1_G2`) over RS485 |

**FoxESS writes are deliberately not implemented.** The register maps come
from community documentation and have not been checked against hardware;
reads are exposed so the maps *can* be checked. The H1's remote-control
watchdog — recorded as data in `foxess::registers::remote_control` — is what
will let a verified write path report `Expiry::InverterTimeout`. Verified
maps are very welcome: run read-only against a real inverter, confirm every
value against its display across charging/discharging/idle, and say which
model, firmware and connection route you tested.

New drivers implement the `Inverter` trait over any `modbus::ModbusBus`,
with addresses kept as data via `register::RegisterDef` — see the
`register` and `modbus` module docs.

## Features

`serial`, `tcp`, `foxess`, `mock` — all on by default, every combination
builds and is tested in CI. The crate is synchronous by design: Modbus over
a serial line does not benefit from async, and a blocking API keeps a
runtime out of your dependency tree.

The minimum supported Rust version is **1.85**, checked in CI; it is set by
the serial stack's dependencies, not by this crate's own code.

## Safety

This is a protocol layer. Leases, power caps, SoC floors, export caps and
watchdogs are policy — they belong to the system deciding what to command,
not silently inside a library. What this crate guarantees: it never quietly
reports a plausible number in place of a failed read, and it never claims a
stronger expiry than the hardware provides. Verify your register map before
enabling writes.

## Licence

MIT. See [LICENSE](LICENSE).
