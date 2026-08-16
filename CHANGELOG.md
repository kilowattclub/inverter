# Changelog

All notable changes to this crate are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.3.0] - 2026-08-16

### Changed

- Non-passive command constructors and `InverterExt` methods now require an
  explicit `Duration` TTL; the implicit five-minute default and
  `holding_for` override were removed.
- Writable drivers must arm a one-shot timeout in the inverter for every
  non-passive command, replace the previous timeout when applying a new
  command, and cancel it on passive. Drivers that cannot provide those
  guarantees must refuse the override without changing inverter state.
- Process lifecycle cleanup remains the application's responsibility; the
  inverter-side timeout is the fail-safe that survives controller failure.

### Fixed

- A zero-length inverter timeout is no longer considered dead-controller-safe.

## [0.2.0] - 2026-08-02

### Changed

- The single-value `InverterExt` reads are now spelled `get_*`
  (`get_soc_pct`, `get_battery_kw`, `get_grid_kw`, `get_load_kw`,
  `get_solar_kw`, `get_export_kw`), and `get_mode` joins them as the sugar
  spelling of `Inverter::mode`. The prefix marks the cost: `get_*` performs
  a hardware read, while the same-named accessors on `Telemetry` are free
  field reads.

### Fixed

- docs.rs failed to build 0.1.0: the `doc_auto_cfg` nightly feature was
  removed in Rust 1.92 (merged into `doc_cfg`), and the attribute is only
  active under docs.rs's `--cfg docsrs`, so no other build saw it. The gate
  now uses `doc_cfg`.

## [0.1.0] - 2026-08-02

The initial release.

### Added

- The `Inverter` trait — `capabilities`, `read_telemetry`, `apply`, `mode` —
  with `InverterExt` conveniences: `charge`/`discharge`/`export`/`passive`
  and single-field telemetry reads.
- Honest command-expiry semantics: every accepted command reports how it
  will end (`Expiry`), and `Expiry::is_dead_controller_safe` separates a real
  one-shot inverter timeout from schedules and standing writes.
- Capability discovery before commanding: `Capabilities` with
  `read_only`/`writable` constructors, per-mode support checks, and a stated
  reason whenever writes are unavailable.
- Modbus transports behind the `serial` (RTU) and `tcp` (bridge) features,
  with retries, incremental frame reassembly, and short replies treated as
  errors rather than decoded as zeroes.
- A read-only FoxESS H1 G1/G2 driver (`foxess` feature) built on community
  register maps, and the H1 remote-control watchdog block recorded as data
  for a future verified write path.
- `MockInverter` (`mock` feature): a simulated battery with a genuine
  one-shot command timeout, driveable in simulated time through `advance`.
