# Changelog

All notable changes to this crate are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
