# FoxESS commissioning

`foxess-write` sends one H1 G2 command over RS485 at 9600 baud, unit 247.
Stop other serial clients, including `kilowatt-pi.service`, before running it.

```sh
scripts/foxess-write /dev/serial/by-id/usb-... charge 5 60
scripts/foxess-write /dev/serial/by-id/usb-... export 1 30
scripts/foxess-write /dev/serial/by-id/usb-... passive
```

Limits: 5 kW, 1–300 seconds, and the intersection of 15–95% SoC with the
inverter's configured SoC range. The tool reads but does not change those
settings. Check the site's power rating and export limit before use.

The tool sends one override, reads telemetry every two seconds, and checks SoC
before and during the test. Completion, telemetry failure or SIGINT/SIGTERM
triggers a passive command. Serial operations can delay cleanup; controller
failure relies on the inverter watchdog.

Commands set inverter AC power. Battery losses, household load and separate
solar generation affect the reported battery/grid readings. Compare them with
the inverter display. Passive selects self-use; it does not restore a previous
custom work mode.

The tool requests passive at the end of the test, so it does not independently
verify native watchdog expiry. See the [driver notes](../README.md#foxess) for
command sequencing and model limitations.

Build and test without connecting to hardware:

```sh
cargo build --release --example foxess_write
cargo test --all-features
cargo test --all-features --example foxess_write
```
