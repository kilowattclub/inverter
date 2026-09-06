# FoxESS commissioning

`foxess-write` builds and runs a single H1 G2 command using this checkout's
driver, over RS485 at 9600 baud and Modbus unit 247. Stop other serial clients
(including `kilowatt-pi.service`) before running it.

```sh
# One 5 kW force-charge request for 60 seconds:
scripts/foxess-write /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG03R9NC-if00-port0 charge 5 60

# Return to self-use:
scripts/foxess-write /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG03R9NC-if00-port0 passive

# Deliberate export, only within the commissioned site export limit:
scripts/foxess-write /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG03R9NC-if00-port0 export 1 30
```

Requests are limited to 5 kW and 1–300 seconds. The tool reads the inverter's
minimum/maximum SoC settings, further restricts the test range to 15–95%, and
checks SoC before and during the override. It never changes those limits.
Power ratings and export permissions must be checked for the connected site.

The driver arms its hardware timeout once; the tool does not keep renewing it.
It prints measured power every two seconds and requests passive/self-use on
completion, telemetry failure, or SIGINT/SIGTERM. Serial operations can delay
software cleanup; SIGKILL or connection loss relies on the inverter timeout.
The timeout still needs verification on each supported model/firmware.

Acknowledgement confirms an inverter AC power setpoint. Battery power differs
because of conversion losses; whole-house grid import/export also includes
household load and any separate solar inverter. Compare the printed readings with the inverter screen. Returning to
passive explicitly selects self-use, rather than restoring a previous custom
work mode. This tool does not enable automatic control in `brain`.

Build without writing using `cargo build --release --example foxess_write`.
Run its regression tests using `cargo test --example foxess_write`.

The driver clears the old power setpoint before enabling, then sets and verifies
the TTL after enabling. H1 G2 resets TTL to 60 seconds during enable. Command
writes are separated by 30 ms; a timeout read-back mismatch aborts the override.

This tool explicitly returns to passive at the end of the requested duration;
that cleanup is not evidence of native watchdog expiry. The commissioning logs
in the parent workspace's `analysis/foxess-modes-2026-09-06` directory include
separate trials that observed expiry before issuing cleanup.

Run the full software suite without touching hardware:

```sh
cargo test --all-features
cargo test --all-features --example foxess_write
```
