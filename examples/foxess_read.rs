//! Read telemetry from a FoxESS H1-series inverter over a serial RS485
//! adapter or a Modbus TCP bridge (Elfin EW11 and similar), using the
//! FoxESS defaults of 9600 baud and unit id 247.
//!
//! Usage:
//!   cargo run --example foxess_read -- serial /dev/serial/by-id/usb-... [g1|g2]
//!   cargo run --example foxess_read -- tcp 10.0.0.5:502 [g1|g2]
//!
//! The register maps are community-sourced and unverified. Running this
//! read-only check against a real unit — and comparing every number with the
//! inverter's own display — is exactly the verification the README asks for.

use inverter::foxess::{registers, FoxEss, RegisterMap};
use inverter::{Inverter, InverterExt};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let (transport, target) = match (args.next(), args.next()) {
        (Some(transport), Some(target)) => (transport, target),
        _ => usage(),
    };
    let map: &'static RegisterMap = match args.next().as_deref() {
        Some("g1") => &registers::H1_G1,
        Some("g2") | None => &registers::H1_G2,
        Some(_) => usage(),
    };

    let mut inverter: Box<dyn Inverter> = match transport.as_str() {
        "serial" => Box::new(FoxEss::open_serial(&target, 9600, 247, map)?),
        "tcp" => Box::new(FoxEss::open_tcp(&target, 247, map)?),
        _ => usage(),
    };

    println!("reading {} ...", inverter.capabilities().model);
    let t = inverter.read_telemetry()?;
    println!("battery: {:>6.1} %", t.soc_pct);
    println!("battery: {:>+6.2} kW  (positive = charging)", t.battery_kw);
    println!("grid:    {:>+6.2} kW  (positive = importing)", t.grid_kw);
    println!("load:    {:>6.2} kW", t.load_kw);
    println!("solar:   {:>6.2} kW", t.solar_kw);

    // Single-value sugar performs a full read; fine for a one-off check.
    println!("export:  {:>6.2} kW", inverter.get_export_kw()?);
    Ok(())
}

fn usage() -> ! {
    eprintln!("usage: foxess_read <serial PORT | tcp HOST:502> [g1|g2]");
    std::process::exit(2);
}
