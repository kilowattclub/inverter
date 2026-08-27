//! Configuration-driven inverter construction.

use crate::{Error, Inverter};

/// Mock telemetry and physical limits used when opening a mock driver.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MockOptions {
    /// Usable battery capacity in kilowatt-hours.
    pub capacity_kwh: f64,
    /// Maximum charge or discharge power in kilowatts.
    pub max_power_kw: f64,
    /// Initial battery state of charge in percent.
    pub soc_pct: f64,
    /// Simulated household load in kilowatts.
    pub load_kw: f64,
    /// Simulated PV generation in kilowatts.
    pub solar_kw: f64,
}

impl Default for MockOptions {
    fn default() -> Self {
        Self {
            capacity_kwh: 10.0,
            max_power_kw: 5.0,
            soc_pct: 50.0,
            load_kw: 0.4,
            solar_kw: 0.0,
        }
    }
}

/// Driver selection and connection settings for [`open`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpenOptions<'a> {
    /// Driver identifier: `mock`, `mock-relay` or `foxess`.
    pub kind: &'a str,
    /// Serial device path used by hardware-backed drivers.
    pub serial_port: &'a str,
    /// Serial baud rate used by hardware-backed drivers.
    pub baud_rate: u32,
    /// Modbus unit/slave identifier. FoxESS uses 247 when this is zero.
    pub unit_id: u8,
    /// Settings used by `mock` and `mock-relay`.
    pub mock: MockOptions,
}

/// Open the selected inverter without substituting a mock after failure.
pub fn open(options: OpenOptions<'_>) -> Result<Box<dyn Inverter>, Error> {
    match options.kind {
        "mock" => open_mock(options.mock),
        "mock-relay" => open_mock_relay(options),
        "foxess" => open_foxess(options),
        other => Err(Error::Unsupported(format!(
            "unknown inverter type {other:?}; expected mock, mock-relay or foxess"
        ))),
    }
}

#[cfg(feature = "mock")]
fn mock(options: MockOptions) -> crate::mock::MockInverter {
    crate::mock::MockInverter::new()
        .with_capacity_kwh(options.capacity_kwh)
        .with_max_power_kw(options.max_power_kw)
        .with_soc_pct(options.soc_pct)
        .with_load_kw(options.load_kw)
        .with_solar_kw(options.solar_kw)
}

#[cfg(feature = "mock")]
fn open_mock(options: MockOptions) -> Result<Box<dyn Inverter>, Error> {
    Ok(Box::new(mock(options)))
}

#[cfg(not(feature = "mock"))]
fn open_mock(_options: MockOptions) -> Result<Box<dyn Inverter>, Error> {
    Err(Error::Unsupported(
        "the mock driver requires the mock feature".into(),
    ))
}

#[cfg(all(feature = "mock", feature = "serial"))]
fn open_mock_relay(options: OpenOptions<'_>) -> Result<Box<dyn Inverter>, Error> {
    mock(options.mock)
        .with_waveshare_relay(options.serial_port, options.baud_rate, options.unit_id)
        .map(|inverter| Box::new(inverter) as Box<dyn Inverter>)
}

#[cfg(not(all(feature = "mock", feature = "serial")))]
fn open_mock_relay(_options: OpenOptions<'_>) -> Result<Box<dyn Inverter>, Error> {
    Err(Error::Unsupported(
        "the mock-relay driver requires the mock and serial features".into(),
    ))
}

#[cfg(all(feature = "foxess", feature = "serial"))]
fn open_foxess(options: OpenOptions<'_>) -> Result<Box<dyn Inverter>, Error> {
    let unit_id = if options.unit_id == 0 {
        247
    } else {
        options.unit_id
    };
    crate::foxess::FoxEss::open_serial(
        options.serial_port,
        options.baud_rate,
        unit_id,
        &crate::foxess::registers::H1_G2,
    )
    .map(|inverter| Box::new(inverter) as Box<dyn Inverter>)
}

#[cfg(not(all(feature = "foxess", feature = "serial")))]
fn open_foxess(_options: OpenOptions<'_>) -> Result<Box<dyn Inverter>, Error> {
    Err(Error::Unsupported(
        "the FoxESS serial driver requires the foxess and serial features".into(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn options(kind: &str) -> OpenOptions<'_> {
        OpenOptions {
            kind,
            serial_port: "/dev/null",
            baud_rate: 9_600,
            unit_id: 1,
            mock: MockOptions {
                capacity_kwh: 12.0,
                max_power_kw: 4.0,
                soc_pct: 60.0,
                load_kw: 0.7,
                solar_kw: 0.2,
            },
        }
    }

    #[test]
    fn unknown_driver_is_rejected() {
        let error = open(options("warpdrive")).err().unwrap();
        assert!(error.to_string().contains("unknown inverter type"));
    }

    #[cfg(feature = "mock")]
    #[test]
    fn mock_uses_the_supplied_telemetry() {
        let mut inverter = open(options("mock")).unwrap();
        let telemetry = inverter.read_telemetry().unwrap();

        assert!((telemetry.soc_pct - 60.0).abs() < 1e-5);
        assert_eq!(telemetry.load_kw, 0.7);
        assert_eq!(telemetry.solar_kw, 0.2);
    }

    #[cfg(not(feature = "mock"))]
    #[test]
    fn mock_reports_its_missing_feature() {
        let error = open(options("mock")).err().unwrap();
        assert!(error.to_string().contains("requires the mock feature"));
    }
}
