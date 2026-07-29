//! Modbus register definitions and value coding.
//!
//! Addresses are data, not logic. Each vendor module keeps its map in one
//! place so a map can be checked against hardware without reading driver code.

use crate::Error;

/// Which Modbus table a register lives in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegKind {
    /// Read-only input registers (function 4).
    Input,
    /// Read/write holding registers (function 3/6).
    Holding,
}

/// One register, described well enough to read, decode and write it.
#[derive(Debug, Clone, Copy)]
pub struct RegisterDef {
    /// Name used in logs and error messages.
    pub name: &'static str,
    /// Modbus address.
    pub address: u16,
    /// Which table it lives in.
    pub kind: RegKind,
    /// Number of consecutive 16-bit registers occupied.
    pub words: u8,
    /// Raw-to-engineering-unit multiplier.
    pub scale: f64,
    /// Whether the raw value is two's-complement signed.
    pub signed: bool,
}

impl RegisterDef {
    /// A single-word unsigned input register.
    pub const fn input(name: &'static str, address: u16) -> Self {
        Self {
            name,
            address,
            kind: RegKind::Input,
            words: 1,
            scale: 1.0,
            signed: false,
        }
    }

    /// A single-word unsigned holding register.
    pub const fn holding(name: &'static str, address: u16) -> Self {
        Self {
            name,
            address,
            kind: RegKind::Holding,
            words: 1,
            scale: 1.0,
            signed: false,
        }
    }

    /// Occupy `n` consecutive registers, most significant word first.
    pub const fn words(mut self, n: u8) -> Self {
        self.words = n;
        self
    }

    /// Apply a raw-to-engineering-unit multiplier.
    pub const fn scale(mut self, s: f64) -> Self {
        self.scale = s;
        self
    }

    /// Interpret the raw value as two's-complement signed.
    pub const fn signed(mut self) -> Self {
        self.signed = true;
        self
    }
}

/// Decode raw words into engineering units, most significant word first.
pub fn decode(reg: &RegisterDef, words: &[u16]) -> f64 {
    let mut raw: i64 = 0;
    for &w in words {
        raw = (raw << 16) | w as i64;
    }
    if reg.signed {
        let bits = 16 * reg.words as u32;
        if raw >= 1i64 << (bits - 1) {
            raw -= 1i64 << bits;
        }
    }
    raw as f64 * reg.scale
}

/// Encode an engineering value into a single raw word.
///
/// Rejects anything that would silently wrap: NaN, infinities, and values
/// outside the register's representable range. Writing a wrapped power
/// setpoint to an inverter is exactly the class of bug this prevents.
pub fn encode(reg: &RegisterDef, value: f64) -> Result<u16, Error> {
    if reg.words != 1 {
        return Err(Error::Range(format!(
            "{} spans {} words; multi-word writes are not supported",
            reg.name, reg.words
        )));
    }
    let raw = (value / reg.scale).round();
    if !raw.is_finite() {
        return Err(Error::Range(format!(
            "value {value} is not finite for register {}",
            reg.name
        )));
    }
    if reg.signed {
        if !(i16::MIN as f64..=i16::MAX as f64).contains(&raw) {
            return Err(Error::Range(format!(
                "value {value} out of range for signed register {} (raw {raw}, valid {}..={})",
                reg.name,
                i16::MIN,
                i16::MAX
            )));
        }
        Ok((raw as i16) as u16)
    } else {
        if !(0.0..=u16::MAX as f64).contains(&raw) {
            return Err(Error::Range(format!(
                "value {value} out of range for register {} (raw {raw}, valid 0..={})",
                reg.name,
                u16::MAX
            )));
        }
        Ok(raw as u16)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_a_scaled_unsigned_word() {
        let reg = RegisterDef::input("soc", 1).scale(0.1);
        assert!((decode(&reg, &[655]) - 65.5).abs() < 1e-9);
    }

    #[test]
    fn decodes_a_negative_two_word_value() {
        let reg = RegisterDef::input("power", 1).words(2).signed();
        // -1000 as a 32-bit two's-complement value.
        assert!((decode(&reg, &[0xFFFF, 0xFC18]) - -1000.0).abs() < 1e-9);
    }

    #[test]
    fn encodes_and_rejects_out_of_range_values() {
        let reg = RegisterDef::holding("current", 1).scale(0.1);
        assert_eq!(encode(&reg, 12.3).unwrap(), 123);
        assert!(encode(&reg, -1.0).is_err());
        assert!(encode(&reg, 1e9).is_err());
        assert!(encode(&reg, f64::NAN).is_err());
        assert!(encode(&reg, f64::INFINITY).is_err());
    }

    #[test]
    fn encodes_negative_values_only_for_signed_registers() {
        let signed = RegisterDef::holding("setpoint", 1).signed();
        assert_eq!(encode(&signed, -1000.0).unwrap(), (-1000i16) as u16);
        assert!(encode(&signed, 40_000.0).is_err());
    }

    #[test]
    fn refuses_to_encode_a_multi_word_register() {
        let wide = RegisterDef::holding("wide", 1).words(2);
        assert!(encode(&wide, 1.0).is_err());
    }

    #[test]
    fn constructors_set_the_register_table() {
        assert_eq!(RegisterDef::input("i", 1).kind, RegKind::Input);
        assert_eq!(RegisterDef::holding("h", 1).kind, RegKind::Holding);
    }

    #[test]
    fn a_negative_scale_normalises_an_inverted_sign_convention() {
        // How the FoxESS maps flip discharge-positive raw values.
        let reg = RegisterDef::input("battery", 1).signed().scale(-1.0);
        assert_eq!(decode(&reg, &[500]), -500.0);
        assert_eq!(decode(&reg, &[(-500i16) as u16]), 500.0);
    }

    #[test]
    fn decodes_the_full_unsigned_range_without_wrapping() {
        let one = RegisterDef::input("one", 1);
        assert_eq!(decode(&one, &[u16::MAX]), 65_535.0);
        let two = RegisterDef::input("two", 1).words(2);
        assert_eq!(decode(&two, &[u16::MAX, u16::MAX]), 4_294_967_295.0);
    }

    #[test]
    fn encode_rounds_to_the_nearest_raw_unit() {
        let reg = RegisterDef::holding("current", 1).scale(0.1);
        assert_eq!(encode(&reg, 12.34).unwrap(), 123);
        assert_eq!(encode(&reg, 12.38).unwrap(), 124);
    }

    #[test]
    fn encode_accepts_the_signed_boundaries_exactly() {
        let reg = RegisterDef::holding("setpoint", 1).signed();
        assert_eq!(encode(&reg, i16::MAX as f64).unwrap(), 0x7FFF);
        assert_eq!(encode(&reg, i16::MIN as f64).unwrap(), 0x8000);
        assert!(encode(&reg, i16::MAX as f64 + 1.0).is_err());
        assert!(encode(&reg, i16::MIN as f64 - 1.0).is_err());
    }

    #[test]
    fn decode_then_encode_round_trips_scaled_values() {
        let reg = RegisterDef::holding("current", 1).scale(0.1);
        for raw in [0u16, 1, 703, 65_535] {
            let value = decode(&reg, &[raw]);
            assert_eq!(encode(&reg, value).unwrap(), raw);
        }
    }
}
