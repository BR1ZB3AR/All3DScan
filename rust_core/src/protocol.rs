//! Revopoint POP4 USB protocol implementation in Rust.

pub const POP4_VENDOR_ID: u16 = 0x2207;
pub const POP4_PRODUCT_ID: u16 = 0x110c;

pub const EXPOSURE_MIN: i32 = 500;
pub const EXPOSURE_MAX: i32 = 6000;
pub const EXPOSURE_DEFAULT: i32 = 3000;

pub const ILLUM_MIN: i32 = 0;
pub const ILLUM_MAX: i32 = 255;
pub const ILLUM_DEFAULT: i32 = 130;

pub const LASER_MIN: i32 = 0;
pub const LASER_MAX: i32 = 255;
pub const LASER_DEFAULT: i32 = 180;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanMode {
    StructuredLight,
    BlueLaserCross,
    BlueLaserSingleLine,
}

#[derive(Debug, Clone)]
pub struct Pop4Controls {
    pub exposure: i32,
    pub illumination_power: i32,
    pub laser_power: i32,
    pub mode: ScanMode,
    pub emitters_on: bool,
}

impl Default for Pop4Controls {
    fn default() -> Self {
        Self {
            exposure: EXPOSURE_DEFAULT,
            illumination_power: ILLUM_DEFAULT,
            laser_power: LASER_DEFAULT,
            mode: ScanMode::StructuredLight,
            emitters_on: false,
        }
    }
}

pub fn build_cs10_payload(emitters_on: bool) -> [u8; 16] {
    let mut payload = [0u8; 16];
    payload[0] = 0x01;
    payload[1] = if emitters_on { 0x01 } else { 0x00 };
    payload[2] = 0x00;
    payload[3] = 0x00;
    payload
}

pub fn build_cs7_payload(controls: &Pop4Controls) -> [u8; 32] {
    let mut payload = [0u8; 32];
    payload[0] = 0x07;
    payload[1] = 0x00;

    let exp = controls.exposure.clamp(EXPOSURE_MIN, EXPOSURE_MAX) as u32;
    payload[2] = (exp & 0xFF) as u8;
    payload[3] = ((exp >> 8) & 0xFF) as u8;
    payload[4] = ((exp >> 16) & 0xFF) as u8;
    payload[5] = ((exp >> 24) & 0xFF) as u8;

    let illum = controls.illumination_power.clamp(ILLUM_MIN, ILLUM_MAX) as u8;
    payload[6] = illum;

    let (cross_val, single_val) = match controls.mode {
        ScanMode::StructuredLight => (0u16, 0u16),
        ScanMode::BlueLaserCross => (
            ((controls.laser_power.clamp(0, 255) * 780) / 255) as u16,
            0u16,
        ),
        ScanMode::BlueLaserSingleLine => (
            0u16,
            ((controls.laser_power.clamp(0, 255) * 706) / 255) as u16,
        ),
    };

    payload[8] = (cross_val & 0xFF) as u8;
    payload[9] = ((cross_val >> 8) & 0xFF) as u8;

    payload[10] = (single_val & 0xFF) as u8;
    payload[11] = ((single_val >> 8) & 0xFF) as u8;

    payload
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cs10_payload() {
        let p_on = build_cs10_payload(true);
        assert_eq!(p_on[0], 0x01);
        assert_eq!(p_on[1], 0x01);

        let p_off = build_cs10_payload(false);
        assert_eq!(p_off[1], 0x00);
    }

    #[test]
    fn test_cs7_payload() {
        let mut ctrl = Pop4Controls::default();
        ctrl.exposure = 3000;
        ctrl.illumination_power = 130;
        ctrl.mode = ScanMode::BlueLaserCross;
        ctrl.laser_power = 255;

        let payload = build_cs7_payload(&ctrl);
        assert_eq!(payload[0], 0x07);
        assert_eq!(payload[6], 130);
        let cross = (payload[8] as u16) | ((payload[9] as u16) << 8);
        assert_eq!(cross, 780);
    }
}
