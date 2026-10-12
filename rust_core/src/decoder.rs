//! Depth map decoding and calibration validation in Rust.

pub const POP4_WIDTH: usize = 640;
pub const POP4_HEIGHT: usize = 512;

#[derive(Debug, Clone)]
pub struct DepthFrameInfo {
    pub width: usize,
    pub height: usize,
    pub valid_points: usize,
    pub min_depth_mm: f32,
    pub max_depth_mm: f32,
}

pub fn analyze_y16_depth_frame(raw_data: &[u8], depth_scale_mm: f32) -> DepthFrameInfo {
    let sample_count = raw_data.len() / 2;
    let mut valid_points = 0;
    let mut min_depth = f32::MAX;
    let mut max_depth = f32::MIN;

    for i in 0..sample_count {
        let val = (raw_data[i * 2] as u16) | ((raw_data[i * 2 + 1] as u16) << 8);
        if val > 0 && val < 65535 {
            valid_points += 1;
            let depth_mm = (val as f32) * depth_scale_mm;
            if depth_mm < min_depth {
                min_depth = depth_mm;
            }
            if depth_mm > max_depth {
                max_depth = depth_mm;
            }
        }
    }

    if valid_points == 0 {
        min_depth = 0.0;
        max_depth = 0.0;
    }

    DepthFrameInfo {
        width: POP4_WIDTH,
        height: sample_count / POP4_WIDTH,
        valid_points,
        min_depth_mm: min_depth,
        max_depth_mm: max_depth,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_y16_depth_analysis() {
        let mut mock_frame = vec![0u8; POP4_WIDTH * POP4_HEIGHT * 2];
        // Insert mock sample value 1000 at pixel 0
        mock_frame[0] = (1000 & 0xFF) as u8;
        mock_frame[1] = ((1000 >> 8) & 0xFF) as u8;

        let info = analyze_y16_depth_frame(&mock_frame, 0.1);
        assert_eq!(info.valid_points, 1);
        assert!((info.min_depth_mm - 100.0).abs() < 1e-3);
    }
}
