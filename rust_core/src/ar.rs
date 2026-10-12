//! Rust-accelerated AR (Augmented Reality) camera projection & spatial anchor module.

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ArIntrinsics {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    pub width: u32,
    pub height: u32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ArPoint2D {
    pub u: f32,
    pub v: f32,
    pub depth_m: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ArPoint3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

/// Project 3D point cloud into 2D AR camera viewport overlay coordinates.
pub fn project_points_to_ar(
    points: &[ArPoint3D],
    intrinsics: &ArIntrinsics,
    out_points: &mut Vec<ArPoint2D>,
) {
    out_points.clear();
    if intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0 || points.is_empty() {
        return;
    }
    out_points.reserve(points.len());

    for p in points {
        if p.z <= 0.05 {
            continue; // Behind camera or too close
        }
        let u = intrinsics.fx * (p.x / p.z) + intrinsics.cx;
        let v = intrinsics.fy * (p.y / p.z) + intrinsics.cy;

        if u >= 0.0 && u < (intrinsics.width as f32) && v >= 0.0 && v < (intrinsics.height as f32) {
            out_points.push(ArPoint2D {
                u,
                v,
                depth_m: p.z,
                r: p.r,
                g: p.g,
                b: p.b,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ar_projection() {
        let intrinsics = ArIntrinsics {
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
            width: 640,
            height: 480,
        };

        let pts = vec![
            ArPoint3D { x: 0.0, y: 0.0, z: 1.0, r: 255, g: 0, b: 0 },
            ArPoint3D { x: 0.1, y: 0.1, z: 1.0, r: 0, g: 255, b: 0 },
        ];

        let mut out = Vec::new();
        project_points_to_ar(&pts, &intrinsics, &mut out);

        assert_eq!(out.len(), 2);
        assert!((out[0].u - 320.0).abs() < 1e-3);
        assert!((out[0].v - 240.0).abs() < 1e-3);
        assert!((out[1].u - 370.0).abs() < 1e-3);
        assert!((out[1].v - 290.0).abs() < 1e-3);
    }
}
