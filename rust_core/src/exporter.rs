//! Safe, high-performance metric 3D point cloud & mesh export module in Rust.

use std::fs::File;
use std::io::{BufWriter, Write};

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

pub fn export_ply_ascii(points: &[Point3D], path: &str) -> Result<(), std::io::Error> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    writeln!(writer, "ply")?;
    writeln!(writer, "format ascii 1.0")?;
    writeln!(writer, "comment Created by All3DScan Rust Core")?;
    writeln!(writer, "element vertex {}", points.len())?;
    writeln!(writer, "property float x")?;
    writeln!(writer, "property float y")?;
    writeln!(writer, "property float z")?;
    writeln!(writer, "property uchar red")?;
    writeln!(writer, "property uchar green")?;
    writeln!(writer, "property uchar blue")?;
    writeln!(writer, "end_header")?;

    for p in points {
        writeln!(writer, "{:.4} {:.4} {:.4} {} {} {}", p.x, p.y, p.z, p.r, p.g, p.b)?;
    }

    writer.flush()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn test_ply_export() {
        let points = vec![
            Point3D { x: 1.0, y: 2.0, z: 3.0, r: 255, g: 0, b: 0 },
            Point3D { x: 4.0, y: 5.0, z: 6.0, r: 0, g: 255, b: 0 },
        ];
        let test_path = "/tmp/test_rust_export.ply";
        let res = export_ply_ascii(&points, test_path);
        assert!(res.is_ok());
        let contents = fs::read_to_string(test_path).unwrap();
        assert!(contents.contains("element vertex 2"));
        assert!(contents.contains("1.0000 2.0000 3.0000 255 0 0"));
        let _ = fs::remove_file(test_path);
    }
}
