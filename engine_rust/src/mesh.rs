//! Turning a scan's depth views into one closed surface, bridged with cxx.
//!
//! The reconstruction is the Qt app's own (src/geometry/TsdfVolume), wrapped
//! by mesh_cpp: the views are melted into a truncated signed distance field
//! and the surface drawn through it, so holes between views close over and
//! the noise a point cloud shows as fur averages out.

use nalgebra::Point3;

// The bridge mirrors the C++ signature, which takes the camera's four
// numbers separately rather than an array cxx cannot carry.
#[allow(clippy::too_many_arguments)]
#[cxx::bridge(namespace = "all3dscan::mesh")]
mod ffi {
    unsafe extern "C++" {
        include!("all3dscan/SurfaceBuilder.h");

        type SurfaceBuilder;

        #[cxx_name = "makeSurfaceBuilder"]
        fn make_surface_builder(voxel_mm: f32) -> Result<UniquePtr<SurfaceBuilder>>;

        /// # Safety
        /// `depth` must point to `depth_count` values and `pose` to 16, both
        /// readable for the length of the call.
        #[cxx_name = "addView"]
        unsafe fn add_view(
            self: Pin<&mut SurfaceBuilder>,
            depth: *const u16,
            depth_count: u64,
            width: u32,
            height: u32,
            fx: f64,
            fy: f64,
            cx: f64,
            cy: f64,
            pose: *const f64,
        ) -> Result<()>;

        /// # Safety
        /// `xyz` must point to `points * 3` floats, readable for the call.
        unsafe fn build(
            self: Pin<&mut SurfaceBuilder>,
            xyz: *const f32,
            points: u64,
            smoothing: i32,
        ) -> Result<bool>;

        fn vertices(self: &SurfaceBuilder) -> &CxxVector<f32>;
        fn normals(self: &SurfaceBuilder) -> &CxxVector<f32>;
        fn indices(self: &SurfaceBuilder) -> &CxxVector<u32>;
        fn triangles(self: &SurfaceBuilder) -> u64;
    }
}

// SAFETY: a SurfaceBuilder is the depth views it was given and the field
// built from them, neither tied to the thread that made them. Calls go
// through & and &mut, so they never overlap.
unsafe impl Send for ffi::SurfaceBuilder {}

/// A closed surface: millimetres, in the axes of the scan it came from.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Surface {
    pub vertices: Vec<Point3<f32>>,
    /// A unit normal per vertex, taken from the triangles sharing it.
    pub normals: Vec<f32>,
    /// Three indices into `vertices` per triangle.
    pub indices: Vec<u32>,
}

impl Surface {
    pub fn triangles(&self) -> usize {
        self.indices.len() / 3
    }

    pub fn is_empty(&self) -> bool {
        self.indices.len() < 3
    }
}

/// Collects a scan's depth views and builds a surface from them.
pub struct SurfaceBuilder {
    inner: cxx::UniquePtr<ffi::SurfaceBuilder>,
}

impl SurfaceBuilder {
    /// A builder whose surface holds detail down to `voxel_mm`. Errors
    /// unless that is between 0.2 and 20 mm.
    pub fn new(voxel_mm: f32) -> Result<Self, cxx::Exception> {
        Ok(Self {
            inner: ffi::make_surface_builder(voxel_mm)?,
        })
    }

    /// Keeps one depth view and the pose it was taken from: a row-major 4x4
    /// taking that camera's points into the model's axes, in millimetres.
    pub fn add_view(
        &mut self,
        depth_mm: &[u16],
        width: u32,
        height: u32,
        intrinsics: [f64; 4],
        pose: &[f64; 16],
    ) -> Result<(), cxx::Exception> {
        let [fx, fy, cx, cy] = intrinsics;
        // SAFETY: both pointers come from slices whose lengths are passed
        // with them, and C++ only reads them during the call.
        unsafe {
            self.inner.pin_mut().add_view(
                depth_mm.as_ptr(),
                depth_mm.len() as u64,
                width,
                height,
                fx,
                fy,
                cx,
                cy,
                pose.as_ptr(),
            )
        }
    }

    /// Builds the surface over the box `points` covers, which is the scan's
    /// own cloud. An empty surface comes back when the views made none.
    pub fn build(
        &mut self,
        points: &[Point3<f32>],
        smoothing: i32,
    ) -> Result<Surface, cxx::Exception> {
        // SAFETY: nalgebra points are three floats each, laid out in order,
        // and C++ only reads them during the call.
        let made = unsafe {
            self.inner.pin_mut().build(
                points.as_ptr().cast::<f32>(),
                points.len() as u64,
                smoothing,
            )?
        };
        if !made {
            return Ok(Surface::default());
        }
        let (xyz, rest) = self.inner.vertices().as_slice().as_chunks::<3>();
        debug_assert!(rest.is_empty(), "C++ vertex buffer is not x, y, z triples");
        Ok(Surface {
            vertices: xyz.iter().map(|&[x, y, z]| Point3::new(x, y, z)).collect(),
            normals: self.inner.normals().as_slice().to_vec(),
            indices: self.inner.indices().as_slice().to_vec(),
        })
    }

    pub fn triangles(&self) -> u64 {
        self.inner.triangles()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Matrix3, Vector3};

    const IDENTITY: [f64; 16] = [
        1.0, 0.0, 0.0, 0.0, //
        0.0, 1.0, 0.0, 0.0, //
        0.0, 0.0, 1.0, 0.0, //
        0.0, 0.0, 0.0, 1.0,
    ];

    /// A depth image of a ball, as a camera at the origin would see it, and
    /// the points it back-projects to. Millimetres.
    fn ball(
        width: u32,
        height: u32,
        radius: f32,
        at: Vector3<f32>,
    ) -> (Vec<u16>, Vec<Point3<f32>>) {
        let (fx, fy) = (0.9 * width as f32, 0.9 * width as f32);
        let (cx, cy) = (0.5 * (width - 1) as f32, 0.5 * (height - 1) as f32);
        let mut depth = vec![0u16; (width * height) as usize];
        let mut points = Vec::new();
        for v in 0..height {
            for u in 0..width {
                let dx = (u as f32 - cx) / fx;
                let dy = (v as f32 - cy) / fy;
                // Where the ray meets the ball, nearest side.
                let a = dx * dx + dy * dy + 1.0;
                let b = -2.0 * (dx * at.x + dy * at.y + at.z);
                let c = at.norm_squared() - radius * radius;
                let disc = b * b - 4.0 * a * c;
                if disc < 0.0 {
                    continue;
                }
                let t = (-b - disc.sqrt()) / (2.0 * a);
                if t <= 0.0 {
                    continue;
                }
                depth[(v * width + u) as usize] = t.round() as u16;
                points.push(Point3::new(dx * t, dy * t, t));
            }
        }
        (depth, points)
    }

    fn intrinsics(width: u32, height: u32) -> [f64; 4] {
        [
            0.9 * width as f64,
            0.9 * width as f64,
            0.5 * (width - 1) as f64,
            0.5 * (height - 1) as f64,
        ]
    }

    #[test]
    fn a_ball_seen_once_makes_a_surface_the_right_size() {
        let (width, height) = (160u32, 160u32);
        let (depth, points) = ball(width, height, 40.0, Vector3::new(0.0, 0.0, 300.0));
        let mut builder = SurfaceBuilder::new(2.0).unwrap();
        builder
            .add_view(&depth, width, height, intrinsics(width, height), &IDENTITY)
            .unwrap();
        let surface = builder.build(&points, 0).unwrap();

        assert!(!surface.is_empty(), "no surface came back");
        assert_eq!(surface.normals.len(), surface.vertices.len() * 3);
        assert!(
            surface
                .indices
                .iter()
                .all(|&i| (i as usize) < surface.vertices.len())
        );
        // The front of a 40 mm ball 300 mm away: the surface should span
        // about its width and sit at its distance.
        let (lo, hi) = surface.vertices.iter().fold(
            (Point3::from([f32::MAX; 3]), Point3::from([f32::MIN; 3])),
            |(lo, hi), p| (lo.inf(p), hi.sup(p)),
        );
        assert!((hi.x - lo.x - 80.0).abs() < 12.0, "{} wide", hi.x - lo.x);
        assert!((lo.z - 260.0).abs() < 12.0, "nearest at {}", lo.z);
    }

    #[test]
    fn the_normals_are_unit_length_and_point_outwards() {
        let (width, height) = (160u32, 160u32);
        let centre = Vector3::new(0.0, 0.0, 300.0);
        let (depth, points) = ball(width, height, 40.0, centre);
        let mut builder = SurfaceBuilder::new(2.0).unwrap();
        builder
            .add_view(&depth, width, height, intrinsics(width, height), &IDENTITY)
            .unwrap();
        let surface = builder.build(&points, 0).unwrap();

        let (normals, _) = surface.normals.as_chunks::<3>();
        assert!(
            normals
                .iter()
                .all(|n| { (Vector3::new(n[0], n[1], n[2]).norm() - 1.0).abs() < 1e-3 })
        );
        // On the front of a ball, a normal should lean away from its centre,
        // which from this side means towards the camera.
        let facing = surface
            .vertices
            .iter()
            .zip(normals)
            .filter(|(p, _)| p.z < centre.z - 20.0)
            .filter(|(p, n)| {
                let out = (p.coords - centre).normalize();
                out.dot(&Vector3::new(n[0], n[1], n[2])) > 0.5
            })
            .count();
        let front = surface
            .vertices
            .iter()
            .filter(|p| p.z < centre.z - 20.0)
            .count();
        assert!(facing * 10 > front * 9, "{facing} of {front} face outwards");
    }

    #[test]
    fn smoothing_does_not_move_the_surface_off_the_object() {
        let (width, height) = (160u32, 160u32);
        let (depth, points) = ball(width, height, 40.0, Vector3::new(0.0, 0.0, 300.0));
        let mut rough = SurfaceBuilder::new(2.0).unwrap();
        let mut smooth = SurfaceBuilder::new(2.0).unwrap();
        for builder in [&mut rough, &mut smooth] {
            builder
                .add_view(&depth, width, height, intrinsics(width, height), &IDENTITY)
                .unwrap();
        }
        let plain = rough.build(&points, 0).unwrap();
        let eased = smooth.build(&points, 8).unwrap();
        assert_eq!(plain.triangles(), eased.triangles());
        let moved: f32 = plain
            .vertices
            .iter()
            .zip(&eased.vertices)
            .map(|(a, b)| (a - b).norm())
            .sum::<f32>()
            / plain.vertices.len() as f32;
        assert!(
            moved < 2.0,
            "smoothing moved vertices {moved} mm on average"
        );
    }

    #[test]
    fn with_no_views_there_is_no_surface() {
        let mut builder = SurfaceBuilder::new(2.0).unwrap();
        let points = vec![Point3::new(0.0, 0.0, 300.0)];
        assert!(builder.build(&points, 0).unwrap().is_empty());
    }

    #[test]
    fn a_silly_voxel_or_a_bad_view_is_refused() {
        for size in [0.0, 0.05, 100.0, f32::NAN] {
            assert!(SurfaceBuilder::new(size).is_err(), "{size}");
        }
        let mut builder = SurfaceBuilder::new(2.0).unwrap();
        // A depth image that is not width x height.
        assert!(
            builder
                .add_view(&[0u16; 10], 8, 8, [1.0, 1.0, 0.0, 0.0], &IDENTITY)
                .is_err()
        );
        // A camera with no focal length.
        assert!(
            builder
                .add_view(&[0u16; 64], 8, 8, [0.0, 0.0, 0.0, 0.0], &IDENTITY)
                .is_err()
        );
    }

    #[test]
    fn a_matrix_keeps_its_shape_through_the_bridge() {
        // The pose is row-major on both sides; a wrong order here would
        // place every view somewhere else.
        let turn = Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        let pose = [
            turn[(0, 0)],
            turn[(0, 1)],
            turn[(0, 2)],
            10.0, //
            turn[(1, 0)],
            turn[(1, 1)],
            turn[(1, 2)],
            20.0, //
            turn[(2, 0)],
            turn[(2, 1)],
            turn[(2, 2)],
            30.0, //
            0.0,
            0.0,
            0.0,
            1.0,
        ];
        let (width, height) = (120u32, 120u32);
        let (depth, points) = ball(width, height, 40.0, Vector3::new(0.0, 0.0, 300.0));
        let mut builder = SurfaceBuilder::new(2.0).unwrap();
        builder
            .add_view(&depth, width, height, intrinsics(width, height), &pose)
            .unwrap();
        // Turned a quarter turn about z and shifted, the ball lands where
        // that takes it; the bounds cloud is given the same treatment.
        let moved: Vec<Point3<f32>> = points
            .iter()
            .map(|p| Point3::new(-p.y + 10.0, p.x + 20.0, p.z + 30.0))
            .collect();
        let surface = builder.build(&moved, 0).unwrap();
        assert!(!surface.is_empty());
        let centre = surface
            .vertices
            .iter()
            .fold(Vector3::zeros(), |sum, p| sum + p.coords)
            / surface.vertices.len() as f32;
        // The ball's front sat near (0, 0, 270) before; after the turn and
        // the shift it should be near (10, 20, 300).
        assert!(
            (centre.x - 10.0).abs() < 20.0 && (centre.y - 20.0).abs() < 20.0,
            "centre at {centre:?}"
        );
    }
}
