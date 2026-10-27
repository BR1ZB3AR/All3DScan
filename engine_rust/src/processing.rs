//! Point-cloud processing on every core (rayon), on nalgebra points.

use std::fmt;

use nalgebra::{Point3, Vector3};
use rayon::prelude::*;

/// Why a processing step refused its input.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ProcessingError {
    InvalidVoxelSize(f32),
    /// A coordinate is so large, for the voxel size, that voxel keys would collide.
    CoordinateOutOfRange(f32),
}

impl fmt::Display for ProcessingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidVoxelSize(size) => write!(
                f,
                "voxel size must be a positive, finite number of millimetres, got {size}"
            ),
            Self::CoordinateOutOfRange(size) => {
                write!(f, "a coordinate is too large to sort into {size} mm voxels")
            }
        }
    }
}

/// Checks a voxel size before any work is done with it.
pub fn check_voxel_size(voxel_mm: f32) -> Result<(), ProcessingError> {
    if voxel_mm.is_finite() && voxel_mm > 0.0 {
        Ok(())
    } else {
        Err(ProcessingError::InvalidVoxelSize(voxel_mm))
    }
}

impl std::error::Error for ProcessingError {}

/// A point cloud with a colour and a normal for every point.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Cloud {
    pub points: Vec<Point3<f32>>,
    pub colors: Vec<[u8; 3]>,
    pub normals: Vec<Vector3<f32>>,
}

/// Thins a cloud to one point per occupied cube of edge `voxel_mm`: the
/// centroid of the points inside it. Points with a NaN or infinite coordinate
/// are dropped. The output is ordered by voxel (z, then y, then x), and the
/// same input always gives the same output.
pub fn voxel_downsample(
    points: &[Point3<f32>],
    voxel_mm: f32,
) -> Result<Vec<Point3<f32>>, ProcessingError> {
    let keyed = voxel_keys(points, voxel_mm)?;
    Ok(keyed
        .par_chunk_by(|a, b| a.0 == b.0)
        .map(|run| centroid(points, run))
        .collect())
}

/// Thins a cloud as voxel_downsample() does, averaging each voxel's colours
/// and normals along with its points.
pub fn voxel_downsample_cloud(cloud: &Cloud, voxel_mm: f32) -> Result<Cloud, ProcessingError> {
    let n = cloud.points.len();
    assert!(
        cloud.colors.len() == n && cloud.normals.len() == n,
        "a cloud needs one colour and one normal per point"
    );
    let keyed = voxel_keys(&cloud.points, voxel_mm)?;
    let merged: Vec<(Point3<f32>, [u8; 3], Vector3<f32>)> = keyed
        .par_chunk_by(|a, b| a.0 == b.0)
        .map(|run| {
            let sum = run.iter().fold([0u32; 3], |mut sum, &(_, i)| {
                for (s, c) in sum.iter_mut().zip(cloud.colors[i as usize]) {
                    *s += u32::from(c);
                }
                sum
            });
            let color = sum.map(|s| (s as f32 / run.len() as f32).round() as u8);
            let finite = run
                .iter()
                .map(|&(_, i)| cloud.normals[i as usize])
                .filter(|n| n.iter().all(|c| c.is_finite()));
            let first = finite.clone().next().unwrap_or(-Vector3::z());
            let normal = finite
                .fold(Vector3::zeros(), |sum, n| sum + n)
                .try_normalize(1e-6)
                .unwrap_or(first); // opposite normals cancelled out
            (centroid(&cloud.points, run), color, normal)
        })
        .collect();
    Ok(Cloud {
        points: merged.iter().map(|m| m.0).collect(),
        colors: merged.iter().map(|m| m.1).collect(),
        normals: merged.iter().map(|m| m.2).collect(),
    })
}

/// Each finite point's voxel key and index, sorted by key. The sort is
/// stable, so each voxel's points stay in input order and the sums over them
/// come out bit-for-bit the same on every run.
fn voxel_keys(
    points: &[Point3<f32>],
    voxel_mm: f32,
) -> Result<Vec<([i64; 3], u32)>, ProcessingError> {
    check_voxel_size(voxel_mm)?;
    // In f64 and range-checked: past 2^63 in magnitude an i64 key saturates,
    // and distinct points would share it.
    let voxel = |c: f32| {
        let q = (f64::from(c) / f64::from(voxel_mm)).floor();
        (q.abs() < 9.0e18).then_some(q as i64)
    };
    let mut keyed: Vec<([i64; 3], u32)> = points
        .par_iter()
        .enumerate()
        .filter(|(_, p)| p.coords.iter().all(|c| c.is_finite()))
        .map(|(i, p)| match (voxel(p.z), voxel(p.y), voxel(p.x)) {
            (Some(z), Some(y), Some(x)) => Ok(([z, y, x], i as u32)),
            _ => Err(ProcessingError::CoordinateOutOfRange(voxel_mm)),
        })
        .collect::<Result<_, _>>()?;
    keyed.par_sort_by_key(|(key, _)| *key);
    Ok(keyed)
}

fn centroid(points: &[Point3<f32>], run: &[([i64; 3], u32)]) -> Point3<f32> {
    let sum = run.iter().fold(Vector3::<f64>::zeros(), |sum, &(_, i)| {
        sum + points[i as usize].coords.cast::<f64>()
    });
    Point3::from((sum / run.len() as f64).cast::<f32>())
}

/// How far, in pixels, a normal looks for the neighbours it is taken from.
/// Adjacent pixels are often closer together than the depth noise, which makes
/// their normals noisy; on the mock sensor, reaching 3 pixels instead of 1
/// cuts the error on its pipe from a median of 23 degrees.
const NORMAL_REACH_PX: isize = 3;

/// Normals for an organised cloud: `points` holds one point for each pixel of
/// a `width` x `height` depth image whose depth is not 0, in row-major order,
/// as a depth camera produces them. Each normal comes from the pixel's
/// neighbours up to NORMAL_REACH_PX away, nearer ones at holes and edges; it
/// ignores neighbours across a depth jump (an object's edge) and faces the
/// camera.
pub fn organized_normals(
    points: &[Point3<f32>],
    depth_mm: &[u16],
    width: usize,
    height: usize,
) -> Vec<Vector3<f32>> {
    assert_eq!(
        depth_mm.len(),
        width * height,
        "depth image is not width x height"
    );
    // The point each pixel produced, if any.
    let mut index = vec![u32::MAX; depth_mm.len()];
    let mut next = 0u32;
    for (slot, &depth) in index.iter_mut().zip(depth_mm) {
        if depth > 0 {
            *slot = next;
            next += 1;
        }
    }
    assert_eq!(
        next as usize,
        points.len(),
        "points do not match the depth image"
    );
    // A slice, so the per-row closures below copy a reference to it.
    let index = index.as_slice();

    let point_at = |u: usize, v: usize, du: isize, dv: isize| -> Option<Point3<f32>> {
        let (u, v) = (u.checked_add_signed(du)?, v.checked_add_signed(dv)?);
        if u >= width || v >= height {
            return None;
        }
        let i = index[v * width + u];
        (i != u32::MAX).then(|| points[i as usize])
    };
    (0..height)
        .into_par_iter()
        .flat_map_iter(|v| {
            (0..width)
                .filter(move |&u| index[v * width + u] != u32::MAX)
                .map(move |u| {
                    let here = points[index[v * width + u] as usize];
                    // The farthest usable neighbour in one direction. A neighbour
                    // more than 3% nearer or farther is across an edge.
                    let reach = |du: isize, dv: isize| {
                        (1..=NORMAL_REACH_PX).rev().find_map(|k| {
                            point_at(u, v, du * k, dv * k)
                                .filter(|q| (q.z - here.z).abs() < 0.03 * here.z.abs().max(1.0))
                        })
                    };
                    let along_u = tangent(reach(1, 0), here, reach(-1, 0));
                    let along_v = tangent(reach(0, 1), here, reach(0, -1));
                    // With x right, y down and z forward, v × u faces the camera.
                    let normal = along_u
                        .zip(along_v)
                        .and_then(|(du, dv)| dv.cross(&du).try_normalize(1e-9))
                        .unwrap_or_else(|| {
                            (-here.coords).try_normalize(1e-9).unwrap_or(-Vector3::z())
                        });
                    if normal.dot(&here.coords) > 0.0 {
                        -normal
                    } else {
                        normal
                    }
                })
        })
        .collect()
}

/// The direction along an image row or column at a point, from whichever
/// neighbours it has.
fn tangent(
    next: Option<Point3<f32>>,
    here: Point3<f32>,
    previous: Option<Point3<f32>>,
) -> Option<Vector3<f32>> {
    match (next, previous) {
        (Some(n), Some(p)) => Some(n - p),
        (Some(n), None) => Some(n - here),
        (None, Some(p)) => Some(here - p),
        (None, None) => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hw::MockDepthSensor;

    fn p(x: f32, y: f32, z: f32) -> Point3<f32> {
        Point3::new(x, y, z)
    }

    #[test]
    fn points_in_one_voxel_merge_into_their_centroid() {
        let out = voxel_downsample(&[p(1.0, 1.0, 1.0), p(3.0, 2.0, 1.0), p(2.0, 3.0, 4.0)], 5.0);
        assert_eq!(out, Ok(vec![p(2.0, 2.0, 2.0)]));
    }

    #[test]
    fn points_in_different_voxels_stay_apart_in_voxel_order() {
        let out = voxel_downsample(
            &[p(12.0, 0.0, 0.0), p(-1.0, 0.0, 0.0), p(0.5, 0.0, 9.0)],
            5.0,
        );
        assert_eq!(
            out,
            Ok(vec![p(-1.0, 0.0, 0.0), p(12.0, 0.0, 0.0), p(0.5, 0.0, 9.0)])
        );
    }

    #[test]
    fn non_finite_points_are_dropped() {
        let cloud = [
            p(f32::NAN, 0.0, 0.0),
            p(0.0, f32::INFINITY, 0.0),
            p(1.0, 1.0, 1.0),
        ];
        assert_eq!(voxel_downsample(&cloud, 5.0), Ok(vec![p(1.0, 1.0, 1.0)]));
    }

    #[test]
    fn voxel_size_must_be_positive_and_finite() {
        for size in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            assert!(
                voxel_downsample(&[p(0.0, 0.0, 0.0)], size).is_err(),
                "{size}"
            );
        }
    }

    #[test]
    fn thins_a_mock_frame_and_keeps_it_in_place() {
        let cloud = MockDepthSensor::new(160, 120, 5).unwrap().capture().points;
        let thinned = voxel_downsample(&cloud, 10.0).unwrap();
        assert!(!thinned.is_empty() && thinned.len() < cloud.len() / 2);

        // Each centroid lies inside the cloud's bounding box.
        let lo = cloud
            .iter()
            .fold(Point3::from([f32::MAX; 3]), |lo, q| lo.inf(q));
        let hi = cloud
            .iter()
            .fold(Point3::from([f32::MIN; 3]), |hi, q| hi.sup(q));
        assert!(thinned.iter().all(|q| lo.inf(q) == lo && hi.sup(q) == hi));
    }

    #[test]
    fn thinning_a_cloud_averages_colours_and_normals() {
        let cloud = Cloud {
            points: vec![p(1.0, 1.0, 1.0), p(3.0, 2.0, 1.0), p(12.0, 0.0, 0.0)],
            colors: vec![[0, 0, 0], [100, 200, 50], [7, 8, 9]],
            normals: vec![
                Vector3::new(0.0, 0.0, -1.0),
                Vector3::new(0.0, -1.0, 0.0),
                Vector3::new(1.0, 0.0, 0.0),
            ],
        };
        let thinned = voxel_downsample_cloud(&cloud, 5.0).unwrap();
        assert_eq!(
            thinned.points,
            voxel_downsample(&cloud.points, 5.0).unwrap()
        );
        assert_eq!(thinned.colors, vec![[50, 100, 25], [7, 8, 9]]);
        let expected = Vector3::new(0.0, -1.0, -1.0).normalize();
        assert!((thinned.normals[0] - expected).norm() < 1e-6);
        assert_eq!(thinned.normals[1], Vector3::new(1.0, 0.0, 0.0));
    }

    /// A `width` x `height` image's worth of points, 1 mm apart, at the
    /// depth `surface(x, y)` gives, with every pixel holding depth.
    fn grid(
        width: usize,
        height: usize,
        surface: impl Fn(f32, f32) -> f32,
    ) -> (Vec<Point3<f32>>, Vec<u16>) {
        let points: Vec<_> = (0..height)
            .flat_map(|v| (0..width).map(move |u| (u as f32, v as f32)))
            .map(|(x, y)| p(x, y, surface(x, y)))
            .collect();
        let depth = points.iter().map(|q| q.z.round() as u16).collect();
        (points, depth)
    }

    #[test]
    fn a_wall_facing_the_camera_has_normals_towards_it() {
        let (points, depth) = grid(5, 4, |_, _| 500.0);
        for n in organized_normals(&points, &depth, 5, 4) {
            assert!((n - Vector3::new(0.0, 0.0, -1.0)).norm() < 1e-5, "{n}");
        }
    }

    #[test]
    fn a_tilted_plane_gets_its_true_normal() {
        let (points, depth) = grid(5, 4, |x, _| 500.0 + x);
        let expected = Vector3::new(1.0, 0.0, -1.0).normalize();
        for n in organized_normals(&points, &depth, 5, 4) {
            assert!((n - expected).norm() < 1e-4, "{n}");
        }
    }

    #[test]
    fn depth_jumps_and_holes_do_not_bend_the_normals() {
        // A near surface on the left, a far wall on the right, one pixel missing.
        let (mut points, mut depth) = grid(7, 5, |x, _| if x < 3.0 { 500.0 } else { 900.0 });
        let hole = 2 * 7 + 2; // u = 2, v = 2
        depth[hole] = 0;
        points.remove(hole);
        for n in organized_normals(&points, &depth, 7, 5) {
            assert!((n - Vector3::new(0.0, 0.0, -1.0)).norm() < 1e-5, "{n}");
        }
    }

    #[test]
    fn a_step_just_over_the_edge_threshold_does_not_bend_the_normals() {
        // A 5% step, past the 3% rule: each side keeps its own flat normal.
        let (points, depth) = grid(8, 4, |x, _| if x < 4.0 { 500.0 } else { 525.0 });
        for n in organized_normals(&points, &depth, 8, 4) {
            assert!((n - Vector3::new(0.0, 0.0, -1.0)).norm() < 1e-5, "{n}");
        }
    }

    #[test]
    fn a_steep_slope_inside_the_threshold_keeps_its_true_normal() {
        // 10 mm deeper per 1 mm across: 2% per pixel, so only adjacent pixels qualify.
        let (points, depth) = grid(9, 5, |x, _| 500.0 + 10.0 * x);
        let expected = Vector3::new(10.0, 0.0, -1.0).normalize();
        for n in organized_normals(&points, &depth, 9, 5) {
            assert!((n - expected).norm() < 1e-4, "{n}");
        }
    }

    #[test]
    fn an_isolated_point_at_the_origin_gets_a_finite_normal() {
        let normals = organized_normals(&[p(0.0, 0.0, 0.0)], &[1], 1, 1);
        assert_eq!(normals, vec![-Vector3::z()]);
    }

    #[test]
    fn coordinates_too_large_for_the_voxels_are_refused() {
        let far = [p(1e30, 0.0, 0.0), p(2e30, 0.0, 0.0)];
        assert_eq!(
            voxel_downsample(&far, 1.0),
            Err(ProcessingError::CoordinateOutOfRange(1.0))
        );
        assert_eq!(
            voxel_downsample(&[p(1.0, 0.0, 0.0)], 1e-30),
            Err(ProcessingError::CoordinateOutOfRange(1e-30))
        );
    }

    #[test]
    fn mock_ball_normals_face_the_camera_and_point_out_of_the_ball() {
        let frame = MockDepthSensor::new(64, 48, 7).unwrap().capture();
        let normals = organized_normals(&frame.points, &frame.depth_mm, 64, 48);
        assert_eq!(normals.len(), frame.points.len());
        assert!(
            normals
                .iter()
                .zip(&frame.points)
                .all(|(n, q)| { (n.norm() - 1.0).abs() < 1e-4 && n.dot(&q.coords) < 0.0 })
        );
        // The nearest point is the front of the ball, facing the camera head-on.
        let front = (0..frame.points.len())
            .min_by(|&a, &b| frame.points[a].z.total_cmp(&frame.points[b].z))
            .unwrap();
        assert!(normals[front].z < -0.95, "{}", normals[front]);
    }
}
