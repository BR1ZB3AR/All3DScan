//! Point-cloud processing on every core (rayon), on nalgebra points.

use std::collections::HashMap;
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

/// How many points lie within `radius_mm` of each point, itself included,
/// paired with each point's index. Exact, not approximate: the points are
/// binned into cubes of that edge, so the 27 cubes around a point hold
/// every point within the radius of it, and each candidate is measured.
fn neighbour_counts(
    points: &[Point3<f32>],
    radius_mm: f32,
) -> Result<Vec<(u32, u32)>, ProcessingError> {
    let keyed = voxel_keys(points, radius_mm)?;
    let mut cubes: HashMap<[i64; 3], &[([i64; 3], u32)]> = HashMap::new();
    for run in keyed.chunk_by(|a, b| a.0 == b.0) {
        cubes.insert(run[0].0, run);
    }
    let radius = radius_mm * radius_mm;
    Ok(keyed
        .par_iter()
        .map(|(key, index)| {
            let here = points[*index as usize];
            let mut near = 0;
            for offset in 0..27 {
                let beside = [
                    key[0] + offset / 9 - 1,
                    key[1] + offset / 3 % 3 - 1,
                    key[2] + offset % 3 - 1,
                ];
                if let Some(cube) = cubes.get(&beside) {
                    near += cube
                        .iter()
                        .filter(|(_, other)| {
                            (points[*other as usize] - here).norm_squared() <= radius
                        })
                        .count() as u32;
                }
            }
            (*index, near)
        })
        .collect())
}

/// How many neighbours a normal wants before it is worth believing. Three
/// points define a plane, so fewer than that cannot; a few more than that
/// averages the sensor's noise without reaching around a corner.
const NORMAL_NEIGHBOURS: usize = 6;

/// How many times the search radius may be doubled looking for enough
/// neighbours, for a cloud thinner than the radius first guessed.
const NORMAL_WIDENINGS: u32 = 5;

/// A unit normal for every point of a cloud that has no order to it.
///
/// `organized_normals` needs the cloud laid out as the depth camera's
/// pixels. A cloud read from a file has no such layout, so each normal is
/// taken instead from the points within `radius_mm`: the direction in which
/// they are most tightly packed, which is across the surface they lie on.
///
/// Normals are turned to face `towards` — the direction the surface is
/// looked at from, which for a scan is the sensor at the origin. A point
/// with too few neighbours to judge gets a zero normal rather than a made-up
/// one, so a caller can tell the difference.
pub fn estimate_normals(
    points: &[Point3<f32>],
    radius_mm: f32,
    towards: Vector3<f32>,
) -> Result<Vec<Vector3<f32>>, ProcessingError> {
    check_voxel_size(radius_mm)?;
    if points.is_empty() {
        return Ok(Vec::new());
    }
    // A radius that finds nothing makes every normal zero, which is useless
    // and silent. Widen until a typical point has neighbours to work with.
    let mut radius = radius_mm;
    for _ in 0..NORMAL_WIDENINGS {
        let counts = neighbour_counts(points, radius)?;
        let mut near: Vec<u32> = counts.iter().map(|(_, n)| *n).collect();
        let middle = near.len() / 2;
        near.select_nth_unstable(middle);
        if near[middle] as usize > NORMAL_NEIGHBOURS {
            break;
        }
        radius *= 2.0;
    }

    let keyed = voxel_keys(points, radius)?;
    let mut cubes: HashMap<[i64; 3], &[([i64; 3], u32)]> = HashMap::new();
    for run in keyed.chunk_by(|a, b| a.0 == b.0) {
        cubes.insert(run[0].0, run);
    }
    let within = radius * radius;
    let mut normals = vec![Vector3::zeros(); points.len()];
    let found: Vec<(u32, Vector3<f32>)> = keyed
        .par_iter()
        .map(|(key, index)| {
            let here = points[*index as usize];
            let mut near: Vec<Point3<f32>> = Vec::with_capacity(32);
            for offset in 0..27 {
                let beside = [
                    key[0] + offset / 9 - 1,
                    key[1] + offset / 3 % 3 - 1,
                    key[2] + offset % 3 - 1,
                ];
                if let Some(cube) = cubes.get(&beside) {
                    near.extend(
                        cube.iter()
                            .map(|(_, other)| points[*other as usize])
                            .filter(|p| (p - here).norm_squared() <= within),
                    );
                }
            }
            (*index, plane_normal(&near).unwrap_or_else(Vector3::zeros))
        })
        .collect();
    for (index, mut normal) in found {
        // Facing the viewer, so neighbouring normals agree with each other
        // and a fit can use them. A cloud from a scanner is seen from the
        // sensor, which sits at the origin.
        if normal.dot(&towards) < 0.0 {
            normal = -normal;
        }
        normals[index as usize] = normal;
    }
    Ok(normals)
}

/// The direction a handful of points are least spread in, which is the
/// normal of the plane through them. None when there are too few to say.
fn plane_normal(near: &[Point3<f32>]) -> Option<Vector3<f32>> {
    if near.len() <= NORMAL_NEIGHBOURS {
        return None;
    }
    let middle = near
        .iter()
        .fold(Vector3::<f64>::zeros(), |sum, p| sum + p.coords.cast::<f64>())
        / near.len() as f64;
    // The covariance of the neighbourhood, whose least eigenvector is the
    // normal. Only the upper triangle is built; it is symmetric.
    let mut covariance = nalgebra::Matrix3::<f64>::zeros();
    for point in near {
        let offset = point.coords.cast::<f64>() - middle;
        for row in 0..3 {
            for column in row..3 {
                covariance[(row, column)] += offset[row] * offset[column];
            }
        }
    }
    for row in 1..3 {
        for column in 0..row {
            covariance[(row, column)] = covariance[(column, row)];
        }
    }
    let eigen = covariance.symmetric_eigen();
    let least = (0..3).min_by(|a, b| eigen.eigenvalues[*a].total_cmp(&eigen.eigenvalues[*b]))?;
    let axis = eigen.eigenvectors.column(least);
    Vector3::new(axis[0] as f32, axis[1] as f32, axis[2] as f32).try_normalize(1e-12)
}

/// The pieces a cloud falls into, as index lists, largest first.
///
/// Two points belong to the same piece when a chain of points no more than
/// `gap_mm` apart joins them. Measured on the voxel grid rather than
/// point-to-point, so it costs one pass rather than a search per point: two
/// points in touching cubes of edge `gap_mm` are within `gap_mm * sqrt(3)`
/// of each other, which is the usual trade and is why the gap is generous
/// rather than exact.
pub fn pieces(points: &[Point3<f32>], gap_mm: f32) -> Result<Vec<Vec<u32>>, ProcessingError> {
    check_voxel_size(gap_mm)?;
    let keyed = voxel_keys(points, gap_mm)?;
    // Number the occupied cubes in the order their first point appears, so
    // the answer never depends on hash order.
    let mut id_of: HashMap<[i64; 3], usize> = HashMap::new();
    let mut cube_of: Vec<usize> = Vec::with_capacity(keyed.len());
    for (key, _) in &keyed {
        let next = id_of.len();
        let id = *id_of.entry(*key).or_insert(next);
        cube_of.push(id);
    }
    let mut parent: Vec<usize> = (0..id_of.len()).collect();
    fn root(parent: &mut [usize], mut at: usize) -> usize {
        while parent[at] != at {
            parent[at] = parent[parent[at]]; // halve the path as we go
            at = parent[at];
        }
        at
    }
    for (key, id) in &id_of {
        for offset in 0..27 {
            let beside = [
                key[0] + offset / 9 - 1,
                key[1] + offset / 3 % 3 - 1,
                key[2] + offset % 3 - 1,
            ];
            if let Some(other) = id_of.get(&beside) {
                let (a, b) = (root(&mut parent, *id), root(&mut parent, *other));
                if a != b {
                    // Join towards the lower number, so the result is the
                    // same however the map was walked.
                    let (low, high) = if a < b { (a, b) } else { (b, a) };
                    parent[high] = low;
                }
            }
        }
    }
    let mut members: HashMap<usize, Vec<u32>> = HashMap::new();
    for (at, (_, index)) in keyed.iter().enumerate() {
        members.entry(root(&mut parent, cube_of[at])).or_default().push(*index);
    }
    let mut found: Vec<Vec<u32>> = members.into_values().collect();
    for piece in &mut found {
        piece.sort_unstable();
    }
    // Largest first, and ties broken by the lowest index, so the order is
    // the same on every run.
    found.sort_by(|a, b| b.len().cmp(&a.len()).then(a[0].cmp(&b[0])));
    Ok(found)
}

/// How far apart two points may be and still count as the same piece, as a
/// multiple of the cloud's own typical spacing. Loose enough to bridge the
/// holes a scan leaves, tight enough not to join an object to the table.
const PIECE_GAP: f32 = 3.0;
/// A plane is only worth removing if it holds at least this share of the
/// cloud: anything less is a face of the object, not the thing it stands on.
const SURFACE_SHARE: f32 = 0.12;
/// ...and only if this much of the cloud is left afterwards. Below it the
/// plane *is* the object, and taking it out would leave nothing worth having.
const LEAST_LEFT: f32 = 0.05;
/// How far from the plane a point must be to count as off it.
const SURFACE_BAND_MM: f32 = 3.0;
/// How many hypotheses the search for that surface tries.
const SURFACE_TRIES: u32 = 400;

/// What isolation did, so a caller can say it rather than silently throwing
/// most of the scan away.
#[derive(Debug, Clone)]
pub struct Isolated {
    pub cloud: Cloud,
    /// Indices into the cloud given, in order.
    pub kept: Vec<u32>,
    /// Whether a surface the object was standing on was found and taken out.
    pub surface_removed: bool,
    /// How many separate pieces the rest fell into.
    pub pieces: usize,
}

/// Keeps the object and drops the room around it.
///
/// A scan of something on a desk is mostly desk, and the walls and clutter
/// behind it come too: on a real POP 4 scan the object is a minority of the
/// points. Two steps, in this order because they depend on each other — the
/// object is usually joined to everything else *through* the surface it
/// stands on, so the surface has to go first or the largest piece is the
/// whole room:
///
/// 1. the flat surface the object stands on or in front of, if the cloud is
///    largely one, and everything behind it;
/// 2. everything not joined to the largest remaining piece.
///
/// Both steps are guarded. A cloud that is mostly one flat thing is probably
/// a scan *of* something flat, so the surface is kept; and if a step would
/// leave almost nothing, it is not taken. `normals` may be empty, in which
/// case no surface is looked for and only step 2 runs.
pub fn isolate(cloud: &Cloud, gap_mm: Option<f32>) -> Result<Isolated, ProcessingError> {
    let n = cloud.points.len();
    let whole = |reason: &str| {
        let _ = reason;
        Isolated {
            cloud: cloud.clone(),
            kept: (0..n as u32).collect(),
            surface_removed: false,
            pieces: 1,
        }
    };
    if n < 32 {
        return Ok(whole("too few points to judge"));
    }
    let gap = match gap_mm {
        Some(given) => {
            check_voxel_size(given)?;
            given
        }
        None => typical_spacing(&cloud.points)? * PIECE_GAP,
    };

    // 1. The surface it stands on, if there is one and it is not the object.
    let mut candidates: Vec<u32> = (0..n as u32).collect();
    let mut surface_removed = false;
    if cloud.normals.len() == n {
        if let Some(plane) = crate::fitting::fit_plane(
            &cloud.points,
            &cloud.normals,
            &candidates,
            crate::fitting::Ransac {
                threshold_mm: SURFACE_BAND_MM,
                min_inliers: (n as f32 * SURFACE_SHARE) as usize,
                iterations: SURFACE_TRIES,
                // Fixed, so isolating the same cloud twice gives the same
                // answer: a tool that moves under you is not usable.
                seed: 0x1501A7E,
            },
        ) {
            // The normal faces the sensor, so the object is on the positive
            // side: in front of a wall, above a desk.
            let in_front: Vec<u32> = candidates
                .iter()
                .copied()
                .filter(|&k| {
                    (cloud.points[k as usize] - plane.origin).dot(&plane.normal)
                        > SURFACE_BAND_MM
                })
                .collect();
            if in_front.len() as f32 >= n as f32 * LEAST_LEFT {
                candidates = in_front;
                surface_removed = true;
            }
        }
    }

    // 2. The largest piece of what is left.
    let kept_points: Vec<Point3<f32>> =
        candidates.iter().map(|&k| cloud.points[k as usize]).collect();
    let found = pieces(&kept_points, gap)?;
    let Some(largest) = found.first() else {
        return Ok(whole("nothing left to judge"));
    };
    let kept: Vec<u32> = largest.iter().map(|&at| candidates[at as usize]).collect();
    if (kept.len() as f32) < n as f32 * LEAST_LEFT {
        return Ok(whole("the largest piece is too small to be the object"));
    }
    Ok(Isolated {
        cloud: take(cloud, &kept),
        kept,
        surface_removed,
        pieces: found.len(),
    })
}

/// The points a cloud holds at these indices, with their colours and normals.
fn take(cloud: &Cloud, which: &[u32]) -> Cloud {
    let has_colors = cloud.colors.len() == cloud.points.len();
    let has_normals = cloud.normals.len() == cloud.points.len();
    Cloud {
        points: which.iter().map(|&k| cloud.points[k as usize]).collect(),
        colors: if has_colors {
            which.iter().map(|&k| cloud.colors[k as usize]).collect()
        } else {
            Vec::new()
        },
        normals: if has_normals {
            which.iter().map(|&k| cloud.normals[k as usize]).collect()
        } else {
            Vec::new()
        },
    }
}

/// Roughly how far apart neighbouring points are, from how many share a
/// cube: a surface spaced s apart crossing a cube of edge c puts about
/// (c / s)^2 points in it.
fn typical_spacing(points: &[Point3<f32>]) -> Result<f32, ProcessingError> {
    const PROBE_MM: f32 = 10.0;
    let keyed = voxel_keys(points, PROBE_MM)?;
    let cubes = keyed.chunk_by(|a, b| a.0 == b.0).count().max(1);
    let per_cube = (keyed.len() as f32 / cubes as f32).max(1.0);
    Ok(PROBE_MM / per_cube.sqrt())
}

/// How many times the radius may be doubled while looking for one at which
/// the points can see each other at all.
const SPECKLE_WIDENINGS: u32 = 4;

/// Drops the points that have far fewer neighbours than the cloud's points
/// typically do: the speckle a depth camera leaves in mid-air, which has
/// nothing around it, where a surface point has its neighbours. Returns the
/// indices of the points kept, in order.
///
/// Both halves of the judgement come from the cloud itself, because how
/// close its points lie depends on the sensor, the distance and how much it
/// has been thinned. `radius_mm` is only a starting point: it is doubled
/// until the typical point has a neighbour, so a cloud sampled more coarsely
/// than the radius is not simply declared empty. A point is then kept when at
/// least `share` of the typical count surrounds it.
///
/// A cloud whose points cannot see each other at any of those radii is
/// merely sparse — nothing in it stands out as speckle — and is kept whole.
pub fn dense_points(
    points: &[Point3<f32>],
    radius_mm: f32,
    share: f32,
) -> Result<Vec<u32>, ProcessingError> {
    check_voxel_size(radius_mm)?;
    let mut radius = radius_mm;
    let mut counted = Vec::new();
    let mut typical = 0.0f32;
    for _ in 0..=SPECKLE_WIDENINGS {
        counted = neighbour_counts(points, radius)?;
        if counted.is_empty() {
            return Ok(Vec::new());
        }
        let mut counts: Vec<u32> = counted.iter().map(|(_, near)| *near).collect();
        let middle = counts.len() / 2;
        counts.par_sort_unstable();
        typical = counts[middle] as f32;
        if typical >= 2.0 {
            break;
        }
        radius *= 2.0;
    }
    if typical < 2.0 {
        let mut all: Vec<u32> = counted.into_iter().map(|(index, _)| index).collect();
        all.par_sort_unstable();
        return Ok(all);
    }
    // A speck has only itself, so two is the lowest count that says anything.
    let least = ((typical * share).round() as u32).max(2);
    let mut kept: Vec<u32> = counted
        .into_par_iter()
        .filter(|(_, near)| *near >= least)
        .map(|(index, _)| index)
        .collect();
    kept.par_sort_unstable();
    Ok(kept)
}

/// The cloud with only the points that `dense_points` keeps.
pub fn remove_speckle(cloud: &Cloud, radius_mm: f32, share: f32) -> Result<Cloud, ProcessingError> {
    let n = cloud.points.len();
    assert!(
        cloud.colors.len() == n && cloud.normals.len() == n,
        "a cloud needs one colour and one normal per point"
    );
    let kept = dense_points(&cloud.points, radius_mm, share)?;
    Ok(Cloud {
        points: kept.iter().map(|&i| cloud.points[i as usize]).collect(),
        colors: kept.iter().map(|&i| cloud.colors[i as usize]).collect(),
        normals: kept.iter().map(|&i| cloud.normals[i as usize]).collect(),
    })
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

    /// A cloud with colours and normals per point, for the isolation tests.
    fn dressed(points: Vec<Point3<f32>>, normals: Vec<Vector3<f32>>) -> Cloud {
        let n = points.len();
        Cloud { points, colors: vec![[200, 200, 200]; n], normals }
    }

    /// A ball of `across` points on a sphere, centred where asked.
    fn ball(centre: Point3<f32>, radius: f32, across: usize) -> (Vec<Point3<f32>>, Vec<Vector3<f32>>) {
        let (mut points, mut normals) = (Vec::new(), Vec::new());
        for i in 0..across {
            for j in 0..across {
                let theta = std::f32::consts::PI * (i as f32 + 0.5) / across as f32;
                let phi = 2.0 * std::f32::consts::PI * j as f32 / across as f32;
                let n = Vector3::new(
                    theta.sin() * phi.cos(),
                    theta.sin() * phi.sin(),
                    theta.cos(),
                );
                points.push(centre + n * radius);
                normals.push(n);
            }
        }
        (points, normals)
    }

    /// A flat sheet in the z = `at` plane, `span` mm across, `step` apart.
    fn sheet(at: f32, span: f32, step: f32) -> (Vec<Point3<f32>>, Vec<Vector3<f32>>) {
        let (mut points, mut normals) = (Vec::new(), Vec::new());
        let n = (span / step) as i32;
        for x in -n..=n {
            for y in -n..=n {
                points.push(Point3::new(x as f32 * step, y as f32 * step, at));
                normals.push(-Vector3::z()); // facing the sensor
            }
        }
        (points, normals)
    }

    #[test]
    fn a_cloud_in_two_lumps_falls_into_two_pieces() {
        let (mut points, _) = ball(Point3::new(0.0, 0.0, 300.0), 20.0, 20);
        let (far, _) = ball(Point3::new(400.0, 0.0, 300.0), 20.0, 20);
        let near_count = points.len();
        points.extend(far);
        let found = pieces(&points, 12.0).unwrap();
        assert_eq!(found.len(), 2, "two separated balls should be two pieces");
        assert_eq!(found[0].len(), near_count, "both balls have the same number of points");
        // Largest first, and every point accounted for exactly once.
        let mut all: Vec<u32> = found.iter().flatten().copied().collect();
        all.sort_unstable();
        assert_eq!(all, (0..points.len() as u32).collect::<Vec<_>>());
    }

    #[test]
    fn the_pieces_of_a_cloud_are_the_same_on_every_run() {
        let (mut points, _) = ball(Point3::new(0.0, 0.0, 300.0), 20.0, 14);
        let (far, _) = ball(Point3::new(300.0, 0.0, 300.0), 15.0, 12);
        points.extend(far);
        let once = pieces(&points, 12.0).unwrap();
        for _ in 0..4 {
            assert_eq!(pieces(&points, 12.0).unwrap(), once);
        }
    }

    #[test]
    fn isolation_keeps_the_object_and_drops_the_desk_it_stands_on() {
        // A ball sitting in front of a large flat surface, with a scrap of
        // clutter off to one side: a scan of something on a desk.
        let (mut points, mut normals) = sheet(400.0, 150.0, 4.0);
        let desk = points.len();
        let (object, object_normals) = ball(Point3::new(0.0, 0.0, 360.0), 30.0, 30);
        let kept_count = object.len();
        points.extend(object);
        normals.extend(object_normals);
        let (clutter, clutter_normals) = ball(Point3::new(250.0, 100.0, 330.0), 8.0, 8);
        points.extend(clutter);
        normals.extend(clutter_normals);

        let found = isolate(&dressed(points, normals), None).unwrap();
        assert!(found.surface_removed, "the flat surface should have been taken out");
        assert_eq!(found.cloud.points.len(), kept_count,
                   "should be exactly the ball: {} of {} kept", found.cloud.points.len(), kept_count);
        assert!(found.kept.iter().all(|&k| k as usize >= desk),
                "a desk point survived isolation");
        assert_eq!(found.cloud.colors.len(), found.cloud.points.len());
        assert_eq!(found.cloud.normals.len(), found.cloud.points.len());
    }

    #[test]
    fn isolation_will_not_eat_a_scan_that_is_itself_flat() {
        // Scanning a flat panel: the dominant plane *is* the object, and
        // taking it out would leave nothing.
        let (points, normals) = sheet(350.0, 120.0, 3.0);
        let whole = points.len();
        let found = isolate(&dressed(points, normals), None).unwrap();
        assert!(!found.surface_removed, "it removed the very thing being scanned");
        assert_eq!(found.cloud.points.len(), whole);
    }

    #[test]
    fn isolation_drops_the_scattered_bits_even_with_no_surface_to_find() {
        let (mut points, mut normals) = ball(Point3::new(0.0, 0.0, 300.0), 25.0, 26);
        let object = points.len();
        for away in [300.0f32, -280.0, 420.0] {
            let (bit, bit_normals) = ball(Point3::new(away, away / 3.0, 300.0), 5.0, 5);
            points.extend(bit);
            normals.extend(bit_normals);
        }
        let found = isolate(&dressed(points, normals), None).unwrap();
        assert!(!found.surface_removed);
        assert!(found.pieces >= 4, "the scattered bits should be separate pieces");
        assert_eq!(found.cloud.points.len(), object);
    }

    #[test]
    fn isolation_leaves_a_cloud_it_cannot_judge_alone() {
        let few = Cloud {
            points: (0..10).map(|i| Point3::new(i as f32, 0.0, 300.0)).collect(),
            colors: vec![[1, 2, 3]; 10],
            normals: vec![-Vector3::z(); 10],
        };
        let found = isolate(&few, None).unwrap();
        assert_eq!(found.cloud.points.len(), 10);
        assert_eq!(found.kept, (0..10).collect::<Vec<u32>>());
    }

    #[test]
    fn isolation_without_normals_still_drops_the_scattered_bits() {
        let (mut points, _) = ball(Point3::new(0.0, 0.0, 300.0), 25.0, 24);
        let object = points.len();
        let (bit, _) = ball(Point3::new(350.0, 0.0, 300.0), 6.0, 6);
        points.extend(bit);
        let n = points.len();
        let bare = Cloud { points, colors: vec![[9, 9, 9]; n], normals: Vec::new() };
        let found = isolate(&bare, None).unwrap();
        assert_eq!(found.cloud.points.len(), object);
        assert!(found.cloud.normals.is_empty(), "normals were invented from nowhere");
    }

    #[test]
    fn isolating_the_same_cloud_twice_gives_the_same_answer() {
        let (mut points, mut normals) = sheet(400.0, 100.0, 4.0);
        let (object, object_normals) = ball(Point3::new(0.0, 0.0, 360.0), 28.0, 24);
        points.extend(object);
        normals.extend(object_normals);
        let cloud = dressed(points, normals);
        let once = isolate(&cloud, None).unwrap();
        for _ in 0..3 {
            assert_eq!(isolate(&cloud, None).unwrap().kept, once.kept);
        }
    }

    #[test]
    fn a_silly_gap_is_refused() {
        let cloud = dressed(ball(Point3::origin(), 10.0, 10).0, ball(Point3::origin(), 10.0, 10).1);
        assert!(isolate(&cloud, Some(0.0)).is_err());
        assert!(isolate(&cloud, Some(f32::NAN)).is_err());
    }
    #[test]
    fn normals_of_a_cloud_with_no_order_point_out_of_its_surface() {
        // A flat sheet in the z = 300 plane, as a scan of a wall would be.
        let mut points = Vec::new();
        for x in 0..40 {
            for y in 0..40 {
                points.push(Point3::new(x as f32 * 2.0 - 40.0, y as f32 * 2.0 - 40.0, 300.0));
            }
        }
        let normals = estimate_normals(&points, 5.0, -Vector3::z()).unwrap();
        assert_eq!(normals.len(), points.len());
        let judged = normals.iter().filter(|n| n.norm() > 0.5).count();
        assert!(judged > points.len() * 9 / 10, "only {judged} normals were worked out");
        for normal in normals.iter().filter(|n| n.norm() > 0.5) {
            assert!(
                (normal.z + 1.0).abs() < 0.05,
                "a sheet in z should have a normal along z, got {normal:?}"
            );
        }
    }

    #[test]
    fn normals_face_the_way_they_are_looked_at() {
        let mut points = Vec::new();
        for x in 0..30 {
            for y in 0..30 {
                points.push(Point3::new(x as f32, y as f32, 100.0));
            }
        }
        for towards in [Vector3::z(), -Vector3::z()] {
            let normals = estimate_normals(&points, 3.0, towards).unwrap();
            for normal in normals.iter().filter(|n| n.norm() > 0.5) {
                assert!(normal.dot(&towards) >= 0.0, "{normal:?} faces away from {towards:?}");
            }
        }
    }

    #[test]
    fn a_point_with_nothing_near_it_gets_no_normal_rather_than_a_guess() {
        let points = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1e4, 1e4, 1e4)];
        let normals = estimate_normals(&points, 1.0, -Vector3::z()).unwrap();
        assert!(normals.iter().all(|n| n.norm() == 0.0), "{normals:?}");
    }

    #[test]
    fn the_radius_widens_until_the_points_can_see_each_other() {
        // Spaced 10 mm apart, asked for with a 0.5 mm radius: too small to
        // find anything, so it has to widen rather than give up.
        let mut points = Vec::new();
        for x in 0..12 {
            for y in 0..12 {
                points.push(Point3::new(x as f32 * 10.0, y as f32 * 10.0, 50.0));
            }
        }
        let normals = estimate_normals(&points, 0.5, -Vector3::z()).unwrap();
        let judged = normals.iter().filter(|n| n.norm() > 0.5).count();
        assert!(judged > points.len() / 2, "only {judged} of {} were judged", points.len());
    }

    #[test]
    fn an_empty_cloud_has_no_normals_and_does_not_fall_over() {
        assert!(estimate_normals(&[], 2.0, -Vector3::z()).unwrap().is_empty());
    }

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
    fn speckle_in_mid_air_goes_and_the_surface_stays() {
        // A plate of points 1 mm apart, and scattered specks well away from it.
        let mut cloud = Cloud::default();
        for i in 0..40 * 40 {
            cloud
                .points
                .push(p((i % 40) as f32, (i / 40) as f32, 400.0));
            cloud.colors.push([200, 200, 200]);
            cloud.normals.push(-Vector3::z());
        }
        let surface = cloud.points.len();
        for i in 0..60 {
            cloud
                .points
                .push(p(i as f32 * 3.0, 60.0 + i as f32, 300.0 + i as f32 * 2.0));
            cloud.colors.push([10, 10, 10]);
            cloud.normals.push(-Vector3::z());
        }
        let cleaned = remove_speckle(&cloud, 3.0, 0.2).unwrap();
        assert_eq!(
            cleaned.points.len(),
            surface,
            "the plate should survive whole"
        );
        assert_eq!(cleaned.colors.len(), surface);
        assert_eq!(cleaned.normals.len(), surface);
        assert!(
            cleaned.colors.iter().all(|&c| c == [200, 200, 200]),
            "a speck got through"
        );
    }

    #[test]
    fn a_cloud_too_sparse_to_judge_is_kept_whole() {
        // Points 400 mm apart: even widened, no point sees another, so
        // nothing stands out as speckle and the cloud survives. Throwing it
        // away would lose the scan.
        let points: Vec<_> = (0..50).map(|i| p(i as f32 * 400.0, 0.0, 400.0)).collect();
        assert_eq!(dense_points(&points, 2.0, 0.2).unwrap().len(), 50);
    }

    #[test]
    fn a_radius_smaller_than_the_spacing_is_widened_until_it_bites() {
        // A plate 5 mm apart with specks around it, asked for with a 1 mm
        // radius: too small to see anything, so it grows until it does.
        let mut cloud = Cloud::default();
        for i in 0..30 * 30 {
            cloud
                .points
                .push(p((i % 30) as f32 * 5.0, (i / 30) as f32 * 5.0, 400.0));
            cloud.colors.push([200, 200, 200]);
            cloud.normals.push(-Vector3::z());
        }
        let surface = cloud.points.len();
        for i in 0..8 {
            cloud.points.push(p(400.0 + 50.0 * i as f32, -200.0, 300.0));
            cloud.colors.push([9, 9, 9]);
            cloud.normals.push(-Vector3::z());
        }
        let cleaned = remove_speckle(&cloud, 1.0, 0.2).unwrap();
        assert_eq!(cleaned.points.len(), surface);
        assert!(cleaned.colors.iter().all(|&c| c == [200, 200, 200]));
    }

    #[test]
    fn a_denser_cloud_loses_the_same_specks() {
        // The same scene sampled finely and coarsely: the speck goes either
        // way, because the threshold follows the cloud.
        for step in [0.5f32, 2.0] {
            let mut cloud = Cloud::default();
            let side = (20.0 / step) as i32;
            for i in 0..side * side {
                cloud
                    .points
                    .push(p((i % side) as f32 * step, (i / side) as f32 * step, 400.0));
                cloud.colors.push([200, 200, 200]);
                cloud.normals.push(-Vector3::z());
            }
            let surface = cloud.points.len();
            cloud.points.push(p(100.0, 100.0, 300.0)); // a speck on its own
            cloud.colors.push([9, 9, 9]);
            cloud.normals.push(-Vector3::z());
            let cleaned = remove_speckle(&cloud, 3.0 * step, 0.2).unwrap();
            assert_eq!(cleaned.points.len(), surface, "step {step}");
        }
    }

    #[test]
    fn the_points_kept_come_back_in_order() {
        let frame = MockDepthSensor::new(320, 240, 5).unwrap().capture();
        let kept = dense_points(&frame.points, 5.0, 0.2).unwrap();
        assert!(
            kept.windows(2).all(|w| w[0] < w[1]),
            "out of order or repeated"
        );
        assert!(!kept.is_empty() && kept.len() <= frame.points.len());
        assert!(kept.iter().all(|&i| (i as usize) < frame.points.len()));
    }

    #[test]
    fn a_bad_radius_is_refused() {
        assert!(dense_points(&[p(0.0, 0.0, 400.0)], 0.0, 0.2).is_err());
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
