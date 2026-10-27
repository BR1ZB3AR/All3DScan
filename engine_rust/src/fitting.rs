//! Fitting planes and cylinders to a cloud with RANSAC, for scan-to-CAD.
//!
//! Hypotheses come from a few points near each other (and, for a cylinder,
//! their normals) and are scored in parallel on a sample of the candidates. The best one is then
//! refined by least squares on all of its inliers. A point counts as an
//! inlier only if it is near the surface and its normal agrees with the
//! surface's, so other surfaces that merely pass through do not count. Other
//! shapes still pass those tests where the surface touches them (a strip of
//! a pipe or a cap of a ball near a plane, a strip of a wall or a band round a
//! ball near a cylinder), so a hypothesis whose inliers have the wrong shape
//! is dropped. A fit keeps only the largest connected piece of its inliers,
//! so that two objects that touch the same surface in different places do not
//! add up to one.
//! Coordinates are the sensor's: millimetres, x right, y down, z forward.

use std::collections::{HashMap, HashSet};

use nalgebra::{Matrix2, Matrix3, Matrix6, Point3, Vector3, Vector6};
use rayon::prelude::*;

/// A plane fitted to part of a cloud.
#[derive(Debug, Clone, PartialEq)]
pub struct PlaneFit {
    /// Unit normal, facing the sensor.
    pub normal: Vector3<f32>,
    /// The inliers' centroid, which lies on the plane.
    pub origin: Point3<f32>,
    /// Two unit axes in the plane, along the inliers' longest and shortest
    /// spread, and the inliers' extent along each, from the origin: together,
    /// the rectangle that holds them most closely.
    pub axes: [Vector3<f32>; 2],
    pub extent: [(f32, f32); 2],
    /// Indices into the cloud of the points on the plane: the largest
    /// connected piece of them.
    pub inliers: Vec<u32>,
    /// The candidates the fit takes from the cloud: see CLAIM_BAND.
    pub claimed: Vec<u32>,
    /// Root-mean-square distance of the inliers from the plane.
    pub rms_mm: f32,
}

/// A cylinder fitted to part of a cloud.
#[derive(Debug, Clone, PartialEq)]
pub struct CylinderFit {
    /// Unit axis direction, with its largest component positive.
    pub axis: Vector3<f32>,
    /// The point on the axis level with the lowest inlier: the cylinder's base.
    pub base: Point3<f32>,
    pub radius_mm: f32,
    /// The inliers' extent along the axis, from the base.
    pub height_mm: f32,
    /// Indices into the cloud of the points on the cylinder: the largest
    /// connected piece of them.
    pub inliers: Vec<u32>,
    /// The candidates the fit takes from the cloud: see CLAIM_BAND.
    pub claimed: Vec<u32>,
    /// Root-mean-square distance of the inliers from the cylinder's surface.
    pub rms_mm: f32,
}

/// Settings shared by the fits.
#[derive(Debug, Clone, Copy)]
pub struct Ransac {
    /// How far a point may be from the surface and still count, in mm.
    pub threshold_mm: f32,
    pub iterations: u32,
    /// The fewest inliers a fit needs before it is reported.
    pub min_inliers: usize,
    /// Seed for the random draws; the same seed gives the same fit.
    pub seed: u64,
}

/// Hypotheses are scored on at most this many candidates, spread over all of them.
const SCORE_SAMPLE: usize = 20_000;
/// An inlier's normal must be within about 37 degrees of a plane's.
const PLANE_NORMAL_AGREEMENT: f32 = 0.8;
/// An inlier's normal must be nearly square to a cylinder's axis...
const CYLINDER_AXIS_TOLERANCE: f32 = 0.3;
/// ...and point away from the axis, within about 45 degrees.
const CYLINDER_RADIAL_AGREEMENT: f32 = 0.7;
/// A fit also claims the candidates up to this many thresholds from its
/// surface whose normals agree with it: the tail of the noise its inliers
/// leave behind, which would otherwise be fitted again as a second surface
/// beside the first.
const CLAIM_BAND: f32 = 2.0;

/// Fits the plane with the most support among `candidates` (indices into
/// `points`, each with a unit normal in `normals`).
pub fn fit_plane(
    points: &[Point3<f32>],
    normals: &[Vector3<f32>],
    candidates: &[u32],
    settings: Ransac,
) -> Option<PlaneFit> {
    if candidates.len() < 3 {
        return None;
    }
    let sample = spread_sample(candidates);
    let neighbourhoods = Neighbourhoods::new(points, candidates);
    let threshold = settings.threshold_mm;
    let near = |k: u32, normal: &Vector3<f32>, on: &Point3<f32>, band: f32| {
        let k = k as usize;
        (points[k] - on).dot(normal).abs() < band
            && normals[k].dot(normal).abs() > PLANE_NORMAL_AGREEMENT
    };
    let supports = |k: u32, normal: &Vector3<f32>, on: &Point3<f32>| near(k, normal, on, threshold);

    let (_, _, mut normal, mut origin) = (0..settings.iterations)
        .into_par_iter()
        .filter_map(|i| {
            let mut rng = SplitMix64::for_iteration(settings.seed, i);
            let [a, b, c] = neighbourhoods.draw(&mut rng).map(|k| points[k as usize]);
            let normal = (b - a).cross(&(c - a)).try_normalize(1e-6)?;
            let bow = Bow::over(
                sample.iter().copied().filter(|&k| supports(k, &normal, &a)),
                points,
                &normal,
                &a,
            );
            (!bow.exceeds(threshold)).then_some((bow.count, std::cmp::Reverse(i), normal, a))
        })
        .max_by_key(|&(score, first, ..)| (score, first))?;

    let mut inliers = gather(candidates, |k| supports(k, &normal, &origin));
    for _ in 0..2 {
        if inliers.len() < settings.min_inliers.max(3) {
            return None;
        }
        (normal, origin) = least_squares_plane(points, &inliers);
        inliers = gather(candidates, |k| supports(k, &normal, &origin));
    }
    let piece = Piece::largest(points, &inliers);
    inliers.retain(|&k| piece.holds(points[k as usize]));
    if inliers.len() < settings.min_inliers.max(3) {
        return None;
    }
    (normal, origin) = least_squares_plane(points, &inliers);
    inliers.retain(|&k| supports(k, &normal, &origin));
    if inliers.len() < settings.min_inliers.max(3)
        || Bow::over(inliers.iter().copied(), points, &normal, &origin).exceeds(threshold)
    {
        return None;
    }
    if normal.dot(&origin.coords) > 0.0 {
        normal = -normal; // face the sensor, which sits at the origin
    }

    let axes = spread_axes(points, &inliers, &origin, &normal);
    let extent = axes.map(|axis| {
        inliers.iter().fold((f32::MAX, f32::MIN), |(lo, hi), &k| {
            let along = (points[k as usize] - origin).dot(&axis);
            (lo.min(along), hi.max(along))
        })
    });
    let rms_mm = rms(&inliers, |k| (points[k as usize] - origin).dot(&normal));
    let claimed = gather(candidates, |k| {
        piece.holds(points[k as usize]) && near(k, &normal, &origin, CLAIM_BAND * threshold)
    });
    Some(PlaneFit {
        normal,
        origin,
        axes,
        extent,
        inliers,
        claimed,
        rms_mm,
    })
}

/// Fits the convex cylinder (seen from outside) with the most support among
/// `candidates`, with a radius between `radius_mm.0` and `radius_mm.1`.
pub fn fit_cylinder(
    points: &[Point3<f32>],
    normals: &[Vector3<f32>],
    candidates: &[u32],
    settings: Ransac,
    radius_mm: (f32, f32),
) -> Option<CylinderFit> {
    if candidates.len() < 2 {
        return None;
    }
    let sample = spread_sample(candidates);
    let neighbourhoods = Neighbourhoods::new(points, candidates);
    let threshold = settings.threshold_mm;
    let near = |k: u32, axis: &Vector3<f32>, centre: &Point3<f32>, radius: f32, band: f32| {
        let k = k as usize;
        let offset = points[k] - centre;
        let radial = offset - axis * offset.dot(axis);
        let distance = radial.norm();
        distance > 0.0
            && (distance - radius).abs() < band
            && normals[k].dot(axis).abs() < CYLINDER_AXIS_TOLERANCE
            && normals[k].dot(&radial) / distance > CYLINDER_RADIAL_AGREEMENT
    };
    let supports = |k: u32, axis: &Vector3<f32>, centre: &Point3<f32>, radius: f32| {
        near(k, axis, centre, radius, threshold)
    };
    let in_range = |radius: f32| (radius_mm.0..=radius_mm.1).contains(&radius);

    let (_, _, mut axis, mut centre, mut radius) = (0..settings.iterations)
        .into_par_iter()
        .filter_map(|i| {
            let mut rng = SplitMix64::for_iteration(settings.seed, i);
            let [a, b] = neighbourhoods.draw(&mut rng).map(|k| k as usize);
            let (axis, centre, radius) =
                cylinder_through(points[a], normals[a], points[b], normals[b])?;
            if !in_range(radius) {
                return None;
            }
            let turns = Turns::over(
                sample
                    .iter()
                    .copied()
                    .filter(|&k| supports(k, &axis, &centre, radius)),
                points,
                normals,
                &axis,
                &centre,
            );
            turns.cylindrical(radius).then_some((
                turns.count(),
                std::cmp::Reverse(i),
                axis,
                centre,
                radius,
            ))
        })
        .max_by_key(|&(score, first, ..)| (score, first))?;

    let mut inliers = gather(candidates, |k| supports(k, &axis, &centre, radius));
    for _ in 0..3 {
        if inliers.len() < settings.min_inliers.max(3) {
            return None;
        }
        (axis, centre, radius) = least_squares_cylinder(points, normals, &inliers, axis);
        inliers = gather(candidates, |k| supports(k, &axis, &centre, radius));
    }
    let piece = Piece::largest(points, &inliers);
    inliers.retain(|&k| piece.holds(points[k as usize]));
    if inliers.len() < settings.min_inliers.max(3) {
        return None;
    }
    (axis, centre, radius) = least_squares_cylinder(points, normals, &inliers, axis);
    inliers.retain(|&k| supports(k, &axis, &centre, radius));
    if inliers.len() < settings.min_inliers.max(3)
        || !in_range(radius)
        || !Turns::over(inliers.iter().copied(), points, normals, &axis, &centre)
            .cylindrical(radius)
    {
        return None;
    }

    let (low, high) = inliers.iter().fold((f32::MAX, f32::MIN), |(lo, hi), &k| {
        let along = (points[k as usize] - centre).dot(&axis);
        (lo.min(along), hi.max(along))
    });
    let rms_mm = rms(&inliers, |k| {
        let offset = points[k as usize] - centre;
        (offset - axis * offset.dot(&axis)).norm() - radius
    });
    let claimed = gather(candidates, |k| {
        piece.holds(points[k as usize]) && near(k, &axis, &centre, radius, CLAIM_BAND * threshold)
    });
    Some(CylinderFit {
        axis,
        base: centre + axis * low,
        radius_mm: radius,
        height_mm: high - low,
        inliers,
        claimed,
        rms_mm,
    })
}

/// How far a candidate plane's inliers bow away from it. A quadric
/// d = a x² + b xy + c y² + d x + e y + f is fitted by least squares to their
/// distances d from the plane over their positions (x, y) in it, and the bow
/// is its curvature across two standard deviations of their spread each way.
/// A wall bows by no more than its noise does. A strip of a pipe or a cap of
/// a ball fills the band of points the threshold lets through, so it bows by
/// about twice the threshold.
struct Bow {
    count: usize,
    /// Sums of the products of the quadric's terms, and of each term times d.
    terms: Matrix6<f64>,
    target: Vector6<f64>,
}

impl Bow {
    /// In-plane positions are taken in units of this, so the terms stay of
    /// similar size.
    const UNIT_MM: f64 = 100.0;

    fn over(
        inliers: impl Iterator<Item = u32>,
        points: &[Point3<f32>],
        normal: &Vector3<f32>,
        on: &Point3<f32>,
    ) -> Self {
        let [u, v] = square_axes(normal);
        let mut bow = Self {
            count: 0,
            terms: Matrix6::zeros(),
            target: Vector6::zeros(),
        };
        for k in inliers {
            let offset = points[k as usize] - on;
            let x = f64::from(offset.dot(&u)) / Self::UNIT_MM;
            let y = f64::from(offset.dot(&v)) / Self::UNIT_MM;
            let row = Vector6::new(x * x, x * y, y * y, x, y, 1.0);
            bow.terms += row * row.transpose();
            bow.target += row * f64::from(offset.dot(normal));
            bow.count += 1;
        }
        bow
    }

    /// Whether the inliers bow by more than `limit_mm`. Too few of them, or
    /// too few directions among them, to fit the quadric to tell nothing.
    fn exceeds(&self, limit_mm: f32) -> bool {
        if self.count < 12 {
            return false;
        }
        let Some(solver) = self.terms.cholesky() else {
            return false;
        };
        let c = solver.solve(&self.target);
        let n = self.count as f64;
        let (mx, my) = (self.terms[(3, 5)] / n, self.terms[(4, 5)] / n);
        let (sxx, sxy, syy) = (
            self.terms[(3, 3)] / n - mx * mx,
            self.terms[(3, 4)] / n - mx * my,
            self.terms[(4, 4)] / n - my * my,
        );
        // The eigenvalues of curvature × spread are the curvature along each
        // principal direction times the spread along it. (They are real: the
        // product is similar to a symmetric matrix.)
        let product =
            Matrix2::new(c[0], c[1] / 2.0, c[1] / 2.0, c[2]) * Matrix2::new(sxx, sxy, sxy, syy);
        let half_trace = product.trace() / 2.0;
        let largest = half_trace.abs()
            + (half_trace * half_trace - product.determinant())
                .max(0.0)
                .sqrt();
        4.0 * largest > f64::from(limit_mm)
    }
}

/// How a candidate cylinder's inlier normals turn, which tells a cylinder
/// apart from the other shapes that pass its distance and normal tests where
/// it touches them. On a cylinder, each normal points straight out from the
/// axis. A band of points round a ball passes too, but there a normal leans
/// along the axis in step with the point's height: n·axis = h / r. A strip of
/// a wall passes where a large cylinder touches it, but there the normals all
/// point the same way instead of turning with the surface, so measured
/// against the direction straight out from the axis, they turn back by a
/// radian for each radian round it.
#[derive(Default)]
struct Turns {
    /// n·axis against the height along the axis.
    lean: Regression,
    /// The normal's sideways component, round the axis, against the angle round it.
    turn: Regression,
}

/// The lean's slope times the radius is 1 on an exact ball and 0 on a
/// cylinder. Measured, bands round the mock's ball score 0.28 to 0.49 and its
/// pipe 0.00: the inlier test keeps only normals nearly square to the axis,
/// which trims the steepest leans off a ball and flattens its slope.
const SPHERE_LEAN: f64 = 0.12;
/// The turn's slope is about -1 on a wall and 0 on a cylinder.
const FLAT_TURN: f64 = -0.5;

impl Turns {
    fn over(
        inliers: impl Iterator<Item = u32>,
        points: &[Point3<f32>],
        normals: &[Vector3<f32>],
        axis: &Vector3<f32>,
        centre: &Point3<f32>,
    ) -> Self {
        let mut turns = Self::default();
        // Angles round the axis are measured from the first inlier, so a strip
        // of a wall, which is narrow, never wraps round at ±180 degrees.
        let mut zero: Option<[Vector3<f32>; 2]> = None;
        for k in inliers {
            let (point, normal) = (points[k as usize], normals[k as usize]);
            let offset = point - centre;
            let height = offset.dot(axis);
            let Some(out) = (offset - axis * height).try_normalize(1e-6) else {
                continue;
            };
            let [x, y] = *zero.get_or_insert_with(|| [out, axis.cross(&out)]);
            let angle = out.dot(&y).atan2(out.dot(&x));
            turns
                .lean
                .add(f64::from(height), f64::from(normal.dot(axis)));
            turns
                .turn
                .add(f64::from(angle), f64::from(normal.dot(&axis.cross(&out))));
        }
        turns
    }

    fn count(&self) -> usize {
        self.lean.count
    }

    /// Whether the normals turn as they do on a cylinder of this radius.
    fn cylindrical(&self, radius: f32) -> bool {
        let sphere = self
            .lean
            .slope()
            .is_some_and(|s| s.abs() * f64::from(radius) > SPHERE_LEAN);
        let flat = self.turn.slope().is_some_and(|s| s < FLAT_TURN);
        !sphere && !flat
    }
}

/// A least-squares line through (x, y) pairs, kept as running sums.
#[derive(Default)]
struct Regression {
    count: usize,
    sum_x: f64,
    sum_y: f64,
    sum_xx: f64,
    sum_xy: f64,
}

impl Regression {
    fn add(&mut self, x: f64, y: f64) {
        self.count += 1;
        self.sum_x += x;
        self.sum_y += y;
        self.sum_xx += x * x;
        self.sum_xy += x * y;
    }

    /// The line's slope, if there are enough pairs, spread out enough, to tell.
    fn slope(&self) -> Option<f64> {
        if self.count < 3 {
            return None;
        }
        let n = self.count as f64;
        let (mean_x, mean_y) = (self.sum_x / n, self.sum_y / n);
        let variance = self.sum_xx / n - mean_x * mean_x;
        (variance > 1e-9).then(|| (self.sum_xy / n - mean_x * mean_y) / variance)
    }
}

/// The largest connected piece of a set of points. Each point falls in a cube
/// about three point spacings across, and points in touching cubes belong to
/// the same piece, so holes of a point or two do not split a surface but a
/// gap between two objects does.
struct Piece {
    cell_mm: f32,
    cells: HashSet<[i64; 3]>,
}

impl Piece {
    /// The spacing comes from how many points share a cube of this edge: a
    /// surface s apart crossing a cube of edge c puts about (c / s)² in it.
    const PROBE_MM: f32 = 10.0;

    fn largest(points: &[Point3<f32>], members: &[u32]) -> Self {
        let cell = |p: Point3<f32>, edge: f32| p.coords.map(|c| (c / edge).floor() as i64).into();
        let probed: HashSet<[i64; 3]> = members
            .iter()
            .map(|&k| cell(points[k as usize], Self::PROBE_MM))
            .collect();
        let per_cube = members.len() as f32 / probed.len().max(1) as f32;
        let cell_mm = 3.0 * Self::PROBE_MM / per_cube.sqrt();

        // Number the occupied cells in order of their first point, so the
        // result never depends on hash order, and join touching ones.
        let mut ids: HashMap<[i64; 3], usize> = HashMap::new();
        let mut keys = Vec::new();
        let member_cells: Vec<usize> = members
            .iter()
            .map(|&k| {
                let key = cell(points[k as usize], cell_mm);
                *ids.entry(key).or_insert_with(|| {
                    keys.push(key);
                    keys.len() - 1
                })
            })
            .collect();
        let mut parent: Vec<usize> = (0..keys.len()).collect();
        fn root(parent: &mut [usize], mut i: usize) -> usize {
            while parent[i] != i {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            i
        }
        for (id, key) in keys.iter().enumerate() {
            for offset in (0..27).filter(|&o| o != 13) {
                let near = [
                    key[0] + offset / 9 - 1,
                    key[1] + offset / 3 % 3 - 1,
                    key[2] + offset % 3 - 1,
                ];
                if let Some(&other) = ids.get(&near) {
                    let (a, b) = (root(&mut parent, id), root(&mut parent, other));
                    parent[a.max(b)] = a.min(b);
                }
            }
        }
        // The piece with the most points; on a tie, the one met first.
        let mut size = vec![0usize; keys.len()];
        for &id in &member_cells {
            size[root(&mut parent, id)] += 1;
        }
        let best = (0..keys.len()).max_by_key(|&id| (size[id], std::cmp::Reverse(id)));
        let cells = keys
            .iter()
            .enumerate()
            .filter(|&(id, _)| Some(root(&mut parent, id)) == best)
            .map(|(_, key)| *key)
            .collect();
        Self { cell_mm, cells }
    }

    fn holds(&self, p: Point3<f32>) -> bool {
        let key: [i64; 3] = p.coords.map(|c| (c / self.cell_mm).floor() as i64).into();
        self.cells.contains(&key)
    }
}

/// The cylinder through two surface points, from their normals: the axis is
/// square to both normals, and the centre is where the normal lines meet.
fn cylinder_through(
    p1: Point3<f32>,
    n1: Vector3<f32>,
    p2: Point3<f32>,
    n2: Vector3<f32>,
) -> Option<(Vector3<f32>, Point3<f32>, f32)> {
    let cross = n1.cross(&n2);
    if cross.norm() < 0.05 {
        return None; // nearly parallel normals fix no axis
    }
    let axis = canonical(cross.normalize());
    // Look along the axis: drop each point and normal onto the plane through the origin.
    let flat = |v: Vector3<f32>| v - axis * v.dot(&axis);
    let (q1, q2) = (flat(p1.coords), flat(p2.coords));
    let (m1, m2) = (flat(n1).try_normalize(1e-6)?, flat(n2).try_normalize(1e-6)?);
    // Closest points of the lines q1 + t m1 and q2 + s m2.
    let w = q1 - q2;
    let (b, d, e) = (m1.dot(&m2), m1.dot(&w), m2.dot(&w));
    let denominator = 1.0 - b * b;
    if denominator < 1e-6 {
        return None;
    }
    let t = (b * e - d) / denominator;
    let s = (e - b * d) / denominator;
    // Outward normals: the centre lies behind both surface points.
    if t >= 0.0 || s >= 0.0 {
        return None;
    }
    let centre = Point3::from((q1 + m1 * t + q2 + m2 * s) / 2.0);
    Some((axis, centre, (q1 - centre.coords).norm()))
}

/// Refines a cylinder on its inliers: the axis is the direction their normals
/// are all square to, and the centre and radius come from a least-squares
/// circle through the inliers seen along the axis.
fn least_squares_cylinder(
    points: &[Point3<f32>],
    normals: &[Vector3<f32>],
    inliers: &[u32],
    previous_axis: Vector3<f32>,
) -> (Vector3<f32>, Point3<f32>, f32) {
    let scatter = inliers.iter().fold(Matrix3::<f64>::zeros(), |sum, &k| {
        let n = normals[k as usize].cast::<f64>();
        sum + n * n.transpose()
    });
    let axis = smallest_eigenvector(scatter)
        .map(|v| canonical(v.cast::<f32>()))
        .unwrap_or(previous_axis);

    // Kasa's circle fit in the plane square to the axis: x² + y² = 2cx·x + 2cy·y + c.
    let [e1, e2] = square_axes(&axis);
    let (mut ata, mut atb) = (Matrix3::<f64>::zeros(), Vector3::<f64>::zeros());
    for &k in inliers {
        let p = points[k as usize].coords;
        let (x, y) = (f64::from(p.dot(&e1)), f64::from(p.dot(&e2)));
        let row = Vector3::new(2.0 * x, 2.0 * y, 1.0);
        ata += row * row.transpose();
        atb += row * (x * x + y * y);
    }
    let solution = ata.lu().solve(&atb).unwrap_or_else(Vector3::zeros);
    let (cx, cy) = (solution.x, solution.y);
    let radius = (solution.z + cx * cx + cy * cy).max(0.0).sqrt() as f32;
    let centre = Point3::from(e1 * cx as f32 + e2 * cy as f32);
    (axis, centre, radius)
}

/// The least-squares plane through the inliers: their centroid, and the
/// direction they spread least along.
fn least_squares_plane(points: &[Point3<f32>], inliers: &[u32]) -> (Vector3<f32>, Point3<f32>) {
    let n = inliers.len() as f64;
    let centroid = inliers.iter().fold(Vector3::<f64>::zeros(), |sum, &k| {
        sum + points[k as usize].coords.cast::<f64>()
    }) / n;
    let scatter = inliers.iter().fold(Matrix3::<f64>::zeros(), |sum, &k| {
        let d = points[k as usize].coords.cast::<f64>() - centroid;
        sum + d * d.transpose()
    });
    let normal = smallest_eigenvector(scatter)
        .unwrap_or_else(Vector3::z)
        .cast::<f32>();
    (normal, Point3::from(centroid.cast::<f32>()))
}

fn smallest_eigenvector(m: Matrix3<f64>) -> Option<Vector3<f64>> {
    let eigen = m.symmetric_eigen();
    let smallest = eigen.eigenvalues.imin();
    eigen
        .eigenvectors
        .column(smallest)
        .into_owned()
        .try_normalize(1e-12)
}

/// Two unit axes square to a unit direction and to each other; the first is
/// also square to y where it can be.
fn square_axes(direction: &Vector3<f32>) -> [Vector3<f32>; 2] {
    let first = direction
        .cross(&Vector3::y())
        .try_normalize(1e-3)
        .unwrap_or_else(|| direction.cross(&Vector3::x()).normalize());
    [first, direction.cross(&first)]
}

/// Two unit axes square to a plane's normal, along the directions its points
/// spread most and least.
fn spread_axes(
    points: &[Point3<f32>],
    inliers: &[u32],
    origin: &Point3<f32>,
    normal: &Vector3<f32>,
) -> [Vector3<f32>; 2] {
    let [u, v] = square_axes(normal);
    let (mut uu, mut uv, mut vv) = (0.0f64, 0.0f64, 0.0f64);
    for &k in inliers {
        let offset = points[k as usize] - origin;
        let (x, y) = (f64::from(offset.dot(&u)), f64::from(offset.dot(&v)));
        uu += x * x;
        uv += x * y;
        vv += y * y;
    }
    let angle = 0.5 * (2.0 * uv).atan2(uu - vv);
    let longest = canonical(u * angle.cos() as f32 + v * angle.sin() as f32);
    [longest, normal.cross(&longest)]
}

/// A direction with its largest component made positive, so that a fit's
/// axis does not flip from one run to the next.
fn canonical(v: Vector3<f32>) -> Vector3<f32> {
    if v[v.iamax()] < 0.0 { -v } else { v }
}

fn gather(candidates: &[u32], keep: impl Fn(u32) -> bool + Sync) -> Vec<u32> {
    candidates
        .par_iter()
        .copied()
        .filter(|&k| keep(k))
        .collect()
}

fn rms(inliers: &[u32], residual: impl Fn(u32) -> f32) -> f32 {
    let sum: f64 = inliers
        .iter()
        .map(|&k| f64::from(residual(k)).powi(2))
        .sum();
    (sum / inliers.len().max(1) as f64).sqrt() as f32
}

/// The candidates grouped by the cube they fall in, a thirty-second of the
/// diagonal of their bounding box across. A hypothesis draws its points from
/// one cube: the points of one surface lie together, so points drawn near
/// each other are far more often on the same surface than points drawn from
/// anywhere, which matters most for the smaller surfaces.
struct Neighbourhoods {
    /// The candidates, each cube's together.
    order: Vec<u32>,
    /// For each entry of `order`, the run of its cube's entries.
    run: Vec<(u32, u32)>,
}

impl Neighbourhoods {
    fn new(points: &[Point3<f32>], candidates: &[u32]) -> Self {
        let (lo, hi) = candidates
            .par_iter()
            .map(|&k| (points[k as usize], points[k as usize]))
            .reduce(
                || (Point3::from([f32::MAX; 3]), Point3::from([f32::MIN; 3])),
                |a, b| (a.0.inf(&b.0), a.1.sup(&b.1)),
            );
        let edge = ((hi - lo).norm() / 32.0).max(1.0);
        let mut keyed: Vec<([i64; 3], u32)> = candidates
            .par_iter()
            .map(|&k| {
                let key = (points[k as usize] - lo).map(|c| (c / edge).floor() as i64);
                (key.into(), k)
            })
            .collect();
        keyed.par_sort_by_key(|(key, _)| *key);
        let mut run = vec![(0, 0); keyed.len()];
        let mut start = 0;
        for cube in keyed.chunk_by(|a, b| a.0 == b.0) {
            let end = start + cube.len();
            run[start..end].fill((start as u32, end as u32));
            start = end;
        }
        Self {
            order: keyed.into_iter().map(|(_, k)| k).collect(),
            run,
        }
    }

    /// A candidate drawn from all of them, and N - 1 more from its cube, or
    /// from all of them if its cube has fewer than N.
    fn draw<const N: usize>(&self, rng: &mut SplitMix64) -> [u32; N] {
        let first = rng.below(self.order.len());
        let (start, end) = match self.run[first] {
            (start, end) if (end - start) as usize >= N => (start as usize, end as usize),
            _ => (0, self.order.len()),
        };
        let mut drawn = [self.order[first]; N];
        for slot in &mut drawn[1..] {
            *slot = self.order[start + rng.below(end - start)];
        }
        drawn
    }
}

/// Up to SCORE_SAMPLE candidates, evenly spread over all of them.
fn spread_sample(candidates: &[u32]) -> Vec<u32> {
    let step = candidates.len().div_ceil(SCORE_SAMPLE).max(1);
    candidates.iter().step_by(step).copied().collect()
}

/// A small, fast random generator; one per RANSAC iteration, so the draws do
/// not depend on which thread runs which iteration.
struct SplitMix64(u64);

impl SplitMix64 {
    fn for_iteration(seed: u64, iteration: u32) -> Self {
        Self(seed ^ u64::from(iteration).wrapping_mul(0x9E37_79B9_7F4A_7C15))
    }

    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hw::MockDepthSensor;
    use crate::processing::organized_normals;

    const SETTINGS: Ransac = Ransac {
        threshold_mm: 1.5,
        iterations: 2000,
        min_inliers: 50,
        seed: 7,
    };

    fn all(n: usize) -> Vec<u32> {
        (0..n as u32).collect()
    }

    /// A little deterministic noise, within ±amplitude.
    fn jitter(i: usize, amplitude: f32) -> f32 {
        let mut rng = SplitMix64::for_iteration(99, i as u32);
        (rng.next() % 10_000) as f32 / 10_000.0 * 2.0 * amplitude - amplitude
    }

    #[test]
    fn a_planes_rectangle_follows_its_points() {
        // A strip 160 mm by 30 mm at 500 mm, turned 30 degrees in the image.
        let (cos, sin) = (30f32.to_radians().cos(), 30f32.to_radians().sin());
        let (mut points, mut normals) = (Vec::new(), Vec::new());
        for i in 0..2400 {
            let (s, t) = ((i % 160) as f32 - 79.5, (i / 160) as f32 * 2.0 - 14.0);
            points.push(Point3::new(
                s * cos - t * sin,
                s * sin + t * cos,
                500.0 + jitter(i, 0.2),
            ));
            normals.push(-Vector3::z());
        }
        let fit = fit_plane(&points, &normals, &all(points.len()), SETTINGS).unwrap();
        assert!(
            fit.axes[0].dot(&Vector3::new(cos, sin, 0.0)).abs() > 0.999,
            "{}",
            fit.axes[0]
        );
        let [(u0, u1), (v0, v1)] = fit.extent;
        assert!(
            (u1 - u0 - 159.0).abs() < 0.5 && (v1 - v0 - 28.0).abs() < 0.5,
            "{:?}",
            fit.extent
        );
    }

    #[test]
    fn finds_a_tilted_plane_among_clutter() {
        // A plane z = 500 + 0.5x seen from the sensor, plus scattered junk points.
        let normal = Vector3::new(0.5, 0.0, -1.0).normalize();
        let mut points = Vec::new();
        let mut normals = Vec::new();
        for i in 0..4000 {
            let (x, y) = ((i % 80) as f32 * 2.0 - 80.0, (i / 80) as f32 * 2.0 - 50.0);
            points.push(Point3::new(x, y, 500.0 + 0.5 * x + jitter(i, 0.3)));
            normals.push(normal);
        }
        for i in 0..1000 {
            points.push(Point3::new(
                jitter(i, 200.0),
                jitter(i + 7, 200.0),
                300.0 + jitter(i + 13, 100.0),
            ));
            normals.push(Vector3::new(jitter(i, 1.0), jitter(i + 1, 1.0), -1.0).normalize());
        }
        let fit = fit_plane(&points, &normals, &all(points.len()), SETTINGS).unwrap();
        assert!((fit.normal - normal).norm() < 0.01, "{}", fit.normal);
        assert!(fit.inliers.len() >= 3990 && fit.inliers.iter().all(|&k| k < 4000));
        assert!(fit.rms_mm < 0.3, "{}", fit.rms_mm);
    }

    #[test]
    fn finds_a_cylinder_and_measures_it() {
        // The visible half of a pipe: radius 30, axis along y through (40, *, 450).
        let mut points = Vec::new();
        let mut normals = Vec::new();
        for i in 0..6000 {
            let angle = std::f32::consts::PI * (0.1 + 0.8 * (i % 100) as f32 / 99.0); // facing the sensor
            let outward = Vector3::new(angle.cos(), 0.0, -angle.sin());
            let y = (i / 100) as f32 * 2.0 - 60.0;
            let r = 30.0 + jitter(i, 0.2);
            points.push(Point3::new(40.0 + outward.x * r, y, 450.0 + outward.z * r));
            normals.push(outward);
        }
        let fit = fit_cylinder(
            &points,
            &normals,
            &all(points.len()),
            SETTINGS,
            (5.0, 500.0),
        )
        .unwrap();
        assert!(
            (fit.radius_mm - 30.0).abs() < 0.1,
            "radius {}",
            fit.radius_mm
        );
        assert!((fit.axis - Vector3::y()).norm() < 0.01, "axis {}", fit.axis);
        assert!(
            (fit.base.x - 40.0).abs() < 0.2 && (fit.base.z - 450.0).abs() < 0.2,
            "{}",
            fit.base
        );
        assert!((fit.base.y + 60.0).abs() < 0.5 && (fit.height_mm - 118.0).abs() < 1.0);
        assert!(fit.inliers.len() > 5900);
    }

    #[test]
    fn finds_a_small_pipe_in_front_of_a_large_wall() {
        // 600 pipe points among 48,400 on the wall: two points drawn from
        // anywhere would both be on the pipe about once in 6,700 draws.
        let (mut points, mut normals) = (Vec::new(), Vec::new());
        for i in 0..48_400 {
            let (x, y) = (
                (i % 220) as f32 * 2.0 - 220.0,
                (i / 220) as f32 * 2.0 - 220.0,
            );
            points.push(Point3::new(x, y, 600.0 + jitter(i, 0.5)));
            normals.push(Vector3::new(jitter(i, 0.1), jitter(i + 3, 0.1), -1.0).normalize());
        }
        for i in 0..600 {
            let angle = std::f32::consts::PI * (0.1 + 0.8 * (i % 40) as f32 / 39.0);
            let outward = Vector3::new(angle.cos(), 0.0, -angle.sin());
            let y = (i / 40) as f32 * 2.0 - 15.0;
            points.push(Point3::new(outward.x * 10.0, y, 400.0 + outward.z * 10.0));
            normals.push(outward);
        }
        for seed in 1..=3 {
            let settings = Ransac { seed, ..SETTINGS };
            let fit = fit_cylinder(
                &points,
                &normals,
                &all(points.len()),
                settings,
                (2.0, 500.0),
            )
            .unwrap_or_else(|| panic!("seed {seed}: no cylinder"));
            assert!(
                (fit.radius_mm - 10.0).abs() < 0.1,
                "seed {seed}: radius {}",
                fit.radius_mm
            );
            assert!(fit.inliers.iter().all(|&k| k >= 48_400));
        }
    }

    #[test]
    fn the_same_seed_gives_the_same_fit() {
        let frame = MockDepthSensor::new(160, 120, 3).unwrap().capture();
        let normals = organized_normals(&frame.points, &frame.depth_mm, 160, 120);
        let candidates = all(frame.points.len());
        let a = fit_plane(&frame.points, &normals, &candidates, SETTINGS);
        let b = fit_plane(&frame.points, &normals, &candidates, SETTINGS);
        assert_eq!(a, b);
    }

    fn mock_frame(seed: u64) -> (Vec<Point3<f32>>, Vec<Vector3<f32>>) {
        let frame = MockDepthSensor::new(640, 480, seed).unwrap().capture();
        let normals = organized_normals(&frame.points, &frame.depth_mm, 640, 480);
        (frame.points, normals)
    }

    fn without(candidates: &mut Vec<u32>, taken: &[u32]) {
        let taken: HashSet<u32> = taken.iter().copied().collect();
        candidates.retain(|k| !taken.contains(k));
    }

    #[test]
    fn isolates_the_mock_wall_then_the_pipe_then_nothing_else() {
        let (points, normals) = mock_frame(1);
        let mut candidates = all(points.len());

        let wall = fit_plane(&points, &normals, &candidates, SETTINGS).unwrap();
        assert!(
            (wall.normal - Vector3::new(0.0, 0.0, -1.0)).norm() < 0.01,
            "{}",
            wall.normal
        );
        assert!((wall.origin.z - 600.0).abs() < 1.0, "{}", wall.origin);
        without(&mut candidates, &wall.claimed);
        // The pipe and the ball are curved: no strip or cap of them is a plane.
        assert_eq!(fit_plane(&points, &normals, &candidates, SETTINGS), None);

        let pipe = fit_cylinder(&points, &normals, &candidates, SETTINGS, (5.0, 500.0)).unwrap();
        assert!(
            (pipe.radius_mm - 40.0).abs() < 0.5,
            "radius {}",
            pipe.radius_mm
        );
        assert!(
            (pipe.axis - Vector3::y()).norm() < 0.02,
            "axis {}",
            pipe.axis
        );
        assert!(
            (pipe.base.x - 175.0).abs() < 1.0 && (pipe.base.z - 470.0).abs() < 1.0,
            "{}",
            pipe.base
        );
        assert!(
            (pipe.height_mm - 220.0).abs() < 3.0,
            "height {}",
            pipe.height_mm
        );
        without(&mut candidates, &pipe.claimed);
        // The ball is all that is left, and it is neither.
        assert_eq!(fit_plane(&points, &normals, &candidates, SETTINGS), None);
        assert_eq!(
            fit_cylinder(&points, &normals, &candidates, SETTINGS, (5.0, 500.0)),
            None
        );
    }

    #[test]
    fn finds_the_pipe_with_the_wall_still_in_the_cloud() {
        // Strips of the wall pass a large cylinder's distance and normal tests.
        for seed in 1..=4 {
            let (points, normals) = mock_frame(seed);
            let pipe = fit_cylinder(
                &points,
                &normals,
                &all(points.len()),
                SETTINGS,
                (5.0, 500.0),
            )
            .unwrap();
            assert!(
                (pipe.radius_mm - 40.0).abs() < 0.5 && (pipe.axis - Vector3::y()).norm() < 0.02,
                "seed {seed}: radius {}, axis {}",
                pipe.radius_mm,
                pipe.axis
            );
        }
    }

    #[test]
    fn a_fit_claims_the_noise_tail_beside_its_inliers() {
        let (points, normals) = mock_frame(2);
        let wall = fit_plane(&points, &normals, &all(points.len()), SETTINGS).unwrap();
        let inliers: HashSet<u32> = wall.inliers.iter().copied().collect();
        assert!(wall.claimed.len() > wall.inliers.len());
        assert!(inliers.iter().all(|k| wall.claimed.contains(k)));
        for &k in &wall.claimed {
            let distance = (points[k as usize] - wall.origin).dot(&wall.normal).abs();
            assert!(distance < CLAIM_BAND * SETTINGS.threshold_mm, "{distance}");
        }
    }

    #[test]
    fn a_curved_strip_is_not_taken_for_a_plane() {
        // A pipe of radius 30 seen from the front, with exact normals.
        let (mut points, mut normals) = (Vec::new(), Vec::new());
        for i in 0..6000 {
            let angle = std::f32::consts::PI * (0.1 + 0.8 * (i % 100) as f32 / 99.0);
            let outward = Vector3::new(angle.cos(), 0.0, -angle.sin());
            points.push(Point3::new(
                outward.x * 30.0,
                (i / 100) as f32 * 2.0,
                450.0 + outward.z * 30.0,
            ));
            normals.push(outward);
        }
        assert_eq!(
            fit_plane(&points, &normals, &all(points.len()), SETTINGS),
            None
        );
    }

    #[test]
    fn a_wall_is_not_taken_for_a_cylinder() {
        // A wall 300 mm across at 600 mm, with normals as noisy as measured ones.
        let (mut points, mut normals) = (Vec::new(), Vec::new());
        for i in 0..22_500 {
            let (x, y) = (
                (i % 150) as f32 * 2.0 - 150.0,
                (i / 150) as f32 * 2.0 - 150.0,
            );
            points.push(Point3::new(x, y, 600.0 + jitter(i, 0.5)));
            normals.push(Vector3::new(jitter(i, 0.15), jitter(i + 3, 0.15), -1.0).normalize());
        }
        assert_eq!(
            fit_cylinder(
                &points,
                &normals,
                &all(points.len()),
                SETTINGS,
                (5.0, 500.0)
            ),
            None
        );
    }

    #[test]
    fn a_plane_keeps_only_its_largest_connected_piece() {
        // Two patches of one plane, 100 mm apart: the larger is the surface found.
        let (mut points, mut normals) = (Vec::new(), Vec::new());
        for (x0, side) in [(-200.0, 60), (100.0, 40)] {
            for i in 0..side * side {
                let (x, y) = (x0 + (i % side) as f32 * 2.0, (i / side) as f32 * 2.0 - 50.0);
                points.push(Point3::new(x, y, 500.0 + jitter(i as usize, 0.3)));
                normals.push(-Vector3::z());
            }
        }
        let fit = fit_plane(&points, &normals, &all(points.len()), SETTINGS).unwrap();
        assert_eq!(fit.inliers.len(), 3600);
        assert!(fit.inliers.iter().chain(&fit.claimed).all(|&k| k < 3600));
    }

    #[test]
    fn a_ball_is_not_taken_for_a_cylinder() {
        // The half of a 120 mm ball that faces the sensor, with exact normals.
        let (mut points, mut normals) = (Vec::new(), Vec::new());
        for i in 0..90 {
            for j in 0..90 {
                let (lat, lon) = (-1.4 + 2.8 * i as f32 / 89.0, -1.4 + 2.8 * j as f32 / 89.0);
                let n = Vector3::new(lat.cos() * lon.sin(), lat.sin(), -lat.cos() * lon.cos());
                points.push(Point3::new(0.0, 0.0, 450.0) + n * 120.0);
                normals.push(n);
            }
        }
        let candidates = all(points.len());
        assert_eq!(
            fit_cylinder(&points, &normals, &candidates, SETTINGS, (5.0, 500.0)),
            None
        );
    }

    #[test]
    fn too_little_support_finds_nothing() {
        let points = vec![Point3::new(0.0, 0.0, 400.0), Point3::new(1.0, 0.0, 400.0)];
        let normals = vec![-Vector3::z(); 2];
        assert_eq!(fit_plane(&points, &normals, &all(2), SETTINGS), None);
        assert_eq!(
            fit_cylinder(&points, &normals, &all(2), SETTINGS, (5.0, 500.0)),
            None
        );
    }
}
