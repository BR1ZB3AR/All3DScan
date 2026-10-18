//! Loop closure for scan tracking.
//!
//! Frame-to-frame alignment is never exact. Each small error carries into the
//! next pose, so after sweeping around an object the far side no longer lines
//! up with where it started — the model gains two skins a few millimetres
//! apart instead of one. The tracker's relocalisation rescues *tracking* when
//! it recognises an earlier keyframe, but it does not go back and repair the
//! drift accumulated on the way round.
//!
//! This repairs it. Given the chain of keyframe poses and the loop closures
//! observed between non-adjacent keyframes, it spreads each closure's error
//! back over the poses in that loop, so the ends meet.
//!
//! The pose chain is the only thing that moves. Nothing here touches depth
//! data, and the caller re-fuses the model from the corrected poses.

/// A rigid transform as a row-major 4x4, matching `RigidTransform` on the C++
/// side so poses can cross the FFI boundary unchanged.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub m: [f64; 16],
}

impl Pose {
    pub const IDENTITY: Pose = Pose {
        m: [
            1.0, 0.0, 0.0, 0.0, //
            0.0, 1.0, 0.0, 0.0, //
            0.0, 0.0, 1.0, 0.0, //
            0.0, 0.0, 0.0, 1.0,
        ],
    };

    pub fn translation(&self) -> [f64; 3] {
        [self.m[3], self.m[7], self.m[11]]
    }

    /// Rigid inverse: transpose the rotation, negate the rotated translation.
    pub fn inverted(&self) -> Pose {
        let m = &self.m;
        let mut out = [0.0f64; 16];
        for r in 0..3 {
            for c in 0..3 {
                out[r * 4 + c] = m[c * 4 + r];
            }
        }
        for r in 0..3 {
            out[r * 4 + 3] =
                -(out[r * 4] * m[3] + out[r * 4 + 1] * m[7] + out[r * 4 + 2] * m[11]);
        }
        out[15] = 1.0;
        Pose { m: out }
    }

    pub fn mul(&self, rhs: &Pose) -> Pose {
        let mut out = [0.0f64; 16];
        for r in 0..4 {
            for c in 0..4 {
                let mut s = 0.0;
                for k in 0..4 {
                    s += self.m[r * 4 + k] * rhs.m[k * 4 + c];
                }
                out[r * 4 + c] = s;
            }
        }
        Pose { m: out }
    }

    /// Nearest true rotation, by Gram-Schmidt. Chained pose arithmetic drifts
    /// away from orthonormal, and an un-repaired chain amplifies that until
    /// poses are visibly stretched.
    pub fn orthonormalized(&self) -> Pose {
        let m = &self.m;
        let mut x = [m[0], m[1], m[2]];
        let mut y = [m[4], m[5], m[6]];
        let nx = (x[0] * x[0] + x[1] * x[1] + x[2] * x[2]).sqrt();
        if nx < 1e-12 {
            return Pose::IDENTITY;
        }
        for v in x.iter_mut() {
            *v /= nx;
        }
        let d = x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
        for i in 0..3 {
            y[i] -= d * x[i];
        }
        let ny = (y[0] * y[0] + y[1] * y[1] + y[2] * y[2]).sqrt();
        if ny < 1e-12 {
            return Pose::IDENTITY;
        }
        for v in y.iter_mut() {
            *v /= ny;
        }
        let z = [
            x[1] * y[2] - x[2] * y[1],
            x[2] * y[0] - x[0] * y[2],
            x[0] * y[1] - x[1] * y[0],
        ];
        Pose {
            m: [
                x[0], x[1], x[2], m[3], //
                y[0], y[1], y[2], m[7], //
                z[0], z[1], z[2], m[11], //
                0.0, 0.0, 0.0, 1.0,
            ],
        }
    }

    /// Rotation angle in radians.
    pub fn rotation_angle(&self) -> f64 {
        let trace = self.m[0] + self.m[5] + self.m[10];
        (((trace - 1.0) * 0.5).clamp(-1.0, 1.0)).acos()
    }

    /// This transform scaled towards identity: `s` of its rotation and
    /// translation. Used to apply a fraction of a correction to one pose.
    pub fn scaled(&self, s: f64) -> Pose {
        let o = self.orthonormalized();
        let angle = o.rotation_angle();
        let t = [o.m[3] * s, o.m[7] * s, o.m[11] * s];
        if angle <= 1e-12 || angle >= std::f64::consts::PI - 1e-9 {
            // Pure translation, or a half turn where the axis is ill-defined.
            let mut m = Pose::IDENTITY.m;
            m[3] = t[0];
            m[7] = t[1];
            m[11] = t[2];
            return Pose { m };
        }
        // Rotation axis from the skew part, angle scaled by s.
        let k = 1.0 / (2.0 * angle.sin());
        let axis = [
            k * (o.m[9] - o.m[6]),
            k * (o.m[2] - o.m[8]),
            k * (o.m[4] - o.m[1]),
        ];
        from_axis_angle_translation(axis, angle * s, t)
    }
}

fn from_axis_angle_translation(axis: [f64; 3], angle: f64, t: [f64; 3]) -> Pose {
    let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    if n < 1e-15 {
        let mut m = Pose::IDENTITY.m;
        m[3] = t[0];
        m[7] = t[1];
        m[11] = t[2];
        return Pose { m };
    }
    let (kx, ky, kz) = (axis[0] / n, axis[1] / n, axis[2] / n);
    let (c, s) = (angle.cos(), angle.sin());
    let cc = 1.0 - c;
    Pose {
        m: [
            c + kx * kx * cc,
            kx * ky * cc - kz * s,
            kx * kz * cc + ky * s,
            t[0],
            ky * kx * cc + kz * s,
            c + ky * ky * cc,
            ky * kz * cc - kx * s,
            t[1],
            kz * kx * cc - ky * s,
            kz * ky * cc + kx * s,
            c + kz * kz * cc,
            t[2],
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    }
}

/// An observed alignment between two keyframes that are not neighbours in the
/// chain: "frame `to` really sits at `relative` from frame `from`".
#[derive(Debug, Clone, Copy)]
pub struct LoopClosure {
    pub from: usize,
    pub to: usize,
    /// Measured transform taking `from`'s frame to `to`'s frame.
    pub relative: Pose,
    /// How much to trust it, 0..1 — the aligner's overlap score.
    pub weight: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct LoopClosureReport {
    pub closures_applied: usize,
    /// Worst mismatch before correcting: how far the chain's own idea of a
    /// keyframe sat from where the loop closure measured it. Reported as both
    /// a distance and an angle, because a sweep that returns to its start can
    /// close almost perfectly in position while being degrees out in
    /// orientation — and it is the orientation error that splits a surface
    /// into two skins.
    pub max_translation_error_before_m: f64,
    pub max_translation_error_after_m: f64,
    pub max_rotation_error_before_rad: f64,
    pub max_rotation_error_after_rad: f64,
}

/// Spreads each closure's error back over the poses between its endpoints.
///
/// This is the cheap, well-behaved form of pose-graph optimisation: rather
/// than solving the whole graph at once, each loop's mismatch is distributed
/// along the chain it spans, weighted by how far round the loop each pose sits.
/// A pose at the start of the loop barely moves, one near the far end takes
/// almost the whole correction, which is how the error actually accumulated.
///
/// `iterations` lets overlapping loops settle against each other.
pub fn apply_loop_closures(
    poses: &mut [Pose],
    closures: &[LoopClosure],
    iterations: usize,
) -> LoopClosureReport {
    let mut report = LoopClosureReport::default();
    if poses.len() < 2 || closures.is_empty() {
        return report;
    }

    // (distance, angle) between where the chain puts `to` and where the
    // closure says it belongs.
    let error_of = |poses: &[Pose], c: &LoopClosure| -> (f64, f64) {
        let predicted = poses[c.from].mul(&c.relative);
        let residual = predicted.mul(&poses[c.to].inverted()).orthonormalized();
        let t = residual.translation();
        (
            (t[0] * t[0] + t[1] * t[1] + t[2] * t[2]).sqrt(),
            residual.rotation_angle(),
        )
    };

    for c in closures {
        if c.from < poses.len() && c.to < poses.len() {
            let (t, r) = error_of(poses, c);
            report.max_translation_error_before_m = report.max_translation_error_before_m.max(t);
            report.max_rotation_error_before_rad = report.max_rotation_error_before_rad.max(r);
        }
    }

    for _ in 0..iterations.max(1) {
        for c in closures {
            if c.from >= poses.len() || c.to >= poses.len() || c.from == c.to {
                continue;
            }
            let (lo, hi) = if c.from < c.to {
                (c.from, c.to)
            } else {
                (c.to, c.from)
            };
            // Where the closure says `to` should be, against where the chain
            // currently puts it. correction maps the latter onto the former.
            let predicted = poses[c.from].mul(&c.relative);
            let correction = predicted.mul(&poses[c.to].inverted()).orthonormalized();

            let span = (hi - lo) as f64;
            let w = c.weight.clamp(0.0, 1.0);
            for i in lo..=hi {
                // Poses early in the loop have accumulated little of the
                // error, so they take little of the correction.
                let frac = if span > 0.0 {
                    (i - lo) as f64 / span
                } else {
                    1.0
                };
                let part = correction.scaled(frac * w);
                poses[i] = part.mul(&poses[i]).orthonormalized();
            }
            // Everything recorded after the loop rides along with its end, so
            // the chain past the loop stays rigid relative to it.
            let tail = correction.scaled(w);
            for i in (hi + 1)..poses.len() {
                poses[i] = tail.mul(&poses[i]).orthonormalized();
            }
            report.closures_applied += 1;
        }
    }

    for c in closures {
        if c.from < poses.len() && c.to < poses.len() {
            let (t, r) = error_of(poses, c);
            report.max_translation_error_after_m = report.max_translation_error_after_m.max(t);
            report.max_rotation_error_after_rad = report.max_rotation_error_after_rad.max(r);
        }
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    fn yaw_translate(deg: f64, tx: f64, ty: f64, tz: f64) -> Pose {
        let a = deg.to_radians();
        Pose {
            m: [
                a.cos(), -a.sin(), 0.0, tx, //
                a.sin(), a.cos(), 0.0, ty, //
                0.0, 0.0, 1.0, tz, //
                0.0, 0.0, 0.0, 1.0,
            ],
        }
    }

    #[test]
    fn inverse_and_multiply_round_trip() {
        let p = yaw_translate(23.0, 0.1, -0.2, 0.3);
        let i = p.inverted().mul(&p);
        for (k, v) in i.m.iter().enumerate() {
            assert!((v - Pose::IDENTITY.m[k]).abs() < 1e-12, "element {k}");
        }
    }

    #[test]
    fn scaling_a_transform_splits_it() {
        let p = yaw_translate(40.0, 0.2, 0.0, 0.0);
        let half = p.scaled(0.5);
        assert!((half.rotation_angle() - p.rotation_angle() / 2.0).abs() < 1e-9);
        assert!((half.translation()[0] - 0.1).abs() < 1e-12);
        // Scaling by one must change nothing.
        let same = p.scaled(1.0);
        for (k, v) in same.m.iter().enumerate() {
            assert!((v - p.m[k]).abs() < 1e-9, "element {k}");
        }
    }

    /// The case this exists for: a sweep that returns to its start, with a
    /// little error added at every step, so the two ends miss each other.
    #[test]
    fn closing_a_loop_removes_the_accumulated_drift() {
        let n = 36;
        let mut poses = Vec::with_capacity(n);
        let mut current = Pose::IDENTITY;
        for _ in 0..n {
            poses.push(current);
            // 10 degrees per step round the object, plus a persistent 1 mm
            // and 0.3 degree error — small per step, 3.6 cm by the end.
            current = current.mul(&yaw_translate(10.3, 0.001, 0.0, 0.0));
        }

        // Having come all the way round, the scan recognises its first
        // keyframe: the true relation between pose 0 and pose 35.
        let truth: Vec<Pose> = (0..n)
            .scan(Pose::IDENTITY, |acc, _| {
                let out = *acc;
                *acc = acc.mul(&yaw_translate(10.0, 0.0, 0.0, 0.0));
                Some(out)
            })
            .collect();
        let relative = truth[0].inverted().mul(&truth[n - 1]);
        let closure = LoopClosure {
            from: 0,
            to: n - 1,
            relative,
            weight: 1.0,
        };

        let report = apply_loop_closures(&mut poses, &[closure], 8);
        // 36 steps that are each 0.3 degrees too far leave the loop about
        // 10 degrees out of true by the time it comes back round.
        assert!(
            report.max_rotation_error_before_rad > 0.15,
            "test should start with real drift, had {} rad",
            report.max_rotation_error_before_rad
        );
        assert!(
            report.max_rotation_error_after_rad < report.max_rotation_error_before_rad * 0.1,
            "rotation drift {} -> {} rad",
            report.max_rotation_error_before_rad,
            report.max_rotation_error_after_rad
        );
        assert!(
            report.max_translation_error_after_m <= report.max_translation_error_before_m + 1e-6,
            "closing the loop must not make position worse"
        );
        // Every pose must stay a true rigid transform.
        for p in &poses {
            let r = p.orthonormalized();
            for (k, v) in r.m.iter().enumerate() {
                assert!((v - p.m[k]).abs() < 1e-9, "pose drifted from rigid at {k}");
            }
        }
    }

    #[test]
    fn a_weak_closure_is_applied_gently() {
        let n = 10;
        let mut strong: Vec<Pose> = (0..n).map(|i| yaw_translate(0.0, i as f64 * 0.01, 0.0, 0.0)).collect();
        let mut weak = strong.clone();
        let relative = yaw_translate(0.0, 0.05, 0.0, 0.0); // says the end is nearer
        let base = LoopClosure { from: 0, to: n - 1, relative, weight: 1.0 };
        apply_loop_closures(&mut strong, &[base], 1);
        apply_loop_closures(&mut weak, &[LoopClosure { weight: 0.25, ..base }], 1);
        let moved = |a: &[Pose], b: &[Pose]| (a[n - 1].translation()[0] - b[n - 1].translation()[0]).abs();
        let original: Vec<Pose> = (0..n).map(|i| yaw_translate(0.0, i as f64 * 0.01, 0.0, 0.0)).collect();
        assert!(moved(&strong, &original) > moved(&weak, &original));
    }

    #[test]
    fn no_closures_changes_nothing() {
        let mut poses = vec![Pose::IDENTITY, yaw_translate(5.0, 0.1, 0.0, 0.0)];
        let before = poses.clone();
        let report = apply_loop_closures(&mut poses, &[], 4);
        assert_eq!(report.closures_applied, 0);
        assert_eq!(poses, before);
    }

    #[test]
    fn out_of_range_closures_are_ignored() {
        let mut poses = vec![Pose::IDENTITY; 3];
        let bad = LoopClosure { from: 0, to: 99, relative: Pose::IDENTITY, weight: 1.0 };
        let report = apply_loop_closures(&mut poses, &[bad], 1);
        assert_eq!(report.closures_applied, 0);
    }
}
