//! Building one cloud out of a scan's frames, bridged with cxx.
//!
//! The alignment is the Qt app's own tracker (src/processing/Registration),
//! wrapped by fusion_cpp: each frame is aligned onto the model built so far
//! with point-to-plane ICP. A frame that cannot be placed is left out rather
//! than smeared across the model.
//!
//! The registration carries only points and colours, so the model is kept
//! here instead: each placed frame's points *and normals* are moved by the
//! pose it was given and merged in, thinned with
//! [`processing::voxel_downsample_cloud`], which averages both. Fitting a
//! cylinder needs real normals, and shading a surface looks wrong without
//! them.

use nalgebra::{Matrix3, Vector3};

use crate::mesh::{self, Surface};
use crate::processing::{self, Cloud, ProcessingError};

#[cxx::bridge(namespace = "all3dscan::fusion")]
mod ffi {
    unsafe extern "C++" {
        include!("all3dscan/FrameFusion.h");

        type FrameFusion;

        #[cxx_name = "makeFrameFusion"]
        fn make_frame_fusion(voxel_mm: f32) -> Result<UniquePtr<FrameFusion>>;

        /// # Safety
        /// `xyz` must point to `points * 3` floats and `rgb` to `points * 3`
        /// bytes, both readable for the length of the call.
        unsafe fn add(
            self: Pin<&mut FrameFusion>,
            xyz: *const f32,
            points: u64,
            rgb: *const u8,
        ) -> Result<bool>;

        #[cxx_name = "lastPose"]
        fn last_pose(self: &FrameFusion) -> &CxxVector<f64>;
        #[cxx_name = "lastOverlap"]
        fn last_overlap(self: &FrameFusion) -> f64;
        #[cxx_name = "lastRmseMm"]
        fn last_rmse_mm(self: &FrameFusion) -> f64;
        fn clear(self: Pin<&mut FrameFusion>);
    }
}

// SAFETY: a FrameFusion is the model cloud and the ICP target built from it,
// neither tied to the thread that made them. Calls go through & and &mut, so
// they never overlap.
unsafe impl Send for ffi::FrameFusion {}

/// Why fusing a frame failed.
#[derive(Debug)]
pub enum FusionError {
    /// The alignment refused the frame's buffers.
    Align(cxx::Exception),
    /// Merging the placed frame into the model failed.
    Merge(ProcessingError),
}

impl std::fmt::Display for FusionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Align(e) => write!(f, "{}", e.what()),
            Self::Merge(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for FusionError {}

impl From<cxx::Exception> for FusionError {
    fn from(e: cxx::Exception) -> Self {
        Self::Align(e)
    }
}

impl From<ProcessingError> for FusionError {
    fn from(e: ProcessingError) -> Self {
        Self::Merge(e)
    }
}

/// How a frame sat on the model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrameFit {
    /// Whether the frame was aligned well enough to be merged.
    pub merged: bool,
    /// The share of the frame's points lying on the model's surface, and
    /// their RMS distance from it. Both are 0 for the first frame, which
    /// becomes the model without being aligned to anything.
    pub overlap: f64,
    pub rmse_mm: f64,
    /// Points in the model after this frame.
    pub model_points: u64,
}

/// A depth view kept so that a surface can be built from the scan later.
/// Only some frames are kept: enough to see the object from every side,
/// without holding every frame of a scan in memory.
struct Keyframe {
    depth_mm: Vec<u16>,
    width: u32,
    height: u32,
    pose: [f64; 16],
}

/// How far the scanner must have moved since the last view kept before
/// another is worth keeping.
const KEYFRAME_SHIFT_MM: f32 = 8.0;
const KEYFRAME_TURN: f32 = 0.10; // radians, about 6 degrees
/// ...but one is kept every this many merged frames however still the
/// scanner is held. A scanner resting on a desk hardly moves, and without
/// this it kept three views of a whole scan, which is not enough to melt a
/// surface out of: the noise has to average over several looks at the same
/// place to settle.
const KEYFRAME_EVERY: u64 = 4;
/// How many are ever held.
const KEYFRAMES: usize = 120;

/// Builds one point cloud out of a scan's frames.
pub struct Fusion {
    inner: cxx::UniquePtr<ffi::FrameFusion>,
    /// The model, with a colour and a normal per point: what the fusion is
    /// for. The C++ side keeps its own copy to align against.
    model: Cloud,
    voxel_mm: f32,
    frames: u64,
    merged: u64,
    /// Views kept for building a surface, and where the last one was taken.
    keyframes: Vec<Keyframe>,
    last_kept: Option<(Matrix3<f32>, Vector3<f32>)>,
    /// Which merged frame the last view was kept from.
    last_kept_at: u64,
}

impl Fusion {
    /// A fusion keeping one point per `voxel_mm` cube. Errors unless the
    /// voxel is over 0 and at most 50 mm.
    pub fn new(voxel_mm: f32) -> Result<Self, cxx::Exception> {
        Ok(Self {
            inner: ffi::make_frame_fusion(voxel_mm)?,
            model: Cloud::default(),
            voxel_mm,
            frames: 0,
            merged: 0,
            keyframes: Vec::new(),
            last_kept: None,
            last_kept_at: 0,
        })
    }

    /// Aligns a frame onto the model and merges it, carrying its colours and
    /// normals across. All three must be the same length.
    pub fn add(&mut self, frame: &Cloud) -> Result<FrameFit, FusionError> {
        let n = frame.points.len();
        assert!(
            frame.colors.len() == n && frame.normals.len() == n,
            "a frame needs one colour and one normal per point"
        );
        self.frames += 1;
        // SAFETY: both pointers come from slices of the asserted length, and
        // C++ only reads them during the call.
        let merged = unsafe {
            self.inner.pin_mut().add(
                frame.points.as_ptr().cast::<f32>(),
                n as u64,
                frame.colors.as_ptr().cast::<u8>(),
            )
        }?;
        if merged {
            self.merged += 1;
            let (rotation, shift) = self.last_pose();
            self.model
                .points
                .extend(frame.points.iter().map(|p| rotation * p + shift));
            self.model
                .normals
                .extend(frame.normals.iter().map(|n| rotation * n));
            self.model.colors.extend_from_slice(&frame.colors);
            self.model = processing::voxel_downsample_cloud(&self.model, self.voxel_mm)?;
        }
        Ok(FrameFit {
            merged,
            overlap: self.inner.last_overlap(),
            rmse_mm: self.inner.last_rmse_mm(),
            model_points: self.model.points.len() as u64,
        })
    }

    /// Keeps this frame's depth image, if the scanner has moved far enough
    /// since the last one kept to show the object from a new side. The
    /// frame must have just been merged, so that the pose is its own.
    pub fn keep_view(&mut self, depth_mm: &[u16], width: u32, height: u32) {
        if depth_mm.len() != (width as usize) * (height as usize) {
            return;
        }
        let (rotation, shift) = self.last_pose();
        if let Some((was_rotation, was_shift)) = self.last_kept {
            let moved = (shift - was_shift).norm();
            // The angle between the two rotations, from the trace of R1ᵀ R2.
            let turned = (((was_rotation.transpose() * rotation).trace() - 1.0) / 2.0)
                .clamp(-1.0, 1.0)
                .acos();
            let waited = self.merged.saturating_sub(self.last_kept_at) >= KEYFRAME_EVERY;
            if moved < KEYFRAME_SHIFT_MM && turned < KEYFRAME_TURN && !waited {
                return;
            }
        }
        if self.keyframes.len() >= KEYFRAMES {
            // Keep the spread: drop every other one and carry on, so the
            // views still cover the whole scan rather than only its start.
            let mut kept = Vec::with_capacity(KEYFRAMES / 2 + 1);
            for (at, frame) in std::mem::take(&mut self.keyframes).into_iter().enumerate() {
                if at % 2 == 0 {
                    kept.push(frame);
                }
            }
            self.keyframes = kept;
        }
        let m = self.inner.last_pose().as_slice();
        let mut pose = [0.0f64; 16];
        pose.copy_from_slice(&m[..16]);
        self.keyframes.push(Keyframe {
            depth_mm: depth_mm.to_vec(),
            width,
            height,
            pose,
        });
        self.last_kept = Some((rotation, shift));
        self.last_kept_at = self.merged;
    }

    /// How many depth views are being held for the surface.
    pub fn views(&self) -> usize {
        self.keyframes.len()
    }

    /// Melts the kept views into one closed surface over the model. The
    /// intrinsics are the depth camera's: fx, fy, cx, cy.
    pub fn surface(
        &self,
        intrinsics: [f64; 4],
        voxel_mm: f32,
        smoothing: i32,
    ) -> Result<Surface, cxx::Exception> {
        let mut builder = mesh::SurfaceBuilder::new(voxel_mm)?;
        for view in &self.keyframes {
            builder.add_view(
                &view.depth_mm,
                view.width,
                view.height,
                intrinsics,
                &view.pose,
            )?;
        }
        builder.build(&self.model.points, smoothing)
    }

    /// Where the last merged frame was placed: its rotation, and its shift
    /// in millimetres.
    fn last_pose(&self) -> (Matrix3<f32>, Vector3<f32>) {
        let m = self.inner.last_pose().as_slice();
        debug_assert_eq!(m.len(), 16, "a pose is a 4x4 matrix");
        let at = |row: usize, column: usize| m[row * 4 + column] as f32;
        (
            // The pose is row-major; Matrix3::new takes rows too.
            Matrix3::new(
                at(0, 0),
                at(0, 1),
                at(0, 2),
                at(1, 0),
                at(1, 1),
                at(1, 2),
                at(2, 0),
                at(2, 1),
                at(2, 2),
            ),
            Vector3::new(at(0, 3), at(1, 3), at(2, 3)),
        )
    }

    /// The model so far, in the frame of the first frame added.
    pub fn cloud(&self) -> &Cloud {
        &self.model
    }

    pub fn model_points(&self) -> u64 {
        self.model.points.len() as u64
    }

    /// Frames handed over, and of those, the ones that were merged.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    pub fn merged(&self) -> u64 {
        self.merged
    }

    pub fn clear(&mut self) {
        self.inner.pin_mut().clear();
        self.model = Cloud::default();
        self.frames = 0;
        self.merged = 0;
        self.keyframes.clear();
        self.last_kept = None;
        self.last_kept_at = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point3, Rotation3, Translation3};

    /// A box-like surface: three faces meeting at a corner, so a rigid fit is
    /// constrained in every direction. Points are millimetres, and every
    /// point carries its face's normal.
    fn corner(step: f32) -> Cloud {
        let mut cloud = Cloud::default();
        let mut at = 0.0;
        while at < 60.0 {
            let mut across = 0.0;
            while across < 60.0 {
                for (point, normal) in [
                    (Point3::new(at, across, 400.0), -Vector3::z()),
                    (Point3::new(at, 0.0, 400.0 + across), -Vector3::y()),
                    (Point3::new(0.0, at, 400.0 + across), -Vector3::x()),
                ] {
                    cloud.points.push(point);
                    cloud.normals.push(normal);
                    cloud.colors.push([120, 120, 120]);
                }
                across += step;
            }
            at += step;
        }
        cloud
    }

    /// The same surface after the scanner moved: turned about the corner, so
    /// it stays in view, then shifted.
    fn moved(cloud: &Cloud, degrees: f32, shift: Vector3<f32>) -> Cloud {
        let turn = Rotation3::from_axis_angle(&Vector3::y_axis(), degrees.to_radians());
        let about = Point3::new(30.0, 30.0, 430.0);
        Cloud {
            points: cloud
                .points
                .iter()
                .map(|p| Translation3::from(shift) * (about + turn * (p - about)))
                .collect(),
            normals: cloud.normals.iter().map(|n| turn * n).collect(),
            colors: cloud.colors.clone(),
        }
    }

    /// The smallest box holding a cloud, as (low, high) in millimetres.
    fn extent(points: &[Point3<f32>]) -> (Point3<f32>, Point3<f32>) {
        points.iter().fold(
            (Point3::from([f32::MAX; 3]), Point3::from([f32::MIN; 3])),
            |(lo, hi), p| (lo.inf(p), hi.sup(p)),
        )
    }

    #[test]
    fn the_first_frame_becomes_the_model() {
        let mut fusion = Fusion::new(1.0).unwrap();
        let frame = corner(2.0);
        let fit = fusion.add(&frame).unwrap();
        assert!(fit.merged);
        assert_eq!((fit.overlap, fit.rmse_mm), (0.0, 0.0)); // nothing to align to
        assert!(fit.model_points > 0 && fit.model_points <= frame.points.len() as u64);
        let model = fusion.cloud();
        assert_eq!(model.points.len() as u64, fit.model_points);
        assert_eq!(model.colors.len() as u64, fit.model_points);
        assert_eq!(model.normals.len() as u64, fit.model_points);
        assert_eq!((fusion.frames(), fusion.merged()), (1, 1));
    }

    #[test]
    fn a_moved_frame_is_put_back_where_it_belongs() {
        // Sampled finer than the voxel, so an aligned frame falls into
        // voxels already taken.
        let mut fusion = Fusion::new(1.0).unwrap();
        let first = corner(0.5);
        fusion.add(&first).unwrap();
        let before = fusion.model_points();
        let (low, high) = extent(&fusion.cloud().points);

        let fit = fusion
            .add(&moved(&first, 4.0, Vector3::new(3.0, -2.0, 5.0)))
            .unwrap();
        assert!(fit.merged, "overlap {} rmse {}", fit.overlap, fit.rmse_mm);
        assert!(fit.overlap > 0.8, "overlap {}", fit.overlap);
        assert!(fit.rmse_mm < 1.0, "rmse {}", fit.rmse_mm);

        // Left unaligned, the frame would sit up to 5 mm off and stretch the
        // model by that much; placed properly it covers the same ground.
        let (merged_low, merged_high) = extent(&fusion.cloud().points);
        assert!(
            (merged_low - low).norm() < 1.5 && (merged_high - high).norm() < 1.5,
            "the model grew from {low:?}..{high:?} to {merged_low:?}..{merged_high:?}"
        );
        assert!(
            fit.model_points < before * 6 / 5,
            "{before} points became {}",
            fit.model_points
        );
    }

    #[test]
    fn the_faces_keep_their_own_normals_through_a_merge() {
        // The registration carries only points, so a merged frame's normals
        // are moved by the pose here. Without that they would be wrong, and
        // fitting a cylinder to a finished scan needs them.
        let mut fusion = Fusion::new(1.0).unwrap();
        let first = corner(0.5);
        fusion.add(&first).unwrap();
        let fit = fusion
            .add(&moved(&first, 4.0, Vector3::new(3.0, -2.0, 5.0)))
            .unwrap();
        assert!(fit.merged);

        let model = fusion.cloud();
        assert_eq!(model.normals.len(), model.points.len());
        assert!(model.normals.iter().all(|n| (n.norm() - 1.0).abs() < 1e-3));
        // Every normal still lies along a face of the corner, within the 4
        // degrees the second frame was turned by. All but the few hundred
        // voxels straddling the corner's three edges, where averaging two
        // faces' normals rightly gives a direction between them: about 60
        // such voxels per 60 mm edge at this voxel size.
        let faces = [-Vector3::x(), -Vector3::y(), -Vector3::z()];
        let strays = model
            .normals
            .iter()
            .filter(|n| !faces.iter().any(|f| f.dot(n) > 4.5f32.to_radians().cos()))
            .count();
        assert!(
            strays < 300 && strays * 20 < model.normals.len(),
            "{strays} of {} normals point away from every face",
            model.normals.len()
        );
    }

    #[test]
    fn a_frame_that_does_not_belong_is_left_out() {
        let mut fusion = Fusion::new(1.0).unwrap();
        let first = corner(1.5);
        fusion.add(&first).unwrap();
        let before = fusion.model_points();

        // The same shape half a metre away: no overlap to align against.
        let fit = fusion
            .add(&moved(&first, 0.0, Vector3::new(0.0, 0.0, 500.0)))
            .unwrap();
        assert!(!fit.merged);
        assert_eq!(fusion.model_points(), before, "the model was left alone");
        assert_eq!((fusion.frames(), fusion.merged()), (2, 1));
    }

    #[test]
    fn the_model_is_thinned_to_the_voxel_it_was_given() {
        let mut coarse = Fusion::new(8.0).unwrap();
        let mut fine = Fusion::new(1.0).unwrap();
        let frame = corner(1.0);
        coarse.add(&frame).unwrap();
        fine.add(&frame).unwrap();
        assert!(
            coarse.model_points() * 4 < fine.model_points(),
            "{} against {}",
            coarse.model_points(),
            fine.model_points()
        );
    }

    #[test]
    fn a_scanner_held_still_still_keeps_views_to_build_from() {
        // Measured on a real scan: held almost still, the motion rule kept
        // three views of a whole scan and the surface came out as 42
        // triangles. Views have to accrue anyway.
        let mut fusion = Fusion::new(1.0).unwrap();
        let frame = corner(0.5);
        let depth = vec![400u16; 64 * 48];
        for _ in 0..16 {
            let fit = fusion.add(&frame).unwrap(); // the same view every time
            assert!(fit.merged);
            fusion.keep_view(&depth, 64, 48);
        }
        assert!(fusion.views() >= 4, "only {} views kept", fusion.views());
    }

    #[test]
    fn a_view_whose_depth_is_the_wrong_size_is_ignored() {
        let mut fusion = Fusion::new(1.0).unwrap();
        fusion.add(&corner(2.0)).unwrap();
        fusion.keep_view(&[400u16; 10], 64, 48);
        assert_eq!(fusion.views(), 0);
    }

    #[test]
    fn clearing_starts_again() {
        let mut fusion = Fusion::new(1.0).unwrap();
        let frame = corner(2.0);
        fusion.add(&frame).unwrap();
        fusion.clear();
        assert_eq!(
            (fusion.model_points(), fusion.frames(), fusion.merged()),
            (0, 0, 0)
        );
        assert!(fusion.cloud().points.is_empty());
        let fit = fusion.add(&frame).unwrap();
        assert_eq!((fit.overlap, fit.rmse_mm), (0.0, 0.0)); // the first frame again
    }

    #[test]
    fn a_silly_voxel_size_is_refused() {
        for size in [0.0, -1.0, 500.0, f32::NAN] {
            assert!(Fusion::new(size).is_err(), "{size}");
        }
    }
}
