//! The C++ sensor layer (hw_cpp), bridged with cxx: the mock depth camera
//! and the Revopoint POP 4.
//!
//! C++ owns each frame's buffers and reuses them on the next capture, so the
//! wrapper copies a frame into Rust `Vec`s before C++ can overwrite it. The
//! borrow checker enforces the order: the views of C++ memory borrow the
//! sensor, and capturing needs the sensor mutably.

use cxx::CxxVector;
use nalgebra::Point3;

#[cxx::bridge(namespace = "all3dscan::hw")]
mod ffi {
    unsafe extern "C++" {
        include!("all3dscan/MockDepthSensor.h");

        type MockDepthSensor;

        #[cxx_name = "makeMockDepthSensor"]
        fn make_mock_depth_sensor(
            width: u32,
            height: u32,
            seed: u64,
        ) -> Result<UniquePtr<MockDepthSensor>>;

        fn capture(self: Pin<&mut MockDepthSensor>);
        fn points(self: &MockDepthSensor) -> &CxxVector<f32>;
        #[cxx_name = "depthMm"]
        fn depth_mm(self: &MockDepthSensor) -> &CxxVector<u16>;
        fn ir(self: &MockDepthSensor) -> &CxxVector<u8>;
        fn rgb(self: &MockDepthSensor) -> &CxxVector<u8>;
        fn width(self: &MockDepthSensor) -> u32;
        fn height(self: &MockDepthSensor) -> u32;
        #[cxx_name = "frameIndex"]
        fn frame_index(self: &MockDepthSensor) -> u64;
    }

    unsafe extern "C++" {
        include!("all3dscan/Pop4Sensor.h");

        type Pop4Sensor;

        #[cxx_name = "openPop4Sensor"]
        fn open_pop4_sensor() -> Result<UniquePtr<Pop4Sensor>>;
        #[cxx_name = "pop4Present"]
        fn pop4_present() -> bool;

        fn capture(self: Pin<&mut Pop4Sensor>, timeout_ms: i32) -> Result<bool>;
        fn points(self: &Pop4Sensor) -> &CxxVector<f32>;
        #[cxx_name = "depthMm"]
        fn depth_mm(self: &Pop4Sensor) -> &CxxVector<u16>;
        fn ir(self: &Pop4Sensor) -> &CxxVector<u8>;
        #[cxx_name = "depthRgb"]
        fn depth_rgb(self: &Pop4Sensor) -> &CxxVector<u8>;
        #[cxx_name = "colorImage"]
        fn color_image(self: &Pop4Sensor) -> &CxxVector<u8>;
        fn width(self: &Pop4Sensor) -> u32;
        fn height(self: &Pop4Sensor) -> u32;
        #[cxx_name = "colorWidth"]
        fn color_width(self: &Pop4Sensor) -> u32;
        #[cxx_name = "colorHeight"]
        fn color_height(self: &Pop4Sensor) -> u32;
        #[cxx_name = "frameIndex"]
        fn frame_index(self: &Pop4Sensor) -> u64;
        #[cxx_name = "validFraction"]
        fn valid_fraction(self: &Pop4Sensor) -> f32;
        fn connected(self: &Pop4Sensor) -> bool;

        #[cxx_name = "setLight"]
        fn set_light(self: Pin<&mut Pop4Sensor>, light: u8, on: bool) -> Result<bool>;
        #[cxx_name = "setLightLevel"]
        fn set_light_level(self: Pin<&mut Pop4Sensor>, light: u8, level: i32) -> Result<bool>;
        #[cxx_name = "lightOn"]
        fn light_on(self: &Pop4Sensor, light: u8) -> Result<bool>;
        #[cxx_name = "lightLevel"]
        fn light_level(self: &Pop4Sensor, light: u8) -> Result<i32>;
        #[cxx_name = "setExposure"]
        fn set_exposure(self: Pin<&mut Pop4Sensor>, microseconds: i32) -> bool;
        fn exposure(self: &Pop4Sensor) -> i32;
        #[cxx_name = "setScanMode"]
        fn set_scan_mode(self: Pin<&mut Pop4Sensor>, mode: u8) -> Result<bool>;
        #[cxx_name = "scanMode"]
        fn scan_mode(self: &Pop4Sensor) -> u8;
        #[cxx_name = "setDepthRange"]
        fn set_depth_range(self: Pin<&mut Pop4Sensor>, near_mm: f32, far_mm: f32);
        fn serial(self: &Pop4Sensor) -> &CxxString;
        #[cxx_name = "backendName"]
        fn backend_name(self: &Pop4Sensor) -> &CxxString;
        #[cxx_name = "lastError"]
        fn last_error(self: &Pop4Sensor) -> &CxxString;
    }
}

// SAFETY: a MockDepthSensor is plain data (its frame buffers and random state)
// with no thread affinity, so it may move to whichever thread drives it. Calls
// still go through & and &mut, so they never overlap.
unsafe impl Send for ffi::MockDepthSensor {}

// SAFETY: a Pop4Sensor holds file descriptors (V4L2), a libusb handle and the
// decoder's CUDA buffers, none of which is tied to the thread that opened
// them. It is not thread-safe, but calls go through & and &mut, so they never
// overlap.
unsafe impl Send for ffi::Pop4Sensor {}

/// One frame, copied out of the sensor's C++ buffers. The images are
/// row-major, `width` x `height` pixels, but for the colour image, which is
/// `rgb_width` x `rgb_height`.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    /// The sensor's frame count after this capture.
    pub index: u64,
    pub width: u32,
    pub height: u32,
    /// Millimetres in the camera frame, one point per pixel that has depth.
    pub points: Vec<Point3<f32>>,
    /// Each point's colour, from where the colour camera sees it.
    pub colors: Vec<[u8; 3]>,
    /// Depth in millimetres; 0 where the pixel has none.
    pub depth_mm: Vec<u16>,
    /// Infrared intensity, one byte per pixel.
    pub ir: Vec<u8>,
    /// The colour camera's image, three bytes (R, G, B) per pixel.
    pub rgb: Vec<u8>,
    pub rgb_width: u32,
    pub rgb_height: u32,
}

/// Each point's colour: points come in pixel order, skipping pixels without
/// depth, and `pixel_rgb` holds a colour for every depth pixel.
fn point_colors(depth_mm: &[u16], pixel_rgb: &[u8]) -> Vec<[u8; 3]> {
    let (pixels, _) = pixel_rgb.as_chunks::<3>();
    depth_mm
        .iter()
        .zip(pixels)
        .filter(|&(&depth, _)| depth > 0)
        .map(|(_, &rgb)| rgb)
        .collect()
}

fn points_of(xyz: &CxxVector<f32>) -> Vec<Point3<f32>> {
    let (xyz, rest) = xyz.as_slice().as_chunks::<3>();
    debug_assert!(rest.is_empty(), "C++ point buffer is not x, y, z triples");
    xyz.iter().map(|&[x, y, z]| Point3::new(x, y, z)).collect()
}

/// The mock depth camera from hw_cpp.
pub struct MockDepthSensor {
    inner: cxx::UniquePtr<ffi::MockDepthSensor>,
}

impl MockDepthSensor {
    /// Creates a `width` x `height` sensor. The same `seed` gives the same frames.
    pub fn new(width: u32, height: u32, seed: u64) -> Result<Self, cxx::Exception> {
        let inner = ffi::make_mock_depth_sensor(width, height, seed)?;
        Ok(Self { inner })
    }

    /// Captures one frame and copies it out of the C++ buffers.
    pub fn capture(&mut self) -> Frame {
        self.inner.pin_mut().capture();
        let sensor = &*self.inner;
        let depth_mm = sensor.depth_mm().as_slice();
        // The mock's colour camera shares the depth camera's viewpoint, so a
        // point's colour is the colour of its own pixel.
        Frame {
            index: sensor.frame_index(),
            width: sensor.width(),
            height: sensor.height(),
            points: points_of(sensor.points()),
            colors: point_colors(depth_mm, sensor.rgb().as_slice()),
            depth_mm: depth_mm.to_vec(),
            ir: sensor.ir().as_slice().to_vec(),
            rgb: sensor.rgb().as_slice().to_vec(),
            rgb_width: sensor.width(),
            rgb_height: sensor.height(),
        }
    }

    pub fn width(&self) -> u32 {
        self.inner.width()
    }

    pub fn height(&self) -> u32 {
        self.inner.height()
    }

    /// Number of frames captured so far.
    pub fn frame_index(&self) -> u64 {
        self.inner.frame_index()
    }
}

/// A built-in light of the POP 4.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Light {
    /// Draws the scan pattern.
    Projector = 0,
    /// Lights the scene for the colour camera.
    FillLight = 1,
}

/// How the POP 4 projects its pattern.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanMode {
    Infrared = 0,
    /// The visible blue laser; measured to give too little depth to scan with yet.
    BlueLaser = 1,
    /// A stronger infrared speckle.
    Speckle = 2,
}

/// The Revopoint POP 4 over USB, from hw_cpp.
pub struct Pop4Sensor {
    inner: cxx::UniquePtr<ffi::Pop4Sensor>,
}

impl Pop4Sensor {
    /// Connects to the scanner and starts it streaming; this takes a few
    /// seconds, longer if the scanner has to be reset first.
    pub fn open() -> Result<Self, cxx::Exception> {
        Ok(Self {
            inner: ffi::open_pop4_sensor()?,
        })
    }

    /// Whether a POP 4 is plugged in, without opening it.
    pub fn present() -> bool {
        ffi::pop4_present()
    }

    /// Waits up to `timeout_ms` for the next frame and copies it out of the
    /// C++ buffers. `Ok(None)` if no frame came; last_error() says why.
    pub fn capture(&mut self, timeout_ms: i32) -> Result<Option<Frame>, cxx::Exception> {
        if !self.inner.pin_mut().capture(timeout_ms)? {
            return Ok(None);
        }
        let sensor = &*self.inner;
        let depth_mm = sensor.depth_mm().as_slice();
        Ok(Some(Frame {
            index: sensor.frame_index(),
            width: sensor.width(),
            height: sensor.height(),
            points: points_of(sensor.points()),
            colors: point_colors(depth_mm, sensor.depth_rgb().as_slice()),
            depth_mm: depth_mm.to_vec(),
            ir: sensor.ir().as_slice().to_vec(),
            rgb: sensor.color_image().as_slice().to_vec(),
            rgb_width: sensor.color_width(),
            rgb_height: sensor.color_height(),
        }))
    }

    pub fn set_light(&mut self, light: Light, on: bool) -> Result<(), String> {
        let ok = self.inner.pin_mut().set_light(light as u8, on);
        self.check(ok)
    }

    pub fn set_light_level(&mut self, light: Light, level: i32) -> Result<(), String> {
        let ok = self.inner.pin_mut().set_light_level(light as u8, level);
        self.check(ok)
    }

    pub fn light_on(&self, light: Light) -> bool {
        self.inner.light_on(light as u8).unwrap_or(false)
    }

    pub fn light_level(&self, light: Light) -> i32 {
        self.inner.light_level(light as u8).unwrap_or(0)
    }

    pub fn set_exposure(&mut self, microseconds: i32) -> Result<(), String> {
        let ok = self.inner.pin_mut().set_exposure(microseconds);
        self.check(Ok(ok))
    }

    pub fn exposure(&self) -> i32 {
        self.inner.exposure()
    }

    /// Switches the scan mode. Going back to infrared reboots the scanner,
    /// which takes about 45 seconds.
    pub fn set_scan_mode(&mut self, mode: ScanMode) -> Result<(), String> {
        let ok = self.inner.pin_mut().set_scan_mode(mode as u8);
        self.check(ok)
    }

    pub fn scan_mode(&self) -> ScanMode {
        match self.inner.scan_mode() {
            1 => ScanMode::BlueLaser,
            2 => ScanMode::Speckle,
            _ => ScanMode::Infrared,
        }
    }

    /// The working distance, in millimetres, the depth decoder searches.
    pub fn set_depth_range(&mut self, near_mm: f32, far_mm: f32) {
        self.inner.pin_mut().set_depth_range(near_mm, far_mm);
    }

    /// The share of the newest frame's pixels that got depth.
    pub fn valid_fraction(&self) -> f32 {
        self.inner.valid_fraction()
    }

    /// Whether the scanner is still there. False once it has unplugged or
    /// re-enumerated, after which capture() never succeeds again.
    pub fn connected(&self) -> bool {
        self.inner.connected()
    }

    pub fn serial(&self) -> String {
        self.inner.serial().to_string_lossy().into_owned()
    }

    pub fn backend_name(&self) -> String {
        self.inner.backend_name().to_string_lossy().into_owned()
    }

    /// Why the last call that failed did.
    pub fn last_error(&self) -> String {
        self.inner.last_error().to_string_lossy().into_owned()
    }

    fn check(&self, ok: Result<bool, cxx::Exception>) -> Result<(), String> {
        match ok {
            Ok(true) => Ok(()),
            Ok(false) => Err(self.last_error()),
            Err(error) => Err(error.what().to_owned()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capture_copies_out_one_point_per_returned_pixel() {
        let mut sensor = MockDepthSensor::new(64, 48, 7).unwrap();
        let frame = sensor.capture();

        // About 2% of pixels drop out; every other pixel gives one point.
        let pixels = 64 * 48;
        let points = &frame.points;
        assert!(
            points.len() <= pixels && points.len() > pixels * 9 / 10,
            "{} points",
            points.len()
        );
        assert_eq!((frame.index, sensor.frame_index()), (1, 1));

        // Depths run from the front of the ball (330 mm) to the wall (600 mm),
        // give or take the 0.5 mm noise.
        let (near, far) = points.iter().fold((f32::MAX, f32::MIN), |(lo, hi), p| {
            (lo.min(p.z), hi.max(p.z))
        });
        assert!((near - 330.0).abs() < 3.0, "nearest depth {near}");
        assert!((far - 600.0).abs() < 3.0, "farthest depth {far}");
    }

    #[test]
    fn images_cover_the_frame_and_agree_with_the_points() {
        let frame = MockDepthSensor::new(64, 48, 7).unwrap().capture();
        let pixels = 64 * 48;
        assert_eq!((frame.width, frame.height), (64, 48));
        assert_eq!(frame.depth_mm.len(), pixels);
        assert_eq!(frame.ir.len(), pixels);
        assert_eq!(frame.rgb.len(), pixels * 3);
        assert_eq!((frame.rgb_width, frame.rgb_height), (64, 48));

        // A pixel has depth exactly when it produced a point, and each point
        // carries its pixel's colour.
        let with_depth: Vec<usize> = (0..pixels).filter(|&i| frame.depth_mm[i] > 0).collect();
        assert_eq!(with_depth.len(), frame.points.len());
        assert_eq!(frame.colors.len(), frame.points.len());
        for (k, &i) in with_depth.iter().enumerate().step_by(97) {
            assert_eq!(
                frame.colors[k],
                [frame.rgb[3 * i], frame.rgb[3 * i + 1], frame.rgb[3 * i + 2]]
            );
        }

        // The orange ball fills the middle of the colour image; the grey wall
        // shows in the corner.
        let colour = |u: usize, v: usize| {
            let i = 3 * (v * 64 + u);
            (frame.rgb[i], frame.rgb[i + 1], frame.rgb[i + 2])
        };
        let (r, g, b) = colour(32, 24);
        assert!(r > g && g > b, "centre {:?}", (r, g, b));
        let (r, g, b) = colour(0, 0);
        assert!(
            r.abs_diff(b) < 30 && r.abs_diff(g) < 30,
            "corner {:?}",
            (r, g, b)
        );
    }

    #[test]
    fn same_seed_gives_the_same_frames() {
        let mut a = MockDepthSensor::new(32, 24, 42).unwrap();
        let mut b = MockDepthSensor::new(32, 24, 42).unwrap();
        for _ in 0..3 {
            assert_eq!(a.capture(), b.capture());
        }
    }

    #[test]
    fn invalid_size_comes_back_as_an_error() {
        let err = MockDepthSensor::new(0, 48, 1)
            .err()
            .expect("a zero-width sensor should be rejected");
        assert!(err.what().contains("width and height"), "{}", err.what());
    }

    /// Needs a POP 4 on USB: `cargo test --release -- --ignored pop4`.
    #[test]
    #[ignore = "needs a POP 4 plugged in"]
    fn pop4_streams_frames_whose_parts_agree() {
        assert!(Pop4Sensor::present(), "no POP 4 on USB");
        let mut sensor = Pop4Sensor::open().unwrap_or_else(|e| panic!("{}", e.what()));
        assert!(
            sensor.backend_name().contains("POP4"),
            "{}",
            sensor.backend_name()
        );
        assert!(!sensor.serial().is_empty());
        let frames: Vec<Frame> = (0..10)
            .filter_map(|_| sensor.capture(2000).unwrap())
            .collect();
        println!(
            "{} of 10 frames; {}; serial {}; last frame {} points, {:.1}% valid, colour {}x{}",
            frames.len(),
            sensor.backend_name(),
            sensor.serial(),
            frames.last().map_or(0, |f| f.points.len()),
            100.0 * sensor.valid_fraction(),
            frames.last().map_or(0, |f| f.rgb_width),
            frames.last().map_or(0, |f| f.rgb_height),
        );
        assert!(frames.len() >= 5, "only {} of 10 frames came", frames.len());
        for frame in &frames {
            let pixels = (frame.width * frame.height) as usize;
            assert_eq!(frame.depth_mm.len(), pixels);
            assert_eq!(frame.ir.len(), pixels);
            assert_eq!(frame.colors.len(), frame.points.len());
            assert_eq!(
                frame.depth_mm.iter().filter(|&&d| d > 0).count(),
                frame.points.len()
            );
            assert_eq!(
                frame.rgb.len(),
                (frame.rgb_width * frame.rgb_height * 3) as usize
            );
            assert!(
                frame
                    .points
                    .iter()
                    .all(|p| p.z > 0.0 && p.coords.iter().all(|c| c.is_finite()))
            );
        }
        assert!(frames.windows(2).all(|w| w[1].index > w[0].index));
    }

    #[test]
    fn sensor_can_be_driven_from_another_thread() {
        let mut sensor = MockDepthSensor::new(16, 12, 3).unwrap();
        let captured = std::thread::spawn(move || sensor.capture().points.len());
        assert!(captured.join().unwrap() > 0);
    }
}
