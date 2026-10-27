use pyo3::prelude::*;

pub mod fitting;
pub mod hw;
pub mod processing;

/// All3DScan's processing engine, imported by the PyQt6 GUI.
///
/// Calls that capture, process or fit a cloud release the GIL while they
/// work, so a worker thread can run them while the GUI thread carries on.
#[pymodule]
mod all3dscan_engine {
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::{Mutex, PoisonError};

    use nalgebra::{Point3, Vector3};
    use numpy::{
        AllowTypeChange, PyArray1, PyArray2, PyArray3, PyArrayLike1, PyArrayLike2, PyArrayMethods,
        PyUntypedArrayMethods,
    };
    use pyo3::exceptions::{PyOSError, PyValueError};
    use pyo3::prelude::*;
    use pyo3::types::PyDict;

    use crate::processing::ProcessingError;
    use crate::{fitting, hw, processing};

    /// The mock depth camera from hw_cpp.
    ///
    /// capture() returns an (N, 3) float32 array of x, y, z in millimetres in
    /// the camera frame (x right, y down, z forward); capture_frame() also
    /// returns the depth, infrared and colour images. The same seed gives the
    /// same frames.
    #[pyclass(frozen)]
    struct MockDepthSensor {
        // Two threads that capture at once take turns instead of racing.
        sensor: Mutex<hw::MockDepthSensor>,
        // Kept outside the lock, so reading them never waits on a capture.
        width: u32,
        height: u32,
        frame_index: AtomicU64,
    }

    /// One captured frame: its point cloud and the images it was made from.
    #[pyclass(frozen)]
    struct Frame {
        /// (N, 3) float32 points in millimetres, camera frame.
        #[pyo3(get)]
        points: Py<PyArray2<f32>>,
        /// (N, 3) uint8 colour of each point.
        #[pyo3(get)]
        colors: Py<PyArray2<u8>>,
        /// (N, 3) float32 unit normal of each point, facing the camera.
        #[pyo3(get)]
        normals: Py<PyArray2<f32>>,
        /// (height, width) uint16 depth in millimetres; 0 where there is none.
        #[pyo3(get)]
        depth_mm: Py<PyArray2<u16>>,
        /// (height, width) uint8 infrared image.
        #[pyo3(get)]
        ir: Py<PyArray2<u8>>,
        /// (height, width, 3) uint8 image from the colour camera, whose size
        /// may differ from the depth's (it is empty until the POP 4's colour
        /// camera has sent one).
        #[pyo3(get)]
        rgb: Py<PyArray3<u8>>,
        /// The sensor's frame count after this capture.
        #[pyo3(get)]
        index: u64,
    }

    #[pymethods]
    impl MockDepthSensor {
        #[new]
        #[pyo3(signature = (width = 640, height = 480, seed = 0))]
        fn new(width: u32, height: u32, seed: u64) -> PyResult<Self> {
            let sensor = hw::MockDepthSensor::new(width, height, seed)
                .map_err(|e| PyValueError::new_err(e.what().to_owned()))?;
            Ok(Self {
                sensor: Mutex::new(sensor),
                width,
                height,
                frame_index: AtomicU64::new(0),
            })
        }

        /// Captures a frame as an (N, 3) array. With voxel_mm, thins it to one
        /// point per voxel first, like voxel_downsample().
        #[pyo3(signature = (voxel_mm = None))]
        fn capture<'py>(
            &self,
            py: Python<'py>,
            voxel_mm: Option<f32>,
        ) -> PyResult<Bound<'py, PyArray2<f32>>> {
            check_voxel_size(voxel_mm)?;
            let xyz = py
                .detach(|| {
                    let frame = self.next_frame();
                    match voxel_mm {
                        Some(size) => {
                            processing::voxel_downsample(&frame.points, size).map(|p| flatten(&p))
                        }
                        None => Ok(flatten(&frame.points)),
                    }
                })
                .map_err(value_error)?;
            into_array(py, xyz)
        }

        /// Captures a frame with its colours, normals and depth, infrared and
        /// colour images. With voxel_mm, thins the points to one per voxel,
        /// averaging their colours and normals, as capture() does.
        #[pyo3(signature = (voxel_mm = None))]
        fn capture_frame(&self, py: Python<'_>, voxel_mm: Option<f32>) -> PyResult<Frame> {
            check_voxel_size(voxel_mm)?;
            let parts = py
                .detach(|| FrameParts::new(self.next_frame(), voxel_mm))
                .map_err(value_error)?;
            parts.into_py(py)
        }

        #[getter]
        fn width(&self) -> u32 {
            self.width
        }

        #[getter]
        fn height(&self) -> u32 {
            self.height
        }

        /// Number of frames captured so far.
        #[getter]
        fn frame_index(&self) -> u64 {
            self.frame_index.load(Ordering::Relaxed)
        }
    }

    impl MockDepthSensor {
        /// The sensor's next frame. Called without the GIL.
        fn next_frame(&self) -> hw::Frame {
            let frame = self
                .sensor
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .capture();
            self.frame_index.store(frame.index, Ordering::Relaxed);
            frame
        }
    }

    /// A frame's arrays, made without the GIL: its normals, taken from the
    /// whole pixel grid, then its points thinned to voxel_mm if asked.
    struct FrameParts {
        xyz: Vec<f32>,
        colors: Vec<u8>,
        normals: Vec<f32>,
        frame: hw::Frame,
    }

    impl FrameParts {
        fn new(mut frame: hw::Frame, voxel_mm: Option<f32>) -> Result<Self, ProcessingError> {
            let (w, h) = (frame.width as usize, frame.height as usize);
            let normals = processing::organized_normals(&frame.points, &frame.depth_mm, w, h);
            let mut cloud = processing::Cloud {
                points: std::mem::take(&mut frame.points),
                colors: std::mem::take(&mut frame.colors),
                normals,
            };
            if let Some(size) = voxel_mm {
                cloud = processing::voxel_downsample_cloud(&cloud, size)?;
            }
            Ok(Self {
                xyz: flatten(&cloud.points),
                colors: cloud.colors.into_iter().flatten().collect(),
                normals: cloud.normals.iter().flat_map(|n| [n.x, n.y, n.z]).collect(),
                frame,
            })
        }

        fn into_py(self, py: Python<'_>) -> PyResult<Frame> {
            let frame = self.frame;
            let (h, w) = (frame.height as usize, frame.width as usize);
            let (rgb_h, rgb_w) = (frame.rgb_height as usize, frame.rgb_width as usize);
            let n = self.xyz.len() / 3;
            Ok(Frame {
                points: into_array(py, self.xyz)?.unbind(),
                colors: PyArray1::from_vec(py, self.colors)
                    .reshape([n, 3])?
                    .unbind(),
                normals: PyArray1::from_vec(py, self.normals)
                    .reshape([n, 3])?
                    .unbind(),
                depth_mm: PyArray1::from_vec(py, frame.depth_mm)
                    .reshape([h, w])?
                    .unbind(),
                ir: PyArray1::from_vec(py, frame.ir).reshape([h, w])?.unbind(),
                rgb: PyArray1::from_vec(py, frame.rgb)
                    .reshape([rgb_h, rgb_w, 3])?
                    .unbind(),
                index: frame.index,
            })
        }
    }

    /// The Revopoint POP 4 over USB.
    ///
    /// Opening one connects to the scanner and starts it streaming, which
    /// takes a few seconds, longer if it has to be reset first; it raises
    /// OSError with the scanner's reason if it cannot. capture_frame() gives
    /// the same Frame as MockDepthSensor, or None if no frame came in time.
    /// The lights are "projector" and "fill light"; the scan modes
    /// "infrared", "blue laser" and "speckle". Leaving blue laser or speckle
    /// reboots the scanner, which takes about 45 seconds. close() lets the
    /// scanner go, as dropping the object does: lights back as connecting
    /// left them, and back in infrared.
    #[pyclass(frozen)]
    struct Pop4Sensor {
        // None once closed. Captures and controls from different threads
        // take turns.
        sensor: Mutex<Option<hw::Pop4Sensor>>,
        frame_index: AtomicU64,
    }

    #[pymethods]
    impl Pop4Sensor {
        #[new]
        fn new(py: Python<'_>) -> PyResult<Self> {
            let sensor = py
                .detach(hw::Pop4Sensor::open)
                .map_err(|e| PyOSError::new_err(e.what().to_owned()))?;
            Ok(Self {
                sensor: Mutex::new(Some(sensor)),
                frame_index: AtomicU64::new(0),
            })
        }

        /// Waits up to timeout_ms for the next frame; None if none came (see
        /// last_error). voxel_mm thins it as MockDepthSensor.capture_frame does.
        #[pyo3(signature = (voxel_mm = None, timeout_ms = 2000))]
        fn capture_frame(
            &self,
            py: Python<'_>,
            voxel_mm: Option<f32>,
            timeout_ms: i32,
        ) -> PyResult<Option<Frame>> {
            check_voxel_size(voxel_mm)?;
            let parts = py.detach(|| -> PyResult<Option<FrameParts>> {
                let frame = self
                    .with(|sensor| sensor.capture(timeout_ms))?
                    .map_err(|e| PyOSError::new_err(e.what().to_owned()))?;
                let Some(frame) = frame else {
                    return Ok(None);
                };
                self.frame_index.store(frame.index, Ordering::Relaxed);
                FrameParts::new(frame, voxel_mm)
                    .map(Some)
                    .map_err(value_error)
            })?;
            parts.map(|parts| parts.into_py(py)).transpose()
        }

        /// Turns a light on or off.
        fn set_light(&self, py: Python<'_>, light: &str, on: bool) -> PyResult<()> {
            let light = parse_light(light)?;
            py.detach(|| self.with(|sensor| sensor.set_light(light, on)))?
                .map_err(PyOSError::new_err)
        }

        /// Sets a light's level, 0 to 255.
        fn set_light_level(&self, py: Python<'_>, light: &str, level: i32) -> PyResult<()> {
            let light = parse_light(light)?;
            py.detach(|| self.with(|sensor| sensor.set_light_level(light, level)))?
                .map_err(PyOSError::new_err)
        }

        fn light_on(&self, light: &str) -> PyResult<bool> {
            let light = parse_light(light)?;
            self.with(|sensor| sensor.light_on(light))
        }

        fn light_level(&self, light: &str) -> PyResult<i32> {
            let light = parse_light(light)?;
            self.with(|sensor| sensor.light_level(light))
        }

        /// Sets the depth camera's exposure, in microseconds.
        fn set_exposure(&self, py: Python<'_>, microseconds: i32) -> PyResult<()> {
            py.detach(|| self.with(|sensor| sensor.set_exposure(microseconds)))?
                .map_err(PyOSError::new_err)
        }

        #[getter]
        fn exposure_us(&self) -> PyResult<i32> {
            self.with(|sensor| sensor.exposure())
        }

        /// Switches the scan mode; back to "infrared" takes about 45 seconds.
        fn set_scan_mode(&self, py: Python<'_>, mode: &str) -> PyResult<()> {
            let mode = match mode {
                "infrared" => hw::ScanMode::Infrared,
                "blue laser" => hw::ScanMode::BlueLaser,
                "speckle" => hw::ScanMode::Speckle,
                other => {
                    return Err(PyValueError::new_err(format!(
                        "scan mode must be \"infrared\", \"blue laser\" or \"speckle\", got {other:?}"
                    )));
                }
            };
            py.detach(|| self.with(|sensor| sensor.set_scan_mode(mode)))?
                .map_err(PyOSError::new_err)
        }

        #[getter]
        fn scan_mode(&self) -> PyResult<&'static str> {
            self.with(|sensor| match sensor.scan_mode() {
                hw::ScanMode::Infrared => "infrared",
                hw::ScanMode::BlueLaser => "blue laser",
                hw::ScanMode::Speckle => "speckle",
            })
        }

        /// The working distance, in millimetres, that the depth decoder searches.
        fn set_depth_range(&self, near_mm: f32, far_mm: f32) -> PyResult<()> {
            if !(near_mm.is_finite() && far_mm.is_finite() && 0.0 < near_mm && near_mm < far_mm) {
                return Err(PyValueError::new_err(format!(
                    "depth range must be 0 < near < far, got {near_mm} to {far_mm}"
                )));
            }
            self.with(|sensor| sensor.set_depth_range(near_mm, far_mm))
        }

        /// The share of the newest frame's pixels that got depth.
        #[getter]
        fn valid_fraction(&self) -> PyResult<f32> {
            self.with(|sensor| sensor.valid_fraction())
        }

        /// Whether the scanner is still there. False once it has unplugged
        /// or re-enumerated, after which capture_frame() never gives another
        /// frame: close this one and open a new one.
        #[getter]
        fn connected(&self) -> bool {
            self.lock().as_ref().is_some_and(hw::Pop4Sensor::connected)
        }

        #[getter]
        fn serial(&self) -> PyResult<String> {
            self.with(|sensor| sensor.serial())
        }

        /// Which decoder and calibration are in use.
        #[getter]
        fn backend(&self) -> PyResult<String> {
            self.with(|sensor| sensor.backend_name())
        }

        /// Why the last call that failed did, such as a capture that returned None.
        #[getter]
        fn last_error(&self) -> PyResult<String> {
            self.with(|sensor| sensor.last_error())
        }

        /// Number of frames captured so far.
        #[getter]
        fn frame_index(&self) -> u64 {
            self.frame_index.load(Ordering::Relaxed)
        }

        #[getter]
        fn closed(&self) -> bool {
            self.lock().is_none()
        }

        /// Lets the scanner go. Calls after this raise ValueError.
        fn close(&self, py: Python<'_>) {
            py.detach(|| drop(self.lock().take()));
        }

        fn __enter__(slf: Py<Self>) -> Py<Self> {
            slf
        }

        fn __exit__(
            &self,
            py: Python<'_>,
            _type: Option<Bound<'_, PyAny>>,
            _value: Option<Bound<'_, PyAny>>,
            _traceback: Option<Bound<'_, PyAny>>,
        ) {
            self.close(py);
        }
    }

    impl Pop4Sensor {
        fn lock(&self) -> std::sync::MutexGuard<'_, Option<hw::Pop4Sensor>> {
            self.sensor.lock().unwrap_or_else(PoisonError::into_inner)
        }

        fn with<R>(&self, f: impl FnOnce(&mut hw::Pop4Sensor) -> R) -> PyResult<R> {
            match self.lock().as_mut() {
                Some(sensor) => Ok(f(sensor)),
                None => Err(PyValueError::new_err("the POP 4 is closed")),
            }
        }
    }

    fn parse_light(light: &str) -> PyResult<hw::Light> {
        match light {
            "projector" => Ok(hw::Light::Projector),
            "fill light" => Ok(hw::Light::FillLight),
            other => Err(PyValueError::new_err(format!(
                "light must be \"projector\" or \"fill light\", got {other:?}"
            ))),
        }
    }

    /// Whether a POP 4 is plugged in, without opening it.
    #[pyfunction]
    fn pop4_present() -> bool {
        hw::Pop4Sensor::present()
    }

    #[pymethods]
    impl Frame {
        fn __repr__(&self, py: Python<'_>) -> String {
            let points = self.points.bind(py).shape()[0];
            let size = self.ir.bind(py).shape();
            format!(
                "Frame(index={}, points={points}, {}x{} pixels)",
                self.index, size[1], size[0]
            )
        }
    }

    /// Thins an (N, 3) array of points to one point per cube of edge voxel_mm:
    /// the centroid of the points inside it. Points with a NaN or infinite
    /// coordinate are dropped.
    #[pyfunction]
    fn voxel_downsample<'py>(
        py: Python<'py>,
        points: PyArrayLike2<'py, f32, AllowTypeChange>,
        voxel_mm: f32,
    ) -> PyResult<Bound<'py, PyArray2<f32>>> {
        // Copied while attached: other Python threads may change the array
        // once the GIL is released.
        let cloud: Vec<Point3<f32>> = triples(&points, "points")?;
        let xyz = py
            .detach(|| processing::voxel_downsample(&cloud, voxel_mm).map(|p| flatten(&p)))
            .map_err(value_error)?;
        into_array(py, xyz)
    }

    /// Fits the plane with the most support to a cloud with RANSAC: points
    /// (N, 3) in millimetres and their unit normals (N, 3), with candidates
    /// (indices into them) limiting the fit to some of the points. A point
    /// supports the plane if it lies within threshold_mm of it and its normal
    /// agrees. Returns None if no plane has min_inliers points, or a dict: the
    /// plane's normal, facing the sensor, and origin (the inliers' centroid);
    /// two axes in the plane and the inliers' extent along each, from the
    /// origin; the inliers' indices, the indices the fit claims (the inliers
    /// and the noise beside them), and the inliers' rms_mm distance from it.
    #[pyfunction]
    #[pyo3(signature = (
        points, normals, candidates = None, *,
        threshold_mm = 1.5, iterations = 2000, min_inliers = 200, seed = 0,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn fit_plane<'py>(
        py: Python<'py>,
        points: PyArrayLike2<'py, f32, AllowTypeChange>,
        normals: PyArrayLike2<'py, f32, AllowTypeChange>,
        candidates: Option<PyArrayLike1<'py, u32, AllowTypeChange>>,
        threshold_mm: f32,
        iterations: u32,
        min_inliers: usize,
        seed: u64,
    ) -> PyResult<Option<Bound<'py, PyDict>>> {
        let (cloud, directions, candidates) = fit_input(&points, &normals, candidates)?;
        let settings = ransac(threshold_mm, iterations, min_inliers, seed)?;
        let Some(fit) =
            py.detach(|| fitting::fit_plane(&cloud, &directions, &candidates, settings))
        else {
            return Ok(None);
        };
        let [u, v] = fit.axes;
        let [(u0, u1), (v0, v1)] = fit.extent;
        let result = PyDict::new(py);
        result.set_item("normal", PyArray1::from_slice(py, fit.normal.as_slice()))?;
        result.set_item(
            "origin",
            PyArray1::from_slice(py, fit.origin.coords.as_slice()),
        )?;
        result.set_item(
            "axes",
            PyArray1::from_vec(py, [u, v].iter().flat_map(|a| [a.x, a.y, a.z]).collect())
                .reshape([2, 3])?,
        )?;
        result.set_item(
            "extent",
            PyArray1::from_vec(py, vec![u0, u1, v0, v1]).reshape([2, 2])?,
        )?;
        result.set_item("inliers", PyArray1::from_vec(py, fit.inliers))?;
        result.set_item("claimed", PyArray1::from_vec(py, fit.claimed))?;
        result.set_item("rms_mm", fit.rms_mm)?;
        Ok(Some(result))
    }

    /// Fits the cylinder with the most support to a cloud with RANSAC, as
    /// fit_plane() does: the convex side of one, seen from outside, with a
    /// radius between radius_mm[0] and radius_mm[1]. Returns None if no
    /// cylinder has min_inliers points, or a dict: the cylinder's unit axis
    /// and its base (the point on the axis level with the lowest inlier),
    /// its radius_mm and height_mm (the inliers' extent along the axis), and
    /// inliers, claimed and rms_mm as fit_plane() gives them.
    #[pyfunction]
    #[pyo3(signature = (
        points, normals, candidates = None, *, radius_mm = (2.0, 1000.0),
        threshold_mm = 1.5, iterations = 2000, min_inliers = 200, seed = 0,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn fit_cylinder<'py>(
        py: Python<'py>,
        points: PyArrayLike2<'py, f32, AllowTypeChange>,
        normals: PyArrayLike2<'py, f32, AllowTypeChange>,
        candidates: Option<PyArrayLike1<'py, u32, AllowTypeChange>>,
        radius_mm: (f32, f32),
        threshold_mm: f32,
        iterations: u32,
        min_inliers: usize,
        seed: u64,
    ) -> PyResult<Option<Bound<'py, PyDict>>> {
        let (cloud, directions, candidates) = fit_input(&points, &normals, candidates)?;
        let settings = ransac(threshold_mm, iterations, min_inliers, seed)?;
        let (lo, hi) = radius_mm;
        if !(lo.is_finite() && hi.is_finite() && 0.0 < lo && lo <= hi) {
            return Err(PyValueError::new_err(format!(
                "radius_mm must be two finite radii, 0 < low <= high, got ({lo}, {hi})"
            )));
        }
        let Some(fit) = py.detach(|| {
            fitting::fit_cylinder(&cloud, &directions, &candidates, settings, radius_mm)
        }) else {
            return Ok(None);
        };
        let result = PyDict::new(py);
        result.set_item("axis", PyArray1::from_slice(py, fit.axis.as_slice()))?;
        result.set_item("base", PyArray1::from_slice(py, fit.base.coords.as_slice()))?;
        result.set_item("radius_mm", fit.radius_mm)?;
        result.set_item("height_mm", fit.height_mm)?;
        result.set_item("inliers", PyArray1::from_vec(py, fit.inliers))?;
        result.set_item("claimed", PyArray1::from_vec(py, fit.claimed))?;
        result.set_item("rms_mm", fit.rms_mm)?;
        Ok(Some(result))
    }

    /// A cloud's points, normals and candidates, copied out of their arrays
    /// and checked against each other.
    #[allow(clippy::type_complexity)]
    fn fit_input(
        points: &PyArrayLike2<'_, f32, AllowTypeChange>,
        normals: &PyArrayLike2<'_, f32, AllowTypeChange>,
        candidates: Option<PyArrayLike1<'_, u32, AllowTypeChange>>,
    ) -> PyResult<(Vec<Point3<f32>>, Vec<Vector3<f32>>, Vec<u32>)> {
        let cloud: Vec<Point3<f32>> = triples(points, "points")?;
        let directions: Vec<Vector3<f32>> = triples(normals, "normals")?;
        if directions.len() != cloud.len() {
            return Err(PyValueError::new_err(format!(
                "normals must match points, got {} normals for {} points",
                directions.len(),
                cloud.len()
            )));
        }
        let n = u32::try_from(cloud.len())
            .map_err(|_| PyValueError::new_err("a cloud may hold at most 2^32 - 1 points"))?;
        let candidates = match candidates {
            None => (0..n).collect(),
            Some(indices) => indices.as_array().to_vec(),
        };
        if let Some(&bad) = candidates.iter().find(|&&k| k >= n) {
            return Err(PyValueError::new_err(format!(
                "candidate {bad} is out of range for {n} points"
            )));
        }
        Ok((cloud, directions, candidates))
    }

    fn ransac(
        threshold_mm: f32,
        iterations: u32,
        min_inliers: usize,
        seed: u64,
    ) -> PyResult<fitting::Ransac> {
        if !(threshold_mm.is_finite() && threshold_mm > 0.0) || iterations == 0 {
            return Err(PyValueError::new_err(format!(
                "threshold_mm must be positive and finite and iterations at least 1, \
                 got {threshold_mm} and {iterations}"
            )));
        }
        Ok(fitting::Ransac {
            threshold_mm,
            iterations,
            min_inliers,
            seed,
        })
    }

    /// The rows of an (N, 3) array as points or vectors, after checking its shape.
    fn triples<T: From<[f32; 3]>>(
        array: &PyArrayLike2<'_, f32, AllowTypeChange>,
        name: &str,
    ) -> PyResult<Vec<T>> {
        let view = array.as_array();
        if view.ncols() != 3 {
            return Err(PyValueError::new_err(format!(
                "{name} must have shape (N, 3), got {:?}",
                view.shape()
            )));
        }
        Ok(view
            .rows()
            .into_iter()
            .map(|r| T::from([r[0], r[1], r[2]]))
            .collect())
    }

    #[pymodule_init]
    fn init(m: &Bound<'_, PyModule>) -> PyResult<()> {
        // Every array this module returns needs numpy; without it, fail here
        // with an ImportError rather than in the middle of a capture.
        m.py().import("numpy")?;
        m.add("__version__", env!("CARGO_PKG_VERSION"))
    }

    fn value_error(e: ProcessingError) -> PyErr {
        PyValueError::new_err(e.to_string())
    }

    /// Refuses a bad voxel size before a frame is captured, so that the
    /// frame is not taken and then thrown away.
    fn check_voxel_size(voxel_mm: Option<f32>) -> PyResult<()> {
        voxel_mm.map_or(Ok(()), |size| {
            processing::check_voxel_size(size).map_err(value_error)
        })
    }

    fn flatten(points: &[Point3<f32>]) -> Vec<f32> {
        points.iter().flat_map(|p| [p.x, p.y, p.z]).collect()
    }

    /// Hands x, y, z triples to numpy as an (N, 3) array without copying them.
    fn into_array(py: Python<'_>, xyz: Vec<f32>) -> PyResult<Bound<'_, PyArray2<f32>>> {
        let n = xyz.len() / 3;
        PyArray1::from_vec(py, xyz).reshape([n, 3])
    }
}
