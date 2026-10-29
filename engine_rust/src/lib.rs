use pyo3::prelude::*;

pub mod fitting;
pub mod fusion;
pub mod hw;
pub mod interchange;
pub mod mesh;
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
    use crate::{fitting, fusion, hw, interchange, mesh, processing};

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

        /// The pinhole the frames are projected through: fx, fy, cx, cy.
        #[getter]
        fn intrinsics(&self) -> (f64, f64, f64, f64) {
            let lens = self
                .sensor
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .intrinsics();
            (lens[0], lens[1], lens[2], lens[3])
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

        /// The depth camera's pinhole: fx, fy, cx, cy. All 0 until the
        /// first frame has come back.
        #[getter]
        fn intrinsics(&self) -> PyResult<(f64, f64, f64, f64)> {
            let lens = self.with(|sensor| sensor.intrinsics())?;
            Ok((lens[0], lens[1], lens[2], lens[3]))
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

    /// Drops the points that have far fewer neighbours within `radius_mm`
    /// than the cloud's points typically do: the speckle a depth camera
    /// leaves in mid-air, which has nothing around it, where a surface
    /// point has its neighbours.
    ///
    /// A point is kept when at least `share` of the typical neighbour count
    /// surrounds it. The measure comes from the cloud itself, so the same
    /// settings suit a dense scan and a sparse one.
    #[pyfunction]
    #[pyo3(signature = (points, colors, normals, radius_mm = 3.0, share = 0.2))]
    fn remove_speckle<'py>(
        py: Python<'py>,
        points: PyArrayLike2<'py, f32, AllowTypeChange>,
        colors: PyArrayLike2<'py, u8, AllowTypeChange>,
        normals: PyArrayLike2<'py, f32, AllowTypeChange>,
        radius_mm: f32,
        share: f32,
    ) -> PyResult<Cloud<'py>> {
        let cloud = processing::Cloud {
            points: triples(&points, "points")?,
            colors: colour_rows(&colors, points.as_array().nrows())?,
            normals: triples(&normals, "normals")?,
        };
        if cloud.normals.len() != cloud.points.len() {
            return Err(PyValueError::new_err(format!(
                "normals must match points, got {} normals for {} points",
                cloud.normals.len(),
                cloud.points.len()
            )));
        }
        let kept = py
            .detach(|| processing::remove_speckle(&cloud, radius_mm, share))
            .map_err(value_error)?;
        cloud_into_py(py, &kept)
    }

    /// Keeps the object and drops the room around it.
    ///
    /// A scan of something on a desk is mostly desk, and the clutter behind
    /// it comes too. This takes out the flat surface the object stands on or
    /// in front of, and then everything not joined to the largest remaining
    /// piece — in that order, since the object is usually joined to the rest
    /// of the room through the surface it stands on.
    ///
    /// Both steps are guarded: a cloud that is mostly one flat thing is
    /// taken to be a scan of something flat and kept whole, and a step that
    /// would leave almost nothing is not taken. `gap_mm` is how far apart
    /// two points may be and still count as joined; None works it out from
    /// the cloud's own spacing.
    ///
    /// Returns the kept cloud, the indices it kept, whether a surface was
    /// removed, and how many separate pieces the cloud fell into.
    #[pyfunction]
    #[pyo3(signature = (points, colors, normals, gap_mm = None))]
    fn isolate<'py>(
        py: Python<'py>,
        points: PyArrayLike2<'py, f32, AllowTypeChange>,
        colors: PyArrayLike2<'py, u8, AllowTypeChange>,
        normals: PyArrayLike2<'py, f32, AllowTypeChange>,
        gap_mm: Option<f32>,
    ) -> PyResult<Bound<'py, PyDict>> {
        let rows = points.as_array().nrows();
        let cloud = processing::Cloud {
            points: triples(&points, "points")?,
            colors: colour_rows(&colors, rows)?,
            // Normals are optional here: without them no surface is looked
            // for, and only the joined-up step runs.
            normals: if normals.as_array().nrows() == 0 {
                Vec::new()
            } else {
                triples(&normals, "normals")?
            },
        };
        if !cloud.normals.is_empty() && cloud.normals.len() != cloud.points.len() {
            return Err(PyValueError::new_err(format!(
                "normals must match points, got {} normals for {} points",
                cloud.normals.len(),
                cloud.points.len()
            )));
        }
        let found = py
            .detach(|| processing::isolate(&cloud, gap_mm))
            .map_err(value_error)?;
        let (kept_points, kept_colors, kept_normals) = cloud_into_py(py, &found.cloud)?;
        let out = PyDict::new(py);
        out.set_item("points", kept_points)?;
        out.set_item("colors", kept_colors)?;
        out.set_item("normals", kept_normals)?;
        out.set_item("kept", PyArray1::from_vec(py, found.kept))?;
        out.set_item("surface_removed", found.surface_removed)?;
        out.set_item("pieces", found.pieces)?;
        Ok(out)
    }

    /// The pieces a cloud falls into, largest first, as arrays of indices.
    ///
    /// Two points belong to the same piece when a chain of points no more
    /// than `gap_mm` apart joins them.
    #[pyfunction]
    #[pyo3(signature = (points, gap_mm))]
    fn pieces<'py>(
        py: Python<'py>,
        points: PyArrayLike2<'py, f32, AllowTypeChange>,
        gap_mm: f32,
    ) -> PyResult<Vec<Bound<'py, PyArray1<u32>>>> {
        let cloud: Vec<Point3<f32>> = triples(&points, "points")?;
        let found = py
            .detach(move || processing::pieces(&cloud, gap_mm))
            .map_err(value_error)?;
        Ok(found.into_iter().map(|piece| PyArray1::from_vec(py, piece)).collect())
    }

    /// Fits one smooth surface through a point cloud, using its normals.
    ///
    /// For a cloud with no depth views behind it: one opened from a file, or
    /// from a scanner All3DScan cannot drive. A scan taken here keeps its
    /// views, and Fusion.surface() makes a better job of those, because it
    /// knows where the scanner looked and saw nothing.
    ///
    /// `detail` is how finely space is divided — each step doubles the
    /// detail and roughly doubles the work. `trim` cuts away the share of
    /// the surface with the least data behind it: the reconstruction closes
    /// over everything, including the places a scan never reached, and this
    /// is what takes that invented part off again. 0 keeps it closed.
    ///
    /// Returns the (N, 3) float32 vertices in millimetres, an (N, 3) unit
    /// normal each, the (M, 3) uint32 triangles, and what the trim removed.
    #[pyfunction]
    #[pyo3(signature = (points, normals, detail = 8, trim = 0.10))]
    fn cloud_surface<'py>(
        py: Python<'py>,
        points: PyArrayLike2<'py, f32, AllowTypeChange>,
        normals: PyArrayLike2<'py, f32, AllowTypeChange>,
        detail: i32,
        trim: f32,
    ) -> PyResult<Bound<'py, PyDict>> {
        let cloud: Vec<Point3<f32>> = triples(&points, "points")?;
        let directions: Vec<Vector3<f32>> = triples(&normals, "normals")?;
        if directions.len() != cloud.len() {
            return Err(PyValueError::new_err(format!(
                "normals must match points, got {} normals for {} points",
                directions.len(),
                cloud.len()
            )));
        }
        let settings = mesh::CloudSurfaceSettings { depth: detail, trim };
        let (surface, trimmed) = py
            .detach(move || mesh::cloud_surface(&cloud, &directions, settings))
            .map_err(|e| PyValueError::new_err(e.what().to_owned()))?;
        let triangles = surface.triangles();
        let found = PyDict::new(py);
        found.set_item("vertices", into_array(py, flatten(&surface.vertices))?)?;
        found.set_item("normals", into_array(py, surface.normals)?)?;
        found.set_item(
            "triangles",
            PyArray1::from_vec(py, surface.indices).reshape([triangles, 3])?,
        )?;
        found.set_item("trimmed_vertices", trimmed.vertices)?;
        found.set_item("trimmed_triangles", trimmed.triangles)?;
        Ok(found)
    }

    /// Whether a POP 4 is plugged in, without opening it.
    #[pyfunction]
    fn pop4_present() -> bool {
        hw::Pop4Sensor::present()
    }

    /// A surface handed back to Python: vertices, their normals, and the
    /// triples of indices that make its triangles.
    type Surface<'py> = (
        Bound<'py, PyArray2<f32>>,
        Bound<'py, PyArray2<f32>>,
        Bound<'py, PyArray2<u32>>,
    );

    /// A cloud handed back to Python: its points, colours and normals.
    type Cloud<'py> = (
        Bound<'py, PyArray2<f32>>,
        Bound<'py, PyArray2<u8>>,
        Bound<'py, PyArray2<f32>>,
    );

    /// A cloud's points, colours and normals, as three numpy arrays.
    fn cloud_into_py<'py>(py: Python<'py>, cloud: &processing::Cloud) -> PyResult<Cloud<'py>> {
        let n = cloud.points.len();
        let rgb: Vec<u8> = cloud.colors.iter().copied().flatten().collect();
        let normals: Vec<f32> = cloud.normals.iter().flat_map(|v| [v.x, v.y, v.z]).collect();
        Ok((
            into_array(py, flatten(&cloud.points))?,
            PyArray1::from_vec(py, rgb).reshape([n, 3])?,
            into_array(py, normals)?,
        ))
    }

    /// The rows of an (N, 3) uint8 array, after checking its shape.
    fn colour_rows(
        array: &PyArrayLike2<'_, u8, AllowTypeChange>,
        points: usize,
    ) -> PyResult<Vec<[u8; 3]>> {
        let view = array.as_array();
        if view.ncols() != 3 || view.nrows() != points {
            return Err(PyValueError::new_err(format!(
                "colors must have shape ({points}, 3), got {:?}",
                view.shape()
            )));
        }
        Ok(view
            .rows()
            .into_iter()
            .map(|r| [r[0], r[1], r[2]])
            .collect())
    }

    /// Builds one point cloud out of a scan's frames.
    ///
    /// Each frame is aligned onto the model built so far, with point-to-plane
    /// ICP against the Qt app's own tracker, and merged into it, thinned to
    /// one point per voxel. The first frame becomes the model and fixes the
    /// axes the rest are placed in. A frame that cannot be placed is left out
    /// rather than smeared across the model, and add() says so.
    #[pyclass(frozen)]
    struct Fusion {
        // Adding from one thread while the GUI reads the model from another
        // takes turns instead of racing.
        fusion: Mutex<fusion::Fusion>,
        model_points: AtomicU64,
    }

    #[pymethods]
    impl Fusion {
        #[new]
        #[pyo3(signature = (voxel_mm = 1.0))]
        fn new(voxel_mm: f32) -> PyResult<Self> {
            let fusion = fusion::Fusion::new(voxel_mm)
                .map_err(|e| PyValueError::new_err(e.what().to_owned()))?;
            Ok(Self {
                fusion: Mutex::new(fusion),
                model_points: AtomicU64::new(0),
            })
        }

        /// Aligns an (N, 3) frame onto the model and merges it, carrying its
        /// colours and normals across. Returns how it sat: whether it was
        /// merged, the share of its points on the model's surface, their RMS
        /// distance in millimetres, and the points in the model afterwards.
        ///
        /// With `depth_mm`, the frame's (height, width) depth image, the
        /// view is kept for building a surface later, if the scanner has
        /// moved far enough since the last one kept.
        #[pyo3(signature = (points, colors, normals, depth_mm = None))]
        fn add<'py>(
            &self,
            py: Python<'py>,
            points: PyArrayLike2<'py, f32, AllowTypeChange>,
            colors: PyArrayLike2<'py, u8, AllowTypeChange>,
            normals: PyArrayLike2<'py, f32, AllowTypeChange>,
            depth_mm: Option<PyArrayLike2<'py, u16, AllowTypeChange>>,
        ) -> PyResult<Bound<'py, PyDict>> {
            // Copied while attached: other Python threads may change the
            // arrays once the GIL is released.
            let frame = processing::Cloud {
                points: triples(&points, "points")?,
                colors: colour_rows(&colors, points.as_array().nrows())?,
                normals: triples(&normals, "normals")?,
            };
            if frame.normals.len() != frame.points.len() {
                return Err(PyValueError::new_err(format!(
                    "normals must match points, got {} normals for {} points",
                    frame.normals.len(),
                    frame.points.len()
                )));
            }
            let view = depth_mm.map(|image| {
                let grid = image.as_array();
                (
                    grid.iter().copied().collect::<Vec<u16>>(),
                    grid.ncols() as u32,
                    grid.nrows() as u32,
                )
            });
            let fit = py
                .detach(|| {
                    let mut fusion = self.locked();
                    let fit = fusion.add(&frame)?;
                    if fit.merged
                        && let Some((depth, width, height)) = view
                    {
                        fusion.keep_view(&depth, width, height);
                    }
                    Ok::<_, fusion::FusionError>(fit)
                })
                .map_err(|e| PyValueError::new_err(e.to_string()))?;
            self.model_points.store(fit.model_points, Ordering::Relaxed);
            let result = PyDict::new(py);
            result.set_item("merged", fit.merged)?;
            result.set_item("overlap", fit.overlap)?;
            result.set_item("rmse_mm", fit.rmse_mm)?;
            result.set_item("model_points", fit.model_points)?;
            Ok(result)
        }

        /// The model so far, as an (N, 3) float32 array of millimetres in the
        /// first frame's axes, and its (N, 3) uint8 colours.
        fn cloud<'py>(&self, py: Python<'py>) -> PyResult<Cloud<'py>> {
            let copied = py.detach(|| self.locked().cloud().clone());
            cloud_into_py(py, &copied)
        }

        /// Melts the depth views kept during the scan into one closed
        /// surface over the model: its (N, 3) float32 vertices in
        /// millimetres, an (N, 3) unit normal each, and an (M, 3) uint32
        /// index per triangle. `intrinsics` is the depth camera's fx, fy,
        /// cx, cy. Comes back empty when there is nothing to build from.
        #[pyo3(signature = (intrinsics, voxel_mm = 1.0, smoothing = 8))]
        fn surface<'py>(
            &self,
            py: Python<'py>,
            intrinsics: (f64, f64, f64, f64),
            voxel_mm: f32,
            smoothing: i32,
        ) -> PyResult<Surface<'py>> {
            let (fx, fy, cx, cy) = intrinsics;
            let built = py
                .detach(|| self.locked().surface([fx, fy, cx, cy], voxel_mm, smoothing))
                .map_err(|e| PyValueError::new_err(e.what().to_owned()))?;
            let triangles = built.triangles();
            Ok((
                into_array(py, flatten(&built.vertices))?,
                into_array(py, built.normals)?,
                PyArray1::from_vec(py, built.indices).reshape([triangles, 3])?,
            ))
        }

        /// How many depth views are being held for the surface.
        #[getter]
        fn views(&self) -> usize {
            self.locked().views()
        }

        /// Throws the model away and starts again.
        fn clear(&self, py: Python<'_>) {
            py.detach(|| self.locked().clear());
            self.model_points.store(0, Ordering::Relaxed);
        }

        #[getter]
        fn model_points(&self) -> u64 {
            self.model_points.load(Ordering::Relaxed)
        }

        /// Frames handed over, and of those, the ones that were merged.
        #[getter]
        fn frames(&self) -> u64 {
            self.locked().frames()
        }

        #[getter]
        fn merged(&self) -> u64 {
            self.locked().merged()
        }
    }

    impl Fusion {
        fn locked(&self) -> std::sync::MutexGuard<'_, fusion::Fusion> {
            self.fusion.lock().unwrap_or_else(PoisonError::into_inner)
        }
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

    /// Works out a unit normal for each point of a cloud that has no order
    /// to it — one read from a file, rather than laid out as a depth
    /// camera's pixels.
    ///
    /// Each normal is the direction its neighbours within radius_mm are
    /// least spread in, turned to face `towards` (the sensor's direction,
    /// (0, 0, -1) by default). A point with too few neighbours to judge gets
    /// a zero normal rather than a made-up one.
    #[pyfunction]
    #[pyo3(signature = (points, radius_mm = 3.0, towards = (0.0, 0.0, -1.0)))]
    fn estimate_normals<'py>(
        py: Python<'py>,
        points: PyArrayLike2<'py, f32, AllowTypeChange>,
        radius_mm: f32,
        towards: (f32, f32, f32),
    ) -> PyResult<Bound<'py, PyArray2<f32>>> {
        let cloud: Vec<Point3<f32>> = triples(&points, "points")?;
        let facing = Vector3::new(towards.0, towards.1, towards.2);
        let normals = py
            .detach(move || processing::estimate_normals(&cloud, radius_mm, facing))
            .map_err(value_error)?;
        into_array(py, normals.iter().flat_map(|n| [n.x, n.y, n.z]).collect())
    }

    /// Reads a scan file that another tool or another scanner wrote.
    ///
    /// Takes .ply, .obj, .stl, .pcd, .asc, .xyz, .pts, .txt and .p3, working
    /// out which from the name and, failing that, from what is inside. Comes
    /// back as a dict: "points" (N, 3) float32, "colors" (N, 3) uint8 or
    /// None, "normals" (N, 3) float32 or None, "triangles" (M, 3) uint32 or
    /// None, and "extent", the (lowest, highest) corner of the box the
    /// points fill — lengths exactly as the file gave them, so a caller can
    /// see from the extent whether it was written in metres.
    #[pyfunction]
    fn read_scan<'py>(py: Python<'py>, path: &str) -> PyResult<Bound<'py, PyDict>> {
        let owned = path.to_owned();
        let scene = py
            .detach(move || interchange::read(std::path::Path::new(&owned)))
            .map_err(scan_file_error)?;
        let n = scene.points.len();
        // Before the arrays below move the scene's parts out of it.
        let extent = scene
            .extent()
            .map(|(low, high)| ((low.x, low.y, low.z), (high.x, high.y, high.z)));
        let found = PyDict::new(py);
        found.set_item("points", into_array(py, flatten(&scene.points))?)?;
        found.set_item(
            "colors",
            if scene.has_colors() {
                let rgb: Vec<u8> = scene.colors.iter().copied().flatten().collect();
                Some(PyArray1::from_vec(py, rgb).reshape([n, 3])?)
            } else {
                None
            },
        )?;
        found.set_item(
            "normals",
            if scene.has_normals() {
                let normals: Vec<f32> =
                    scene.normals.iter().flat_map(|v| [v.x, v.y, v.z]).collect();
                Some(into_array(py, normals)?)
            } else {
                None
            },
        )?;
        let triangles = scene.triangles();
        found.set_item(
            "triangles",
            if triangles > 0 {
                Some(PyArray1::from_vec(py, scene.indices).reshape([triangles, 3])?)
            } else {
                None
            },
        )?;
        found.set_item("extent", extent)?;
        Ok(found)
    }

    /// Writes a scan for another tool to open.
    ///
    /// `format` is one of the names scan_formats() gives, or None to take it
    /// from the path's extension. Triangles are (M, 3) indices into the
    /// points; passing them to a format that holds no mesh drops them, and
    /// passing none to STL, which holds nothing else, is an error.
    #[pyfunction]
    #[pyo3(signature = (path, points, colors = None, normals = None, triangles = None, format = None))]
    fn write_scan(
        py: Python<'_>,
        path: &str,
        points: PyArrayLike2<'_, f32, AllowTypeChange>,
        colors: Option<PyArrayLike2<'_, u8, AllowTypeChange>>,
        normals: Option<PyArrayLike2<'_, f32, AllowTypeChange>>,
        triangles: Option<PyArrayLike2<'_, u32, AllowTypeChange>>,
        format: Option<&str>,
    ) -> PyResult<String> {
        let scene = interchange::Scene {
            points: triples(&points, "points")?,
            colors: match &colors {
                Some(rgb) => colour_rows(rgb, points.shape()[0])?,
                None => Vec::new(),
            },
            normals: match &normals {
                Some(n) => triples(n, "normals")?,
                None => Vec::new(),
            },
            indices: match &triangles {
                Some(faces) => {
                    let shape = faces.shape();
                    if shape.len() != 2 || shape[1] != 3 {
                        return Err(PyValueError::new_err(format!(
                            "triangles must be an (M, 3) array, got {shape:?}"
                        )));
                    }
                    faces.as_array().iter().copied().collect()
                }
                None => Vec::new(),
            },
        };
        let chosen = match format {
            Some(name) => Some(named_format(name)?),
            None => None,
        };
        let owned = path.to_owned();
        let written = py
            .detach(move || {
                let path = std::path::Path::new(&owned);
                let path = match chosen {
                    Some(f) if interchange::Format::of_path(path) != Some(f) => {
                        interchange::with_extension(path, f)
                    }
                    _ => path.to_path_buf(),
                };
                interchange::write(&path, &scene, chosen).map(|()| path)
            })
            .map_err(scan_file_error)?;
        Ok(written.display().to_string())
    }

    /// The formats write_scan() takes: each a (name, what it is, file
    /// extension, whether it keeps triangles, whether it keeps colour).
    #[pyfunction]
    fn scan_formats() -> Vec<(&'static str, &'static str, &'static str, bool, bool)> {
        use interchange::Format;
        [
            (Format::Ply, "ply", "PLY, binary — points, colour, normals and mesh"),
            (Format::PlyText, "ply-text", "PLY, text — the same, readable"),
            (Format::Stl, "stl", "STL, binary — a mesh on its own, for printing"),
            (Format::StlText, "stl-text", "STL, text"),
            (Format::Obj, "obj", "OBJ — a mesh or a cloud, widely read"),
            (Format::Pcd, "pcd", "PCD — the point cloud format of PCL"),
            (Format::Asc, "asc", "Plain columns of numbers"),
        ]
        .into_iter()
        .map(|(format, name, what)| {
            (
                name,
                what,
                match format {
                    Format::Ply | Format::PlyText => "ply",
                    Format::Stl | Format::StlText => "stl",
                    Format::Obj => "obj",
                    Format::Pcd => "pcd",
                    Format::Asc => "asc",
                },
                format.keeps_triangles(),
                format.keeps_colors(),
            )
        })
        .collect()
    }

    fn named_format(name: &str) -> PyResult<interchange::Format> {
        use interchange::Format;
        Ok(match name {
            "ply" => Format::Ply,
            "ply-text" => Format::PlyText,
            "stl" => Format::Stl,
            "stl-text" => Format::StlText,
            "obj" => Format::Obj,
            "pcd" => Format::Pcd,
            "asc" => Format::Asc,
            other => {
                return Err(PyValueError::new_err(format!(
                    "{other} is not a scan format: try ply, ply-text, stl, stl-text, \
                     obj, pcd or asc"
                )))
            }
        })
    }

    /// A file that would not read or write, as the Python error that fits
    /// it: something the caller got wrong, or something the disk did.
    fn scan_file_error(why: interchange::InterchangeError) -> PyErr {
        use interchange::InterchangeError;
        match why {
            InterchangeError::Io(_) => PyOSError::new_err(why.to_string()),
            _ => PyValueError::new_err(why.to_string()),
        }
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
