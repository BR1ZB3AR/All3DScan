pub mod ar;
pub mod decoder;
pub mod exporter;
pub mod protocol;

use std::ffi::CStr;
use std::os::raw::c_char;

/// C FFI interface function to check rust_core version
#[no_mangle]
pub extern "C" fn rust_core_version() -> *const c_char {
    let s = concat!(env!("CARGO_PKG_VERSION"), "\0");
    s.as_ptr() as *const c_char
}

/// C FFI interface function to build CS10 projector arming payload
#[no_mangle]
pub extern "C" fn rust_build_cs10_payload(emitters_on: bool, out_buffer: *mut u8) {
    if out_buffer.is_null() {
        return;
    }
    let payload = protocol::build_cs10_payload(emitters_on);
    unsafe {
        std::ptr::copy_nonoverlapping(payload.as_ptr(), out_buffer, 16);
    }
}

/// C FFI interface function to analyze raw depth frame
#[no_mangle]
pub extern "C" fn rust_analyze_depth_frame(
    raw_data: *const u8,
    data_len: usize,
    depth_scale_mm: f32,
    out_valid_points: *mut usize,
    out_min_depth: *mut f32,
    out_max_depth: *mut f32,
) -> bool {
    if raw_data.is_null() || data_len == 0 {
        return false;
    }
    let slice = unsafe { std::slice::from_raw_parts(raw_data, data_len) };
    let info = decoder::analyze_y16_depth_frame(slice, depth_scale_mm);

    unsafe {
        if !out_valid_points.is_null() {
            *out_valid_points = info.valid_points;
        }
        if !out_min_depth.is_null() {
            *out_min_depth = info.min_depth_mm;
        }
        if !out_max_depth.is_null() {
            *out_max_depth = info.max_depth_mm;
        }
    }
    true
}

/// C FFI interface function to export point cloud to ASCII PLY
#[no_mangle]
pub extern "C" fn rust_export_ply(
    points_ptr: *const exporter::Point3D,
    points_len: usize,
    output_path: *const c_char,
) -> bool {
    if points_ptr.is_null() || output_path.is_null() {
        return false;
    }
    let path_str = match unsafe { CStr::from_ptr(output_path) }.to_str() {
        Ok(s) => s,
        Err(_) => return false,
    };
    let slice = unsafe { std::slice::from_raw_parts(points_ptr, points_len) };
    exporter::export_ply_ascii(slice, path_str).is_ok()
}

/// C FFI interface function to project 3D points into 2D AR camera viewport overlay
#[no_mangle]
pub extern "C" fn rust_project_ar_points(
    points_ptr: *const ar::ArPoint3D,
    points_len: usize,
    intrinsics: *const ar::ArIntrinsics,
    out_2d_buffer: *mut ar::ArPoint2D,
    buffer_capacity: usize,
    out_written_count: *mut usize,
) -> bool {
    if points_ptr.is_null() || intrinsics.is_null() || out_2d_buffer.is_null() || out_written_count.is_null() {
        return false;
    }
    let points_slice = unsafe { std::slice::from_raw_parts(points_ptr, points_len) };
    let intrin = unsafe { &*intrinsics };

    let mut projected = Vec::new();
    ar::project_points_to_ar(points_slice, intrin, &mut projected);

    let copy_count = projected.len().min(buffer_capacity);
    unsafe {
        std::ptr::copy_nonoverlapping(projected.as_ptr(), out_2d_buffer, copy_count);
        *out_written_count = copy_count;
    }
    true
}
