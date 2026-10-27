//! Compiles the C++ sensor layer (../hw_cpp) and its cxx bridge into the engine.
//!
//! The POP 4 goes through the scanner backend the legacy app uses: ../src/usb,
//! ../src/protocol and ../src/decoder, compiled here too, with the decoder's
//! CUDA stereo matcher when nvcc is found (ALL3DSCAN_CUDA_ARCH picks the GPU,
//! "native" by default) and its CPU matcher otherwise. That backend needs
//! libusb-1.0; without it, the POP 4 is compiled out and opening one says so.
//! libturbojpeg, when found, decodes the colour camera.

use std::env;
use std::path::{Path, PathBuf};

fn main() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
    let hw_cpp = root.join("hw_cpp");
    let legacy = root.join("src");

    let libusb = pkg_config::Config::new()
        .cargo_metadata(false)
        .probe("libusb-1.0")
        .ok();
    if libusb.is_none() {
        println!("cargo:warning=libusb-1.0 not found: building without POP 4 support");
    }

    cxx_build::bridge("src/hw.rs")
        .file(hw_cpp.join("src/MockDepthSensor.cpp"))
        .file(hw_cpp.join("src/Pop4Presence.cpp"))
        .include(hw_cpp.join("include"))
        .std("c++17")
        .flag_if_supported("-Wall")
        .flag_if_supported("-Wextra")
        .compile("all3dscan_hw");

    let Some(libusb) = libusb else {
        cc::Build::new()
            .cpp(true)
            .std("c++17")
            .file(hw_cpp.join("src/Pop4SensorStub.cpp"))
            .include(hw_cpp.join("include"))
            .compile("all3dscan_pop4");
        rerun_if_changed(&[&hw_cpp]);
        return;
    };

    // The legacy sources keep their own warnings out of this build's output.
    let mut pop4 = cc::Build::new();
    pop4.cpp(true)
        .std("c++17")
        .warnings(false)
        .flag("-fopenmp")
        .include(hw_cpp.join("include"))
        .include(&legacy)
        .includes(&libusb.include_paths)
        .file(hw_cpp.join("src/Pop4Sensor.cpp"))
        .files(
            [
                "usb/UsbDevice.cpp",
                "usb/V4l2Capture.cpp",
                "protocol/RevopointPop4Protocol.cpp",
                "decoder/Pop4DepthDecoder.cpp",
                "decoder/StereoRig.cpp",
                "decoder/RgbCalibration.cpp",
            ]
            .map(|file| legacy.join(file)),
        );
    let turbojpeg = pkg_config::Config::new()
        .cargo_metadata(false)
        .probe("libturbojpeg")
        .ok();
    if let Some(turbojpeg) = &turbojpeg {
        pop4.define("ALL3DSCAN_HAVE_TURBOJPEG", "1")
            .includes(&turbojpeg.include_paths);
    } else {
        println!("cargo:warning=libturbojpeg not found: POP 4 frames will have no colour");
    }
    let nvcc = find_nvcc();
    if nvcc.is_none() {
        println!("cargo:warning=nvcc not found: the POP 4 depth decoder runs on the CPU");
        pop4.file(legacy.join("decoder/Pop4StereoCudaStub.cpp"));
    }
    pop4.compile("all3dscan_pop4");

    if let Some(nvcc) = &nvcc {
        let arch = env::var("ALL3DSCAN_CUDA_ARCH").unwrap_or_else(|_| "native".into());
        // cc finds nvcc through $NVCC; set as the compiler, it would be taken
        // for gcc and given gcc's flags.
        // SAFETY: the build script is single-threaded here.
        unsafe { env::set_var("NVCC", nvcc) };
        cc::Build::new()
            .cuda(true)
            .cudart("none") // linked below, statically, so the module finds it anywhere
            .file(legacy.join("decoder/Pop4StereoCuda.cu"))
            .include(&legacy)
            .flag("-std=c++17")
            .flag(format!("-arch={arch}"))
            // No fused multiply-add: keeps the GPU's floats bit-equal to the
            // CPU matcher's, as the CMake build does.
            .flag("--fmad=false")
            .flag("-Xcompiler")
            .flag("-fPIC")
            .compile("all3dscan_cuda");
        let cuda = nvcc
            .parent()
            .and_then(Path::parent)
            .expect("nvcc sits in <cuda>/bin");
        for lib in ["lib64", "targets/x86_64-linux/lib"] {
            println!(
                "cargo:rustc-link-search=native={}",
                cuda.join(lib).display()
            );
        }
        println!("cargo:rustc-link-lib=static=cudart_static");
        for lib in ["dl", "rt", "pthread"] {
            println!("cargo:rustc-link-lib={lib}");
        }
    }

    // After the static libraries above, which need them.
    for library in std::iter::once(&libusb).chain(&turbojpeg) {
        for path in &library.link_paths {
            println!("cargo:rustc-link-search=native={}", path.display());
        }
        for lib in &library.libs {
            println!("cargo:rustc-link-lib={lib}");
        }
    }
    println!("cargo:rustc-link-lib=gomp");

    rerun_if_changed(&[
        &hw_cpp,
        &legacy.join("usb"),
        &legacy.join("protocol"),
        &legacy.join("decoder"),
    ]);
    for var in ["ALL3DSCAN_CUDA_ARCH", "NVCC", "CUDA_PATH"] {
        println!("cargo:rerun-if-env-changed={var}");
    }
}

/// nvcc from $NVCC, $CUDA_PATH/bin, the PATH or /opt/cuda/bin (Arch's place for it).
fn find_nvcc() -> Option<PathBuf> {
    let candidates = env::var_os("NVCC")
        .map(PathBuf::from)
        .into_iter()
        .chain(env::var_os("CUDA_PATH").map(|cuda| PathBuf::from(cuda).join("bin/nvcc")));
    let on_path = env::var_os("PATH")
        .into_iter()
        .flat_map(|path| env::split_paths(&path).collect::<Vec<_>>())
        .map(|dir| dir.join("nvcc"));
    candidates
        .chain(on_path)
        .chain([PathBuf::from("/opt/cuda/bin/nvcc")])
        .find(|nvcc| nvcc.is_file())
}

fn rerun_if_changed(paths: &[&Path]) {
    println!("cargo:rerun-if-changed=src/hw.rs");
    for path in paths {
        println!("cargo:rerun-if-changed={}", path.display());
    }
}
