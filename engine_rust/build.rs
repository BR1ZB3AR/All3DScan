//! Compiles the C++ layers (../hw_cpp, ../fusion_cpp) and their cxx bridges
//! into the engine.
//!
//! Fusion reuses the Qt app's registration (../src/processing/Registration,
//! ../src/pointcloud), which needs nothing but the standard library and
//! OpenMP, so it is always built.
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

    let fusion_cpp = root.join("fusion_cpp");
    let mesh_cpp = root.join("mesh_cpp");
    // Screened Poisson reconstruction, for clouds with no depth views behind
    // them. Header-only: an include path is the whole of it.
    let poisson = root.join("third_party/PoissonRecon/Src");
    cxx_build::bridges(["src/hw.rs", "src/fusion.rs", "src/mesh.rs"])
        .file(hw_cpp.join("src/MockDepthSensor.cpp"))
        .file(hw_cpp.join("src/Pop4Presence.cpp"))
        .include(hw_cpp.join("include"))
        .include(fusion_cpp.join("include"))
        .include(mesh_cpp.join("include"))
        .include(&legacy) // the shims reach the registration and geometry headers
        .std("c++17")
        .flag_if_supported("-Wall")
        .flag_if_supported("-Wextra")
        .compile("all3dscan_hw");

    // The registration's own warnings stay out of this build's output.
    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .warnings(false)
        .flag("-fopenmp") // ICP's hot loops
        .include(fusion_cpp.join("include"))
        .include(&legacy)
        .file(fusion_cpp.join("src/FrameFusion.cpp"))
        .file(mesh_cpp.join("src/SurfaceBuilder.cpp"))
        .file(legacy.join("pointcloud/PointCloud.cpp"))
        .file(legacy.join("processing/Registration.cpp"))
        .file(legacy.join("geometry/TsdfVolume.cpp"))
        .include(mesh_cpp.join("include"))
        .compile("all3dscan_fusion");

    // Poisson is built on its own: it is third-party template code whose
    // warnings are not ours to fix, and it takes a while, so it does not
    // share a target with anything that changes often.
    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .warnings(false)
        .flag("-fopenmp")
        .include(mesh_cpp.join("include"))
        .include(&poisson)
        .file(mesh_cpp.join("src/CloudSurface.cpp"))
        .compile("all3dscan_cloud_surface");
    println!("cargo:rustc-link-lib=gomp");

    let Some(libusb) = libusb else {
        cc::Build::new()
            .cpp(true)
            .std("c++17")
            .file(hw_cpp.join("src/Pop4SensorStub.cpp"))
            .include(hw_cpp.join("include"))
            .compile("all3dscan_pop4");
        rerun_if_changed(&[&hw_cpp, &fusion_cpp, &mesh_cpp, &legacy.join("processing")]);
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

    rerun_if_changed(&[
        &hw_cpp,
        &fusion_cpp,
        &mesh_cpp,
        &legacy.join("geometry"),
        &legacy.join("usb"),
        &legacy.join("protocol"),
        &legacy.join("decoder"),
        &legacy.join("pointcloud"),
        &legacy.join("processing"),
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
    println!("cargo:rerun-if-changed=src/fusion.rs");
    println!("cargo:rerun-if-changed=src/mesh.rs");
    for path in paths {
        println!("cargo:rerun-if-changed={}", path.display());
    }
}
