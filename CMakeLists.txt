cmake_minimum_required(VERSION 3.18)
project(all3dscan VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

option(BUILD_TESTS "Build unit tests" ${BUILD_TESTING})
option(ALL3DSCAN_WITH_CUDA "Enable CUDA POP4 stereo depth decode" ON)

set(ALL3DSCAN_CUDA_ENABLED OFF)
if(ALL3DSCAN_WITH_CUDA)
    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
            # native picks the host GPU (e.g. RTX 5060 → sm_120 with CUDA 13.x).
            set(CMAKE_CUDA_ARCHITECTURES native)
        endif()
        enable_language(CUDA)
        find_package(CUDAToolkit QUIET)
        if(CUDAToolkit_FOUND)
            set(ALL3DSCAN_CUDA_ENABLED ON)
            message(STATUS "All3DScan CUDA stereo decode: ON (${CUDAToolkit_VERSION})")
        else()
            message(STATUS "All3DSCAN_WITH_CUDA=ON but CUDAToolkit not found; CPU-only")
        endif()
    else()
        message(STATUS "All3DSCAN_WITH_CUDA=ON but no CUDA compiler; CPU-only")
    endif()
else()
    message(STATUS "All3DScan CUDA stereo decode: OFF")
endif()

if(NOT TARGET MPI::MPI_C)
    add_library(MPI::MPI_C INTERFACE IMPORTED)
endif()
find_package(PCL QUIET COMPONENTS common io filters kdtree segmentation registration surface)
if(PCL_FOUND)
    message(STATUS "Point Cloud Library (PCL) found: ${PCL_VERSION}")
    add_compile_definitions(ALL3DSCAN_HAS_PCL=1)
    include_directories(${PCL_INCLUDE_DIRS})
    link_libraries(${PCL_LIBRARIES})
else()
    message(STATUS "PCL not found; using high-performance built-in PointCloud algorithms (install libpcl-dev to enable PCL features)")
endif()

find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL Network)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
find_package(nlohmann_json REQUIRED)
find_package(OpenGL REQUIRED)

add_library(util
    src/util/ResourcePaths.h
    src/util/ResourcePaths.cpp
)
target_include_directories(util PUBLIC src)

add_library(usb
    src/usb/UsbDevice.h
    src/usb/UsbDevice.cpp
    src/usb/V4l2Capture.h
    src/usb/V4l2Capture.cpp
)
target_include_directories(usb PUBLIC src ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(usb PUBLIC ${LIBUSB_LIBRARIES} nlohmann_json::nlohmann_json)
target_compile_options(usb PRIVATE ${LIBUSB_CFLAGS_OTHER})

set(DECODER_SOURCES
    src/decoder/Pop4DepthDecoder.h
    src/decoder/Pop4DepthDecoder.cpp
    src/decoder/Pop4StereoCuda.h
)
if(ALL3DSCAN_CUDA_ENABLED)
    list(APPEND DECODER_SOURCES src/decoder/Pop4StereoCuda.cu)
else()
    list(APPEND DECODER_SOURCES src/decoder/Pop4StereoCudaStub.cpp)
endif()

add_library(decoder ${DECODER_SOURCES})
target_include_directories(decoder PUBLIC src)
# Speckle SGM runs on the UI poll thread; OpenMP keeps it near frame rate.
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
    target_link_libraries(decoder PRIVATE OpenMP::OpenMP_CXX)
endif()
if(ALL3DSCAN_CUDA_ENABLED)
    set_target_properties(decoder PROPERTIES CUDA_SEPARABLE_COMPILATION OFF)
    target_compile_definitions(decoder PUBLIC ALL3DSCAN_HAS_CUDA=1)
    target_link_libraries(decoder PUBLIC CUDA::cudart)
    set_source_files_properties(src/decoder/Pop4StereoCuda.cu PROPERTIES LANGUAGE CUDA)
    # No fused multiply-add: keeps the GPU float steps bit-equal to the CPU matcher.
    target_compile_options(decoder PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>)
endif()

add_library(protocol
    src/protocol/ScannerProtocol.h
    src/protocol/MockProtocol.cpp
    src/protocol/RevopointPop4Protocol.h
    src/protocol/RevopointPop4Protocol.cpp
)
target_include_directories(protocol PUBLIC src)
target_link_libraries(protocol PUBLIC usb decoder)

add_library(pointcloud
    src/pointcloud/PointCloud.h
    src/pointcloud/PointCloud.cpp
)
target_include_directories(pointcloud PUBLIC src)
target_link_libraries(pointcloud PUBLIC protocol)

add_library(document
    src/document/ScanDocument.h
    src/document/ScanDocument.cpp
    src/geometry/TriangleMesh.h
    src/geometry/DepthMesh.h
    src/geometry/DepthMesh.cpp
    src/geometry/TsdfVolume.h
    src/geometry/TsdfVolume.cpp
)
target_include_directories(document PUBLIC src)
target_link_libraries(document PUBLIC pointcloud)
if(OpenMP_CXX_FOUND)
    target_link_libraries(document PRIVATE OpenMP::OpenMP_CXX) # TSDF integration
endif()

add_library(io
    src/io/PointCloudExport.h
    src/io/PointCloudExport.cpp
    src/io/GeometryExport.h
    src/io/GeometryExport.cpp
    src/io/PlyImport.h
    src/io/PlyImport.cpp
)
target_include_directories(io PUBLIC src)
target_link_libraries(io PUBLIC pointcloud document rust_bridge)

add_library(python_bridge
    src/bridge/PythonWorker.h
    src/bridge/PythonWorker.cpp
)
target_include_directories(python_bridge PUBLIC src)
target_link_libraries(python_bridge PUBLIC util nlohmann_json::nlohmann_json)

# Rust Core Integration (Option B)
set(RUST_STATIC_LIB "${CMAKE_SOURCE_DIR}/target/release/liball3dscan_rust.a")

add_custom_command(
    OUTPUT ${RUST_STATIC_LIB}
    COMMAND cargo build --release
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/rust_core
    DEPENDS ${CMAKE_SOURCE_DIR}/rust_core/src/lib.rs ${CMAKE_SOURCE_DIR}/rust_core/src/ar.rs ${CMAKE_SOURCE_DIR}/rust_core/src/protocol.rs ${CMAKE_SOURCE_DIR}/rust_core/src/decoder.rs ${CMAKE_SOURCE_DIR}/rust_core/src/exporter.rs
    COMMENT "Building Rust Core static library..."
)

add_custom_target(cargo_build_rust_core DEPENDS ${RUST_STATIC_LIB})

add_library(rust_core_lib STATIC IMPORTED GLOBAL)
add_dependencies(rust_core_lib cargo_build_rust_core)
set_target_properties(rust_core_lib PROPERTIES IMPORTED_LOCATION ${RUST_STATIC_LIB})

add_library(rust_bridge
    src/bridge/RustBridge.h
    src/bridge/RustBridge.cpp
)
target_include_directories(rust_bridge PUBLIC src)
target_link_libraries(rust_bridge PUBLIC rust_core_lib ${CMAKE_DL_LIBS} pthread)

add_library(registration
    src/processing/Registration.h
    src/processing/Registration.cpp
)
target_include_directories(registration PUBLIC src)
target_link_libraries(registration PUBLIC pointcloud)

add_library(processing
    src/processing/ScanPipeline.h
    src/processing/ScanPipeline.cpp
)
target_include_directories(processing PUBLIC src)
target_link_libraries(processing PUBLIC document io python_bridge registration Qt5::Core)

add_library(net
    src/net/ZxCmdProtocol.h
    src/net/ZxCmdProtocol.cpp
    src/net/RevopointWifiControl.h
    src/net/RevopointWifiControl.cpp
)
target_include_directories(net PUBLIC src)
target_link_libraries(net PUBLIC Qt5::Core Qt5::Network)

add_executable(all3dscan
    src/app/main.cpp
    src/app/MainWindow.h
    src/app/MainWindow.cpp
    src/app/PointCloudView.h
    src/app/PointCloudView.cpp
    src/app/ScannerController.h
    src/app/ScannerController.cpp
    src/app/ScannerControlPanel.h
    src/app/ScannerControlPanel.cpp
    src/app/EditPanel.h
    src/app/EditPanel.cpp
    src/app/Theme.h
    src/app/Theme.cpp
    src/app/DistanceScaleWidget.h
    src/app/DistanceScaleWidget.cpp
    src/app/CameraPanel.h
    src/app/CameraPanel.cpp
    src/app/ArOverlayView.h
    src/app/ArOverlayView.cpp
)
target_include_directories(all3dscan PRIVATE src)
target_link_libraries(all3dscan PRIVATE
    Qt5::Widgets
    Qt5::OpenGL
    Qt5::Network
    OpenGL::GL
    protocol
    pointcloud
    document
    processing
    io
    usb
    decoder
    python_bridge
    rust_bridge
    registration
    net
    util
)

install(TARGETS all3dscan RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
# Absolute Exec path so launchers work when the bin dir is not on the session PATH
# (e.g. a ~/.local install under KDE).
file(READ resources/io.github.all3dscan.All3DScan.desktop ALL3DSCAN_DESKTOP)
string(REPLACE "Exec=all3dscan" "Exec=${CMAKE_INSTALL_FULL_BINDIR}/all3dscan"
       ALL3DSCAN_DESKTOP "${ALL3DSCAN_DESKTOP}")
file(WRITE ${CMAKE_BINARY_DIR}/io.github.all3dscan.All3DScan.desktop "${ALL3DSCAN_DESKTOP}")
install(
    FILES ${CMAKE_BINARY_DIR}/io.github.all3dscan.All3DScan.desktop
    DESTINATION ${CMAKE_INSTALL_DATADIR}/applications
)
install(
    FILES resources/io.github.all3dscan.All3DScan.svg
    DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps
)
install(
    FILES resources/io.github.all3dscan.All3DScan.metainfo.xml
    DESTINATION ${CMAKE_INSTALL_DATADIR}/metainfo
)
install(
    FILES resources/devices.json resources/param_pop4.json
    DESTINATION ${CMAKE_INSTALL_DATADIR}/all3dscan
)
install(
    FILES resources/70-all3dscan-scanners.rules
    DESTINATION lib/udev/rules.d
)
install(
    PROGRAMS tools/pipeline/mesh_worker.py
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/all3dscan
)
install(
    FILES
        LICENSE
        README.md
        docs/protocol/RE_NOTES.md
        docs/protocol/REVOSCAN6_RE.md
        docs/protocol/APK_REVOSCAN_6.0.2.4.md
        docs/protocol/USBCAP_CHECKLIST.md
    DESTINATION ${CMAKE_INSTALL_DOCDIR}
)

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

set(CPACK_PACKAGE_NAME "all3dscan")
set(CPACK_PACKAGE_VENDOR "All3DScan contributors")
set(CPACK_PACKAGE_CONTACT "All3DScan contributors")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Open-source native Linux scanning app for Revopoint, Creality and Einstar scanners")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_INSTALL_DIRECTORY "All3DScan")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "All3DScan contributors")
set(CPACK_DEBIAN_PACKAGE_SECTION "graphics")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_DEPENDS "python3 (>= 3.10)")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_GENERATOR "DEB")
include(CPack)
