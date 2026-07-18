cmake_minimum_required(VERSION 3.16)
project(all3dscan VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

option(BUILD_TESTS "Build unit tests" ${BUILD_TESTING})

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
find_package(nlohmann_json REQUIRED)

add_library(util
    src/util/ResourcePaths.h
    src/util/ResourcePaths.cpp
)
target_include_directories(util PUBLIC src)

add_library(protocol
    src/protocol/ScannerProtocol.h
    src/protocol/MockProtocol.cpp
)
target_include_directories(protocol PUBLIC src)

add_library(usb
    src/usb/UsbDevice.h
    src/usb/UsbDevice.cpp
)
target_include_directories(usb PUBLIC src ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(usb PUBLIC ${LIBUSB_LIBRARIES} nlohmann_json::nlohmann_json)
target_compile_options(usb PRIVATE ${LIBUSB_CFLAGS_OTHER})

add_library(pointcloud
    src/pointcloud/PointCloud.h
    src/pointcloud/PointCloud.cpp
)
target_include_directories(pointcloud PUBLIC src)
target_link_libraries(pointcloud PUBLIC protocol)

add_library(io
    src/io/PointCloudExport.h
    src/io/PointCloudExport.cpp
)
target_include_directories(io PUBLIC src)
target_link_libraries(io PUBLIC pointcloud)

add_library(python_bridge
    src/bridge/PythonWorker.h
    src/bridge/PythonWorker.cpp
)
target_include_directories(python_bridge PUBLIC src)
target_link_libraries(python_bridge PUBLIC util nlohmann_json::nlohmann_json)

add_executable(all3dscan
    src/app/main.cpp
    src/app/MainWindow.h
    src/app/MainWindow.cpp
    src/app/PointCloudView.h
    src/app/PointCloudView.cpp
    src/app/ScannerController.h
    src/app/ScannerController.cpp
    src/app/Theme.h
    src/app/Theme.cpp
    src/app/DistanceScaleWidget.h
    src/app/DistanceScaleWidget.cpp
    src/app/CameraPanel.h
    src/app/CameraPanel.cpp
)
target_include_directories(all3dscan PRIVATE src)
target_link_libraries(all3dscan PRIVATE Qt5::Widgets protocol pointcloud io usb python_bridge util)

install(TARGETS all3dscan RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(
    FILES resources/io.github.all3dscan.All3DScan.desktop
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
    FILES resources/devices.json
    DESTINATION ${CMAKE_INSTALL_DATADIR}/all3dscan
)
install(
    FILES resources/99-all3dscan-scanners.rules
    DESTINATION lib/udev/rules.d
)
install(
    PROGRAMS tools/pipeline/mesh_worker.py
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/all3dscan
)
install(
    FILES LICENSE README.md docs/protocol/RE_NOTES.md docs/protocol/REVOSCAN6_RE.md
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
