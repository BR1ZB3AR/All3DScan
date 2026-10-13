find_package(Catch2 3 REQUIRED)

add_executable(unit_tests
    test_mock_protocol.cpp
    test_pointcloud.cpp
    test_export.cpp
    test_usb_devices.cpp
    test_pop4_protocol.cpp
    test_decoder.cpp
    test_scan_document.cpp
    test_geometry_export.cpp
    test_zxcmd.cpp
    test_registration.cpp
    test_depth_mesh.cpp
    test_tsdf.cpp
    test_stereo_rig.cpp
)
target_link_libraries(unit_tests PRIVATE
    Catch2::Catch2WithMain
    protocol
    pointcloud
    document
    io
    decoder
    usb
    net
    registration
    util
)
target_include_directories(unit_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)

include(CTest)
add_test(NAME unit_tests COMMAND unit_tests)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(
    NAME python_tools
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_python_tools.py
)
