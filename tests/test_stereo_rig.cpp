#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "decoder/StereoRig.h"

using namespace revoscan;

namespace {

// Synthetic rig in the camparam.yaml format (not real device values).
const char* kYaml = R"(%YAML:1.0
---
deviceID: "TEST0001"
lastTime: "20260101000000"
width: "1280"
height: "1024"
cameraMatrixL: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 1500., 0., 640., 0., 1500., 512., 0., 0., 1. ]
distCoeffL: !!opencv-matrix
   rows: 5
   cols: 1
   dt: d
   data: [ 0.06, -0.1, 0.0, 3.0e-04,
       -0.02 ]
cameraMatrixR: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 1495., 0., 645., 0., 1495., 508., 0., 0., 1. ]
distCoeffR: !!opencv-matrix
   rows: 5
   cols: 1
   dt: d
   data: [ 0.05, -0.09, 1.0e-05, 2.0e-04, -0.03 ]
steroRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 0.9393609728855362, 0.0066306273346100975, 0.3428658591937792,
       -0.003166247087303454, 0.9999381360670124, -0.010662969483740212,
       -0.3429153503401693, 0.00893077936008689, 0.9393238545257436 ]
steroTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [ -117., 0.7, 20. ]
pL: !!opencv-matrix
   rows: 3
   cols: 4
   dt: d
   data: [ 1497.5, 0., 315.8496627807617, 0., 0., 1497.5, 510.3926544189453, 0.,
       0., 0., 1., 0. ]
pR: !!opencv-matrix
   rows: 3
   cols: 4
   dt: d
   data: [ 1497.5, 0., 900.0, -178000., 0., 1497.5, 510.3926544189453, 0.,
       0., 0., 1., 0. ]
)";

} // namespace

TEST_CASE("camparam.yaml parses and rectifies like cv::stereoRectify") {
    std::string error;
    const auto rig = decoder::parseCamparamYaml(kYaml, &error);
    REQUIRE(rig.has_value());
    REQUIRE(rig->deviceId == "TEST0001");
    REQUIRE(rig->width == 1280);
    REQUIRE(std::fabs(rig->baselineMm() - std::sqrt(117.0 * 117 + 0.49 + 400)) < 1e-9);

    // Expected values computed with OpenCV 5.0 cv::stereoRectify (alpha=-1).
    const double expR1[9] = {0.9837117603041781, -0.0008659613634429915, 0.17975100208391767,
                             0.0017057393484470745, 0.9999883417723305, -0.004517385604524318,
                             -0.1797449946244143, 0.004750413702176699, 0.983701769072887};
    const double expR2[9] = {0.9856851761016341, -0.005897261737369028, -0.16849319249599257,
                             0.0066839977104885834, 0.9999692465131703, 0.004102462978345214,
                             0.16846381744486505, -0.005169945056136764, 0.9856942801295032};
    std::array<double, 9> R1{}, R2{};
    decoder::stereoRectifyRotations(rig->R, rig->T, R1, R2);
    for (int i = 0; i < 9; ++i) {
        REQUIRE(std::fabs(R1[i] - expR1[i]) < 1e-9);
        REQUIRE(std::fabs(R2[i] - expR2[i]) < 1e-9);
    }

    // cv::initUndistortRectifyMap(S·KL, DL, R1, S·P1, 320×512), S = diag(1/4, 1/2, 1).
    std::vector<float> mx, my;
    decoder::buildRectifyMap(rig->KL, rig->DL, R1, rig->PL, 0.25, 0.5, 320, 512, 0.0, mx, my);
    REQUIRE(std::fabs(mx[100 * 320 + 40] - 49.705677f) < 2e-3f);
    REQUIRE(std::fabs(my[100 * 320 + 40] - 97.8989f) < 2e-3f);
    REQUIRE(std::fabs(mx[400 * 320 + 300] - 299.26608f) < 2e-3f);
    REQUIRE(std::fabs(my[400 * 320 + 300] - 393.12256f) < 2e-3f);
}

TEST_CASE("camparam.yaml parser rejects missing matrices") {
    std::string error;
    REQUIRE_FALSE(decoder::parseCamparamYaml("%YAML:1.0\n---\nwidth: \"1280\"\n", &error));
    REQUIRE(error.find("cameraMatrixL") != std::string::npos);
}

#include <cstring>

#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "decoder/RgbCalibration.h"

// The colour calibration blobs are raw float32 with no header, so a layout
// mistake would silently mis-project every texture sample. Pin the exact
// bytes this POP4 returns (device D26545575B6S00E1B).
TEST_CASE("RGB calibration parses the scanner's blob layout", "[rgb]") {
    auto f32 = [](std::vector<float> v) {
        std::vector<uint8_t> out(v.size() * 4);
        std::memcpy(out.data(), v.data(), out.size());
        return out;
    };
    // Prgb.bin leads with one pad float, then the 3x3 matrix row-major.
    const auto prgb = f32({0.0f, 1557.54749f, 0.0f, 633.88849f, 0.0f, 1557.54749f, 518.03156f,
                           0.0f, 0.0f, 1.0f});
    const auto distort = f32({0.12374f, -0.60751f, -0.00030f, -0.00024f, 1.09100f});
    const auto lcRt = f32({0.99518f, 0.00278f, -0.09806f, -0.00258f, 0.99999f, 0.00210f,
                           0.09807f, -0.00184f, 0.99518f, -24.03342f, 0.02720f, -2.40354f});

    std::string error;
    const auto cal = revoscan::decoder::parseRgbCalibration(prgb, distort, lcRt, &error);
    REQUIRE(cal.has_value());
    REQUIRE_THAT(cal->fx(), Catch::Matchers::WithinAbs(1557.547, 1e-2));
    REQUIRE_THAT(cal->fy(), Catch::Matchers::WithinAbs(1557.547, 1e-2));
    // Principal point near the centre of the 1280x1024 colour sensor.
    REQUIRE_THAT(cal->cx(), Catch::Matchers::WithinAbs(633.888, 1e-2));
    REQUIRE_THAT(cal->cy(), Catch::Matchers::WithinAbs(518.032, 1e-2));
    REQUIRE_THAT(cal->T[0], Catch::Matchers::WithinAbs(-24.033, 1e-2));

    // A point 400 mm straight ahead of the depth camera must land inside the
    // colour image, and left of centre: the colour camera sits 24 mm away.
    double u = 0.0, v = 0.0;
    REQUIRE(revoscan::decoder::projectToRgb(*cal, 0.0, 0.0, 400.0, &u, &v));
    REQUIRE(u > 0.0);
    REQUIRE(u < 1280.0);
    REQUIRE(v > 0.0);
    REQUIRE(v < 1024.0);
    REQUIRE(u < cal->cx()); // shifted by the baseline

    // Parallax must shrink as things get further away.
    double uNear = 0.0, uFar = 0.0, tmp = 0.0;
    revoscan::decoder::projectToRgb(*cal, 0.0, 0.0, 250.0, &uNear, &tmp);
    revoscan::decoder::projectToRgb(*cal, 0.0, 0.0, 2000.0, &uFar, &tmp);
    REQUIRE(uNear < uFar);

    // Behind the camera is rejected, and malformed blobs never parse.
    REQUIRE_FALSE(revoscan::decoder::projectToRgb(*cal, 0.0, 0.0, -10.0, &u, &v));
    REQUIRE_FALSE(revoscan::decoder::parseRgbCalibration({}, distort, lcRt, &error).has_value());
    REQUIRE_FALSE(revoscan::decoder::parseRgbCalibration(prgb, distort, prgb, &error).has_value());
}
