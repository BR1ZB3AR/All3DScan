#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace revoscan::decoder {

// Factory colour-camera calibration, read from the scanner's own files over
// the XU file channel (all little-endian float32):
//
//   /data/camparam/Prgb.bin     40 B = pad + 3x3 colour camera matrix
//   /data/camparam/Distort.bin  20 B = k1 k2 p1 p2 k3 for that camera
//   /data/camparam/LC_RT.bin    48 B = 3x3 rotation + 3 translation (mm)
//
// The rotation/translation map the *rectified left depth* frame to the colour
// camera. That is the frame our point cloud is already in: the vendor stores
// Pl.bin beside these, and it is byte-identical to camparam.yaml's `pL`, the
// rectified left projection we back-project with.
struct RgbCalibration {
    std::array<double, 9> K{};  // colour camera matrix, row-major
    std::array<double, 5> D{};  // k1 k2 p1 p2 k3
    std::array<double, 9> R{};  // rectified left -> colour rotation
    std::array<double, 3> T{};  // rectified left -> colour translation, mm
    int width = 1280;
    int height = 1024;

    double fx() const { return K[0]; }
    double fy() const { return K[4]; }
    double cx() const { return K[2]; }
    double cy() const { return K[5]; }
};

// Parses the three blobs. Returns nullopt (and sets error) when a size is
// wrong or the values are not a plausible camera.
std::optional<RgbCalibration> parseRgbCalibration(const std::vector<uint8_t>& prgb,
                                                  const std::vector<uint8_t>& distort,
                                                  const std::vector<uint8_t>& lcRt,
                                                  std::string* error = nullptr);

// Projects a point given in the rectified left depth frame (millimetres) into
// colour-image pixels. Returns false when the point is behind the camera.
bool projectToRgb(const RgbCalibration& cal, double xMm, double yMm, double zMm, double* u,
                  double* v);

} // namespace revoscan::decoder
