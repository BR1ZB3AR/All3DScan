#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace revoscan::decoder {

// Factory stereo calibration stored on the scanner at
// /data/camparam/camparam.yaml (OpenCV FileStorage YAML, read over the XU file
// channel). Values are for the sensors' native 1280×1024 grid; "L" is the
// camera delivered as packed images 0–4, "R" as images 5–9.
struct StereoRig {
    std::string deviceId;
    int width = 1280;
    int height = 1024;
    std::array<double, 9> KL{}, KR{};      // camera matrices (row-major)
    std::array<double, 5> DL{}, DR{};      // k1 k2 p1 p2 k3
    std::array<double, 9> R{};             // stereo rotation (L -> R)
    std::array<double, 3> T{};             // stereo translation, mm
    std::array<double, 12> PL{}, PR{};     // rectified projections (stereoRectify)

    double baselineMm() const;
};

// Parses the subset of OpenCV YAML used by camparam.yaml. Returns nullopt
// (and sets error) if any required matrix is missing or malformed.
std::optional<StereoRig> parseCamparamYaml(const std::string& text, std::string* error = nullptr);

// Reads and parses a camparam.yaml file.
std::optional<StereoRig> loadCamparamFile(const std::string& path, std::string* error = nullptr);

// Bouguet rectification rotations, identical to cv::stereoRectify's R1/R2.
void stereoRectifyRotations(const std::array<double, 9>& R, const std::array<double, 3>& T,
                            std::array<double, 9>& R1, std::array<double, 9>& R2);

// Per-pixel source coordinates (cv::initUndistortRectifyMap) for an output
// grid of outW×outH that is the native grid scaled by (sx, sy) — the POP4
// delivers 320×512 images, i.e. sx = 1/4, sy = 1/2. `P` is the rectified
// projection (3×4); `cxShift` moves the output principal point left by that
// many output pixels (used to keep disparities positive).
void buildRectifyMap(const std::array<double, 9>& K, const std::array<double, 5>& D,
                     const std::array<double, 9>& Rrect, const std::array<double, 12>& P,
                     double sx, double sy, int outW, int outH, double cxShift,
                     std::vector<float>& mapX, std::vector<float>& mapY);

} // namespace revoscan::decoder
