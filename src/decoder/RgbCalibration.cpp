#include "decoder/RgbCalibration.h"

#include <cmath>
#include <cstring>

namespace revoscan::decoder {
namespace {

// The blobs are little-endian float32. Reading them byte-wise keeps the
// parser independent of the host's alignment and float representation
// assumptions; every scanner file seen so far is IEEE-754 little-endian.
float readF32(const uint8_t* p) {
    uint32_t bits = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                    (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
    float out = 0.0f;
    std::memcpy(&out, &bits, sizeof(out));
    return out;
}

bool finite(double v) {
    return std::isfinite(v);
}

} // namespace

std::optional<RgbCalibration> parseRgbCalibration(const std::vector<uint8_t>& prgb,
                                                  const std::vector<uint8_t>& distort,
                                                  const std::vector<uint8_t>& lcRt,
                                                  std::string* error) {
    auto fail = [&](const char* why) -> std::optional<RgbCalibration> {
        if (error) {
            *error = why;
        }
        return std::nullopt;
    };
    // Prgb.bin leads with one padding float before the 3x3 matrix; Pl.bin,
    // which is byte-identical to camparam.yaml's pL, has the same shape.
    if (prgb.size() != 40) {
        return fail("Prgb.bin: expected 40 bytes");
    }
    if (distort.size() != 20) {
        return fail("Distort.bin: expected 20 bytes");
    }
    if (lcRt.size() != 48) {
        return fail("LC_RT.bin: expected 48 bytes");
    }

    RgbCalibration cal;
    for (int i = 0; i < 9; ++i) {
        cal.K[static_cast<size_t>(i)] = readF32(prgb.data() + 4 * (i + 1));
    }
    for (int i = 0; i < 5; ++i) {
        cal.D[static_cast<size_t>(i)] = readF32(distort.data() + 4 * i);
    }
    for (int i = 0; i < 9; ++i) {
        cal.R[static_cast<size_t>(i)] = readF32(lcRt.data() + 4 * i);
    }
    for (int i = 0; i < 3; ++i) {
        cal.T[static_cast<size_t>(i)] = readF32(lcRt.data() + 4 * (9 + i));
    }

    for (double v : cal.K) {
        if (!finite(v)) {
            return fail("Prgb.bin: non-finite value");
        }
    }
    for (double v : cal.R) {
        if (!finite(v)) {
            return fail("LC_RT.bin: non-finite value");
        }
    }
    // A plausible colour camera for this sensor: focal lengths of the right
    // order and a principal point inside the image.
    if (cal.fx() < 100.0 || cal.fx() > 20000.0 || cal.fy() < 100.0 || cal.fy() > 20000.0) {
        return fail("Prgb.bin: implausible focal length");
    }
    if (cal.cx() < 0.0 || cal.cx() > cal.width || cal.cy() < 0.0 || cal.cy() > cal.height) {
        return fail("Prgb.bin: principal point outside the image");
    }
    // The depth and colour cameras sit side by side, tens of millimetres
    // apart, so a sane rig has a small baseline and a near-identity rotation.
    const double baseline =
        std::sqrt(cal.T[0] * cal.T[0] + cal.T[1] * cal.T[1] + cal.T[2] * cal.T[2]);
    if (!finite(baseline) || baseline < 1.0 || baseline > 500.0) {
        return fail("LC_RT.bin: implausible baseline");
    }
    return cal;
}

bool projectToRgb(const RgbCalibration& cal, double xMm, double yMm, double zMm, double* u,
                  double* v) {
    const double X = cal.R[0] * xMm + cal.R[1] * yMm + cal.R[2] * zMm + cal.T[0];
    const double Y = cal.R[3] * xMm + cal.R[4] * yMm + cal.R[5] * zMm + cal.T[1];
    const double Z = cal.R[6] * xMm + cal.R[7] * yMm + cal.R[8] * zMm + cal.T[2];
    if (!(Z > 1e-6)) {
        return false; // behind, or on, the colour camera
    }
    const double x = X / Z;
    const double y = Y / Z;
    const double r2 = x * x + y * y;
    const double radial = 1.0 + cal.D[0] * r2 + cal.D[1] * r2 * r2 + cal.D[4] * r2 * r2 * r2;
    const double xd = x * radial + 2.0 * cal.D[2] * x * y + cal.D[3] * (r2 + 2.0 * x * x);
    const double yd = y * radial + cal.D[2] * (r2 + 2.0 * y * y) + 2.0 * cal.D[3] * x * y;
    *u = cal.fx() * xd + cal.cx();
    *v = cal.fy() * yd + cal.cy();
    return std::isfinite(*u) && std::isfinite(*v);
}

} // namespace revoscan::decoder
