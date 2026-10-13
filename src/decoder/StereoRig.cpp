#include "decoder/StereoRig.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace revoscan::decoder {
namespace {

using Mat3 = std::array<double, 9>;

Mat3 mul(const Mat3& a, const Mat3& b) {
    Mat3 c{};
    for (int r = 0; r < 3; ++r) {
        for (int k = 0; k < 3; ++k) {
            double s = 0.0;
            for (int i = 0; i < 3; ++i) {
                s += a[r * 3 + i] * b[i * 3 + k];
            }
            c[r * 3 + k] = s;
        }
    }
    return c;
}

Mat3 transpose(const Mat3& a) {
    return {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]};
}

// Rotation vector -> matrix (Rodrigues).
Mat3 rodrigues(const std::array<double, 3>& w) {
    const double th = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    if (th < 1e-15) {
        return {1, 0, 0, 0, 1, 0, 0, 0, 1};
    }
    const double kx = w[0] / th, ky = w[1] / th, kz = w[2] / th;
    const double c = std::cos(th), s = std::sin(th), C = 1.0 - c;
    return {c + kx * kx * C,      kx * ky * C - kz * s, kx * kz * C + ky * s,
            ky * kx * C + kz * s, c + ky * ky * C,      ky * kz * C - kx * s,
            kz * kx * C - ky * s, kz * ky * C + kx * s, c + kz * kz * C};
}

// Matrix -> rotation vector (inverse Rodrigues), robust near 0 and pi.
std::array<double, 3> rotationVector(const Mat3& R) {
    const double cosTh = std::fmax(-1.0, std::fmin(1.0, (R[0] + R[4] + R[8] - 1.0) * 0.5));
    const double th = std::acos(cosTh);
    if (th < 1e-12) {
        return {0, 0, 0};
    }
    const double s = 2.0 * std::sin(th);
    std::array<double, 3> axis{(R[7] - R[5]) / s, (R[2] - R[6]) / s, (R[3] - R[1]) / s};
    return {axis[0] * th, axis[1] * th, axis[2] * th};
}

// Reads "name: !!opencv-matrix ... data: [ ... ]" into `out`.
template <size_t N>
bool readMatrix(const std::string& text, const std::string& name, std::array<double, N>& out,
                std::string* error) {
    const std::string key = "\n" + name + ":";
    const size_t at = ("\n" + text).find(key);
    if (at == std::string::npos) {
        if (error) {
            *error = "camparam: missing " + name;
        }
        return false;
    }
    const size_t open = text.find('[', at);
    const size_t close = text.find(']', open);
    if (open == std::string::npos || close == std::string::npos) {
        if (error) {
            *error = "camparam: malformed " + name;
        }
        return false;
    }
    std::string body = text.substr(open + 1, close - open - 1);
    for (char& c : body) {
        if (c == ',' || c == '\n' || c == '\r') {
            c = ' ';
        }
    }
    std::istringstream in(body);
    size_t n = 0;
    std::string tok;
    while (in >> tok) {
        char* end = nullptr;
        const double v = std::strtod(tok.c_str(), &end);
        if (end == tok.c_str() || n >= N) {
            if (error) {
                *error = "camparam: bad values in " + name;
            }
            return false;
        }
        out[n++] = v;
    }
    if (n != N) {
        if (error) {
            *error = "camparam: " + name + " has " + std::to_string(n) + " values, expected " +
                     std::to_string(N);
        }
        return false;
    }
    return true;
}

std::string readScalar(const std::string& text, const std::string& name) {
    const std::string key = "\n" + name + ":";
    const size_t at = ("\n" + text).find(key);
    if (at == std::string::npos) {
        return {};
    }
    const size_t lineEnd = text.find('\n', at);
    std::string v = text.substr(at + name.size() + 1, lineEnd - at - name.size() - 1);
    const size_t a = v.find_first_not_of(" \t\"");
    const size_t b = v.find_last_not_of(" \t\"\r");
    return a == std::string::npos ? std::string{} : v.substr(a, b - a + 1);
}

} // namespace

double StereoRig::baselineMm() const {
    return std::sqrt(T[0] * T[0] + T[1] * T[1] + T[2] * T[2]);
}

std::optional<StereoRig> parseCamparamYaml(const std::string& text, std::string* error) {
    StereoRig rig;
    rig.deviceId = readScalar(text, "deviceID");
    if (const std::string w = readScalar(text, "width"); !w.empty()) {
        rig.width = std::atoi(w.c_str());
    }
    if (const std::string h = readScalar(text, "height"); !h.empty()) {
        rig.height = std::atoi(h.c_str());
    }
    // The vendor spells it "stero".
    if (!readMatrix(text, "cameraMatrixL", rig.KL, error) ||
        !readMatrix(text, "distCoeffL", rig.DL, error) ||
        !readMatrix(text, "cameraMatrixR", rig.KR, error) ||
        !readMatrix(text, "distCoeffR", rig.DR, error) ||
        !readMatrix(text, "steroRotation", rig.R, error) ||
        !readMatrix(text, "steroTranslation", rig.T, error) ||
        !readMatrix(text, "pL", rig.PL, error) || !readMatrix(text, "pR", rig.PR, error)) {
        return std::nullopt;
    }
    if (rig.width <= 0 || rig.height <= 0 || rig.KL[0] <= 0 || rig.KR[0] <= 0 ||
        rig.PL[0] <= 0 || rig.baselineMm() < 1.0) {
        if (error) {
            *error = "camparam: implausible values";
        }
        return std::nullopt;
    }
    return rig;
}

std::optional<StereoRig> loadCamparamFile(const std::string& path, std::string* error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        if (error) {
            *error = "cannot open " + path;
        }
        return std::nullopt;
    }
    std::stringstream ss;
    ss << in.rdbuf();
    return parseCamparamYaml(ss.str(), error);
}

void stereoRectifyRotations(const std::array<double, 9>& R, const std::array<double, 3>& T,
                            std::array<double, 9>& R1, std::array<double, 9>& R2) {
    // Same steps as OpenCV's cvStereoRectify: rotate each camera half-way,
    // then rotate both so the baseline lies along x.
    std::array<double, 3> om = rotationVector(R);
    for (double& v : om) {
        v *= -0.5;
    }
    const Mat3 rr = rodrigues(om);
    const std::array<double, 3> t{rr[0] * T[0] + rr[1] * T[1] + rr[2] * T[2],
                                  rr[3] * T[0] + rr[4] * T[1] + rr[5] * T[2],
                                  rr[6] * T[0] + rr[7] * T[1] + rr[8] * T[2]};
    const int idx = std::fabs(t[0]) > std::fabs(t[1]) ? 0 : 1;
    const double c = t[idx];
    const double nt = std::sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
    std::array<double, 3> uu{0, 0, 0};
    uu[idx] = c > 0 ? 1 : -1;
    std::array<double, 3> ww{t[1] * uu[2] - t[2] * uu[1], t[2] * uu[0] - t[0] * uu[2],
                             t[0] * uu[1] - t[1] * uu[0]};
    const double nw = std::sqrt(ww[0] * ww[0] + ww[1] * ww[1] + ww[2] * ww[2]);
    if (nw > 0.0) {
        const double scale = std::acos(std::fabs(c) / nt) / nw;
        for (double& v : ww) {
            v *= scale;
        }
    }
    const Mat3 wR = rodrigues(ww);
    R1 = mul(wR, transpose(rr));
    R2 = mul(wR, rr);
}

void buildRectifyMap(const std::array<double, 9>& K, const std::array<double, 5>& D,
                     const std::array<double, 9>& Rrect, const std::array<double, 12>& P,
                     double sx, double sy, int outW, int outH, double cxShift,
                     std::vector<float>& mapX, std::vector<float>& mapY) {
    // Scaled camera and rectified projection (cv::initUndistortRectifyMap with
    // K' = S·K, P' = S·P, S = diag(sx, sy, 1)).
    const double fx = K[0] * sx, fy = K[4] * sy, cx = K[2] * sx, cy = K[5] * sy;
    const double pfx = P[0] * sx, pfy = P[5] * sy, pcx = P[2] * sx - cxShift, pcy = P[6] * sy;
    const double k1 = D[0], k2 = D[1], p1 = D[2], p2 = D[3], k3 = D[4];
    const Mat3 Ri = transpose(Rrect); // rectified ray -> camera ray
    mapX.resize(static_cast<size_t>(outW) * outH);
    mapY.resize(mapX.size());
    for (int v = 0; v < outH; ++v) {
        for (int u = 0; u < outW; ++u) {
            const double xr = (u - pcx) / pfx;
            const double yr = (v - pcy) / pfy;
            const double X = Ri[0] * xr + Ri[1] * yr + Ri[2];
            const double Y = Ri[3] * xr + Ri[4] * yr + Ri[5];
            const double W = Ri[6] * xr + Ri[7] * yr + Ri[8];
            const double x = X / W, y = Y / W;
            const double r2 = x * x + y * y;
            const double radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
            const double xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
            const double yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
            const size_t i = static_cast<size_t>(v) * outW + u;
            mapX[i] = static_cast<float>(fx * xd + cx);
            mapY[i] = static_cast<float>(fy * yd + cy);
        }
    }
}

} // namespace revoscan::decoder
