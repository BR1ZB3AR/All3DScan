#include "Pop4DepthDecoder.h"

#include "Pop4StereoCuda.h"

#include <algorithm>
#include <chrono>
#include <optional>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <vector>

namespace revoscan::decoder {
namespace {

// Packed 640×2560 Y16 is really ten 320×512 images back to back (row stride
// 320). Verified 2026-10 on a live POP4 by autocorrelation (row period 320)
// and by rendering: images 0–4 = right camera, 5–9 = left camera; images 4
// and 9 carry the projected speckle. Images 0–3 / 5–8 carry four-step fringes
// once streaming has run ~1 s, but in a different geometry from the speckle
// images and not yet usable (RE_NOTES 2026-10-13). Depth is speckle-only.
constexpr uint32_t kPackedWidth = 640;
constexpr uint32_t kPackedHeight = 2560;
constexpr uint32_t kW = 320;
constexpr uint32_t kH = 512;
constexpr size_t kImagePixels = static_cast<size_t>(kW) * kH;
constexpr int kRightSpeckle = 4;
constexpr int kLeftSpeckle = 9;
constexpr int kRightAmbientFirst = 0;
constexpr int kLeftAmbientFirst = 5;

constexpr uint32_t kMinValidDepth = 200;
// Fallback disparity search band, used only if calibration can't derive one.
constexpr int kFallbackDispMin = 8;
constexpr int kFallbackDispMax = 80;
constexpr int kDispSearchPad = 6;
constexpr int kDispSearchFloor = 1;
constexpr int kDispSearchCeil = 160;

// 7×7 window → 48 bits. 9×7 (62 bits) was tried: rougher planes (0.297 →
// 0.315 px RMS) and lower coverage on real frames at the same P1/P2.
constexpr int kCensusRadiusX = 3;
constexpr int kCensusRadiusY = 3;
constexpr uint8_t kCensusInvalidCost = 48; // no right pixel at this disparity

bool envFlag(const char* name, bool defaultValue) {
    if (const char* value = std::getenv(name)) {
        if (value[0] == '0' || value[0] == 'n' || value[0] == 'N' ||
            value[0] == 'f' || value[0] == 'F') {
            return false;
        }
        if (value[0] == '1' || value[0] == 'y' || value[0] == 'Y' ||
            value[0] == 't' || value[0] == 'T') {
            return true;
        }
    }
    return defaultValue;
}

float envFloat(const char* name, float defaultValue, float lo, float hi) {
    if (const char* value = std::getenv(name)) {
        char* end = nullptr;
        const float parsed = std::strtof(value, &end);
        if (end != value && std::isfinite(parsed) && parsed >= lo && parsed <= hi) {
            return parsed;
        }
    }
    return defaultValue;
}

// SGM / filtering knobs; every one is overridable from the environment so
// live tuning needs no rebuild.
struct MatchTuning {
    int p1 = 16;                // small disparity change penalty (census units)
    int p2 = 120;               // large disparity jump penalty
    float uniqueness = 0.05f;   // best*(1+u) must beat the best rival outside ±1
    float lrMaxDiff = 1.0f;     // left/right consistency tolerance (px)
    float textureMin = 1.5f;    // min local std (0..255 scale) of the left speckle
    int speckleMinRegion = 80;  // drop connected disparity blobs smaller than this
    float speckleMaxDiff = 1.0f;
    bool doMedian = true;
    bool equiangular = false;   // sub-pixel fit: equiangular (V) vs parabola
    int edgeRadius = 2;         // flying-pixel filter window (0 disables)
    float edgeMaxJump = 1.5f;   // disparity jump (px) that marks a depth edge
    bool debug = false;
};

MatchTuning readMatchTuning() {
    MatchTuning t;
    t.p1 = static_cast<int>(envFloat("ALL3DSCAN_POP4_P1", static_cast<float>(t.p1), 0, 200));
    t.p2 = static_cast<int>(envFloat("ALL3DSCAN_POP4_P2", static_cast<float>(t.p2), 0, 400));
    t.p2 = std::max(t.p2, t.p1);
    t.uniqueness = envFloat("ALL3DSCAN_POP4_UNIQ", t.uniqueness, 0.0f, 1.0f);
    t.lrMaxDiff = envFloat("ALL3DSCAN_POP4_LRDIFF", t.lrMaxDiff, 0.0f, 8.0f);
    t.textureMin = envFloat("ALL3DSCAN_POP4_TEXTURE", t.textureMin, 0.0f, 64.0f);
    t.speckleMinRegion = static_cast<int>(envFloat(
        "ALL3DSCAN_POP4_SPECKLE_REGION", static_cast<float>(t.speckleMinRegion), 0, 5000));
    t.speckleMaxDiff = envFloat("ALL3DSCAN_POP4_SPECKLE_DIFF", t.speckleMaxDiff, 0.1f, 16.0f);
    t.doMedian = envFlag("ALL3DSCAN_POP4_MEDIAN", t.doMedian);
    t.equiangular = envFlag("ALL3DSCAN_POP4_SUBPIX_EQUI", t.equiangular);
    t.edgeRadius = static_cast<int>(
        envFloat("ALL3DSCAN_POP4_EDGE_RADIUS", static_cast<float>(t.edgeRadius), 0, 8));
    t.edgeMaxJump = envFloat("ALL3DSCAN_POP4_EDGE_JUMP", t.edgeMaxJump, 0.2f, 20.0f);
    t.debug = envFlag("ALL3DSCAN_POP4_DEBUG", false);
    return t;
}

bool swapCameras() {
    return envFlag("ALL3DSCAN_POP4_SWAP_LR", false);
}

const uint16_t* imageAt(const protocol::RawFrame& raw, int index) {
    return raw.depthMm.data() + static_cast<size_t>(index) * kImagePixels;
}

// Y16 samples are an 8-bit value duplicated into both bytes; /257 → 0..255.
void toFloat(const uint16_t* src, std::vector<float>& out) {
    out.resize(kImagePixels);
    for (size_t i = 0; i < kImagePixels; ++i) {
        out[i] = static_cast<float>(src[i]) / 257.0f;
    }
}

// Separable [1 4 6 4 1]/16 (≈ Gaussian σ=1) with clamped borders. Speckle dots
// are 1–2 px; this keeps census bits stable across the two viewpoints.
void blur5(const std::vector<float>& in, std::vector<float>& out) {
    static constexpr float k[5] = {1 / 16.f, 4 / 16.f, 6 / 16.f, 4 / 16.f, 1 / 16.f};
    std::vector<float> tmp(kImagePixels);
    for (uint32_t y = 0; y < kH; ++y) {
        const float* row = in.data() + static_cast<size_t>(y) * kW;
        for (uint32_t x = 0; x < kW; ++x) {
            float s = 0.0f;
            for (int i = -2; i <= 2; ++i) {
                const int xx = std::clamp(static_cast<int>(x) + i, 0, static_cast<int>(kW) - 1);
                s += k[i + 2] * row[xx];
            }
            tmp[static_cast<size_t>(y) * kW + x] = s;
        }
    }
    out.resize(kImagePixels);
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            float s = 0.0f;
            for (int i = -2; i <= 2; ++i) {
                const int yy = std::clamp(static_cast<int>(y) + i, 0, static_cast<int>(kH) - 1);
                s += k[i + 2] * tmp[static_cast<size_t>(yy) * kW + x];
            }
            out[static_cast<size_t>(y) * kW + x] = s;
        }
    }
}

void censusTransform(const std::vector<float>& img, std::vector<uint64_t>& out) {
    out.assign(kImagePixels, 0);
#pragma omp parallel for schedule(static)
    for (int y = 0; y < static_cast<int>(kH); ++y) {
        for (int x = 0; x < static_cast<int>(kW); ++x) {
            const float c = img[static_cast<size_t>(y) * kW + x];
            uint64_t bits = 0;
            for (int dy = -kCensusRadiusY; dy <= kCensusRadiusY; ++dy) {
                const int yy = std::clamp(y + dy, 0, static_cast<int>(kH) - 1);
                const float* row = img.data() + static_cast<size_t>(yy) * kW;
                for (int dx = -kCensusRadiusX; dx <= kCensusRadiusX; ++dx) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    const int xx = std::clamp(x + dx, 0, static_cast<int>(kW) - 1);
                    bits = (bits << 1) | (row[xx] < c ? 1u : 0u);
                }
            }
            out[static_cast<size_t>(y) * kW + x] = bits;
        }
    }
}

// Local standard deviation over a (2r+1)² window via integral images.
void localStd(const std::vector<float>& img, int r, std::vector<float>& out) {
    const size_t stride = kW + 1;
    std::vector<double> s1((kH + 1) * stride, 0.0);
    std::vector<double> s2((kH + 1) * stride, 0.0);
    for (uint32_t y = 0; y < kH; ++y) {
        double r1 = 0.0;
        double r2 = 0.0;
        for (uint32_t x = 0; x < kW; ++x) {
            const double v = img[static_cast<size_t>(y) * kW + x];
            r1 += v;
            r2 += v * v;
            s1[(y + 1) * stride + x + 1] = s1[y * stride + x + 1] + r1;
            s2[(y + 1) * stride + x + 1] = s2[y * stride + x + 1] + r2;
        }
    }
    out.resize(kImagePixels);
    for (int y = 0; y < static_cast<int>(kH); ++y) {
        const int y0 = std::max(0, y - r);
        const int y1 = std::min(static_cast<int>(kH), y + r + 1);
        for (int x = 0; x < static_cast<int>(kW); ++x) {
            const int x0 = std::max(0, x - r);
            const int x1 = std::min(static_cast<int>(kW), x + r + 1);
            const double n = static_cast<double>((y1 - y0) * (x1 - x0));
            auto box = [&](const std::vector<double>& s) {
                return s[y1 * stride + x1] - s[y0 * stride + x1] - s[y1 * stride + x0] +
                       s[y0 * stride + x0];
            };
            const double mean = box(s1) / n;
            const double var = std::max(0.0, box(s2) / n - mean * mean);
            out[static_cast<size_t>(y) * kW + x] = static_cast<float>(std::sqrt(var));
        }
    }
}

// Sub-pixel offset of the cost minimum from its two neighbours. Census
// (Hamming) costs are V-shaped, so the equiangular fit avoids the "pixel
// locking" a parabola shows on L1-like costs; the parabola stays available.
float subpixelOffset(float cp, float cb, float cn, bool equiangular) {
    if (equiangular) {
        const float rise = std::max(cp, cn) - cb;
        return rise > 1e-6f ? std::clamp(0.5f * (cp - cn) / rise, -0.5f, 0.5f) : 0.0f;
    }
    const float den = cp - 2.0f * cb + cn;
    return den > 1e-6f ? std::clamp(0.5f * (cp - cn) / den, -0.5f, 0.5f) : 0.0f;
}

// One SGM scan direction, accumulated into `sum`. `start` is the first pixel
// of the line, `step` the pixel stride along it.
void aggregatePath(const std::vector<uint8_t>& cost,
                   int numDisp,
                   size_t start,
                   std::ptrdiff_t step,
                   uint32_t length,
                   int p1,
                   int p2,
                   std::vector<uint16_t>& sum,
                   std::vector<uint16_t>& prev,
                   std::vector<uint16_t>& cur) {
    size_t p = start;
    const uint8_t* c = cost.data() + p * numDisp;
    uint16_t minPrev = std::numeric_limits<uint16_t>::max();
    for (int d = 0; d < numDisp; ++d) {
        prev[d] = c[d];
        sum[p * numDisp + d] += prev[d];
        minPrev = std::min(minPrev, prev[d]);
    }
    for (uint32_t k = 1; k < length; ++k) {
        p = static_cast<size_t>(static_cast<std::ptrdiff_t>(p) + step);
        c = cost.data() + p * numDisp;
        uint16_t minCur = std::numeric_limits<uint16_t>::max();
        const int jump = minPrev + p2;
        for (int d = 0; d < numDisp; ++d) {
            int v = prev[d];
            if (d > 0) {
                v = std::min(v, prev[d - 1] + p1);
            }
            if (d + 1 < numDisp) {
                v = std::min(v, prev[d + 1] + p1);
            }
            v = std::min(v, jump);
            const uint16_t l = static_cast<uint16_t>(c[d] + v - minPrev);
            cur[d] = l;
            minCur = std::min(minCur, l);
            sum[p * numDisp + d] += l;
        }
        prev.swap(cur);
        minPrev = minCur;
    }
}

void medianFilterDisparity(const std::vector<float>& in, std::vector<float>& out) {
    out.assign(in.size(), -1.0f);
    for (int y = 0; y < static_cast<int>(kH); ++y) {
        for (int x = 0; x < static_cast<int>(kW); ++x) {
            const size_t ci = static_cast<size_t>(y) * kW + x;
            if (in[ci] < 0.0f) {
                continue;
            }
            float window[9];
            int n = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                const int yy = y + dy;
                if (yy < 0 || yy >= static_cast<int>(kH)) {
                    continue;
                }
                for (int dx = -1; dx <= 1; ++dx) {
                    const int xx = x + dx;
                    if (xx < 0 || xx >= static_cast<int>(kW)) {
                        continue;
                    }
                    const float v = in[static_cast<size_t>(yy) * kW + xx];
                    if (v >= 0.0f) {
                        window[n++] = v;
                    }
                }
            }
            std::sort(window, window + n);
            out[ci] = window[n / 2];
        }
    }
}

// Drops "flying pixels": SGM smears depth across object outlines, which
// back-projects to streaks along the viewing rays. A pixel is removed when any
// valid pixel within `radius` differs by more than `maxJump` px of disparity.
void filterDepthEdges(std::vector<float>& disp, int radius, float maxJump) {
    if (radius <= 0) {
        return;
    }
    const std::vector<float> in = disp;
    for (int y = 0; y < static_cast<int>(kH); ++y) {
        for (int x = 0; x < static_cast<int>(kW); ++x) {
            const size_t p = static_cast<size_t>(y) * kW + x;
            const float c = in[p];
            if (c < 0.0f) {
                continue;
            }
            bool edge = false;
            for (int dy = -radius; dy <= radius && !edge; ++dy) {
                const int yy = y + dy;
                if (yy < 0 || yy >= static_cast<int>(kH)) {
                    continue;
                }
                for (int dx = -radius; dx <= radius; ++dx) {
                    const int xx = x + dx;
                    if (xx < 0 || xx >= static_cast<int>(kW)) {
                        continue;
                    }
                    const float v = in[static_cast<size_t>(yy) * kW + xx];
                    if (v >= 0.0f && std::fabs(v - c) > maxJump) {
                        edge = true;
                        break;
                    }
                }
            }
            if (edge) {
                disp[p] = -1.0f;
            }
        }
    }
}

// OpenCV filterSpeckles semantics: 4-connected regions whose neighbouring
// disparities differ by <= maxDiff form a blob; blobs under minRegion pixels
// are mismatches and are invalidated.
void filterSpeckles(std::vector<float>& disp, int minRegion, float maxDiff) {
    if (minRegion <= 0) {
        return;
    }
    std::vector<int> label(disp.size(), 0);
    std::vector<uint32_t> stack;
    std::vector<uint32_t> region;
    int next = 0;
    for (uint32_t i = 0; i < disp.size(); ++i) {
        if (disp[i] < 0.0f || label[i] != 0) {
            continue;
        }
        ++next;
        region.clear();
        stack.assign(1, i);
        label[i] = next;
        while (!stack.empty()) {
            const uint32_t p = stack.back();
            stack.pop_back();
            region.push_back(p);
            const uint32_t x = p % kW;
            const uint32_t y = p / kW;
            const uint32_t nbr[4] = {x > 0 ? p - 1 : p, x + 1 < kW ? p + 1 : p,
                                     y > 0 ? p - kW : p, y + 1 < kH ? p + kW : p};
            for (uint32_t q : nbr) {
                if (q != p && label[q] == 0 && disp[q] >= 0.0f &&
                    std::fabs(disp[q] - disp[p]) <= maxDiff) {
                    label[q] = next;
                    stack.push_back(q);
                }
            }
        }
        if (static_cast<int>(region.size()) < minRegion) {
            for (uint32_t p : region) {
                disp[p] = -1.0f;
            }
        }
    }
}

float depthFromDisparity(float disparity, const protocol::ScannerCalibration& cal,
                         float depthMinMm, float depthMaxMm) {
    const double q23 = cal.qMatrix[2 * 4 + 3];
    const double q32 = cal.qMatrix[3 * 4 + 2];
    const double q33 = cal.qMatrix[3 * 4 + 3];
    const double denom = disparity * q32 + q33;
    if (std::abs(denom) < 1e-9) {
        return 0.0f;
    }
    const double z = q23 / denom * cal.depthScale;
    if (!std::isfinite(z) || z < depthMinMm || z > depthMaxMm) {
        return 0.0f;
    }
    return static_cast<float>(z);
}

// Inverts the Q-matrix depth formula to find the disparity band covering
// [depthMinMm, depthMaxMm] so the search only looks where the working-distance
// preset says the object is.
void computeDispRange(const protocol::ScannerCalibration& cal, float depthMinMm,
                      float depthMaxMm, int& dispMinOut, int& dispMaxOut) {
    const double q23 = cal.qMatrix[2 * 4 + 3];
    const double q32 = cal.qMatrix[3 * 4 + 2];
    const double q33 = cal.qMatrix[3 * 4 + 3];
    auto dispForDepth = [&](double zMm) -> std::optional<double> {
        if (std::abs(q32) < 1e-12 || zMm <= 0.0) {
            return std::nullopt;
        }
        const double d = (q23 / zMm - q33) / q32;
        return std::isfinite(d) ? std::optional<double>(d) : std::nullopt;
    };
    const auto dNear = dispForDepth(static_cast<double>(depthMinMm));
    const auto dFar = dispForDepth(static_cast<double>(depthMaxMm));
    if (!dNear || !dFar) {
        dispMinOut = kFallbackDispMin;
        dispMaxOut = kFallbackDispMax;
        return;
    }
    const double lo = std::max<double>(std::min(*dNear, *dFar) - kDispSearchPad,
                                       kDispSearchFloor);
    const double hi = std::min<double>(std::max(*dNear, *dFar) + kDispSearchPad,
                                       kDispSearchCeil);
    if (hi <= lo + 2) {
        dispMinOut = kFallbackDispMin;
        dispMaxOut = kFallbackDispMax;
        return;
    }
    dispMinOut = static_cast<int>(std::floor(lo));
    dispMaxOut = static_cast<int>(std::ceil(hi));
}

// Grayscale IR texture for the left view: the mean of its ambient exposures,
// auto-gained so the 99th percentile lands near white.
void ambientTexture(const protocol::RawFrame& raw, int firstAmbient, std::vector<uint8_t>& rgb) {
    std::vector<float> mean(kImagePixels, 0.0f);
    for (int i = 0; i < 4; ++i) {
        const uint16_t* src = imageAt(raw, firstAmbient + i);
        for (size_t p = 0; p < kImagePixels; ++p) {
            mean[p] += static_cast<float>(src[p]) / (257.0f * 4.0f);
        }
    }
    std::vector<float> sorted = mean;
    const size_t k = sorted.size() * 99 / 100;
    std::nth_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(k), sorted.end());
    const float gain = 230.0f / std::max(sorted[k], 1.0f);
    rgb.resize(kImagePixels * 3);
    for (size_t p = 0; p < kImagePixels; ++p) {
        const auto v = static_cast<uint8_t>(std::clamp(mean[p] * gain, 0.0f, 255.0f));
        rgb[p * 3 + 0] = v;
        rgb[p * 3 + 1] = v;
        rgb[p * 3 + 2] = v;
    }
}

// Steps 1-4 on the CPU: row-align, census, 4-path SGM, winner-take-all with
// uniqueness / sub-pixel / left-right / texture gates. Returns raw disparity
// (<0 = invalid) before median and speckle clean-up. Pop4StereoCuda.cu mirrors
// this exactly so both backends feed the same post-processing.
std::vector<float> matchCpu(const uint16_t* leftRaw,
                            const uint16_t* rightRaw,
                            const std::vector<float>& rectMapY,
                            const MatchTuning& t,
                            int dispMin,
                            int numDisp) {
    // 1. Left speckle, and right speckle resampled onto left rows.
    std::vector<float> rawL;
    std::vector<float> rawR;
    toFloat(leftRaw, rawL);
    toFloat(rightRaw, rawR);
    std::vector<float> rectR(kImagePixels, 0.0f);
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            const float sy = rectMapY[static_cast<size_t>(y) * kW + x];
            if (sy < 0.0f || sy > static_cast<float>(kH - 1)) {
                continue;
            }
            const auto y0 = static_cast<uint32_t>(sy);
            const uint32_t y1 = std::min(y0 + 1, kH - 1);
            const float a = sy - static_cast<float>(y0);
            rectR[static_cast<size_t>(y) * kW + x] =
                (1.0f - a) * rawR[static_cast<size_t>(y0) * kW + x] +
                a * rawR[static_cast<size_t>(y1) * kW + x];
        }
    }
    std::vector<float> L;
    std::vector<float> R;
    blur5(rawL, L);
    blur5(rectR, R);

    // 2. Census matching cost volume [pixel][d - dispMin].
    std::vector<uint64_t> cL;
    std::vector<uint64_t> cR;
    censusTransform(L, cL);
    censusTransform(R, cR);
    std::vector<uint8_t> cost(kImagePixels * numDisp, kCensusInvalidCost);
#pragma omp parallel for schedule(static)
    for (int yi = 0; yi < static_cast<int>(kH); ++yi) {
        const auto y = static_cast<uint32_t>(yi);
        for (uint32_t x = 0; x < kW; ++x) {
            const size_t p = static_cast<size_t>(y) * kW + x;
            uint8_t* c = cost.data() + p * numDisp;
            for (int di = 0; di < numDisp; ++di) {
                const int xr = static_cast<int>(x) - (dispMin + di);
                if (xr < 0) {
                    break;
                }
                c[di] = static_cast<uint8_t>(__builtin_popcountll(cL[p] ^ cR[p - (dispMin + di)]));
            }
        }
    }

    // 3. Semi-global aggregation along four directions.
    // Lines within one direction touch disjoint pixels, so each direction is
    // parallel over lines; directions run one after another.
    std::vector<uint16_t> sum(cost.size(), 0);
    const auto W = static_cast<std::ptrdiff_t>(kW);
    const struct {
        int lines;
        std::ptrdiff_t lineStride;
        std::ptrdiff_t first;
        std::ptrdiff_t step;
        uint32_t length;
    } passes[4] = {
        {static_cast<int>(kH), W, 0, 1, kW},                           // left → right
        {static_cast<int>(kH), W, W - 1, -1, kW},                      // right → left
        {static_cast<int>(kW), 1, 0, W, kH},                           // top → bottom
        {static_cast<int>(kW), 1, static_cast<std::ptrdiff_t>(kH - 1) * W, -W, kH}, // bottom → top
    };
    for (const auto& pass : passes) {
#pragma omp parallel
        {
            std::vector<uint16_t> prev(numDisp);
            std::vector<uint16_t> cur(numDisp);
#pragma omp for schedule(static)
            for (int line = 0; line < pass.lines; ++line) {
                aggregatePath(cost, numDisp,
                              static_cast<size_t>(pass.first + line * pass.lineStride),
                              pass.step, pass.length, t.p1, t.p2, sum, prev, cur);
            }
        }
    }

    // 4. Winner-take-all + uniqueness + sub-pixel; right-view winners for LR.
    std::vector<float> stdL;
    localStd(L, 4, stdL);
    std::vector<float> disp(kImagePixels, -1.0f);
#pragma omp parallel for schedule(static)
    for (int yi = 0; yi < static_cast<int>(kH); ++yi) {
        const auto y = static_cast<uint32_t>(yi);
        int dispRight[kW];
        for (uint32_t xr = 0; xr < kW; ++xr) {
            int bestD = -1;
            uint32_t best = std::numeric_limits<uint32_t>::max();
            for (int di = 0; di < numDisp; ++di) {
                const uint32_t xl = xr + static_cast<uint32_t>(dispMin + di);
                if (xl >= kW) {
                    break;
                }
                const uint16_t s = sum[(static_cast<size_t>(y) * kW + xl) * numDisp + di];
                if (s < best) {
                    best = s;
                    bestD = di;
                }
            }
            dispRight[xr] = bestD < 0 ? -1 : dispMin + bestD;
        }
        for (uint32_t x = static_cast<uint32_t>(dispMin); x < kW; ++x) {
            const size_t p = static_cast<size_t>(y) * kW + x;
            if (stdL[p] < t.textureMin) {
                continue;
            }
            const uint16_t* s = sum.data() + p * numDisp;
            const int validDisp = std::min(numDisp, static_cast<int>(x) - dispMin + 1);
            int bestD = 0;
            for (int di = 1; di < validDisp; ++di) {
                if (s[di] < s[bestD]) {
                    bestD = di;
                }
            }
            uint32_t rival = std::numeric_limits<uint32_t>::max();
            for (int di = 0; di < validDisp; ++di) {
                if (std::abs(di - bestD) > 1) {
                    rival = std::min<uint32_t>(rival, s[di]);
                }
            }
            if (rival != std::numeric_limits<uint32_t>::max() &&
                static_cast<float>(s[bestD]) * (1.0f + t.uniqueness) >= static_cast<float>(rival)) {
                continue;
            }
            float d = static_cast<float>(dispMin + bestD);
            if (bestD > 0 && bestD + 1 < validDisp) {
                const float cp = s[bestD - 1];
                const float cb = s[bestD];
                const float cn = s[bestD + 1];
                d += subpixelOffset(cp, cb, cn, t.equiangular);
            } else {
                continue; // winner at the band edge: object is outside the preset
            }
            const int xr = static_cast<int>(x) - static_cast<int>(std::lround(d));
            if (xr < 0 || dispRight[xr] < 0 ||
                std::fabs(static_cast<float>(dispRight[xr]) - d) > t.lrMaxDiff + 0.5f) {
                continue;
            }
            disp[p] = d;
        }
    }
    return disp;
}

} // namespace

RowRectification defaultPop4RowRectification() {
    // Fitted 2026-10-12 on a live POP4 (405 speckle correspondences, residual
    // 0.15 px) with tools/re_probe/pop4_rectify_fit.py. Roll ≈ −2.6° plus keystone (uv term).
    RowRectification coeffs{{-0.00437, -0.03004, 1.01091, -0.01107, 0.13940, -0.00572}};
    if (const char* value = std::getenv("ALL3DSCAN_POP4_RECT")) {
        RowRectification parsed{};
        std::stringstream ss(value);
        std::string item;
        size_t n = 0;
        while (n < parsed.size() && std::getline(ss, item, ',')) {
            char* end = nullptr;
            parsed[n] = std::strtod(item.c_str(), &end);
            if (end == item.c_str() || !std::isfinite(parsed[n])) {
                break;
            }
            ++n;
        }
        if (n == parsed.size() && std::fabs(parsed[2]) > 0.5) {
            coeffs = parsed;
        }
    }
    return coeffs;
}

protocol::ScannerCalibration makeApproximatePop4FullFieldCalibration() {
    constexpr double kPi = 3.14159265358979323846;
    // 46° across the 320 px axis; square pixels.
    const double fx = (static_cast<double>(kW) * 0.5) / std::tan(46.0 * kPi / 360.0);
    const double fy = fx;
    const double cx = kW * 0.5;
    const double cy = kH * 0.5;
    // Provisional: chosen so the ~35 px disparity of a surface measured at
    // ~300 mm maps to 300 mm. Replace with a ruler-checked value (Z ∝ B).
    double baselineMm = 28.0;
    if (const char* value = std::getenv("ALL3DSCAN_POP4_BASELINE_MM")) {
        char* end = nullptr;
        const double parsed = std::strtod(value, &end);
        if (end != value && parsed >= 5.0 && parsed <= 150.0 && std::isfinite(parsed)) {
            baselineMm = parsed;
        }
    }

    protocol::ScannerCalibration cal;
    cal.valid = true;
    cal.source = "ApproximatePop4FullField";
    cal.depthScale = 1.0;
    cal.baselineMm = baselineMm;
    cal.depth = {.fx = fx, .fy = fy, .cx = cx, .cy = cy};
    cal.depthRight = cal.depth;
    cal.stereoTranslation = {-baselineMm / 1000.0, 0.0, 0.0};
    cal.qMatrix = {
        1, 0, 0, -cx,
        0, 1, 0, -cy,
        0, 0, 0, fx * baselineMm,
        0, 0, 1, 0,
    };
    return cal;
}

DisparityBias defaultPop4DisparityBias() {
    // Mean of two independent planar fits (box 24.22/-0.64/0.17, carton
    // 24.66/-0.85/-0.05), 2026-10-12, same POP4 unit as the row map.
    DisparityBias coeffs{{24.44, -0.745, 0.06}};
    if (const char* value = std::getenv("ALL3DSCAN_POP4_DISPX")) {
        DisparityBias parsed{};
        std::stringstream ss(value);
        std::string item;
        size_t n = 0;
        while (n < parsed.size() && std::getline(ss, item, ',')) {
            char* end = nullptr;
            parsed[n] = std::strtod(item.c_str(), &end);
            if (end == item.c_str() || !std::isfinite(parsed[n]) || std::fabs(parsed[n]) > 80.0) {
                break;
            }
            ++n;
        }
        if (n == parsed.size()) {
            coeffs = parsed;
        }
    }
    return coeffs;
}

Pop4DepthDecoder::Pop4DepthDecoder() {
    setRowRectification(defaultPop4RowRectification());
    setDisparityBias(defaultPop4DisparityBias());
}

void Pop4DepthDecoder::setDisparityBias(const DisparityBias& coeffs) {
    if (coeffs == bias_ && !biasMap_.empty()) {
        return;
    }
    bias_ = coeffs;
    biasMap_.resize(kImagePixels);
    biasMin_ = std::numeric_limits<float>::max();
    biasMax_ = std::numeric_limits<float>::lowest();
    for (uint32_t y = 0; y < kH; ++y) {
        const double v = (static_cast<double>(y) - 256.0) / 256.0;
        for (uint32_t x = 0; x < kW; ++x) {
            const double u = (static_cast<double>(x) - 160.0) / 160.0;
            const auto b = static_cast<float>(coeffs[0] * u * u + coeffs[1] * u * v + coeffs[2] * v * v);
            biasMap_[static_cast<size_t>(y) * kW + x] = b;
            biasMin_ = std::min(biasMin_, b);
            biasMax_ = std::max(biasMax_, b);
        }
    }
}

void Pop4DepthDecoder::setCalibration(const protocol::ScannerCalibration& calibration) {
    calibration_ = calibration;
}

void Pop4DepthDecoder::setRowRectification(const RowRectification& coeffs) {
    if (coeffs == rect_ && !rectMapY_.empty()) {
        return;
    }
    rect_ = coeffs;
    rebuildRectificationMap();
}

void Pop4DepthDecoder::rebuildRectificationMap() {
    // For each output pixel find the raw right-image row yR whose mapped row
    // equals the output row (x is unchanged): Newton on yL(x, yR) = y.
    rectMapY_.resize(kImagePixels);
    const auto& c = rect_;
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            const double u = (static_cast<double>(x) - 160.0) / 160.0;
            double yR = y;
            for (int it = 0; it < 8; ++it) {
                const double v = (yR - 256.0) / 256.0;
                const double f =
                    256.0 + 256.0 * (c[0] + c[1] * u + c[2] * v + c[3] * u * u +
                                     c[4] * u * v + c[5] * v * v);
                const double df = c[2] + c[4] * u + 2.0 * c[5] * v;
                if (std::fabs(df) < 1e-6) {
                    break;
                }
                yR -= (f - static_cast<double>(y)) / df;
            }
            rectMapY_[static_cast<size_t>(y) * kW + x] = static_cast<float>(yR);
        }
    }
}

void Pop4DepthDecoder::setDepthRange(float minMm, float maxMm) {
    if (!std::isfinite(minMm) || !std::isfinite(maxMm)) {
        return;
    }
    const float clampedMin = std::clamp(minMm, 50.0f, 2000.0f);
    const float clampedMax = std::clamp(maxMm, 50.0f, 2000.0f);
    if (clampedMax <= clampedMin) {
        return;
    }
    depthMinMm_ = clampedMin;
    depthMaxMm_ = clampedMax;
}

std::string Pop4DepthDecoder::backendName() {
    return cuda_stereo::isBuilt() && cuda_stereo::available() ? "CUDA" : "CPU";
}

std::optional<protocol::RawFrame> Pop4DepthDecoder::decode(const protocol::RawFrame& rawY16) {
    lastError_.clear();
    if (rawY16.encoding != protocol::DepthEncoding::VendorRawY16) {
        lastError_ = "POP4 decoder expects VendorRawY16 input";
        return std::nullopt;
    }
    if (!calibration_.hasStereoCalibration()) {
        lastError_ = "POP4 metric decode blocked: stereo calibration not loaded";
        return std::nullopt;
    }
    if (rawY16.width != kPackedWidth || rawY16.height != kPackedHeight) {
        lastError_ = "POP4 decode expects 640x2560 VendorRawY16 (ten 320x512 images)";
        return std::nullopt;
    }
    if (rawY16.depthMm.size() < kImagePixels * 10) {
        lastError_ = "POP4 packed frame truncated";
        return std::nullopt;
    }

    const MatchTuning t = readMatchTuning();
    const bool swap = swapCameras();
    const int leftIdx = swap ? kRightSpeckle : kLeftSpeckle;
    const int rightIdx = swap ? kLeftSpeckle : kRightSpeckle;
    const int leftAmbient = swap ? kRightAmbientFirst : kLeftAmbientFirst;

    int dispMin = kFallbackDispMin;
    int dispMax = kFallbackDispMax;
    computeDispRange(calibration_, depthMinMm_, depthMaxMm_, dispMin, dispMax);
    // Matched disparities carry the lens bias, so search the biased band.
    dispMin = std::max(kDispSearchFloor, dispMin + static_cast<int>(std::floor(biasMin_)));
    dispMax = std::min(kDispSearchCeil, dispMax + static_cast<int>(std::ceil(biasMax_)));
    const int numDisp = dispMax - dispMin + 1;

    // 1-4. Matching: GPU when built and available, otherwise CPU.
    const auto matchStart = std::chrono::steady_clock::now();
    std::vector<float> disp;
    std::string backend = "CPU";
    if (cuda_stereo::isBuilt() && cuda_stereo::available()) {
        cuda_stereo::SgmParams params;
        params.dispMin = dispMin;
        params.numDisp = numDisp;
        params.p1 = t.p1;
        params.p2 = t.p2;
        params.uniqueness = t.uniqueness;
        params.lrMaxDiff = t.lrMaxDiff;
        params.textureMin = t.textureMin;
        std::string cudaError;
        if (cuda_stereo::computeDisparity(imageAt(rawY16, leftIdx), imageAt(rawY16, rightIdx),
                                          rectMapY_.data(), params, disp, &cudaError)) {
            backend = "CUDA";
        } else if (t.debug) {
            std::fprintf(stderr, "[POP4] %s; using CPU\n", cudaError.c_str());
        }
    }
    if (backend != "CUDA") {
        disp = matchCpu(imageAt(rawY16, leftIdx), imageAt(rawY16, rightIdx), rectMapY_, t,
                        dispMin, numDisp);
    }

    const double matchMs = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - matchStart)
                               .count();

    // Remove the lens bias so flat surfaces come out flat.
    for (size_t p = 0; p < kImagePixels; ++p) {
        if (disp[p] >= 0.0f) {
            const float d = disp[p] - biasMap_[p];
            disp[p] = d > 0.5f ? d : -1.0f;
        }
    }

    // 5. Clean-up.
    if (t.doMedian) {
        std::vector<float> tmp;
        medianFilterDisparity(disp, tmp);
        disp.swap(tmp);
    }
    filterSpeckles(disp, t.speckleMinRegion, t.speckleMaxDiff);
    filterDepthEdges(disp, t.edgeRadius, t.edgeMaxJump);

    // 6. Metric depth + texture.
    protocol::RawFrame out;
    out.width = kW;
    out.height = kH;
    out.timestampUs = rawY16.timestampUs;
    out.encoding = protocol::DepthEncoding::MetricMillimeters;
    out.depthMm.assign(kImagePixels, 0);
    ambientTexture(rawY16, leftAmbient, out.rgb);

    uint32_t validDepth = 0;
    double sumD = 0.0;
    double sumD2 = 0.0;
    for (size_t p = 0; p < kImagePixels; ++p) {
        if (disp[p] < 0.0f) {
            continue;
        }
        const float z = depthFromDisparity(disp[p], calibration_, depthMinMm_, depthMaxMm_);
        if (z <= 0.0f) {
            disp[p] = -1.0f;
            continue;
        }
        out.depthMm[p] = static_cast<uint16_t>(std::clamp(z, 1.0f, 65535.0f));
        ++validDepth;
        sumD += disp[p];
        sumD2 += static_cast<double>(disp[p]) * disp[p];
    }
    float dispStd = 0.0f;
    if (validDepth > 1) {
        const double mean = sumD / validDepth;
        dispStd = static_cast<float>(std::sqrt(std::max(0.0, sumD2 / validDepth - mean * mean)));
    }
    lastDisparity_ = std::move(disp);

    lastStats_.validDepth = validDepth;
    lastStats_.totalPixels = static_cast<uint32_t>(kImagePixels);
    lastStats_.dispStdPx = dispStd;
    lastStats_.backend = backend;
    lastStats_.attempted = true;
    if (t.debug) {
        std::fprintf(stderr, "[POP4] %s match %.1f ms band d=[%d,%d] valid=%.1f%% (%u) dispStd=%.2f\n", backend.c_str(), matchMs, dispMin,
                     dispMax, 100.0 * validDepth / static_cast<double>(kImagePixels), validDepth,
                     static_cast<double>(dispStd));
    }

    if (validDepth < kMinValidDepth) {
        lastError_ = "POP4 speckle match produced too few metric samples (" +
                     std::to_string(validDepth) + ")";
        return std::nullopt;
    }
    return out;
}

std::optional<protocol::RawFrame> Pop4DepthDecoder::validateMetricPlane(
    const protocol::RawFrame& metricCandidate,
    const protocol::ScannerCalibration& calibration,
    std::string* errorOut) {
    auto fail = [&](const std::string& msg) -> std::optional<protocol::RawFrame> {
        if (errorOut) {
            *errorOut = msg;
        }
        return std::nullopt;
    };

    if (!calibration.hasDepthIntrinsics()) {
        return fail("calibration missing depth intrinsics");
    }
    if (metricCandidate.encoding != protocol::DepthEncoding::MetricMillimeters) {
        return fail("expected MetricMillimeters encoding");
    }
    const size_t expected = static_cast<size_t>(metricCandidate.width) *
                            static_cast<size_t>(metricCandidate.height);
    if (metricCandidate.width == 0 || metricCandidate.height == 0 ||
        metricCandidate.depthMm.size() != expected) {
        return fail("malformed metric frame dimensions");
    }

    uint32_t nonzero = 0;
    for (uint16_t d : metricCandidate.depthMm) {
        if (d != 0) {
            ++nonzero;
        }
    }
    if (nonzero < 16) {
        return fail("too few nonzero metric samples");
    }

    protocol::RawFrame out = metricCandidate;
    if (std::abs(calibration.depthScale - 1.0) > 1e-9) {
        for (uint16_t& d : out.depthMm) {
            if (d == 0) {
                continue;
            }
            const double scaled = static_cast<double>(d) * calibration.depthScale;
            d = static_cast<uint16_t>(std::clamp(scaled, 0.0, 65535.0));
        }
    }
    return out;
}

} // namespace revoscan::decoder
