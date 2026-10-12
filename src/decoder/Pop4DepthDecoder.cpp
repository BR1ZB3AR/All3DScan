#include "Pop4DepthDecoder.h"

#include "Pop4StereoCuda.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

namespace revoscan::decoder {
namespace {

constexpr uint32_t kPlaneWidth = 640;
constexpr uint32_t kPlaneHeight = 512;
// Live 640×512 planes are a 2×2 of 320×256 tiles (strong seams at col 320 and
// row 256). Stereo L|R is the top row; the bottom row is a darker near-copy /
// second exposure of the same pair — NOT a taller 320×512 camera.
constexpr uint32_t kViewWidth = 320;
constexpr uint32_t kViewHeight = 256;
constexpr uint32_t kPackedHeight = 2560;
constexpr int kPhasePlanes = 4;
// APK libRPASpeckleStereo rejects modulation < 3.0f.
constexpr float kModulationMin = 2.5f;
constexpr float kTwoPi = 6.283185307179586f;
// Fallback disparity search band, used only if calibration can't derive one
// (e.g. missing Q matrix). Matches the historical 200-650mm hardcoded band.
constexpr int kFallbackDispMin = 8;
constexpr int kFallbackDispMax = 120;
constexpr int kDispSearchPad = 6;    // px slack around the derived band
constexpr int kDispSearchFloor = 1;  // never search disparity 0 (degenerate Q)
constexpr uint32_t kMinValidDepth = 200;
constexpr float kTextureGain = 1.8f;
// Re-balanced 2026-07: the first stricter pass (0.15 / 0.03 / uniq 0.75 /
// lrDiff 1.5 / speckle 6) rejected almost every real-device pixel, so decode
// fell back to raw Y16. These looser defaults keep enough matches to clear
// kMinValidDepth while margin + ratio + intensity + speckle still kill the
// periodic constant-fringe slab.
constexpr float kMaxPhaseCost = 0.35f;
constexpr float kMinCostMargin = 0.001f;
// Weight on |albedoL−albedoR| (0..255) added to phase circular diff. Breaks
// single-frequency fringe ambiguity on real textured surfaces.
constexpr float kIntensityWeight = 0.05f;

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

// Matching-quality tuning shared by the CPU path and (via DecodeParams) the
// CUDA path. Wrapped single-frequency phase is periodic, so a naive per-pixel
// winner collapses to a near-constant disparity band → fronto-parallel slab →
// ICP fuses slabs into a cube. These robustness gates recover real relief.
struct MatchTuning {
    float maxPhaseCost = kMaxPhaseCost;
    float minCostMargin = kMinCostMargin;
    float uniqRatio = 0.93f;      // best < 0.93 * rival
    float intensityWeight = kIntensityWeight;
    bool lrCheck = true;
    float lrMaxDiff = 1.2f;       // strict L↔R consistency keeps true surface relief
    bool doMedian = true;
    bool doSpeckle = true;
    float speckleMaxDiff = 3.0f;
    int speckleMinSupport = 3;    // 3 of 24 neighbours rejects noise
    bool debug = false;
};

MatchTuning readMatchTuning() {
    MatchTuning t;
    t.maxPhaseCost = envFloat("ALL3DSCAN_POP4_MAXCOST", t.maxPhaseCost, 0.01f, 3.14f);
    t.minCostMargin = envFloat("ALL3DSCAN_POP4_MARGIN", t.minCostMargin, 0.0f, 3.14f);
    t.uniqRatio = envFloat("ALL3DSCAN_POP4_UNIQ", t.uniqRatio, 0.1f, 1.0f);
    t.intensityWeight =
        envFloat("ALL3DSCAN_POP4_INTENSITY", t.intensityWeight, 0.0f, 1.0f);
    t.lrCheck = envFlag("ALL3DSCAN_POP4_LRCHECK", t.lrCheck);
    t.lrMaxDiff = envFloat("ALL3DSCAN_POP4_LRDIFF", t.lrMaxDiff, 0.5f, 8.0f);
    t.doMedian = envFlag("ALL3DSCAN_POP4_MEDIAN", t.doMedian);
    t.doSpeckle = envFlag("ALL3DSCAN_POP4_SPECKLE", t.doSpeckle);
    t.speckleMaxDiff = envFloat("ALL3DSCAN_POP4_SPECKLE_DIFF", t.speckleMaxDiff, 0.5f, 16.0f);
    t.speckleMinSupport = static_cast<int>(
        envFloat("ALL3DSCAN_POP4_SPECKLE_SUPPORT",
                 static_cast<float>(t.speckleMinSupport), 0.0f, 24.0f));
    t.debug = envFlag("ALL3DSCAN_POP4_DEBUG", false);
    return t;
}

// SWAP_LR: 0/false = left is cols 0..319; 1/true = swapped; auto/unset
// (default) = pick the assignment that yields more valid depth with spread.
enum class SwapLrMode { ForceNo, ForceYes, Auto };

SwapLrMode swapLrMode() {
    if (const char* value = std::getenv("ALL3DSCAN_POP4_SWAP_LR")) {
        if (value[0] == 'a' || value[0] == 'A') {
            return SwapLrMode::Auto;
        }
        if (value[0] == '1' || value[0] == 'y' || value[0] == 'Y' ||
            value[0] == 't' || value[0] == 'T') {
            return SwapLrMode::ForceYes;
        }
        if (value[0] == '0' || value[0] == 'n' || value[0] == 'N' ||
            value[0] == 'f' || value[0] == 'F') {
            return SwapLrMode::ForceNo;
        }
    }
    return SwapLrMode::Auto;
}

// Process-lifetime Auto decision: -1 unknown, 0 = no swap, 1 = swap.
int& autoSwapCache() {
    static int cached = -1;
    return cached;
}

void resetAutoSwapCache() {
    autoSwapCache() = -1;
}

bool swapStereoHalves() {
    switch (swapLrMode()) {
    case SwapLrMode::ForceYes:
        return true;
    case SwapLrMode::ForceNo:
        return false;
    case SwapLrMode::Auto:
        return autoSwapCache() == 1;
    }
    return false;
}

void commitAutoSwap(bool swap, uint32_t validPoints) {
    if (swapLrMode() != SwapLrMode::Auto) {
        return;
    }
    if (validPoints < 1500) {
        return; // Don't lock auto-swap on unilluminated startup or sparse frames
    }
    int& cached = autoSwapCache();
    if (cached < 0) {
        cached = swap ? 1 : 0;
        if (envFlag("ALL3DSCAN_POP4_DEBUG", false)) {
            std::fprintf(stderr, "[POP4] auto SWAP_LR=%d (valid=%u)\n", cached, validPoints);
        }
    }
}

uint32_t stereoBandY0() {
    if (const char* value = std::getenv("ALL3DSCAN_POP4_STEREO_BAND")) {
        if (value[0] == 'b' || value[0] == 'B' || value[0] == '1') {
            return kViewHeight;
        }
    }
    return 0;
}

float intensityFromY16(uint16_t sample) {
    return static_cast<float>(sample) / 257.0f;
}

float phaseCircDiff(float a, float b) {
    float d = std::fabs(a - b);
    if (d > kTwoPi - d) {
        d = kTwoPi - d;
    }
    return d;
}

void fourStepPhase(const uint16_t* plane0,
                   const uint16_t* plane1,
                   const uint16_t* plane2,
                   const uint16_t* plane3,
                   uint32_t width,
                   uint32_t height,
                   uint32_t stride,
                   uint32_t x0,
                   uint32_t y0,
                   std::vector<float>& phaseOut,
                   std::vector<uint8_t>& validOut,
                   std::vector<float>& albedoOut) {
    const size_t n = static_cast<size_t>(width) * height;
    phaseOut.assign(n, 0.0f);
    validOut.assign(n, 0);
    albedoOut.assign(n, 0.0f);
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            const size_t src =
                static_cast<size_t>(y0 + y) * stride + x0 + x;
            const size_t dst = static_cast<size_t>(y) * width + x;
            const float i0 = intensityFromY16(plane0[src]);
            const float i1 = intensityFromY16(plane1[src]);
            const float i2 = intensityFromY16(plane2[src]);
            const float i3 = intensityFromY16(plane3[src]);
            albedoOut[dst] = 0.25f * (i0 + i1 + i2 + i3);
            const float dx = i0 - i2;
            const float dy = i1 - i3;
            const float mod = std::hypot(dx, dy);
            if (mod < kModulationMin) {
                continue;
            }
            float phase = std::atan2(dy, dx);
            if (phase < 0.0f) {
                phase += kTwoPi;
            }
            phaseOut[dst] = phase;
            validOut[dst] = 1;
        }
    }
}

// Phase-guided disparity search for one direction. `dir` is -1 when the source
// is the LEFT view (target pixel = x - d) and +1 when the source is the RIGHT
// view (target pixel = x + d). Returns sub-pixel disparity (>=0) or -1 when the
// match is weak or ambiguous (fails the uniqueness / ratio gates).
// Hybrid cost = phase circular diff + intensityWeight * |albedoSrc-albedoTgt|
// so textured IR breaks single-frequency fringe period collisions.
void matchDisparity(const std::vector<float>& phaseSrc,
                    const std::vector<uint8_t>& validSrc,
                    const std::vector<float>& albedoSrc,
                    const std::vector<float>& phaseTgt,
                    const std::vector<uint8_t>& validTgt,
                    const std::vector<float>& albedoTgt,
                    int dir,
                    int dispMin,
                    int dispMax,
                    const MatchTuning& t,
                    std::vector<float>& dispOut) {
    dispOut.assign(phaseSrc.size(), -1.0f);
    for (uint32_t y = 0; y < kViewHeight; ++y) {
        for (uint32_t x = 0; x < kViewWidth; ++x) {
            const size_t si = static_cast<size_t>(y) * kViewWidth + x;
            if (!validSrc[si]) {
                continue;
            }
            float bestCost = std::numeric_limits<float>::max();
            float bestPhase = std::numeric_limits<float>::max();
            int bestD = -1;
            for (int d = dispMin; d <= dispMax; ++d) {
                const int xt = static_cast<int>(x) + dir * d;
                if (xt < 0 || xt >= static_cast<int>(kViewWidth)) {
                    continue;
                }
                const size_t ti =
                    static_cast<size_t>(y) * kViewWidth + static_cast<size_t>(xt);
                if (!validTgt[ti]) {
                    continue;
                }
                const float phaseCost = phaseCircDiff(phaseSrc[si], phaseTgt[ti]);
                const float cost =
                    phaseCost +
                    t.intensityWeight * std::fabs(albedoSrc[si] - albedoTgt[ti]);
                if (cost < bestCost) {
                    bestCost = cost;
                    bestPhase = phaseCost;
                    bestD = d;
                }
            }
            // Gate on phase agreement only — intensity is a tie-break / ranking
            // term and must not reject otherwise-good fringe matches.
            if (bestD < 0 || bestPhase > t.maxPhaseCost) {
                continue;
            }

            // Best rival at a *different* fringe period (excludes local bowl ±5px
            // around winner). This tests uniqueness against actual rival periods.
            float rivalCost = std::numeric_limits<float>::max();
            for (int d = dispMin; d <= dispMax; ++d) {
                if (std::abs(d - bestD) < 6) {
                    continue;
                }
                const int xt = static_cast<int>(x) + dir * d;
                if (xt < 0 || xt >= static_cast<int>(kViewWidth)) {
                    continue;
                }
                const size_t ti =
                    static_cast<size_t>(y) * kViewWidth + static_cast<size_t>(xt);
                if (!validTgt[ti]) {
                    continue;
                }
                const float cost =
                    phaseCircDiff(phaseSrc[si], phaseTgt[ti]) +
                    t.intensityWeight * std::fabs(albedoSrc[si] - albedoTgt[ti]);
                rivalCost = std::min(rivalCost, cost);
            }
            if (rivalCost < std::numeric_limits<float>::max()) {
                if ((rivalCost - bestCost) < t.minCostMargin) {
                    continue; // additive margin
                }
                if (bestCost > t.uniqRatio * rivalCost) {
                    continue; // ratio test
                }
            }

            // Parabolic sub-pixel refine using the two adjacent disparities.
            float costPrev = bestCost + 1.0f; // d - 1
            float costNext = bestCost + 1.0f; // d + 1
            const int xtPrev = static_cast<int>(x) + dir * (bestD - 1);
            const int xtNext = static_cast<int>(x) + dir * (bestD + 1);
            if (xtPrev >= 0 && xtPrev < static_cast<int>(kViewWidth) &&
                validTgt[static_cast<size_t>(y) * kViewWidth +
                         static_cast<size_t>(xtPrev)]) {
                const size_t ti = static_cast<size_t>(y) * kViewWidth +
                                  static_cast<size_t>(xtPrev);
                costPrev = phaseCircDiff(phaseSrc[si], phaseTgt[ti]) +
                           t.intensityWeight *
                               std::fabs(albedoSrc[si] - albedoTgt[ti]);
            }
            if (xtNext >= 0 && xtNext < static_cast<int>(kViewWidth) &&
                validTgt[static_cast<size_t>(y) * kViewWidth +
                         static_cast<size_t>(xtNext)]) {
                const size_t ti = static_cast<size_t>(y) * kViewWidth +
                                  static_cast<size_t>(xtNext);
                costNext = phaseCircDiff(phaseSrc[si], phaseTgt[ti]) +
                           t.intensityWeight *
                               std::fabs(albedoSrc[si] - albedoTgt[ti]);
            }
            float disparity = static_cast<float>(bestD);
            const float denom = costPrev - 2.0f * bestCost + costNext;
            if (std::fabs(denom) > 1e-6f) {
                const float delta = 0.5f * (costPrev - costNext) / denom;
                if (std::fabs(delta) < 1.0f) {
                    disparity += delta;
                }
            }
            dispOut[si] = disparity;
        }
    }
}

// Keep only left disparities that agree with the right→left match at the
// corresponding pixel (mutual consistency); invalidate the rest.
void applyLrConsistency(std::vector<float>& dispL, const std::vector<float>& dispR,
                        float lrMaxDiff) {
    for (uint32_t y = 0; y < kViewHeight; ++y) {
        for (uint32_t x = 0; x < kViewWidth; ++x) {
            const size_t li = static_cast<size_t>(y) * kViewWidth + x;
            const float d = dispL[li];
            if (d < 0.0f) {
                continue;
            }
            const int xr = static_cast<int>(x) - static_cast<int>(std::lround(d));
            if (xr < 0 || xr >= static_cast<int>(kViewWidth)) {
                dispL[li] = -1.0f;
                continue;
            }
            const float dr =
                dispR[static_cast<size_t>(y) * kViewWidth + static_cast<size_t>(xr)];
            if (dr < 0.0f || std::fabs(dr - d) > lrMaxDiff) {
                dispL[li] = -1.0f;
            }
        }
    }
}

// 3×3 median of valid disparities (invalid neighbours ignored). Removes
// salt/pepper mismatches without smearing edges much.
void medianFilterDisparity(const std::vector<float>& in, std::vector<float>& out) {
    out.assign(in.size(), -1.0f);
    for (uint32_t y = 0; y < kViewHeight; ++y) {
        for (uint32_t x = 0; x < kViewWidth; ++x) {
            const size_t ci = static_cast<size_t>(y) * kViewWidth + x;
            if (in[ci] < 0.0f) {
                continue;
            }
            float window[9];
            int n = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                const int yy = static_cast<int>(y) + dy;
                if (yy < 0 || yy >= static_cast<int>(kViewHeight)) {
                    continue;
                }
                for (int dx = -1; dx <= 1; ++dx) {
                    const int xx = static_cast<int>(x) + dx;
                    if (xx < 0 || xx >= static_cast<int>(kViewWidth)) {
                        continue;
                    }
                    const float v =
                        in[static_cast<size_t>(yy) * kViewWidth +
                           static_cast<size_t>(xx)];
                    if (v >= 0.0f) {
                        window[n++] = v;
                    }
                }
            }
            if (n == 0) {
                continue;
            }
            std::sort(window, window + n);
            out[ci] = window[n / 2];
        }
    }
}

// Local-support speckle rejection: drop a disparity if too few neighbours in a
// 5×5 window agree with it (removes isolated wrong-fringe blobs).
void speckleFilterDisparity(const std::vector<float>& in, std::vector<float>& out,
                            float maxDiff, int minSupport) {
    out = in;
    for (uint32_t y = 0; y < kViewHeight; ++y) {
        for (uint32_t x = 0; x < kViewWidth; ++x) {
            const size_t ci = static_cast<size_t>(y) * kViewWidth + x;
            const float c = in[ci];
            if (c < 0.0f) {
                continue;
            }
            int support = 0;
            for (int dy = -2; dy <= 2; ++dy) {
                const int yy = static_cast<int>(y) + dy;
                if (yy < 0 || yy >= static_cast<int>(kViewHeight)) {
                    continue;
                }
                for (int dx = -2; dx <= 2; ++dx) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    const int xx = static_cast<int>(x) + dx;
                    if (xx < 0 || xx >= static_cast<int>(kViewWidth)) {
                        continue;
                    }
                    const float v =
                        in[static_cast<size_t>(yy) * kViewWidth +
                           static_cast<size_t>(xx)];
                    if (v >= 0.0f && std::fabs(v - c) <= maxDiff) {
                        ++support;
                    }
                }
            }
            if (support < minSupport) {
                out[ci] = -1.0f;
            }
        }
    }
}

void debugReport(const std::vector<float>& disp, uint32_t validDepth, int dispMin,
                 int dispMax) {
    static int frame = 0;
    if ((frame++ % 15) != 0) {
        return;
    }
    const size_t total = disp.size();
    size_t valid = 0;
    float dmin = 1e30f, dmax = -1e30f;
    double sum = 0.0, sum2 = 0.0;
    for (float d : disp) {
        if (d < 0.0f) {
            continue;
        }
        ++valid;
        dmin = std::min(dmin, d);
        dmax = std::max(dmax, d);
        sum += d;
        sum2 += static_cast<double>(d) * d;
    }
    if (valid == 0) {
        std::fprintf(stderr, "[POP4] valid disparities: 0 / %zu\n", total);
        return;
    }
    const double mean = sum / static_cast<double>(valid);
    const double var = sum2 / static_cast<double>(valid) - mean * mean;
    const double stddev = std::sqrt(std::max(0.0, var));
    // Coarse 8-bin histogram across the search range.
    int hist[8] = {0};
    const float span = static_cast<float>(dispMax - dispMin);
    for (float d : disp) {
        if (d < 0.0f) {
            continue;
        }
        int b = static_cast<int>((d - dispMin) / span * 8.0f);
        b = std::clamp(b, 0, 7);
        ++hist[b];
    }
    std::fprintf(stderr,
                 "[POP4] valid=%.1f%% (%zu/%zu) depthOut=%u disp[min=%.1f max=%.1f "
                 "mean=%.1f std=%.2f] hist=[%d %d %d %d %d %d %d %d]\n",
                 100.0 * static_cast<double>(valid) / static_cast<double>(total),
                 valid, total, validDepth, static_cast<double>(dmin),
                 static_cast<double>(dmax), mean, stddev, hist[0], hist[1], hist[2],
                 hist[3], hist[4], hist[5], hist[6], hist[7]);
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
    double z = q23 / denom;
    z *= cal.depthScale;
    if (!std::isfinite(z) || z < depthMinMm || z > depthMaxMm) {
        return 0.0f;
    }
    return static_cast<float>(z);
}

// Inverts the Q-matrix depth formula to find the disparity band that covers
// [depthMinMm, depthMaxMm], so the phase search actually looks where the
// current working-distance hint says the object is. Falls back to the
// historical fixed band if the calibration can't invert (degenerate Q).
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
    double lo = std::min(*dNear, *dFar) - kDispSearchPad;
    double hi = std::max(*dNear, *dFar) + kDispSearchPad;
    lo = std::max<double>(lo, static_cast<double>(kDispSearchFloor));
    hi = std::min<double>(hi, static_cast<double>(kViewWidth) - 1.0);
    if (hi <= lo) {
        dispMinOut = kFallbackDispMin;
        dispMaxOut = kFallbackDispMax;
        return;
    }
    dispMinOut = static_cast<int>(std::floor(lo));
    dispMaxOut = static_cast<int>(std::ceil(hi));
}

struct DecodeScore {
    uint32_t validDepth = 0;
    float dispStd = 0.0f;
    std::vector<float> dispL;
    std::vector<float> albedoL;
};

DecodeScore matchOrientation(const uint16_t** planes,
                             uint32_t xLeft,
                             uint32_t xRight,
                             uint32_t y0,
                             int dispMin,
                             int dispMax,
                             const MatchTuning& tuning) {
    DecodeScore score;
    std::vector<float> phaseL;
    std::vector<float> phaseR;
    std::vector<uint8_t> validL;
    std::vector<uint8_t> validR;
    std::vector<float> albedoR;
    fourStepPhase(planes[0], planes[1], planes[2], planes[3], kViewWidth,
                  kViewHeight, kPlaneWidth, xLeft, y0, phaseL, validL,
                  score.albedoL);
    fourStepPhase(planes[0], planes[1], planes[2], planes[3], kViewWidth,
                  kViewHeight, kPlaneWidth, xRight, y0, phaseR, validR, albedoR);

    matchDisparity(phaseL, validL, score.albedoL, phaseR, validR, albedoR,
                   /*dir=*/-1, dispMin, dispMax, tuning, score.dispL);
    if (tuning.lrCheck) {
        std::vector<float> dispR;
        matchDisparity(phaseR, validR, albedoR, phaseL, validL, score.albedoL,
                       /*dir=*/+1, dispMin, dispMax, tuning, dispR);
        applyLrConsistency(score.dispL, dispR, tuning.lrMaxDiff);
    }
    if (tuning.doMedian) {
        std::vector<float> tmp;
        medianFilterDisparity(score.dispL, tmp);
        score.dispL.swap(tmp);
    }
    if (tuning.doSpeckle) {
        std::vector<float> tmp;
        speckleFilterDisparity(score.dispL, tmp, tuning.speckleMaxDiff,
                               tuning.speckleMinSupport);
        score.dispL.swap(tmp);
    }

    double sum = 0.0;
    double sumSq = 0.0;
    for (float d : score.dispL) {
        if (d < 0.0f) {
            continue;
        }
        ++score.validDepth;
        sum += d;
        sumSq += static_cast<double>(d) * d;
    }
    if (score.validDepth > 1) {
        const double mean = sum / score.validDepth;
        const double var =
            std::max(0.0, sumSq / score.validDepth - mean * mean);
        score.dispStd = static_cast<float>(std::sqrt(var));
    }
    // validDepth here counts disparities before depth-range clamp; callers
    // re-count after Q mapping. Use dispStd for Auto swap ranking.
    return score;
}

std::optional<protocol::RawFrame> decodeCpu(const protocol::RawFrame& rawY16,
                                            const protocol::ScannerCalibration& cal,
                                            float depthMinMm,
                                            float depthMaxMm,
                                            std::string* errorOut,
                                            protocol::DepthDecodeStats* statsOut = nullptr) {
    const size_t planeSamples =
        static_cast<size_t>(kPlaneWidth) * static_cast<size_t>(kPlaneHeight);
    const uint16_t* planes[kPhasePlanes];
    for (int p = 0; p < kPhasePlanes; ++p) {
        planes[p] = rawY16.depthMm.data() + static_cast<size_t>(p) * planeSamples;
    }

    const uint32_t y0 = stereoBandY0();
    const MatchTuning tuning = readMatchTuning();
    int dispMin = kFallbackDispMin;
    int dispMax = kFallbackDispMax;
    computeDispRange(cal, depthMinMm, depthMaxMm, dispMin, dispMax);

    bool swapLr = swapStereoHalves();
    DecodeScore scored;
    if (swapLrMode() == SwapLrMode::Auto && autoSwapCache() < 0) {
        // Score both L|R assignments once; prefer denser + higher disparity std
        // (collapsed fringe slabs have near-zero std).
        auto scoreNo = matchOrientation(planes, 0, kViewWidth, y0, dispMin, dispMax, tuning);
        auto scoreYes =
            matchOrientation(planes, kViewWidth, 0, y0, dispMin, dispMax, tuning);
        const uint32_t nNo = scoreNo.validDepth;
        const uint32_t nYes = scoreYes.validDepth;
        const float stdNo = scoreNo.dispStd;
        const float stdYes = scoreYes.dispStd;
        // True surface matches have focused disparity variance (std <= 12.0px),
        // whereas false fringe collisions produce extreme std (std > 18.0px).
        const double penaltyNo = (stdNo > 18.0f || stdNo < 1.0f) ? 0.25 : 1.0;
        const double penaltyYes = (stdYes > 18.0f || stdYes < 1.0f) ? 0.25 : 1.0;
        const double rankNo = static_cast<double>(nNo) * penaltyNo;
        const double rankYes = static_cast<double>(nYes) * penaltyYes;
        swapLr = rankYes > rankNo;
        commitAutoSwap(swapLr, std::max(nNo, nYes));
        scored = swapLr ? std::move(scoreYes) : std::move(scoreNo);
        if (tuning.debug) {
            std::fprintf(stderr,
                         "[POP4] auto-swap score no=%u/std%.1f yes=%u/std%.1f -> %s\n",
                         nNo, stdNo, nYes, stdYes, swapLr ? "SWAP" : "noswap");
        }
    } else {
        const uint32_t xLeft = swapLr ? kViewWidth : 0;
        const uint32_t xRight = swapLr ? 0 : kViewWidth;
        scored = matchOrientation(planes, xLeft, xRight, y0, dispMin, dispMax, tuning);
    }

    protocol::RawFrame out;
    out.width = kViewWidth;
    out.height = kViewHeight;
    out.timestampUs = rawY16.timestampUs;
    out.encoding = protocol::DepthEncoding::MetricMillimeters;
    out.depthMm.assign(static_cast<size_t>(kViewWidth) * kViewHeight, 0);
    out.rgb.assign(static_cast<size_t>(kViewWidth) * kViewHeight * 3, 0);

    for (uint32_t y = 0; y < kViewHeight; ++y) {
        for (uint32_t x = 0; x < kViewWidth; ++x) {
            const size_t li = static_cast<size_t>(y) * kViewWidth + x;
            const uint8_t v = static_cast<uint8_t>(std::clamp(
                scored.albedoL[li] * kTextureGain, 0.0f, 255.0f));
            const size_t dst = li * 3;
            out.rgb[dst + 0] = v;
            out.rgb[dst + 1] = v;
            out.rgb[dst + 2] = v;
        }
    }

    uint32_t validDepth = 0;
    for (size_t li = 0; li < scored.dispL.size(); ++li) {
        const float disparity = scored.dispL[li];
        if (disparity < 0.0f) {
            continue;
        }
        const float depthMm = depthFromDisparity(disparity, cal, depthMinMm, depthMaxMm);
        if (depthMm <= 0.0f) {
            continue;
        }
        out.depthMm[li] =
            static_cast<uint16_t>(std::clamp(depthMm, 1.0f, 65535.0f));
        ++validDepth;
    }

    if (tuning.debug) {
        debugReport(scored.dispL, validDepth, dispMin, dispMax);
    }

    if (statsOut) {
        statsOut->validDepth = validDepth;
        statsOut->totalPixels = static_cast<uint32_t>(scored.dispL.size());
        statsOut->dispStdPx = scored.dispStd;
        statsOut->backend = "CPU";
        statsOut->attempted = true;
    }

    if (validDepth < kMinValidDepth) {
        if (errorOut) {
            *errorOut =
                "POP4 phase match produced too few metric samples (" +
                std::to_string(validDepth) + ")";
        }
        return std::nullopt;
    }
    return out;
}

} // namespace

protocol::ScannerCalibration makeApproximatePop4FullFieldCalibration() {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double fovH = 46.0 * kPi / 180.0;
    constexpr double fovV = 37.0 * kPi / 180.0;
    constexpr double w = static_cast<double>(kViewWidth);
    constexpr double h = static_cast<double>(kViewHeight);
    const double fx = (w * 0.5) / std::tan(fovH * 0.5);
    const double fy = (h * 0.5) / std::tan(fovV * 0.5);
    const double cx = w * 0.5;
    const double cy = h * 0.5;
    double baselineMm = 70.0;
    if (const char* value = std::getenv("ALL3DSCAN_POP4_BASELINE_MM")) {
        char* end = nullptr;
        const double parsed = std::strtod(value, &end);
        if (end != value && parsed >= 20.0 && parsed <= 120.0 &&
            std::isfinite(parsed)) {
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

void Pop4DepthDecoder::setCalibration(const protocol::ScannerCalibration& calibration) {
    calibration_ = calibration;
    resetAutoSwapCache();
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
    // Prefer reporting the compile/env preference without forcing a device
    // probe at UI construction time; decode() still checks available().
    if (cuda_stereo::isBuilt() && cuda_stereo::enabledByEnv()) {
        return cuda_stereo::available() ? "CUDA" : "CPU";
    }
    return "CPU";
}

std::optional<protocol::RawFrame> Pop4DepthDecoder::decode(const protocol::RawFrame& rawY16) {
    lastError_.clear();
    if (rawY16.encoding != protocol::DepthEncoding::VendorRawY16) {
        lastError_ = "POP4 decoder expects VendorRawY16 input";
        return std::nullopt;
    }
    if (!calibration_.hasStereoCalibration()) {
        lastError_ =
            "POP4 metric decode blocked: stereo calibration not loaded";
        return std::nullopt;
    }
    if (rawY16.width != kPlaneWidth || rawY16.height != kPackedHeight) {
        lastError_ =
            "POP4 packed decode expects 640x2560 VendorRawY16 "
            "(five 640x512 planes)";
        return std::nullopt;
    }
    const size_t expected =
        static_cast<size_t>(kPlaneWidth) * static_cast<size_t>(kPackedHeight);
    if (rawY16.depthMm.size() < expected) {
        lastError_ = "POP4 packed frame truncated";
        return std::nullopt;
    }

    // Auto L|R: first frame scores both assignments on CPU and commits a
    // process-lifetime choice so CUDA and CPU stay consistent.
    if (swapLrMode() == SwapLrMode::Auto && autoSwapCache() < 0) {
        std::string cpuError;
        auto out = decodeCpu(rawY16, calibration_, depthMinMm_, depthMaxMm_, &cpuError,
                             &lastStats_);
        if (out) {
            lastError_.clear();
            return out;
        }
        // Choice may still be committed; fall through to CUDA/CPU with it.
        if (!cpuError.empty()) {
            lastError_ = cpuError;
        }
    }

    const bool swapLr = swapStereoHalves();
    const uint32_t xLeft = swapLr ? kViewWidth : 0;
    const uint32_t xRight = swapLr ? 0 : kViewWidth;
    const uint32_t y0 = stereoBandY0();
    int dispMin = kFallbackDispMin;
    int dispMax = kFallbackDispMax;
    computeDispRange(calibration_, depthMinMm_, depthMaxMm_, dispMin, dispMax);

    if (cuda_stereo::isBuilt() && cuda_stereo::enabledByEnv() &&
        cuda_stereo::available()) {
        cuda_stereo::DecodeParams params;
        params.xLeft = xLeft;
        params.xRight = xRight;
        params.y0 = y0;
        params.q23 = static_cast<float>(calibration_.qMatrix[2 * 4 + 3]);
        params.q32 = static_cast<float>(calibration_.qMatrix[3 * 4 + 2]);
        params.q33 = static_cast<float>(calibration_.qMatrix[3 * 4 + 3]);
        params.depthScale = static_cast<float>(calibration_.depthScale);
        params.depthMinMm = depthMinMm_;
        params.depthMaxMm = depthMaxMm_;
        params.dispMin = dispMin;
        params.dispMax = dispMax;

        const MatchTuning tuning = readMatchTuning();
        params.maxPhaseCost = tuning.maxPhaseCost;
        params.minCostMargin = tuning.minCostMargin;
        params.uniqRatio = tuning.uniqRatio;
        params.intensityWeight = tuning.intensityWeight;
        params.lrCheck = tuning.lrCheck ? 1 : 0;
        params.lrMaxDiff = tuning.lrMaxDiff;
        params.doMedian = tuning.doMedian ? 1 : 0;
        params.doSpeckle = tuning.doSpeckle ? 1 : 0;
        params.speckleMaxDiff = tuning.speckleMaxDiff;
        params.speckleMinSupport = tuning.speckleMinSupport;

        std::vector<uint16_t> depthMm;
        std::vector<uint8_t> rgb;
        std::string cudaError;
        const uint32_t valid =
            cuda_stereo::decodePacked(rawY16.depthMm.data(), params, depthMm, rgb,
                                      &cudaError);
        lastStats_.validDepth = valid;
        lastStats_.totalPixels = static_cast<uint32_t>(kViewWidth) * kViewHeight;
        lastStats_.dispStdPx = 0.0f; // not computed on the GPU path
        lastStats_.backend = "CUDA";
        lastStats_.attempted = true;
        if (valid >= kMinValidDepth &&
            depthMm.size() == static_cast<size_t>(kViewWidth) * kViewHeight &&
            rgb.size() == static_cast<size_t>(kViewWidth) * kViewHeight * 3) {
            protocol::RawFrame out;
            out.width = kViewWidth;
            out.height = kViewHeight;
            out.timestampUs = rawY16.timestampUs;
            out.encoding = protocol::DepthEncoding::MetricMillimeters;
            out.depthMm = std::move(depthMm);
            out.rgb = std::move(rgb);
            return out;
        }
        // Device errors sticky-disable CUDA inside decodePacked; CPU owns the
        // rest of this process. Sparse-but-successful CUDA still tries CPU.
        lastError_ = cudaError.empty()
                         ? ("CUDA stereo too sparse (" + std::to_string(valid) +
                            "); trying CPU")
                         : (cudaError + "; trying CPU");
    }

    std::string cpuError;
    auto out = decodeCpu(rawY16, calibration_, depthMinMm_, depthMaxMm_, &cpuError,
                         &lastStats_);
    if (!out) {
        lastError_ = cpuError.empty() ? lastError_ : cpuError;
        if (lastError_.empty()) {
            lastError_ = "POP4 phase match failed";
        }
        return std::nullopt;
    }
    lastError_.clear();
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
    double sum = 0.0;
    for (uint16_t d : metricCandidate.depthMm) {
        if (d == 0) {
            continue;
        }
        ++nonzero;
        sum += static_cast<double>(d) * calibration.depthScale;
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
    (void)sum;
    return out;
}

} // namespace revoscan::decoder
