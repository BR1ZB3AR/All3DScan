#include "Pop4StereoCuda.h"

#include <cuda_runtime.h>

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>

namespace revoscan::decoder::cuda_stereo {
namespace {

constexpr uint32_t kPlaneWidth = 640;
constexpr uint32_t kPlaneHeight = 512;
constexpr uint32_t kViewWidth = 320;
constexpr uint32_t kViewHeight = 256;
constexpr uint32_t kPackedHeight = 2560;
constexpr int kPhasePlanes = 4;
constexpr float kModulationMin = 2.5f;
constexpr float kTwoPi = 6.283185307179586f;
constexpr float kTextureGain = 1.8f;
constexpr float kInvalidDisp = -1.0f;

constexpr size_t kPackedSamples =
    static_cast<size_t>(kPlaneWidth) * static_cast<size_t>(kPackedHeight);
constexpr size_t kPlaneSamples =
    static_cast<size_t>(kPlaneWidth) * static_cast<size_t>(kPlaneHeight);
constexpr size_t kViewSamples =
    static_cast<size_t>(kViewWidth) * static_cast<size_t>(kViewHeight);

__device__ __forceinline__ float intensityFromY16(uint16_t sample) {
    return static_cast<float>(sample) * (1.0f / 257.0f);
}

__device__ __forceinline__ float phaseCircDiff(float a, float b) {
    float d = fabsf(a - b);
    const float wrap = kTwoPi - d;
    if (wrap < d) {
        d = wrap;
    }
    return d;
}

__global__ void computePhaseKernel(const uint16_t* packed,
                                   uint32_t x0,
                                   uint32_t y0,
                                   float* phaseOut,
                                   uint8_t* validOut,
                                   float* albedoOut) {
    const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
    const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= kViewWidth || y >= kViewHeight) {
        return;
    }
    const size_t dst = static_cast<size_t>(y) * kViewWidth + x;
    const size_t row = static_cast<size_t>(y0 + y) * kPlaneWidth + x0 + x;

    const float i0 = intensityFromY16(packed[0 * kPlaneSamples + row]);
    const float i1 = intensityFromY16(packed[1 * kPlaneSamples + row]);
    const float i2 = intensityFromY16(packed[2 * kPlaneSamples + row]);
    const float i3 = intensityFromY16(packed[3 * kPlaneSamples + row]);
    albedoOut[dst] = 0.25f * (i0 + i1 + i2 + i3);
    const float dx = i0 - i2;
    const float dy = i1 - i3;
    const float mod = hypotf(dx, dy);
    if (mod < kModulationMin) {
        phaseOut[dst] = 0.0f;
        validOut[dst] = 0;
        return;
    }
    float phase = atan2f(dy, dx);
    if (phase < 0.0f) {
        phase += kTwoPi;
    }
    phaseOut[dst] = phase;
    validOut[dst] = 1;
}

__global__ void fillAlbedoKernel(const uint16_t* packed,
                                 uint32_t xLeft,
                                 uint32_t y0,
                                 uint8_t* rgbOut) {
    const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
    const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= kViewWidth || y >= kViewHeight) {
        return;
    }
    const size_t row = static_cast<size_t>(y0 + y) * kPlaneWidth + xLeft + x;
    const float dc = 0.25f * (intensityFromY16(packed[0 * kPlaneSamples + row]) +
                              intensityFromY16(packed[1 * kPlaneSamples + row]) +
                              intensityFromY16(packed[2 * kPlaneSamples + row]) +
                              intensityFromY16(packed[3 * kPlaneSamples + row]));
    float v = dc * kTextureGain;
    if (v < 0.0f) {
        v = 0.0f;
    }
    if (v > 255.0f) {
        v = 255.0f;
    }
    const uint8_t b = static_cast<uint8_t>(v);
    const size_t dst = (static_cast<size_t>(y) * kViewWidth + x) * 3;
    rgbOut[dst + 0] = b;
    rgbOut[dst + 1] = b;
    rgbOut[dst + 2] = b;
}

// Phase-guided disparity search for one direction. dir=-1: source is LEFT view,
// target pixel = x - d. dir=+1: source is RIGHT view, target pixel = x + d.
// Writes sub-pixel disparity (>=0) or kInvalidDisp when weak/ambiguous.
__global__ void matchKernel(const float* phaseSrc,
                            const uint8_t* validSrc,
                            const float* albedoSrc,
                            const float* phaseTgt,
                            const uint8_t* validTgt,
                            const float* albedoTgt,
                            int dir,
                            int dispMin,
                            int dispMax,
                            float maxCost,
                            float minMargin,
                            float uniqRatio,
                            float intensityWeight,
                            float* dispOut) {
    const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
    const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= kViewWidth || y >= kViewHeight) {
        return;
    }
    const size_t si = static_cast<size_t>(y) * kViewWidth + x;
    dispOut[si] = kInvalidDisp;
    if (!validSrc[si]) {
        return;
    }

    float bestCost = 1.0e30f;
    float bestPhase = 1.0e30f;
    int bestD = -1;
    for (int d = dispMin; d <= dispMax; ++d) {
        const int xt = static_cast<int>(x) + dir * d;
        if (xt < 0 || xt >= static_cast<int>(kViewWidth)) {
            continue;
        }
        const size_t ti = static_cast<size_t>(y) * kViewWidth + static_cast<size_t>(xt);
        if (!validTgt[ti]) {
            continue;
        }
        const float phaseCost = phaseCircDiff(phaseSrc[si], phaseTgt[ti]);
        const float cost = phaseCost +
                           intensityWeight * fabsf(albedoSrc[si] - albedoTgt[ti]);
        if (cost < bestCost) {
            bestCost = cost;
            bestPhase = phaseCost;
            bestD = d;
        }
    }
    // Gate on phase agreement only; intensity ranks / breaks fringe ties.
    if (bestD < 0 || bestPhase > maxCost) {
        return;
    }

    float rivalCost = 1.0e30f;
    for (int d = dispMin; d <= dispMax; ++d) {
        if (abs(d - bestD) < 6) {
            continue;
        }
        const int xt = static_cast<int>(x) + dir * d;
        if (xt < 0 || xt >= static_cast<int>(kViewWidth)) {
            continue;
        }
        const size_t ti = static_cast<size_t>(y) * kViewWidth + static_cast<size_t>(xt);
        if (!validTgt[ti]) {
            continue;
        }
        const float cost = phaseCircDiff(phaseSrc[si], phaseTgt[ti]) +
                           intensityWeight * fabsf(albedoSrc[si] - albedoTgt[ti]);
        if (cost < rivalCost) {
            rivalCost = cost;
        }
    }
    if (rivalCost < 1.0e30f) {
        if ((rivalCost - bestCost) < minMargin) {
            return;
        }
        if (bestCost > uniqRatio * rivalCost) {
            return;
        }
    }

    float costPrev = bestCost + 1.0f; // d - 1
    float costNext = bestCost + 1.0f; // d + 1
    const int xtPrev = static_cast<int>(x) + dir * (bestD - 1);
    const int xtNext = static_cast<int>(x) + dir * (bestD + 1);
    if (xtPrev >= 0 && xtPrev < static_cast<int>(kViewWidth)) {
        const size_t ti = static_cast<size_t>(y) * kViewWidth + static_cast<size_t>(xtPrev);
        if (validTgt[ti]) {
            costPrev = phaseCircDiff(phaseSrc[si], phaseTgt[ti]) +
                       intensityWeight * fabsf(albedoSrc[si] - albedoTgt[ti]);
        }
    }
    if (xtNext >= 0 && xtNext < static_cast<int>(kViewWidth)) {
        const size_t ti = static_cast<size_t>(y) * kViewWidth + static_cast<size_t>(xtNext);
        if (validTgt[ti]) {
            costNext = phaseCircDiff(phaseSrc[si], phaseTgt[ti]) +
                       intensityWeight * fabsf(albedoSrc[si] - albedoTgt[ti]);
        }
    }

    float disparity = static_cast<float>(bestD);
    const float denom = costPrev - 2.0f * bestCost + costNext;
    if (fabsf(denom) > 1e-6f) {
        const float delta = 0.5f * (costPrev - costNext) / denom;
        if (fabsf(delta) < 1.0f) {
            disparity += delta;
        }
    }
    dispOut[si] = disparity;
}

// Invalidate left disparities that disagree with the right→left match.
__global__ void lrConsistencyKernel(const float* dispL,
                                    const float* dispR,
                                    float lrMaxDiff,
                                    float* dispOut) {
    const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
    const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= kViewWidth || y >= kViewHeight) {
        return;
    }
    const size_t li = static_cast<size_t>(y) * kViewWidth + x;
    const float d = dispL[li];
    dispOut[li] = d;
    if (d < 0.0f) {
        return;
    }
    const int xr = static_cast<int>(x) - __float2int_rn(d);
    if (xr < 0 || xr >= static_cast<int>(kViewWidth)) {
        dispOut[li] = kInvalidDisp;
        return;
    }
    const float dr = dispR[static_cast<size_t>(y) * kViewWidth + static_cast<size_t>(xr)];
    if (dr < 0.0f || fabsf(dr - d) > lrMaxDiff) {
        dispOut[li] = kInvalidDisp;
    }
}

// 3×3 median of valid disparities (invalid neighbours ignored).
__global__ void medianKernel(const float* in, float* out) {
    const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
    const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= kViewWidth || y >= kViewHeight) {
        return;
    }
    const size_t ci = static_cast<size_t>(y) * kViewWidth + x;
    out[ci] = kInvalidDisp;
    if (in[ci] < 0.0f) {
        return;
    }
    float w[9];
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
            const float v = in[static_cast<size_t>(yy) * kViewWidth + static_cast<size_t>(xx)];
            if (v >= 0.0f) {
                w[n++] = v;
            }
        }
    }
    if (n == 0) {
        return;
    }
    for (int i = 1; i < n; ++i) { // insertion sort (n <= 9)
        const float key = w[i];
        int j = i - 1;
        while (j >= 0 && w[j] > key) {
            w[j + 1] = w[j];
            --j;
        }
        w[j + 1] = key;
    }
    out[ci] = w[n / 2];
}

// Local-support speckle rejection over a 5×5 window.
__global__ void speckleKernel(const float* in, float* out, float maxDiff,
                              int minSupport) {
    const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
    const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= kViewWidth || y >= kViewHeight) {
        return;
    }
    const size_t ci = static_cast<size_t>(y) * kViewWidth + x;
    const float c = in[ci];
    out[ci] = c;
    if (c < 0.0f) {
        return;
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
            const float v = in[static_cast<size_t>(yy) * kViewWidth + static_cast<size_t>(xx)];
            if (v >= 0.0f && fabsf(v - c) <= maxDiff) {
                ++support;
            }
        }
    }
    if (support < minSupport) {
        out[ci] = kInvalidDisp;
    }
}

// Convert the filtered disparity field to metric depth (mm).
__global__ void depthKernel(const float* disp,
                            float q23,
                            float q32,
                            float q33,
                            float depthScale,
                            float depthMinMm,
                            float depthMaxMm,
                            uint16_t* depthOut,
                            unsigned long long* validCount) {
    const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
    const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= kViewWidth || y >= kViewHeight) {
        return;
    }
    const size_t li = static_cast<size_t>(y) * kViewWidth + x;
    depthOut[li] = 0;
    const float disparity = disp[li];
    if (disparity < 0.0f) {
        return;
    }
    const float qDenom = disparity * q32 + q33;
    if (fabsf(qDenom) < 1e-9f) {
        return;
    }
    float z = (q23 / qDenom) * depthScale;
    if (!isfinite(z) || z < depthMinMm || z > depthMaxMm) {
        return;
    }
    if (z < 1.0f) {
        z = 1.0f;
    }
    if (z > 65535.0f) {
        z = 65535.0f;
    }
    depthOut[li] = static_cast<uint16_t>(z);
    atomicAdd(validCount, 1ULL);
}

struct DeviceBuffers {
    uint16_t* packed = nullptr;
    float* phaseL = nullptr;
    float* phaseR = nullptr;
    float* albedoL = nullptr;
    float* albedoR = nullptr;
    uint8_t* validL = nullptr;
    uint8_t* validR = nullptr;
    float* dispL = nullptr;
    float* dispR = nullptr;
    float* dispScratch = nullptr;
    uint16_t* depth = nullptr;
    uint8_t* rgb = nullptr;
    unsigned long long* validCount = nullptr;
    bool ready = false;

    bool ensure() {
        if (ready) {
            return true;
        }
        auto fail = [this]() {
            freeAll();
            return false;
        };
        if (cudaMalloc(&packed, kPackedSamples * sizeof(uint16_t)) != cudaSuccess) {
            return fail();
        }
        if (cudaMalloc(&phaseL, kViewSamples * sizeof(float)) != cudaSuccess) {
            return fail();
        }
        if (cudaMalloc(&phaseR, kViewSamples * sizeof(float)) != cudaSuccess) {
            return fail();
        }
        if (cudaMalloc(&albedoL, kViewSamples * sizeof(float)) != cudaSuccess) {
            return fail();
        }
        if (cudaMalloc(&albedoR, kViewSamples * sizeof(float)) != cudaSuccess) {
            return fail();
        }
        if (cudaMalloc(&validL, kViewSamples * sizeof(uint8_t)) != cudaSuccess) {
            return fail();
        }
        if (cudaMalloc(&validR, kViewSamples * sizeof(uint8_t)) != cudaSuccess) {
            return fail();
        }
        if (cudaMalloc(&dispL, kViewSamples * sizeof(float)) != cudaSuccess) {
            return fail();
        }
        if (cudaMalloc(&dispR, kViewSamples * sizeof(float)) != cudaSuccess) {
            return fail();
        }
        if (cudaMalloc(&dispScratch, kViewSamples * sizeof(float)) != cudaSuccess) {
            return fail();
        }
        if (cudaMalloc(&depth, kViewSamples * sizeof(uint16_t)) != cudaSuccess) {
            return fail();
        }
        if (cudaMalloc(&rgb, kViewSamples * 3 * sizeof(uint8_t)) != cudaSuccess) {
            return fail();
        }
        if (cudaMalloc(&validCount, sizeof(unsigned long long)) != cudaSuccess) {
            return fail();
        }
        ready = true;
        return true;
    }

    void freeAll() {
        // cudaFree(nullptr) is a no-op; always null after free to avoid double-free.
        cudaFree(packed);
        cudaFree(phaseL);
        cudaFree(phaseR);
        cudaFree(albedoL);
        cudaFree(albedoR);
        cudaFree(validL);
        cudaFree(validR);
        cudaFree(dispL);
        cudaFree(dispR);
        cudaFree(dispScratch);
        cudaFree(depth);
        cudaFree(rgb);
        cudaFree(validCount);
        packed = nullptr;
        phaseL = phaseR = nullptr;
        albedoL = albedoR = nullptr;
        validL = validR = nullptr;
        dispL = dispR = dispScratch = nullptr;
        depth = nullptr;
        rgb = nullptr;
        validCount = nullptr;
        ready = false;
    }
};

DeviceBuffers& buffers() {
    static DeviceBuffers b;
    return b;
}

std::mutex& mutex() {
    static std::mutex m;
    return m;
}

// Sticky disable after a fatal device error so we stop thrashing a broken GPU
// context and let the CPU fallback own decoding for the rest of the process.
std::atomic<bool>& stickyDisabled() {
    static std::atomic<bool> disabled{false};
    return disabled;
}

enum class AvailState : int { Unknown = 0, Yes = 1, No = 2 };
std::atomic<int>& availCache() {
    static std::atomic<int> cache{static_cast<int>(AvailState::Unknown)};
    return cache;
}

std::string cudaErrorString(cudaError_t err) {
    return std::string("CUDA: ") + cudaGetErrorString(err);
}

bool paramsInBounds(const DecodeParams& params) {
    // Stereo tiles are 320×256 within each 640×512 plane.
    if (params.xLeft > kPlaneWidth - kViewWidth ||
        params.xRight > kPlaneWidth - kViewWidth) {
        return false;
    }
    if (params.y0 > kPlaneHeight - kViewHeight) {
        return false;
    }
    (void)kPhasePlanes;
    return true;
}

void markDeviceBroken() {
    stickyDisabled().store(true, std::memory_order_release);
    availCache().store(static_cast<int>(AvailState::No), std::memory_order_release);
    buffers().freeAll();
    // Drop sticky CUDA error so later CPU-only runs don't trip on old state.
    (void)cudaGetLastError();
    (void)cudaDeviceSynchronize();
    (void)cudaGetLastError();
}

} // namespace

bool isBuilt() {
    return true;
}

bool available() {
    if (stickyDisabled().load(std::memory_order_acquire)) {
        return false;
    }
    if (!enabledByEnv()) {
        return false;
    }
    const int cached = availCache().load(std::memory_order_acquire);
    if (cached == static_cast<int>(AvailState::Yes)) {
        return true;
    }
    if (cached == static_cast<int>(AvailState::No)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex());
    if (stickyDisabled().load(std::memory_order_relaxed)) {
        return false;
    }
    const int again = availCache().load(std::memory_order_relaxed);
    if (again == static_cast<int>(AvailState::Yes)) {
        return true;
    }
    if (again == static_cast<int>(AvailState::No)) {
        return false;
    }

    int count = 0;
    const cudaError_t err = cudaGetDeviceCount(&count);
    const bool ok = (err == cudaSuccess && count > 0);
    availCache().store(static_cast<int>(ok ? AvailState::Yes : AvailState::No),
                       std::memory_order_release);
    return ok;
}

bool enabledByEnv() {
    if (const char* value = std::getenv("ALL3DSCAN_POP4_CUDA")) {
        if (value[0] == '0' || value[0] == 'n' || value[0] == 'N' ||
            value[0] == 'f' || value[0] == 'F') {
            return false;
        }
    }
    return true;
}

uint32_t decodePacked(const uint16_t* packedY16,
                      const DecodeParams& params,
                      std::vector<uint16_t>& depthMm,
                      std::vector<uint8_t>& rgb,
                      std::string* errorOut) {
    auto fail = [&](const std::string& msg) -> uint32_t {
        if (errorOut) {
            *errorOut = msg;
        }
        depthMm.clear();
        rgb.clear();
        return 0;
    };

    if (!packedY16) {
        return fail("CUDA decode: null packed buffer");
    }
    if (!paramsInBounds(params)) {
        return fail("CUDA decode: stereo crop out of bounds");
    }
    if (stickyDisabled().load(std::memory_order_acquire)) {
        return fail("CUDA decode: disabled after prior device error");
    }
    if (!available()) {
        return fail("CUDA decode: no device");
    }

    std::lock_guard<std::mutex> lock(mutex());
    if (stickyDisabled().load(std::memory_order_relaxed)) {
        return fail("CUDA decode: disabled after prior device error");
    }

    auto& buf = buffers();
    if (!buf.ensure()) {
        markDeviceBroken();
        return fail("CUDA decode: device buffer alloc failed");
    }

    auto failDevice = [&](cudaError_t err) -> uint32_t {
        markDeviceBroken();
        return fail(cudaErrorString(err));
    };

    // Clear any stale sticky error from a previous caller before launching work.
    (void)cudaGetLastError();

    cudaError_t err =
        cudaMemcpy(buf.packed, packedY16, kPackedSamples * sizeof(uint16_t),
                   cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        return failDevice(err);
    }

    const dim3 block(16, 16);
    const dim3 grid((kViewWidth + block.x - 1) / block.x,
                    (kViewHeight + block.y - 1) / block.y);

    computePhaseKernel<<<grid, block>>>(buf.packed, params.xLeft, params.y0,
                                        buf.phaseL, buf.validL, buf.albedoL);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return failDevice(err);
    }
    computePhaseKernel<<<grid, block>>>(buf.packed, params.xRight, params.y0,
                                        buf.phaseR, buf.validR, buf.albedoR);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return failDevice(err);
    }
    fillAlbedoKernel<<<grid, block>>>(buf.packed, params.xLeft, params.y0, buf.rgb);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return failDevice(err);
    }

    err = cudaMemset(buf.validCount, 0, sizeof(unsigned long long));
    if (err != cudaSuccess) {
        return failDevice(err);
    }

    // Disparity field pipeline: match both directions, enforce L↔R consistency,
    // median + speckle clean, then convert to depth. Mirrors decodeCpu().
    matchKernel<<<grid, block>>>(buf.phaseL, buf.validL, buf.albedoL, buf.phaseR,
                                 buf.validR, buf.albedoR, /*dir=*/-1,
                                 params.dispMin, params.dispMax,
                                 params.maxPhaseCost, params.minCostMargin,
                                 params.uniqRatio, params.intensityWeight,
                                 buf.dispL);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return failDevice(err);
    }

    float* cur = buf.dispL;
    float* scratch = buf.dispScratch;
    if (params.lrCheck) {
        matchKernel<<<grid, block>>>(buf.phaseR, buf.validR, buf.albedoR, buf.phaseL,
                                     buf.validL, buf.albedoL, /*dir=*/+1,
                                     params.dispMin, params.dispMax,
                                     params.maxPhaseCost, params.minCostMargin,
                                     params.uniqRatio, params.intensityWeight,
                                     buf.dispR);
        err = cudaGetLastError();
        if (err != cudaSuccess) {
            return failDevice(err);
        }
        lrConsistencyKernel<<<grid, block>>>(cur, buf.dispR, params.lrMaxDiff,
                                             scratch);
        err = cudaGetLastError();
        if (err != cudaSuccess) {
            return failDevice(err);
        }
        float* t = cur;
        cur = scratch;
        scratch = t;
    }
    if (params.doMedian) {
        medianKernel<<<grid, block>>>(cur, scratch);
        err = cudaGetLastError();
        if (err != cudaSuccess) {
            return failDevice(err);
        }
        float* t = cur;
        cur = scratch;
        scratch = t;
    }
    if (params.doSpeckle) {
        speckleKernel<<<grid, block>>>(cur, scratch, params.speckleMaxDiff,
                                       params.speckleMinSupport);
        err = cudaGetLastError();
        if (err != cudaSuccess) {
            return failDevice(err);
        }
        float* t = cur;
        cur = scratch;
        scratch = t;
    }

    depthKernel<<<grid, block>>>(cur, params.q23, params.q32, params.q33,
                                 params.depthScale, params.depthMinMm,
                                 params.depthMaxMm, buf.depth, buf.validCount);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return failDevice(err);
    }
    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        return failDevice(err);
    }

    depthMm.assign(kViewSamples, 0);
    rgb.assign(kViewSamples * 3, 0);
    err = cudaMemcpy(depthMm.data(), buf.depth, kViewSamples * sizeof(uint16_t),
                     cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        depthMm.clear();
        rgb.clear();
        return failDevice(err);
    }
    err = cudaMemcpy(rgb.data(), buf.rgb, kViewSamples * 3 * sizeof(uint8_t),
                     cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        depthMm.clear();
        rgb.clear();
        return failDevice(err);
    }

    unsigned long long valid = 0;
    err = cudaMemcpy(&valid, buf.validCount, sizeof(unsigned long long),
                     cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        depthMm.clear();
        rgb.clear();
        return failDevice(err);
    }
    return static_cast<uint32_t>(valid);
}

} // namespace revoscan::decoder::cuda_stereo
