// GPU port of Pop4DepthDecoder's matchCpu(): speckle stereo with census +
// 4-path semi-global matching. Every step mirrors the CPU code (same blur
// weights, census bit order, integer SGM recurrence and gates) so the two
// backends produce matching disparity maps; tests compare them directly.
// Build with --fmad=false (see CMakeLists.txt) to keep float steps bit-equal.

#include "Pop4StereoCuda.h"

#include <cuda_runtime.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <mutex>

namespace revoscan::decoder::cuda_stereo {
namespace {

constexpr int kW = 320;
constexpr int kH = 512;
constexpr int kPixels = kW * kH;
constexpr int kMaxDisp = 192; // >= CPU kDispSearchCeil (160) band width
constexpr int kCensusRadius = 3;
constexpr uint8_t kCensusInvalidCost = 48;

__constant__ float kBlur[5] = {1 / 16.f, 4 / 16.f, 6 / 16.f, 4 / 16.f, 1 / 16.f};

// --- 1. row alignment ------------------------------------------------------

// Bilinear sample of a raw Y16 image (as /257 floats), 0 outside; the same
// expression as the CPU sampleBilinear so results stay bit-equal.
__device__ float sampleRaw(const uint16_t* __restrict__ img, float sx, float sy) {
    if (!(sx >= 0.0f && sy >= 0.0f && sx <= static_cast<float>(kW - 1) &&
          sy <= static_cast<float>(kH - 1))) {
        return 0.0f;
    }
    const int x0 = static_cast<int>(sx);
    const int y0 = static_cast<int>(sy);
    const int x1 = min(x0 + 1, kW - 1);
    const int y1 = min(y0 + 1, kH - 1);
    const float ax = sx - static_cast<float>(x0);
    const float ay = sy - static_cast<float>(y0);
    const float p00 = static_cast<float>(img[y0 * kW + x0]) / 257.0f;
    const float p10 = static_cast<float>(img[y0 * kW + x1]) / 257.0f;
    const float p01 = static_cast<float>(img[y1 * kW + x0]) / 257.0f;
    const float p11 = static_cast<float>(img[y1 * kW + x1]) / 257.0f;
    const float top = (1.0f - ax) * p00 + ax * p10;
    const float bot = (1.0f - ax) * p01 + ax * p11;
    return (1.0f - ay) * top + ay * bot;
}

__global__ void prepareKernel(const uint16_t* __restrict__ left,
                              const uint16_t* __restrict__ right,
                              const float* __restrict__ mapLx,
                              const float* __restrict__ mapLy,
                              const float* __restrict__ mapRx,
                              const float* __restrict__ mapRy,
                              float* __restrict__ outL,
                              float* __restrict__ outR) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= kW || y >= kH) {
        return;
    }
    const int p = y * kW + x;
    outL[p] = sampleRaw(left, mapLx[p], mapLy[p]);
    outR[p] = sampleRaw(right, mapRx[p], mapRy[p]);
}

__global__ void blurRowsKernel(const float* __restrict__ in, float* __restrict__ out) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= kW || y >= kH) {
        return;
    }
    float s = 0.0f;
    for (int i = -2; i <= 2; ++i) {
        const int xx = min(max(x + i, 0), kW - 1);
        s += kBlur[i + 2] * in[y * kW + xx];
    }
    out[y * kW + x] = s;
}

__global__ void blurColsKernel(const float* __restrict__ in, float* __restrict__ out) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= kW || y >= kH) {
        return;
    }
    float s = 0.0f;
    for (int i = -2; i <= 2; ++i) {
        const int yy = min(max(y + i, 0), kH - 1);
        s += kBlur[i + 2] * in[yy * kW + x];
    }
    out[y * kW + x] = s;
}

// --- 2. census + cost volume ----------------------------------------------

__global__ void censusKernel(const float* __restrict__ img, unsigned long long* __restrict__ out) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= kW || y >= kH) {
        return;
    }
    const float c = img[y * kW + x];
    unsigned long long bits = 0;
    for (int dy = -kCensusRadius; dy <= kCensusRadius; ++dy) {
        const int yy = min(max(y + dy, 0), kH - 1);
        for (int dx = -kCensusRadius; dx <= kCensusRadius; ++dx) {
            if (dx == 0 && dy == 0) {
                continue;
            }
            const int xx = min(max(x + dx, 0), kW - 1);
            bits = (bits << 1) | (img[yy * kW + xx] < c ? 1ull : 0ull);
        }
    }
    out[y * kW + x] = bits;
}

// One thread per (pixel, disparity); cost layout [pixel][d - dispMin].
__global__ void costKernel(const unsigned long long* __restrict__ cL,
                           const unsigned long long* __restrict__ cR,
                           int dispMin,
                           int numDisp,
                           uint8_t* __restrict__ cost,
                           uint16_t* __restrict__ sum) {
    const long long i = static_cast<long long>(blockIdx.x) * blockDim.x + threadIdx.x;
    if (i >= static_cast<long long>(kPixels) * numDisp) {
        return;
    }
    const int p = static_cast<int>(i / numDisp);
    const int di = static_cast<int>(i % numDisp);
    const int x = p % kW;
    const int d = dispMin + di;
    cost[i] = x - d < 0 ? kCensusInvalidCost
                        : static_cast<uint8_t>(__popcll(cL[p] ^ cR[p - d]));
    sum[i] = 0;
}

// --- 3. semi-global aggregation -------------------------------------------

// One warp per scan line; lane l holds disparities [l*K, l*K + K) in registers
// and swaps edge values with its neighbours by shuffle, so a pixel step needs
// no block barriers. The integer recurrence is identical to the CPU
// aggregatePath(), so sums match exactly.
template <int K>
__global__ void aggregateKernel(const uint8_t* __restrict__ cost,
                                uint16_t* __restrict__ sum,
                                int numDisp,
                                int lines,
                                int lineStride,
                                int first,
                                int step,
                                int length,
                                int p1,
                                int p2) {
    constexpr int kBig = 1 << 20;
    constexpr unsigned kAll = 0xffffffffu;
    const int lane = threadIdx.x & 31;
    const int line = blockIdx.x * (blockDim.x >> 5) + (threadIdx.x >> 5);
    if (line >= lines) {
        return; // whole warp exits together
    }
    const int d0 = lane * K;
    int prev[K];
    int minPrev = 0;
    long long p = static_cast<long long>(first) + static_cast<long long>(line) * lineStride;
    for (int k = 0; k < length; ++k, p += step) {
        const long long base = p * numDisp;
        // Neighbour disparities across lane boundaries (previous step's values).
        const int fromLeft = __shfl_up_sync(kAll, prev[K - 1], 1);
        const int fromRight = __shfl_down_sync(kAll, prev[0], 1);
        int cur[K];
        int m = kBig;
#pragma unroll
        for (int j = 0; j < K; ++j) {
            const int d = d0 + j;
            if (d >= numDisp) {
                cur[j] = kBig;
                continue;
            }
            const int c = cost[base + d];
            int l;
            if (k == 0) {
                l = c;
            } else {
                int v = prev[j];
                if (d > 0) {
                    v = min(v, (j > 0 ? prev[j - 1] : fromLeft) + p1);
                }
                if (d + 1 < numDisp) {
                    v = min(v, (j + 1 < K ? prev[j + 1] : fromRight) + p1);
                }
                v = min(v, minPrev + p2);
                l = static_cast<uint16_t>(c + v - minPrev);
            }
            sum[base + d] = static_cast<uint16_t>(sum[base + d] + l);
            cur[j] = l;
            m = min(m, l);
        }
#pragma unroll
        for (int off = 16; off > 0; off >>= 1) {
            m = min(m, __shfl_xor_sync(kAll, m, off));
        }
#pragma unroll
        for (int j = 0; j < K; ++j) {
            prev[j] = cur[j];
        }
        minPrev = m;
    }
}

template <int K>
void launchAggregate(const uint8_t* cost, uint16_t* sum, int numDisp, int lines,
                     int lineStride, int first, int step, int length, int p1, int p2) {
    constexpr int kWarpsPerBlock = 4;
    const int blocks = (lines + kWarpsPerBlock - 1) / kWarpsPerBlock;
    aggregateKernel<K><<<blocks, 32 * kWarpsPerBlock>>>(cost, sum, numDisp, lines, lineStride,
                                                        first, step, length, p1, p2);
}

void aggregate(const uint8_t* cost, uint16_t* sum, int numDisp, int lines, int lineStride,
               int first, int step, int length, int p1, int p2) {
    switch ((numDisp + 31) / 32) {
    case 1: launchAggregate<1>(cost, sum, numDisp, lines, lineStride, first, step, length, p1, p2); break;
    case 2: launchAggregate<2>(cost, sum, numDisp, lines, lineStride, first, step, length, p1, p2); break;
    case 3: launchAggregate<3>(cost, sum, numDisp, lines, lineStride, first, step, length, p1, p2); break;
    case 4: launchAggregate<4>(cost, sum, numDisp, lines, lineStride, first, step, length, p1, p2); break;
    case 5: launchAggregate<5>(cost, sum, numDisp, lines, lineStride, first, step, length, p1, p2); break;
    default: launchAggregate<6>(cost, sum, numDisp, lines, lineStride, first, step, length, p1, p2); break;
    }
}

// --- 4. winner-take-all ----------------------------------------------------

__global__ void rightDisparityKernel(const uint16_t* __restrict__ sum,
                                     int dispMin,
                                     int numDisp,
                                     int* __restrict__ dispRight) {
    const int xr = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (xr >= kW || y >= kH) {
        return;
    }
    int bestD = -1;
    unsigned best = 0xffffffffu;
    for (int di = 0; di < numDisp; ++di) {
        const int xl = xr + dispMin + di;
        if (xl >= kW) {
            break;
        }
        const unsigned s = sum[static_cast<long long>(y * kW + xl) * numDisp + di];
        if (s < best) {
            best = s;
            bestD = di;
        }
    }
    dispRight[y * kW + xr] = bestD < 0 ? -1 : dispMin + bestD;
}

// Local std over a 9×9 window clamped at the borders (CPU: localStd(L, 4)).
__device__ float localStd9(const float* img, int x, int y) {
    const int y0 = max(0, y - 4);
    const int y1 = min(kH, y + 5);
    const int x0 = max(0, x - 4);
    const int x1 = min(kW, x + 5);
    double s1 = 0.0;
    double s2 = 0.0;
    for (int yy = y0; yy < y1; ++yy) {
        for (int xx = x0; xx < x1; ++xx) {
            const double v = img[yy * kW + xx];
            s1 += v;
            s2 += v * v;
        }
    }
    const double n = static_cast<double>((y1 - y0) * (x1 - x0));
    const double mean = s1 / n;
    return static_cast<float>(sqrt(fmax(0.0, s2 / n - mean * mean)));
}

__global__ void winnerKernel(const uint16_t* __restrict__ sum,
                             const int* __restrict__ dispRight,
                             const float* __restrict__ blurredL,
                             SgmParams prm,
                             float* __restrict__ disp) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= kW || y >= kH) {
        return;
    }
    const int p = y * kW + x;
    disp[p] = -1.0f;
    if (x < prm.dispMin || localStd9(blurredL, x, y) < prm.textureMin) {
        return;
    }
    const uint16_t* s = sum + static_cast<long long>(p) * prm.numDisp;
    const int validDisp = min(prm.numDisp, x - prm.dispMin + 1);
    int bestD = 0;
    for (int di = 1; di < validDisp; ++di) {
        if (s[di] < s[bestD]) {
            bestD = di;
        }
    }
    unsigned rival = 0xffffffffu;
    for (int di = 0; di < validDisp; ++di) {
        if (abs(di - bestD) > 1) {
            rival = min(rival, static_cast<unsigned>(s[di]));
        }
    }
    if (rival != 0xffffffffu &&
        static_cast<float>(s[bestD]) * (1.0f + prm.uniqueness) >= static_cast<float>(rival)) {
        return;
    }
    if (!(bestD > 0 && bestD + 1 < validDisp)) {
        return; // winner at the band edge
    }
    float d = static_cast<float>(prm.dispMin + bestD);
    const float cp = s[bestD - 1];
    const float cb = s[bestD];
    const float cn = s[bestD + 1];
    const float den = cp - 2.0f * cb + cn;
    if (den > 1e-6f) {
        d += fminf(fmaxf(0.5f * (cp - cn) / den, -0.5f), 0.5f);
    }
    const int xr = x - static_cast<int>(lroundf(d));
    if (xr < 0) {
        return;
    }
    const int dr = dispRight[y * kW + xr];
    if (dr < 0 || fabsf(static_cast<float>(dr) - d) > prm.lrMaxDiff + 0.5f) {
        return;
    }
    disp[p] = d;
}

// --- device state ------------------------------------------------------------

struct DeviceBuffers {
    uint16_t* left = nullptr;
    uint16_t* right = nullptr;
    float* mapLx = nullptr;
    float* mapLy = nullptr;
    float* mapRx = nullptr;
    float* mapRy = nullptr;
    float* rawL = nullptr;
    float* rectR = nullptr;
    float* tmp = nullptr;
    float* L = nullptr;
    float* R = nullptr;
    unsigned long long* cL = nullptr;
    unsigned long long* cR = nullptr;
    uint8_t* cost = nullptr;
    uint16_t* sum = nullptr;
    int* dispRight = nullptr;
    float* disp = nullptr;
    bool ready = false;

    bool ensure() {
        if (ready) {
            return true;
        }
        const size_t px = kPixels;
        bool ok = cudaMalloc(&left, px * sizeof(uint16_t)) == cudaSuccess &&
                  cudaMalloc(&right, px * sizeof(uint16_t)) == cudaSuccess &&
                  cudaMalloc(&mapLx, px * sizeof(float)) == cudaSuccess &&
                  cudaMalloc(&mapLy, px * sizeof(float)) == cudaSuccess &&
                  cudaMalloc(&mapRx, px * sizeof(float)) == cudaSuccess &&
                  cudaMalloc(&mapRy, px * sizeof(float)) == cudaSuccess &&
                  cudaMalloc(&rawL, px * sizeof(float)) == cudaSuccess &&
                  cudaMalloc(&rectR, px * sizeof(float)) == cudaSuccess &&
                  cudaMalloc(&tmp, px * sizeof(float)) == cudaSuccess &&
                  cudaMalloc(&L, px * sizeof(float)) == cudaSuccess &&
                  cudaMalloc(&R, px * sizeof(float)) == cudaSuccess &&
                  cudaMalloc(&cL, px * sizeof(unsigned long long)) == cudaSuccess &&
                  cudaMalloc(&cR, px * sizeof(unsigned long long)) == cudaSuccess &&
                  cudaMalloc(&cost, px * kMaxDisp) == cudaSuccess &&
                  cudaMalloc(&sum, px * kMaxDisp * sizeof(uint16_t)) == cudaSuccess &&
                  cudaMalloc(&dispRight, px * sizeof(int)) == cudaSuccess &&
                  cudaMalloc(&disp, px * sizeof(float)) == cudaSuccess;
        if (!ok) {
            freeAll();
            return false;
        }
        ready = true;
        return true;
    }

    void freeAll() {
        for (void* p : {static_cast<void*>(left), static_cast<void*>(right),
                        static_cast<void*>(mapLx), static_cast<void*>(mapLy),
                        static_cast<void*>(mapRx), static_cast<void*>(mapRy),
                        static_cast<void*>(rawL),
                        static_cast<void*>(rectR), static_cast<void*>(tmp),
                        static_cast<void*>(L), static_cast<void*>(R),
                        static_cast<void*>(cL), static_cast<void*>(cR),
                        static_cast<void*>(cost), static_cast<void*>(sum),
                        static_cast<void*>(dispRight), static_cast<void*>(disp)}) {
            if (p) {
                cudaFree(p);
            }
        }
        *this = DeviceBuffers{};
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

void markDeviceBroken() {
    stickyDisabled().store(true, std::memory_order_release);
    availCache().store(static_cast<int>(AvailState::No), std::memory_order_release);
    buffers().freeAll();
    (void)cudaGetLastError();
    (void)cudaDeviceSynchronize();
    (void)cudaGetLastError();
}

} // namespace

bool isBuilt() {
    return true;
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

bool available() {
    if (stickyDisabled().load(std::memory_order_acquire) || !enabledByEnv()) {
        return false;
    }
    const int cached = availCache().load(std::memory_order_acquire);
    if (cached != static_cast<int>(AvailState::Unknown)) {
        return cached == static_cast<int>(AvailState::Yes);
    }
    std::lock_guard<std::mutex> lock(mutex());
    const int again = availCache().load(std::memory_order_relaxed);
    if (again != static_cast<int>(AvailState::Unknown)) {
        return again == static_cast<int>(AvailState::Yes);
    }
    int count = 0;
    const bool ok = cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
    availCache().store(static_cast<int>(ok ? AvailState::Yes : AvailState::No),
                       std::memory_order_release);
    return ok;
}

bool computeDisparity(const uint16_t* leftY16,
                      const uint16_t* rightY16,
                      const float* mapLx,
                      const float* mapLy,
                      const float* mapRx,
                      const float* mapRy,
                      const SgmParams& params,
                      std::vector<float>& disparity,
                      std::string* errorOut) {
    auto fail = [&](const std::string& msg) {
        if (errorOut) {
            *errorOut = msg;
        }
        disparity.clear();
        return false;
    };
    if (!leftY16 || !rightY16 || !mapLx || !mapLy || !mapRx || !mapRy) {
        return fail("CUDA decode: null input");
    }
    if (params.numDisp < 3 || params.numDisp > kMaxDisp || params.dispMin < 0) {
        return fail("CUDA decode: disparity band out of range");
    }
    if (!available()) {
        return fail("CUDA decode: no usable device");
    }

    std::lock_guard<std::mutex> lock(mutex());
    if (stickyDisabled().load(std::memory_order_relaxed)) {
        return fail("CUDA decode: disabled after prior device error");
    }
    auto& b = buffers();
    if (!b.ensure()) {
        markDeviceBroken();
        return fail("CUDA decode: device buffer allocation failed");
    }
    auto check = [&](cudaError_t err) {
        if (err == cudaSuccess) {
            return true;
        }
        markDeviceBroken();
        fail(std::string("CUDA: ") + cudaGetErrorString(err));
        return false;
    };
    (void)cudaGetLastError();

    const size_t px = kPixels;
    if (!check(cudaMemcpy(b.left, leftY16, px * sizeof(uint16_t), cudaMemcpyHostToDevice)) ||
        !check(cudaMemcpy(b.right, rightY16, px * sizeof(uint16_t), cudaMemcpyHostToDevice)) ||
        // Four 640 KB maps per frame; cheaper than tracking map changes.
        !check(cudaMemcpy(b.mapLx, mapLx, px * sizeof(float), cudaMemcpyHostToDevice)) ||
        !check(cudaMemcpy(b.mapLy, mapLy, px * sizeof(float), cudaMemcpyHostToDevice)) ||
        !check(cudaMemcpy(b.mapRx, mapRx, px * sizeof(float), cudaMemcpyHostToDevice)) ||
        !check(cudaMemcpy(b.mapRy, mapRy, px * sizeof(float), cudaMemcpyHostToDevice))) {
        return false;
    }

    // ALL3DSCAN_POP4_DEBUG=1: per-stage GPU timing via events.
    const char* dbg = std::getenv("ALL3DSCAN_POP4_DEBUG");
    const bool timing = dbg && dbg[0] == '1';
    cudaEvent_t ev[6];
    if (timing) {
        for (auto& e : ev) {
            cudaEventCreate(&e);
        }
    }
    auto mark = [&](int i) {
        if (timing) {
            cudaEventRecord(ev[i]);
        }
    };
    mark(0);
    const dim3 block(32, 8);
    const dim3 grid((kW + block.x - 1) / block.x, (kH + block.y - 1) / block.y);
    prepareKernel<<<grid, block>>>(b.left, b.right, b.mapLx, b.mapLy, b.mapRx, b.mapRy, b.rawL,
                                   b.rectR);
    blurRowsKernel<<<grid, block>>>(b.rawL, b.tmp);
    blurColsKernel<<<grid, block>>>(b.tmp, b.L);
    blurRowsKernel<<<grid, block>>>(b.rectR, b.tmp);
    blurColsKernel<<<grid, block>>>(b.tmp, b.R);
    censusKernel<<<grid, block>>>(b.L, b.cL);
    censusKernel<<<grid, block>>>(b.R, b.cR);
    mark(1);

    const long long volume = static_cast<long long>(px) * params.numDisp;
    costKernel<<<static_cast<unsigned>((volume + 255) / 256), 256>>>(
        b.cL, b.cR, params.dispMin, params.numDisp, b.cost, b.sum);
    mark(2);

    // left→right, right→left, top→bottom, bottom→top (same order as CPU).
    aggregate(b.cost, b.sum, params.numDisp, kH, kW, 0, 1, kW, params.p1, params.p2);
    aggregate(b.cost, b.sum, params.numDisp, kH, kW, kW - 1, -1, kW, params.p1, params.p2);
    aggregate(b.cost, b.sum, params.numDisp, kW, 1, 0, kW, kH, params.p1, params.p2);
    aggregate(b.cost, b.sum, params.numDisp, kW, 1, (kH - 1) * kW, -kW, kH, params.p1, params.p2);
    mark(3);
    rightDisparityKernel<<<grid, block>>>(b.sum, params.dispMin, params.numDisp, b.dispRight);
    winnerKernel<<<grid, block>>>(b.sum, b.dispRight, b.L, params, b.disp);
    mark(4);
    if (!check(cudaGetLastError())) {
        return false;
    }

    disparity.resize(px);
    const bool ok = check(cudaMemcpy(disparity.data(), b.disp, px * sizeof(float),
                                     cudaMemcpyDeviceToHost));
    if (timing) {
        mark(5);
        cudaEventSynchronize(ev[5]);
        float t[5];
        for (int i = 0; i < 5; ++i) {
            cudaEventElapsedTime(&t[i], ev[i], ev[i + 1]);
        }
        std::fprintf(stderr,
                     "[POP4] CUDA ms: prep+census %.2f cost %.2f sgm %.2f wta %.2f download %.2f\n",
                     t[0], t[1], t[2], t[3], t[4]);
        for (auto& e : ev) {
            cudaEventDestroy(e);
        }
    }
    return ok;
}

} // namespace revoscan::decoder::cuda_stereo
