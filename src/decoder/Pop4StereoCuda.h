#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace revoscan::decoder::cuda_stereo {

// True when this binary was compiled with ALL3DSCAN_HAS_CUDA.
bool isBuilt();

// True when a CUDA device is present and usable. Result is cached; sticky
// device failures disable CUDA for the rest of the process (CPU fallback).
bool available();

// ALL3DSCAN_POP4_CUDA=0/false disables GPU decode (CPU fallback).
bool enabledByEnv();

struct DecodeParams {
    uint32_t xLeft = 0;
    uint32_t xRight = 320;
    uint32_t y0 = 0;
    float q23 = 0.0f;
    float q32 = 1.0f;
    float q33 = 0.0f;
    float depthScale = 1.0f;
    float depthMinMm = 210.0f;
    float depthMaxMm = 500.0f;

    // Disparity search band (pixels), derived from [depthMinMm, depthMaxMm]
    // via the Q matrix by the caller (Pop4DepthDecoder::computeDispRange).
    // Defaults match the historical fixed 220-650mm-ish band.
    int dispMin = 8;
    int dispMax = 120;

    // Matching-quality tuning (mirror of the CPU path). Wrapped four-step phase
    // is periodic, so single-shot winner-take-all disparities collapse into a
    // narrow band (fronto-parallel slab). These knobs reject the ambiguous /
    // wrong-fringe matches so the real surface relief survives.
    // Re-balanced 2026-07 to match the CPU path: the first stricter set rejected
    // nearly all real-device pixels (decode fell back to raw Y16). Looser gates
    // keep enough matches to clear kMinValidDepth while margin + ratio + speckle
    // still kill the periodic constant-fringe slab.
    float maxPhaseCost = 0.35f;   // reject weak phase agreement
    float minCostMargin = 0.010f; // best must beat the next fringe by this
    float uniqRatio = 0.92f;      // best < uniqRatio * secondBest (ratio test)
    float intensityWeight = 0.015f; // |albedoL-albedoR| term vs fringe collisions
    int lrCheck = 1;              // left→right / right→left mutual consistency
    float lrMaxDiff = 3.5f;       // max |dispL - dispR| (px) to keep a match
    int doMedian = 1;            // 3×3 disparity median (kills salt/pepper)
    int doSpeckle = 1;           // local-support speckle rejection
    float speckleMaxDiff = 3.0f;  // neighbour disparity tolerance (px)
    int speckleMinSupport = 3;   // min supporting neighbours in 5×5 window
};

// Packed POP4 Y16: 640×2560 little-endian uint16 (first four 640×512 planes used).
// On success fills depthMm (320×256) and rgb (320×256×3 grayscale IR) and returns
// the nonzero depth count. On failure returns 0 and sets errorOut.
uint32_t decodePacked(const uint16_t* packedY16,
                      const DecodeParams& params,
                      std::vector<uint16_t>& depthMm,
                      std::vector<uint8_t>& rgb,
                      std::string* errorOut);

} // namespace revoscan::decoder::cuda_stereo
