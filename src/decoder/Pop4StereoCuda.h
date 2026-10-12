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

// Mirror of the CPU MatchTuning fields that affect matching.
struct SgmParams {
    int dispMin = 8;
    int numDisp = 64;          // search band [dispMin, dispMin + numDisp - 1]
    int p1 = 16;
    int p2 = 120;
    float uniqueness = 0.05f;
    float lrMaxDiff = 1.0f;
    float textureMin = 1.5f;
};

// GPU port of Pop4DepthDecoder's matchCpu(): row-align the right speckle image
// with `rectMapY` (320×512 source rows), blur, 7×7 census, 4-path SGM and
// winner-take-all with uniqueness / sub-pixel / left-right / texture gates.
// `leftY16` / `rightY16` are raw 320×512 Y16 images. Fills `disparity`
// (320×512, <0 = invalid) and returns true; on failure returns false and sets
// errorOut (device errors also disable CUDA for the process).
bool computeDisparity(const uint16_t* leftY16,
                      const uint16_t* rightY16,
                      const float* rectMapY,
                      const SgmParams& params,
                      std::vector<float>& disparity,
                      std::string* errorOut);

} // namespace revoscan::decoder::cuda_stereo
