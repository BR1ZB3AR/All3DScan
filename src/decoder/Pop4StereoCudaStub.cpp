#include "Pop4StereoCuda.h"

namespace revoscan::decoder::cuda_stereo {

bool isBuilt() {
    return false;
}

bool available() {
    return false;
}

bool enabledByEnv() {
    return false;
}

uint32_t decodePacked(const uint16_t*,
                      const DecodeParams&,
                      std::vector<uint16_t>& depthMm,
                      std::vector<uint8_t>& rgb,
                      std::string* errorOut) {
    depthMm.clear();
    rgb.clear();
    if (errorOut) {
        *errorOut = "CUDA stereo decode not built into this binary";
    }
    return 0;
}

} // namespace revoscan::decoder::cuda_stereo
