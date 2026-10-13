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

bool computeDisparity(const uint16_t*,
                      const uint16_t*,
                      const float*,
                      const float*,
                      const float*,
                      const float*,
                      const SgmParams&,
                      std::vector<float>& disparity,
                      std::string* errorOut) {
    disparity.clear();
    if (errorOut) {
        *errorOut = "CUDA stereo decode not built into this binary";
    }
    return false;
}

} // namespace revoscan::decoder::cuda_stereo
