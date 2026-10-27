// Pop4Sensor for builds without libusb-1.0, where the POP 4 backend cannot
// be compiled: opening one fails with that reason, so nothing else is reached.

#include <stdexcept>

#include "all3dscan/Pop4Sensor.h"

namespace all3dscan::hw {

Pop4Sensor::Pop4Sensor() {
    throw std::runtime_error(
        "POP 4: this engine was built without POP 4 support (libusb-1.0 was not found)");
}

Pop4Sensor::~Pop4Sensor() = default;
bool Pop4Sensor::capture(int32_t) { return false; }
bool Pop4Sensor::connected() const { return false; }
bool Pop4Sensor::captureFirstFrame() { return false; }
float Pop4Sensor::validFraction() const { return 0.0f; }
bool Pop4Sensor::setLight(uint8_t, bool) { return false; }
bool Pop4Sensor::setLightLevel(uint8_t, int32_t) { return false; }
bool Pop4Sensor::lightOn(uint8_t) const { return false; }
int32_t Pop4Sensor::lightLevel(uint8_t) const { return 0; }
bool Pop4Sensor::setExposure(int32_t) { return false; }
int32_t Pop4Sensor::exposure() const { return 0; }
bool Pop4Sensor::setScanMode(uint8_t) { return false; }
uint8_t Pop4Sensor::scanMode() const { return kInfrared; }
void Pop4Sensor::setDepthRange(float, float) {}
bool Pop4Sensor::check(bool ok) { return ok; }

std::unique_ptr<Pop4Sensor> openPop4Sensor() {
    return std::make_unique<Pop4Sensor>();
}

} // namespace all3dscan::hw
