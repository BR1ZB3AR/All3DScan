#include <filesystem>
#include <fstream>
#include <string>

#include "all3dscan/Pop4Sensor.h"

namespace all3dscan::hw {

namespace {

std::string readLine(const std::filesystem::path& path) {
    std::ifstream in(path);
    std::string line;
    std::getline(in, line);
    return line;
}

} // namespace

bool pop4Present() {
    // The POP 4's USB ids (src/protocol/RevopointPop4Protocol.h), as sysfs
    // writes them.
    std::error_code error;
    for (const auto& device : std::filesystem::directory_iterator("/sys/bus/usb/devices", error)) {
        if (readLine(device.path() / "idVendor") == "2207" &&
            readLine(device.path() / "idProduct") == "110c") {
            return true;
        }
    }
    return false;
}

} // namespace all3dscan::hw
