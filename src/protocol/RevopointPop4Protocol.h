#pragma once

#include <array>
#include <cstdint>

namespace revoscan::protocol::pop4 {

constexpr uint16_t kVendorId = 0x2207;
constexpr uint16_t kProductId = 0x110c;
constexpr uint8_t kDepthInterface = 0;
constexpr uint8_t kXuUnitId = 4;
constexpr uint8_t kEnableSelector = 10;
constexpr uint8_t kSettingsSelector = 7;
constexpr uint32_t kCaptureWidth = 640;
constexpr uint32_t kCaptureHeight = 512;

std::array<uint8_t, 4> makeEnablePayload();
std::array<uint8_t, 60> makeSettingsPayload(int32_t gain = 1,
                                            int32_t exposure = 800,
                                            int32_t irLum = 130,
                                            int32_t crossLaserLight = 780,
                                            int32_t singleLaserLight = 706);

} // namespace revoscan::protocol::pop4
