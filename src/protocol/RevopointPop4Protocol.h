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
// Preferred packed full-field mode: five stacked 640×512 temporal planes.
constexpr uint32_t kCaptureHeightPacked = 2560;
// Fallback single-plane mode when the node rejects 640×2560.
constexpr uint32_t kCaptureHeightFallback = 512;
// Legacy alias used by older call sites / docs.
constexpr uint32_t kCaptureHeight = kCaptureHeightPacked;

std::array<uint8_t, 4> makeEnablePayload();
// Defaults from RevoScan APK assets/param_pop4.json (model 0E1, high-accuracy
// feature/general) for exposure/gain, with empirically confirmed CS7 IR/laser
// field magnitudes from Linux arm probes.
std::array<uint8_t, 60> makeSettingsPayload(int32_t gain = 1,
                                            int32_t exposure = 3000,
                                            int32_t irLum = 130,
                                            int32_t crossLaserLight = 780,
                                            int32_t singleLaserLight = 706);

} // namespace revoscan::protocol::pop4
