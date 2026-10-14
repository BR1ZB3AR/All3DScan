#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "protocol/ScannerProtocol.h"

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

// Read-only file transfer from the scanner's own filesystem, as the vendor
// SDK's libuvcReadFile does (RevoScan 6.0.2.4 lib3DCamera.so @0x72c450):
//   open : SET_CUR XU4 selector 1, 60 B = [0x01, path..., NUL]
//   read : GET_CUR XU4 selector 2, 60 B = [len LE32 (<=56), data...], until len 0
//   close: SET_CUR XU4 selector 1, 60 B of zeros
// The factory calibration lives under /data/camparam/ (camparam.yaml, maps,
// Q/P matrices). Must not run while the depth interface is claimed elsewhere.
std::optional<std::vector<uint8_t>> readDeviceFile(const std::string& path,
                                                   std::string* error = nullptr,
                                                   size_t maxBytes = 8u << 20);

// Scanner shell commands (sent on XU4 selector 7) that switch the built-in
// lights. These are exactly what the vendor SDK's cs::Camera::ledAndLaserCtrl
// (RevoScan 6.0.2.4 lib3DCamera.so @0x630fd4) sends for POP_4 / POP_4_PRO
// (camera types 0x20/0x21), which its support check (@0x63c4ec) allows:
//   blue laser  on: 0xb04 1                    off: 0xb04 0
//   fill light  on: 0xb03 1, 0xb02 1, 0xb12 255  off: 0xb03 0
//   IR projector on: 0xb01 1                   off: 0xb01 0
// and the levels (0..255): laser 0x48d, fill light 0xb12, IR 0xb07.
std::vector<std::string> lightSwitchCommands(Light light, bool on);

// Depth-mode switch. cs::Camera::setSpeckleMode (lib3DCamera.so @0x63725c) is
// gated to camera types 0x20/0x21 (POP_4 / POP_4_PRO) and sends
// "echo s 0x3001 12"; the normal mode for the same types is "echo s 0x3001 7"
// (@0x635410). RevoScan's own POP4 presets pair speckle mode with
// laserLuminance 180 (resources/param_pop4.json, accuracymode "speckle"), and
// the matching factory calibration is /data/camparam_blue/camparam.yaml.
std::vector<std::string> scanModeCommands(ScanMode mode);

// The vendor's "laser line" path for POP_4 / POP_4_PRO (lib3DCamera.so
// @0x635234): "echo s 0x3001 7" then the enter-laser-line-group command
// "echo s 0x3010 0x17 1", plus the laser rail (ledid 2) and its luminance.
// Verified on hardware 2026-10-14: this is what makes the POP4 project
// visible blue laser lines. Sending only the 0x3001 7 half leaves every
// emitter off, which is why an earlier attempt looked dead.
std::vector<std::string> laserLineCommands(bool on);
std::string lightLevelCommand(Light light, int level);

} // namespace revoscan::protocol::pop4
