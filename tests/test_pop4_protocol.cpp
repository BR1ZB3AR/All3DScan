#include <catch2/catch_test_macros.hpp>

#include <algorithm>

#include "protocol/RevopointPop4Protocol.h"

using namespace revoscan;

TEST_CASE("POP4 projector enable payload is little-endian one", "[pop4]") {
    const auto payload = protocol::pop4::makeEnablePayload();
    REQUIRE(payload.size() == 4);
    REQUIRE(payload[0] == 0x01);
    REQUIRE(payload[1] == 0x00);
    REQUIRE(payload[2] == 0x00);
    REQUIRE(payload[3] == 0x00);
}

TEST_CASE("POP4 capture size constants prefer packed full-field", "[pop4]") {
    REQUIRE(protocol::pop4::kCaptureWidth == 640);
    REQUIRE(protocol::pop4::kCaptureHeightPacked == 2560);
    REQUIRE(protocol::pop4::kCaptureHeightFallback == 512);
    REQUIRE(protocol::pop4::kCaptureHeight == 2560);
}

TEST_CASE("POP4 settings payload matches APK-informed CS7 defaults", "[pop4]") {
    const auto payload = protocol::pop4::makeSettingsPayload();
    REQUIRE(payload.size() == 60);

    const uint8_t expectedPrefix[] = {
        0x01, 0x00, 0x00, 0x00, // gain 1
        0xb8, 0x0b, 0x00, 0x00, // exposure 3000 (param_pop4.json)
        0x82, 0x00, 0x00, 0x00, // IR 130
        0x0c, 0x03, 0x00, 0x00, // cross laser 780
        0xc2, 0x02, 0x00, 0x00, // single laser 706
    };
    REQUIRE(std::equal(
        std::begin(expectedPrefix), std::end(expectedPrefix), payload.begin()));
    REQUIRE(std::all_of(payload.begin() + 20,
                        payload.end(),
                        [](uint8_t value) { return value == 0; }));
}

// The light and mode commands are transcribed from the RevoScan 6.0.2.4
// Android SDK (lib3DCamera.so) and verified on hardware; a typo here would
// silently send a different register to the scanner, so pin them exactly.
TEST_CASE("POP4 light commands match the vendor SDK registers", "[pop4][lights]") {
    using protocol::Light;

    // ledAndLaserCtrl @0x630fd4 ledid 2 ("laser") -> 0xb04. Measured on
    // hardware: this is the rail that draws the scan pattern.
    REQUIRE(protocol::pop4::lightSwitchCommands(Light::Projector, true) ==
            std::vector<std::string>{"echo s 0xb04 1 > /dev/rk_preisp"});
    REQUIRE(protocol::pop4::lightSwitchCommands(Light::Projector, false) ==
            std::vector<std::string>{"echo s 0xb04 0 > /dev/rk_preisp"});

    // ledid 0 ("IR led") -> 0xb01. Accepted but inert on a POP4.
    REQUIRE(protocol::pop4::lightSwitchCommands(Light::IrLed, true) ==
            std::vector<std::string>{"echo s 0xb01 1 > /dev/rk_preisp"});

    // ledid 1 (RGB LED), ctrltype 0; POP_4 / POP_4_PRO also get 0xb12 255.
    REQUIRE(protocol::pop4::lightSwitchCommands(Light::FillLight, true) ==
            std::vector<std::string>{"echo s 0xb03 1 > /dev/rk_preisp",
                                     "echo s 0xb02 1 > /dev/rk_preisp",
                                     "echo s 0xb12 255 > /dev/rk_preisp"});
    REQUIRE(protocol::pop4::lightSwitchCommands(Light::FillLight, false) ==
            std::vector<std::string>{"echo s 0xb03 0 > /dev/rk_preisp"});

    // Luminance registers: laser 0x48d, RGB 0xb12, IR 0xb07; all 0..255.
    REQUIRE(protocol::pop4::lightLevelCommand(Light::Projector, 180) ==
            "echo s 0x48d 180 > /dev/rk_preisp");
    REQUIRE(protocol::pop4::lightLevelCommand(Light::FillLight, 128) ==
            "echo s 0xb12 128 > /dev/rk_preisp");
    REQUIRE(protocol::pop4::lightLevelCommand(Light::IrLed, 64) ==
            "echo s 0xb07 64 > /dev/rk_preisp");
    // Out-of-range values are clamped, never sent raw.
    REQUIRE(protocol::pop4::lightLevelCommand(Light::FillLight, 999) ==
            "echo s 0xb12 255 > /dev/rk_preisp");
    REQUIRE(protocol::pop4::lightLevelCommand(Light::FillLight, -5) ==
            "echo s 0xb12 0 > /dev/rk_preisp");

    // Every command must fit the 60-byte XU payload with room for the NUL.
    for (bool on : {true, false}) {
        for (Light l : {Light::Projector, Light::FillLight, Light::IrLed}) {
            for (const auto& cmd : protocol::pop4::lightSwitchCommands(l, on)) {
                REQUIRE(cmd.size() < 60);
            }
            REQUIRE(protocol::pop4::lightLevelCommand(l, 255).size() < 60);
        }
    }
}

TEST_CASE("POP4 scan mode commands match cs::Camera::setSpeckleMode", "[pop4][lights]") {
    // setSpeckleMode @0x63725c is gated to camera types 0x20/0x21 (POP_4,
    // POP_4_PRO) and sends 0x3001 12; RevoScan's POP4 speckle presets pair it
    // with laserLuminance 180 (resources/param_pop4.json).
    const auto blue = protocol::pop4::scanModeCommands(protocol::ScanMode::BlueLaser);
    REQUIRE(blue == std::vector<std::string>{"echo s 0x3001 12 > /dev/rk_preisp",
                                             "echo s 0x48d 180 > /dev/rk_preisp"});
    const auto ir = protocol::pop4::scanModeCommands(protocol::ScanMode::StructuredLight);
    REQUIRE(ir == std::vector<std::string>{"echo s 0x3001 7 > /dev/rk_preisp"});
    for (const auto& cmd : blue) {
        REQUIRE(cmd.size() < 60);
    }
}
