#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>

#include "usb/UsbDevice.h"
#include "util/ResourcePaths.h"

namespace fs = std::filesystem;

TEST_CASE("loadKnownDeviceIds reads POP4 entry", "[usb]") {
    const auto path = revoscan::util::findDevicesJson();
    REQUIRE(path.has_value());

    const auto ids = revoscan::usb::loadKnownDeviceIds(*path);
    REQUIRE_FALSE(ids.empty());
    REQUIRE(ids.front().vendorId == 0x2207);
    REQUIRE(ids.front().productId == 0x110c);
}

TEST_CASE("loadKnownDeviceIds covers multiple vendors", "[usb]") {
    const auto path = revoscan::util::findDevicesJson();
    REQUIRE(path.has_value());
    const auto ids = revoscan::usb::loadKnownDeviceIds(*path);

    // Confirmed POP4 and Creality Otter entries must be present and matchable.
    REQUIRE(revoscan::usb::deviceMatches(
        revoscan::usb::DeviceId{0x2207, 0x110c, "", false, "Revopoint"}, 0x2207, 0x110c));

    bool sawCreality = false;
    bool sawVendorWildcard = false;
    for (const auto& id : ids) {
        if (id.vendorId == 0x2bc5) {
            sawCreality = true;
        }
        if (id.matchAnyProduct) {
            sawVendorWildcard = true;
        }
    }
    REQUIRE(sawCreality);
    REQUIRE(sawVendorWildcard);
}

TEST_CASE("deviceMatches honors vendor-only wildcards", "[usb]") {
    revoscan::usb::DeviceId wildcard{0x2207, 0x0000, "", true, "Revopoint"};
    REQUIRE(revoscan::usb::deviceMatches(wildcard, 0x2207, 0x9999));
    REQUIRE_FALSE(revoscan::usb::deviceMatches(wildcard, 0x2bc5, 0x9999));

    revoscan::usb::DeviceId exact{0x2bc5, 0x06da, "", false, "Creality"};
    REQUIRE(revoscan::usb::deviceMatches(exact, 0x2bc5, 0x06da));
    REQUIRE_FALSE(revoscan::usb::deviceMatches(exact, 0x2bc5, 0x06db));
}

TEST_CASE("loadKnownDeviceIds ignores malformed JSON", "[usb]") {
    const auto temp = fs::temp_directory_path() / "all3dscan-bad-devices.json";
    {
        std::ofstream out(temp);
        out << "{ \"devices\": [ { \"vendorId\": 1 } ] }";
    }
    const auto ids = revoscan::usb::loadKnownDeviceIds(temp.string());
    REQUIRE(ids.empty());
    fs::remove(temp);
}

TEST_CASE("loadKnownDeviceIds skips disabled entries", "[usb]") {
    const auto temp = fs::temp_directory_path() / "all3dscan-disabled-devices.json";
    {
        std::ofstream out(temp);
        out << R"({ "devices": [
            { "vendorId": "2207", "productId": "110c", "enabled": true },
            { "vendorId": "1234", "productId": "5678", "enabled": false }
        ] })";
    }
    const auto ids = revoscan::usb::loadKnownDeviceIds(temp.string());
    REQUIRE(ids.size() == 1);
    REQUIRE(ids.front().vendorId == 0x2207);
    fs::remove(temp);
}
