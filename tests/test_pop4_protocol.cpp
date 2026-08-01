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

TEST_CASE("POP4 settings payload matches confirmed CS7 replay", "[pop4]") {
    const auto payload = protocol::pop4::makeSettingsPayload();
    REQUIRE(payload.size() == 60);

    const uint8_t expectedPrefix[] = {
        0x01, 0x00, 0x00, 0x00, // gain 1
        0x20, 0x03, 0x00, 0x00, // exposure 800
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
