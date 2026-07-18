#include <catch2/catch_test_macros.hpp>

#include "protocol/ScannerProtocol.h"

using namespace revoscan::protocol;

TEST_CASE("MockProtocol connect/stream lifecycle") {
    auto proto = makeMockProtocol();
    REQUIRE(proto->state() == ConnectionState::Disconnected);

    REQUIRE(proto->connect());
    REQUIRE(proto->state() == ConnectionState::Connected);

    // No frames while not streaming.
    REQUIRE_FALSE(proto->readFrame(0).has_value());

    REQUIRE(proto->startStream());
    REQUIRE(proto->state() == ConnectionState::Streaming);

    auto frame = proto->readFrame(0);
    REQUIRE(frame.has_value());
    REQUIRE(frame->width > 0);
    REQUIRE(frame->height > 0);
    REQUIRE(frame->depthMm.size() == static_cast<size_t>(frame->width) * frame->height);

    proto->stopStream();
    REQUIRE(proto->state() == ConnectionState::Connected);

    proto->disconnect();
    REQUIRE(proto->state() == ConnectionState::Disconnected);
}

TEST_CASE("MockProtocol produces a nonzero-depth sphere") {
    auto proto = makeMockProtocol();
    proto->connect();
    proto->startStream();
    auto frame = proto->readFrame(0);
    REQUIRE(frame.has_value());

    bool anyNonZero = false;
    for (auto d : frame->depthMm) {
        if (d != 0) {
            anyNonZero = true;
            break;
        }
    }
    REQUIRE(anyNonZero);
}
