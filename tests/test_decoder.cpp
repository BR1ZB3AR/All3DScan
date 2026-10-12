#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "decoder/Pop4DepthDecoder.h"
#include "decoder/Pop4StereoCuda.h"

using namespace revoscan;

namespace {

constexpr uint32_t kW = 640;
constexpr uint32_t kH = 512;
constexpr uint32_t kPackedH = 2560;
constexpr float kTwoPi = 6.28318530718f;

uint16_t y16FromIntensity(float intensity) {
    const float clamped = std::clamp(intensity, 0.0f, 255.0f);
    const uint8_t b = static_cast<uint8_t>(std::lround(clamped));
    return static_cast<uint16_t>((static_cast<uint16_t>(b) << 8) | b);
}

protocol::RawFrame makeSyntheticPackedFrame(float disparityPx) {
    protocol::RawFrame raw;
    raw.width = kW;
    raw.height = kPackedH;
    raw.encoding = protocol::DepthEncoding::VendorRawY16;
    raw.depthMm.assign(static_cast<size_t>(kW) * kPackedH, 0);

    // Stereo lives in the top 320×256 L|R tiles; bottom band left blank.
    constexpr uint32_t kViewH = 256;
    const size_t planeSamples = static_cast<size_t>(kW) * kH;
    for (int p = 0; p < 4; ++p) {
        const float step = static_cast<float>(p) * (kTwoPi * 0.25f);
        uint16_t* plane = raw.depthMm.data() + static_cast<size_t>(p) * planeSamples;
        for (uint32_t y = 0; y < kViewH; ++y) {
            for (uint32_t x = 0; x < 320; ++x) {
                const float phaseL = kTwoPi * (static_cast<float>(x) / 320.0f);
                const float iL = 80.0f + 40.0f * std::cos(phaseL - step);
                plane[y * kW + x] = y16FromIntensity(iL);

                const float phaseR =
                    kTwoPi * ((static_cast<float>(x) + disparityPx) / 320.0f);
                const float iR = 80.0f + 40.0f * std::cos(phaseR - step);
                plane[y * kW + 320 + x] = y16FromIntensity(iR);
            }
        }
    }
    return raw;
}

} // namespace

TEST_CASE("approximate POP4 calib exposes stereo hooks") {
    const auto cal = decoder::makeApproximatePop4FullFieldCalibration();
    REQUIRE(cal.valid);
    REQUIRE(cal.source == "ApproximatePop4FullField");
    REQUIRE(cal.hasDepthIntrinsics());
    REQUIRE(cal.hasStereoCalibration());
    REQUIRE(cal.baselineMm > 0.0);
    REQUIRE(cal.depth.fx > 0.0);
}

TEST_CASE("Pop4DepthDecoder refuses non-packed frames") {
    decoder::Pop4DepthDecoder decoder;
    decoder.setCalibration(decoder::makeApproximatePop4FullFieldCalibration());

    protocol::RawFrame raw;
    raw.width = 640;
    raw.height = 512;
    raw.encoding = protocol::DepthEncoding::VendorRawY16;
    raw.depthMm.assign(640u * 512u, 100);

    REQUIRE_FALSE(decoder.decode(raw).has_value());
    REQUIRE(decoder.lastError().find("640x2560") != std::string::npos);
}

TEST_CASE("Pop4DepthDecoder recovers metric depth from synthetic four-step stereo") {
    decoder::Pop4DepthDecoder decoder;
    const auto cal = decoder::makeApproximatePop4FullFieldCalibration();
    decoder.setCalibration(cal);

    constexpr float disparity = 50.0f;
    auto raw = makeSyntheticPackedFrame(disparity);
    // Force L|R assignment so Auto scoring is deterministic in CI.
    setenv("ALL3DSCAN_POP4_SWAP_LR", "0", 1);
    auto metric = decoder.decode(raw);
    REQUIRE(metric.has_value());
    REQUIRE(metric->encoding == protocol::DepthEncoding::MetricMillimeters);
    REQUIRE(metric->width == 320);
    REQUIRE(metric->height == 256);

    const auto nonzero = std::count_if(
        metric->depthMm.begin(), metric->depthMm.end(),
        [](uint16_t d) { return d != 0; });
    REQUIRE(nonzero > 400);

    // Expected Z ≈ fx * B / d
    const double expectedMm = cal.depth.fx * cal.baselineMm / disparity;
    uint32_t nearExpected = 0;
    for (uint16_t d : metric->depthMm) {
        if (d == 0) {
            continue;
        }
        if (std::fabs(static_cast<double>(d) - expectedMm) < 25.0) {
            ++nearExpected;
        }
    }
    REQUIRE(nearExpected > 200);
}

#ifdef ALL3DSCAN_HAS_CUDA
TEST_CASE("Pop4DepthDecoder CUDA path recovers synthetic stereo depth") {
    REQUIRE(decoder::cuda_stereo::isBuilt());
    if (!decoder::cuda_stereo::available()) {
        SKIP("No CUDA device present");
    }

    decoder::Pop4DepthDecoder decoder;
    const auto cal = decoder::makeApproximatePop4FullFieldCalibration();
    decoder.setCalibration(cal);

    constexpr float disparity = 50.0f;
    auto raw = makeSyntheticPackedFrame(disparity);
    // Prefer CUDA explicitly for this case.
    setenv("ALL3DSCAN_POP4_CUDA", "1", 1);
    auto metric = decoder.decode(raw);
    REQUIRE(metric.has_value());
    REQUIRE(metric->width == 320);
    REQUIRE(metric->height == 256);

    const double expectedMm = cal.depth.fx * cal.baselineMm / disparity;
    uint32_t nearExpected = 0;
    for (uint16_t d : metric->depthMm) {
        if (d == 0) {
            continue;
        }
        if (std::fabs(static_cast<double>(d) - expectedMm) < 25.0) {
            ++nearExpected;
        }
    }
    REQUIRE(nearExpected > 200);
}
#endif

TEST_CASE("validateMetricPlane scales and accepts dense metric frames") {
    protocol::ScannerCalibration cal;
    cal.valid = true;
    cal.depthScale = 2.0;
    cal.depth = {.fx = 100, .fy = 100, .cx = 1, .cy = 1};

    protocol::RawFrame frame;
    frame.width = 4;
    frame.height = 4;
    frame.encoding = protocol::DepthEncoding::MetricMillimeters;
    frame.depthMm.assign(16, 100);

    std::string error;
    auto out = decoder::Pop4DepthDecoder::validateMetricPlane(frame, cal, &error);
    REQUIRE(out.has_value());
    REQUIRE(out->depthMm[0] == 200);
    REQUIRE(error.empty());
}
