#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "decoder/Pop4DepthDecoder.h"

using namespace revoscan;

namespace {

constexpr uint32_t kImgW = 320;
constexpr uint32_t kImgH = 512;
constexpr uint32_t kPackedW = 640;
constexpr uint32_t kPackedH = 2560;

uint16_t y16FromIntensity(float intensity) {
    const float clamped = std::clamp(intensity, 0.0f, 255.0f);
    const uint8_t b = static_cast<uint8_t>(std::lround(clamped));
    return static_cast<uint16_t>((static_cast<uint16_t>(b) << 8) | b);
}

// Ten 320x512 images back to back: 0-4 right camera, 5-9 left camera, with the
// projected speckle in images 4 (right) and 9 (left). The right speckle is the
// left pattern shifted by `disparityPx`, i.e. already row-aligned.
protocol::RawFrame makeSyntheticSpeckleFrame(int disparityPx) {
    const uint32_t patternW = kImgW + static_cast<uint32_t>(disparityPx);
    std::vector<float> pattern(static_cast<size_t>(patternW) * kImgH, 20.0f);
    uint32_t state = 12345u;
    for (float& v : pattern) {
        state = state * 1664525u + 1013904223u;
        if ((state >> 24) < 40) { // ~16% bright dots
            v = 200.0f;
        }
    }
    protocol::RawFrame raw;
    raw.width = kPackedW;
    raw.height = kPackedH;
    raw.encoding = protocol::DepthEncoding::VendorRawY16;
    raw.depthMm.assign(static_cast<size_t>(kPackedW) * kPackedH, y16FromIntensity(20.0f));
    const size_t image = static_cast<size_t>(kImgW) * kImgH;
    uint16_t* right = raw.depthMm.data() + 4 * image;
    uint16_t* left = raw.depthMm.data() + 9 * image;
    for (uint32_t y = 0; y < kImgH; ++y) {
        for (uint32_t x = 0; x < kImgW; ++x) {
            left[y * kImgW + x] = y16FromIntensity(pattern[y * patternW + x]);
            right[y * kImgW + x] =
                y16FromIntensity(pattern[y * patternW + x + static_cast<uint32_t>(disparityPx)]);
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

TEST_CASE("Pop4DepthDecoder recovers metric depth from synthetic speckle stereo") {
    decoder::Pop4DepthDecoder decoder;
    const auto cal = decoder::makeApproximatePop4FullFieldCalibration();
    decoder.setCalibration(cal);
    decoder.setRowRectification(decoder::kIdentityRowRectification);
    decoder.setDepthRange(200.0f, 650.0f);
    unsetenv("ALL3DSCAN_POP4_SWAP_LR");

    constexpr int disparity = 35;
    auto metric = decoder.decode(makeSyntheticSpeckleFrame(disparity));
    REQUIRE(metric.has_value());
    REQUIRE(metric->encoding == protocol::DepthEncoding::MetricMillimeters);
    REQUIRE(metric->width == 320);
    REQUIRE(metric->height == 512);
    REQUIRE(metric->rgb.size() == 320u * 512u * 3u);

    // Expected Z = fx * B / d
    const double expectedMm = cal.depth.fx * cal.baselineMm / disparity;
    uint32_t nonzero = 0;
    uint32_t nearExpected = 0;
    for (uint16_t d : metric->depthMm) {
        if (d == 0) {
            continue;
        }
        ++nonzero;
        if (std::fabs(static_cast<double>(d) - expectedMm) < 5.0) {
            ++nearExpected;
        }
    }
    // Every pixel right of the disparity band edge is matchable.
    REQUIRE(nonzero > 320u * 512u / 2);
    REQUIRE(nearExpected > nonzero * 95 / 100);
}

TEST_CASE("Pop4DepthDecoder rejects frames without projected speckle") {
    decoder::Pop4DepthDecoder decoder;
    decoder.setCalibration(decoder::makeApproximatePop4FullFieldCalibration());
    decoder.setRowRectification(decoder::kIdentityRowRectification);

    protocol::RawFrame raw;
    raw.width = 640;
    raw.height = 2560;
    raw.encoding = protocol::DepthEncoding::VendorRawY16;
    raw.depthMm.assign(640u * 2560u, y16FromIntensity(30.0f));

    REQUIRE_FALSE(decoder.decode(raw).has_value());
    REQUIRE(decoder.lastStats().validDepth < 200);
}

TEST_CASE("default POP4 row rectification is a mild roll, not identity") {
    const auto c = decoder::defaultPop4RowRectification();
    REQUIRE(c[2] > 0.9);
    REQUIRE(c[2] < 1.1);
    REQUIRE(std::fabs(c[4]) > 0.01);
}

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
