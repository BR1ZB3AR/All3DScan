#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <algorithm>
#include <cmath>

#include "pointcloud/PointCloud.h"

using revoscan::pointcloud::ForegroundIsolation;
using revoscan::pointcloud::isolateForegroundObject;
using namespace revoscan;

TEST_CASE("isolateForegroundObject keeps the near object and drops far background") {
    revoscan::pointcloud::PointCloud cloud;
    // Near object: dense 8x8x8 block around z=0.30 m.
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            for (int k = 0; k < 8; ++k) {
                revoscan::pointcloud::Point p;
                p.x = static_cast<float>(i) * 0.01f;
                p.y = static_cast<float>(j) * 0.01f;
                p.z = 0.30f + static_cast<float>(k) * 0.01f;
                cloud.points.push_back(p);
            }
        }
    }
    // Far background sheet at z=0.75 m (beyond the near band, separate cluster).
    for (int i = 0; i < 40; ++i) {
        for (int j = 0; j < 40; ++j) {
            revoscan::pointcloud::Point p;
            p.x = static_cast<float>(i) * 0.02f - 0.4f;
            p.y = static_cast<float>(j) * 0.02f - 0.4f;
            p.z = 0.75f;
            cloud.points.push_back(p);
        }
    }

    ForegroundIsolation params;
    params.nearClipM = 0.10f;
    params.farClipM = 0.90f;
    params.nearBandM = 0.15f;
    params.clusterVoxelM = 0.02f;
    const auto out = isolateForegroundObject(cloud, params);
    REQUIRE(out.points.size() == 512); // only the near block survives
    for (const auto& p : out.points) {
        REQUIRE(p.z < 0.6f);
    }
}

TEST_CASE("isolateForegroundObject prefers small near object over large background") {
    revoscan::pointcloud::PointCloud cloud;
    // Small near object (bolt-sized) at z≈0.30 m, near optical axis.
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 10; ++k) {
                revoscan::pointcloud::Point p;
                p.x = static_cast<float>(i) * 0.005f - 0.015f;
                p.y = static_cast<float>(j) * 0.005f - 0.015f;
                p.z = 0.28f + static_cast<float>(k) * 0.004f;
                cloud.points.push_back(p);
            }
        }
    }
    // Large background sheet still inside a wide farClip, farther and off-axis.
    for (int i = 0; i < 50; ++i) {
        for (int j = 0; j < 50; ++j) {
            revoscan::pointcloud::Point p;
            p.x = static_cast<float>(i) * 0.01f - 0.25f;
            p.y = static_cast<float>(j) * 0.01f - 0.25f;
            p.z = 0.42f;
            cloud.points.push_back(p);
        }
    }

    ForegroundIsolation params;
    params.nearClipM = 0.20f;
    params.farClipM = 0.50f;
    params.nearBandM = 0.20f; // band wide enough that both clusters survive the cut
    params.clusterVoxelM = 0.02f;
    params.lateralPadM = 0.05f;
    params.minClusterPoints = 30;
    const auto out = isolateForegroundObject(cloud, params);
    REQUIRE(out.points.size() >= 300);
    REQUIRE(out.points.size() <= 360);
    for (const auto& p : out.points) {
        REQUIRE(p.z < 0.36f);
        REQUIRE(std::fabs(p.x) < 0.08f);
        REQUIRE(std::fabs(p.y) < 0.08f);
    }
}

TEST_CASE("isolateForegroundObject removes a detached speck near the object") {
    revoscan::pointcloud::PointCloud cloud;
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            for (int k = 0; k < 8; ++k) {
                revoscan::pointcloud::Point p;
                p.x = static_cast<float>(i) * 0.01f;
                p.y = static_cast<float>(j) * 0.01f;
                p.z = 0.30f + static_cast<float>(k) * 0.01f;
                cloud.points.push_back(p);
            }
        }
    }
    // A tiny detached speck (within the depth band but far in x/y).
    for (int i = 0; i < 3; ++i) {
        revoscan::pointcloud::Point p;
        p.x = 0.5f + static_cast<float>(i) * 0.005f;
        p.y = 0.5f;
        p.z = 0.31f;
        cloud.points.push_back(p);
    }

    ForegroundIsolation params;
    params.nearClipM = 0.10f;
    params.farClipM = 0.90f;
    params.nearBandM = 0.15f;
    params.clusterVoxelM = 0.02f;
    const auto out = isolateForegroundObject(cloud, params);
    REQUIRE(out.points.size() == 512);
}

namespace {
// A wide fronto-parallel slab at z=0.33 plus an object cube (z=0.30..0.34)
// that overlaps the slab laterally — the "collapsed depth" blob scenario.
revoscan::pointcloud::PointCloud slabPlusObject() {
    revoscan::pointcloud::PointCloud cloud;
    for (int i = 0; i < 30; ++i) {
        for (int j = 0; j < 30; ++j) {
            revoscan::pointcloud::Point p;
            p.x = static_cast<float>(i) * 0.01f - 0.15f;
            p.y = static_cast<float>(j) * 0.01f - 0.15f;
            p.z = 0.33f;
            cloud.points.push_back(p);
        }
    }
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            for (int k = 0; k < 9; ++k) {
                revoscan::pointcloud::Point p;
                p.x = static_cast<float>(i) * 0.01f - 0.05f;
                p.y = static_cast<float>(j) * 0.01f - 0.05f;
                p.z = 0.300f + static_cast<float>(k) * 0.005f; // 0.300..0.340
                cloud.points.push_back(p);
            }
        }
    }
    return cloud;
}
} // namespace

TEST_CASE("isolateForegroundObject strips a dominant fronto-parallel slab") {
    const auto cloud = slabPlusObject();
    ForegroundIsolation params;
    params.nearClipM = 0.25f;
    params.farClipM = 0.50f;
    params.nearBandM = 0.15f;
    params.clusterVoxelM = 0.02f;
    params.minClusterPoints = 30;
    const auto out = isolateForegroundObject(cloud, params);
    // The flat slab (z≈0.33) is removed; the 3D object relief survives.
    REQUIRE(out.points.size() > 300);
    REQUIRE(out.points.size() < 1500);
    const auto onSlab = std::count_if(
        out.points.begin(), out.points.end(),
        [](const revoscan::pointcloud::Point& p) {
            return std::fabs(p.z - 0.33f) <= 0.002f;
        });
    REQUIRE(onSlab == 0);
}

TEST_CASE("isolateForegroundObject keeps the slab when plane removal is disabled") {
    const auto cloud = slabPlusObject();
    ForegroundIsolation params;
    params.nearClipM = 0.25f;
    params.farClipM = 0.50f;
    params.nearBandM = 0.15f;
    params.clusterVoxelM = 0.02f;
    params.minClusterPoints = 30;
    params.removeDominantPlane = false; // knob off → slab is retained
    const auto out = isolateForegroundObject(cloud, params);
    const auto onSlab = std::count_if(
        out.points.begin(), out.points.end(),
        [](const revoscan::pointcloud::Point& p) {
            return std::fabs(p.z - 0.33f) <= 0.002f;
        });
    REQUIRE(onSlab > 400); // slab survives when the knob is off
}

TEST_CASE("fromDepthFrame skips zero-depth pixels and back-projects correctly") {
    protocol::RawFrame frame;
    frame.width = 2;
    frame.height = 1;
    frame.depthMm = {0, 1000}; // first pixel invalid, second at 1m
    frame.rgb = {0, 0, 0, 10, 20, 30};

    protocol::CameraIntrinsics intrinsics{.fx = 100.0, .fy = 100.0, .cx = 1.0, .cy = 0.0};

    auto cloud = pointcloud::fromDepthFrame(frame, intrinsics);
    REQUIRE(cloud.points.size() == 1);

    const auto& p = cloud.points[0];
    REQUIRE(p.z == Catch::Approx(1.0f));
    // pixel x=1, cx=1 -> (x - cx) = 0, so px should be 0 regardless of z/fx.
    REQUIRE(p.x == Catch::Approx(0.0f));
    REQUIRE(p.r == 10);
    REQUIRE(p.g == 20);
    REQUIRE(p.b == 30);
}

TEST_CASE("fromDepthFrame rejects malformed metric frames") {
    protocol::RawFrame frame;
    frame.width = 2;
    frame.height = 2;
    frame.depthMm = {1000}; // should contain four samples
    protocol::CameraIntrinsics intrinsics{
        .fx = 100.0, .fy = 100.0, .cx = 1.0, .cy = 1.0};

    REQUIRE(pointcloud::fromDepthFrame(frame, intrinsics).points.empty());
}

TEST_CASE("fromDepthFrame never interprets vendor raw Y16 as millimeters") {
    protocol::RawFrame frame;
    frame.width = 2;
    frame.height = 1;
    frame.depthMm = {1000, 2000};
    frame.encoding = protocol::DepthEncoding::VendorRawY16;
    protocol::CameraIntrinsics intrinsics{
        .fx = 100.0, .fy = 100.0, .cx = 1.0, .cy = 0.0};

    REQUIRE(pointcloud::fromDepthFrame(frame, intrinsics).points.empty());
}

TEST_CASE("previewFromVendorRawY16 maps into POP4 working distance") {
    protocol::RawFrame frame;
    frame.width = 4;
    frame.height = 2;
    frame.encoding = protocol::DepthEncoding::VendorRawY16;
    frame.depthMm = {10, 0, 100, 0, 50, 0, 200, 0};

    pointcloud::PreviewDepthScale scale;
    scale.zMinMm = 210.0f;
    scale.zMaxMm = 500.0f;
    scale.depthScale = 1.0;
    scale.fx = 560.0f;
    scale.fy = 560.0f;

    auto cloud = pointcloud::previewFromVendorRawY16(frame, scale);
    REQUIRE(cloud.points.size() == 4);
    for (const auto& p : cloud.points) {
        REQUIRE(p.z >= Catch::Approx(0.210f).margin(0.001f));
        REQUIRE(p.z <= Catch::Approx(0.500f).margin(0.001f));
    }
    // Highest raw (200) should land near the far end of the working volume.
    const auto maxIt = std::max_element(
        cloud.points.begin(), cloud.points.end(),
        [](const pointcloud::Point& a, const pointcloud::Point& b) { return a.z < b.z; });
    REQUIRE(maxIt->z == Catch::Approx(0.500f).margin(0.02f));
}

TEST_CASE("previewFromVendorRawY16 respects depthScale multiplier") {
    protocol::RawFrame frame;
    frame.width = 2;
    frame.height = 1;
    frame.encoding = protocol::DepthEncoding::VendorRawY16;
    frame.depthMm = {100, 200};

    pointcloud::PreviewDepthScale scale;
    scale.depthScale = 2.0;
    scale.zMinMm = 200.0f;
    scale.zMaxMm = 400.0f;
    auto cloud = pointcloud::previewFromVendorRawY16(frame, scale);
    REQUIRE(cloud.points.size() == 2);
    REQUIRE(cloud.points[0].z == Catch::Approx(0.200f).margin(0.01f));
    REQUIRE(cloud.points[1].z == Catch::Approx(0.400f).margin(0.01f));
}
