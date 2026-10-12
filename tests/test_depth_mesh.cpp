#include <catch2/catch_test_macros.hpp>

#include <vector>

#include "geometry/DepthMesh.h"

using revoscan::geometry::DepthMeshOptions;
using revoscan::geometry::meshFromOrganizedDepth;
using revoscan::geometry::mergeMeshVoxel;
using revoscan::protocol::CameraIntrinsics;

TEST_CASE("meshFromOrganizedDepth builds a plane from constant depth") {
    constexpr uint32_t w = 32;
    constexpr uint32_t h = 24;
    std::vector<uint16_t> depth(static_cast<size_t>(w) * h, 300); // 0.3 m
    CameraIntrinsics K{.fx = 100.0, .fy = 100.0, .cx = 16.0, .cy = 12.0};
    DepthMeshOptions opts;
    opts.stride = 1;
    opts.maxEdgeLengthM = 0.05f;
    opts.nearM = 0.1f;
    opts.farM = 0.6f;

    auto mesh = meshFromOrganizedDepth(depth.data(), w, h, K, nullptr, 0, opts);
    REQUIRE_FALSE(mesh.empty());
    REQUIRE(mesh.vertices.size() == static_cast<size_t>(w) * h);
    REQUIRE(mesh.triangleCount() > 0);
    REQUIRE(mesh.metric);
}

TEST_CASE("meshFromOrganizedDepth rejects depth discontinuities") {
    constexpr uint32_t w = 8;
    constexpr uint32_t h = 8;
    std::vector<uint16_t> depth(static_cast<size_t>(w) * h, 300);
    // Left half close, right half far — edges should not bridge.
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = w / 2; x < w; ++x) {
            depth[static_cast<size_t>(y) * w + x] = 500;
        }
    }
    CameraIntrinsics K{.fx = 80.0, .fy = 80.0, .cx = 4.0, .cy = 4.0};
    DepthMeshOptions opts;
    opts.stride = 1;
    opts.maxEdgeLengthM = 0.05f; // 50 mm — 200 mm jump is rejected

    auto mesh = meshFromOrganizedDepth(depth.data(), w, h, K, nullptr, 0, opts);
    REQUIRE_FALSE(mesh.empty());
    // Full grid would be 7*7*2 = 98 tris; discontinuity cuts the mid seam.
    REQUIRE(mesh.triangleCount() < 98);
}

TEST_CASE("mergeMeshVoxel grows then welds duplicate voxels") {
    constexpr uint32_t w = 16;
    constexpr uint32_t h = 16;
    std::vector<uint16_t> depth(static_cast<size_t>(w) * h, 300);
    CameraIntrinsics K{.fx = 90.0, .fy = 90.0, .cx = 8.0, .cy = 8.0};
    DepthMeshOptions opts;
    opts.stride = 1;
    opts.maxEdgeLengthM = 0.05f;

    auto a = meshFromOrganizedDepth(depth.data(), w, h, K, nullptr, 0, opts);
    auto b = meshFromOrganizedDepth(depth.data(), w, h, K, nullptr, 0, opts);
    REQUIRE_FALSE(a.empty());

    revoscan::geometry::TriangleMesh dst;
    mergeMeshVoxel(dst, a, 0.005f);
    const size_t afterFirst = dst.vertices.size();
    mergeMeshVoxel(dst, b, 0.005f);
    // Same voxels: vertex count should not roughly double.
    REQUIRE(dst.vertices.size() <= afterFirst + 8);
    REQUIRE(dst.triangleCount() > 0);
}
