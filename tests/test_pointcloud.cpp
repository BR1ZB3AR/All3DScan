#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "pointcloud/PointCloud.h"

using namespace revoscan;

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
