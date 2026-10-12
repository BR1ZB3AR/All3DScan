#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "processing/Registration.h"

using namespace revoscan::processing;
using revoscan::pointcloud::Point;
using revoscan::pointcloud::PointCloud;

namespace {

// Deterministic, non-degenerate 3D point set (LCG-scattered in a cube).
PointCloud makeCloud(size_t n, float side = 0.2f) {
    PointCloud c;
    uint32_t state = 0x1234567u;
    auto rnd = [&]() {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) / static_cast<float>(1u << 24);
    };
    for (size_t i = 0; i < n; ++i) {
        Point p;
        p.x = (rnd() - 0.5f) * side;
        p.y = (rnd() - 0.5f) * side;
        p.z = 0.3f + (rnd() - 0.5f) * side; // centred ~0.3 m in front
        c.points.push_back(p);
    }
    return c;
}

PointCloud rotateAboutZ(const PointCloud& in, float deg, float tx, float ty, float tz) {
    const float a = deg * 3.14159265358979f / 180.0f;
    const float ca = std::cos(a);
    const float sa = std::sin(a);
    PointCloud out;
    for (const auto& p : in.points) {
        Point q;
        q.x = ca * p.x - sa * p.y + tx;
        q.y = sa * p.x + ca * p.y + ty;
        q.z = p.z + tz;
        out.points.push_back(q);
    }
    return out;
}

double maxPointError(const PointCloud& a, const PointCloud& b) {
    double worst = 0.0;
    for (size_t i = 0; i < a.points.size(); ++i) {
        const double dx = a.points[i].x - b.points[i].x;
        const double dy = a.points[i].y - b.points[i].y;
        const double dz = a.points[i].z - b.points[i].z;
        worst = std::max(worst, std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    return worst;
}

} // namespace

TEST_CASE("voxelDownsample reduces density but preserves extent") {
    PointCloud c = makeCloud(2000);
    PointCloud down = voxelDownsample(c, 0.02f);
    REQUIRE(down.points.size() < c.points.size());
    REQUIRE(down.points.size() > 10);
}

TEST_CASE("ICP returns identity/high fitness for already-aligned clouds") {
    PointCloud c = makeCloud(200);
    IcpResult r = icpPointToPoint(c, c);
    REQUIRE(r.fitness > 0.99);
    REQUIRE(r.transform.translationNorm() < 1e-4);
    REQUIRE(r.transform.rotationAngle() < 1e-3);
}

TEST_CASE("ICP recovers a pure translation") {
    PointCloud src = makeCloud(200);
    PointCloud tgt = rotateAboutZ(src, 0.0f, 0.02f, -0.01f, 0.015f);

    IcpOptions opts;
    opts.maxCorrespondenceDistM = 0.06f;
    opts.maxIterations = 60;
    IcpResult r = icpPointToPoint(src, tgt, RigidTransform::identity(), opts);

    REQUIRE(r.fitness > 0.9);
    REQUIRE(r.rmseM < 1e-3);
    REQUIRE(maxPointError(transformed(src, r.transform), tgt) < 2e-3);
}

TEST_CASE("ICP recovers a small rotation + translation") {
    PointCloud src = makeCloud(250);
    PointCloud tgt = rotateAboutZ(src, 8.0f, 0.01f, 0.005f, 0.0f);

    IcpOptions opts;
    opts.maxCorrespondenceDistM = 0.06f;
    opts.maxIterations = 80;
    IcpResult r = icpPointToPoint(src, tgt, RigidTransform::identity(), opts);

    REQUIRE(r.fitness > 0.9);
    REQUIRE(r.rmseM < 2e-3);
    REQUIRE(maxPointError(transformed(src, r.transform), tgt) < 4e-3);
}

TEST_CASE("ICP reports zero fitness for undersized clouds") {
    PointCloud tiny;
    for (int i = 0; i < 5; ++i) {
        tiny.points.push_back(Point{static_cast<float>(i) * 0.01f, 0, 0.3f, 0, 0, 0});
    }
    IcpResult r = icpPointToPoint(tiny, tiny);
    REQUIRE(r.fitness == 0.0);
}
