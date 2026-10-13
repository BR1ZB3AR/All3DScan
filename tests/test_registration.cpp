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

namespace {

// Inside corner of a box: three orthogonal 6 cm faces sampled on a 2 mm grid.
// Plane-rich and feature-poor, like the cartons people scan.
PointCloud makeBoxCorner() {
    PointCloud c;
    for (int i = 0; i < 30; ++i) {
        for (int j = 0; j < 30; ++j) {
            const float a = 0.002f * static_cast<float>(i);
            const float b = 0.002f * static_cast<float>(j);
            c.points.push_back(Point{a, b, 0.30f});
            c.points.push_back(Point{a, 0.0f, 0.30f - b});
            c.points.push_back(Point{0.0f, a, 0.30f - b});
        }
    }
    return c;
}

} // namespace

TEST_CASE("point-to-plane ICP recovers motion on a plane-only box corner") {
    const PointCloud target = makeBoxCorner();
    // Small rotation plus a 6 mm translation partly along the faces.
    const PointCloud source = rotateAboutZ(target, 2.0f, 0.004f, -0.003f, 0.003f);

    PlaneIcpOptions opts;
    opts.maxCorrespondenceDistM = 0.02f;
    opts.maxIterations = 40;
    const IcpResult r = icpPointToPlane(source, target, RigidTransform::identity(), opts);
    REQUIRE(r.fitness > 0.95);

    // Source mapped back must land on the target within a fraction of a mm.
    const PointCloud aligned = transformed(source, r.transform);
    double maxErr = 0.0;
    for (size_t i = 0; i < target.points.size(); ++i) {
        const double dx = aligned.points[i].x - target.points[i].x;
        const double dy = aligned.points[i].y - target.points[i].y;
        const double dz = aligned.points[i].z - target.points[i].z;
        maxErr = std::max(maxErr, std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    REQUIRE(maxErr < 0.0005);
}

namespace {

double orthonormalityError(const RigidTransform& t) {
    double worst = 0.0;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double d = 0.0;
            for (int k = 0; k < 3; ++k) {
                d += t.m[r * 4 + k] * t.m[c * 4 + k];
            }
            worst = std::max(worst, std::fabs(d - (r == c ? 1.0 : 0.0)));
        }
    }
    return worst;
}

RigidTransform rotationAboutZ(double deg, double tx, double ty, double tz) {
    const double a = deg * 3.14159265358979 / 180.0;
    RigidTransform t;
    t.m = {std::cos(a), -std::sin(a), 0, tx, std::sin(a), std::cos(a), 0, ty,
           0,           0,            1, tz, 0,           0,           0, 1};
    return t;
}

} // namespace

TEST_CASE("rigid helpers: inverse, re-orthonormalization, scaled motion") {
    const RigidTransform t = rotationAboutZ(20.0, 0.01, -0.02, 0.03);
    const RigidTransform id = inverted(t) * t;
    for (int i = 0; i < 16; ++i) {
        REQUIRE_THAT(id.m[static_cast<size_t>(i)],
                     Catch::Matchers::WithinAbs(RigidTransform::identity().m[static_cast<size_t>(i)], 1e-12));
    }

    RigidTransform skewed = t;
    skewed.m[0] *= 1.05; // a stretched, sheared "rotation"
    skewed.m[1] += 0.02;
    const RigidTransform fixed = orthonormalized(skewed);
    REQUIRE(orthonormalityError(fixed) < 1e-12);
    REQUIRE(fixed.m[3] == t.m[3]);
    REQUIRE(fixed.rotationAngle() < 0.4); // still close to the 20° rotation

    const RigidTransform half = scaledMotion(t, 0.5);
    REQUIRE_THAT(half.rotationAngle(), Catch::Matchers::WithinAbs(t.rotationAngle() / 2, 1e-9));
    REQUIRE_THAT(half.translationNorm(), Catch::Matchers::WithinAbs(t.translationNorm() / 2, 1e-12));
    const RigidTransform same = scaledMotion(t, 1.0);
    for (int i = 0; i < 16; ++i) {
        REQUIRE_THAT(same.m[static_cast<size_t>(i)],
                     Catch::Matchers::WithinAbs(t.m[static_cast<size_t>(i)], 1e-12));
    }
}

TEST_CASE("constant-velocity pose prediction stays rigid over long scans") {
    // Regression: extrapolating last * prev^-1 * last from transposes let
    // round-off grow ~2.4x per frame until poses were stretched 2-3x.
    RigidTransform prev = RigidTransform::identity();
    RigidTransform last = rotationAboutZ(0.7, 0.001, 0.0005, -0.0002);
    for (int frame = 0; frame < 500; ++frame) {
        const RigidTransform step = inverted(prev) * last;
        const RigidTransform next = orthonormalized(last * scaledMotion(step, 0.7));
        prev = last;
        last = next;
    }
    REQUIRE(orthonormalityError(last) < 1e-12);
    REQUIRE(orthonormalityError(inverted(prev) * last) < 1e-12);
}

TEST_CASE("alignment quality separates a real fit from a near miss or another shape") {
    const PlaneIcpTarget target(makeBoxCorner(), 0.006f);
    const double tol = 0.004;

    const AlignmentQuality same =
        evaluateAlignment(makeBoxCorner(), target, RigidTransform::identity(), tol);
    REQUIRE(same.overlap > 0.99);
    REQUIRE(same.rmseM < 0.0005);

    // Off by 1 cm on every axis: every face leaves its plane.
    const AlignmentQuality shifted =
        evaluateAlignment(makeBoxCorner(), target, rotationAboutZ(0.0, 0.01, 0.01, -0.01), tol);
    REQUIRE(shifted.overlap < 0.1);

    // A different object (sphere, r = 3 cm) sitting in the corner.
    PointCloud sphere;
    for (int i = 0; i < 40; ++i) {
        for (int j = 0; j < 80; ++j) {
            const double th = 3.14159265358979 * (i + 0.5) / 40.0;
            const double ph = 2.0 * 3.14159265358979 * j / 80.0;
            sphere.points.push_back(Point{static_cast<float>(0.03 + 0.03 * std::sin(th) * std::cos(ph)),
                                          static_cast<float>(0.03 + 0.03 * std::sin(th) * std::sin(ph)),
                                          static_cast<float>(0.27 + 0.03 * std::cos(th))});
        }
    }
    const AlignmentQuality other =
        evaluateAlignment(sphere, target, RigidTransform::identity(), tol);
    REQUIRE(other.overlap < 0.3);
}
