#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

#include "geometry/TsdfVolume.h"

using namespace revoscan;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr float kRadius = 0.05f; // 5 cm sphere at the origin

// Camera at distance 0.3 m looking at the origin from (yaw, pitch); returns
// camera→world pose and a rendered depth map of the sphere.
geometry::TsdfFrame renderSphere(double yaw, double pitch, std::vector<uint16_t>& depth,
                                 std::vector<uint8_t>& rgb) {
    const uint32_t w = 160, h = 160;
    protocol::CameraIntrinsics k{.fx = 200, .fy = 200, .cx = 80, .cy = 80};
    // Camera z axis points at the origin.
    const double cxw = 0.3 * std::cos(pitch) * std::sin(yaw);
    const double cyw = 0.3 * std::sin(pitch);
    const double czw = -0.3 * std::cos(pitch) * std::cos(yaw);
    double fz[3] = {-cxw / 0.3, -cyw / 0.3, -czw / 0.3};
    double up[3] = {0, 1, 0};
    if (std::fabs(fz[1]) > 0.95) {
        up[0] = 1;
        up[1] = 0;
    }
    double fx[3] = {up[1] * fz[2] - up[2] * fz[1], up[2] * fz[0] - up[0] * fz[2],
                    up[0] * fz[1] - up[1] * fz[0]};
    const double nx = std::sqrt(fx[0] * fx[0] + fx[1] * fx[1] + fx[2] * fx[2]);
    for (double& v : fx) {
        v /= nx;
    }
    const double fy[3] = {fz[1] * fx[2] - fz[2] * fx[1], fz[2] * fx[0] - fz[0] * fx[2],
                          fz[0] * fx[1] - fz[1] * fx[0]};
    geometry::TsdfFrame f;
    f.width = w;
    f.height = h;
    f.intrinsics = k;
    f.cameraToWorld = {fx[0], fy[0], fz[0], cxw, fx[1], fy[1], fz[1], cyw,
                       fx[2], fy[2], fz[2], czw, 0, 0, 0, 1};
    depth.assign(static_cast<size_t>(w) * h, 0);
    rgb.assign(depth.size() * 3, 200);
    for (uint32_t v = 0; v < h; ++v) {
        for (uint32_t u = 0; u < w; ++u) {
            // Ray in world space; intersect with the sphere.
            const double dc[3] = {(u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0};
            double d[3];
            for (int i = 0; i < 3; ++i) {
                d[i] = fx[i] * dc[0] + fy[i] * dc[1] + fz[i] * dc[2];
            }
            const double o[3] = {cxw, cyw, czw};
            const double a = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
            const double b = 2 * (o[0] * d[0] + o[1] * d[1] + o[2] * d[2]);
            const double c = o[0] * o[0] + o[1] * o[1] + o[2] * o[2] - kRadius * kRadius;
            const double disc = b * b - 4 * a * c;
            if (disc < 0) {
                continue;
            }
            const double t = (-b - std::sqrt(disc)) / (2 * a); // camera-z depth (dc.z = 1)
            depth[static_cast<size_t>(v) * w + u] = static_cast<uint16_t>(std::lround(t * 1000.0));
        }
    }
    f.depthMm = &depth;
    f.rgb = &rgb;
    return f;
}

} // namespace

TEST_CASE("TSDF + Surface Nets reconstructs a closed sphere from six views") {
    const double views[6][2] = {{0, 0}, {kPi / 2, 0}, {kPi, 0}, {3 * kPi / 2, 0},
                                {0, 1.3}, {0, -1.3}};
    std::vector<std::vector<uint16_t>> depths(6);
    std::vector<std::vector<uint8_t>> rgbs(6);
    std::vector<geometry::TsdfFrame> frames;
    for (int i = 0; i < 6; ++i) {
        frames.push_back(renderSphere(views[i][0], views[i][1], depths[i], rgbs[i]));
    }
    pointcloud::PointCloud bounds;
    bounds.points.push_back({-kRadius, -kRadius, -kRadius});
    bounds.points.push_back({kRadius, kRadius, kRadius});
    geometry::TsdfParams params;
    params.voxelM = 0.002f;
    const auto mesh = geometry::reconstructTsdf(frames, bounds, 0.015f, params);
    REQUIRE(mesh.triangleCount() > 1000);

    // Vertices on the sphere (depth is quantised to 1 mm).
    double maxErr = 0.0;
    for (const auto& v : mesh.vertices) {
        const double r = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        maxErr = std::max(maxErr, std::fabs(r - kRadius));
    }
    REQUIRE(maxErr < 0.002);

    // Faces wound outward: normal · centroid > 0 for nearly all triangles.
    size_t outward = 0;
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const auto& a = mesh.vertices[mesh.indices[t]];
        const auto& b = mesh.vertices[mesh.indices[t + 1]];
        const auto& c = mesh.vertices[mesh.indices[t + 2]];
        const double ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
        const double vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
        const double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        const double mx = a.x + b.x + c.x, my = a.y + b.y + c.y, mz = a.z + b.z + c.z;
        outward += (nx * mx + ny * my + nz * mz) > 0.0;
    }
    REQUIRE(outward > mesh.triangleCount() * 99 / 100);
    REQUIRE(mesh.hasColors);
    REQUIRE(mesh.vertices[0].r == 200);
}

TEST_CASE("cleanMesh removes surface away from the model and small fragments") {
    // Two separate quads: one near the "model" cloud, one far away; plus a
    // tiny fragment near the model that is far smaller than the main part.
    geometry::TriangleMesh mesh;
    auto addQuadGrid = [&](float x0, float z, int n, float step) {
        const auto base = static_cast<uint32_t>(mesh.vertices.size());
        for (int j = 0; j <= n; ++j) {
            for (int i = 0; i <= n; ++i) {
                mesh.vertices.push_back({x0 + i * step, j * step, z});
            }
        }
        for (int j = 0; j < n; ++j) {
            for (int i = 0; i < n; ++i) {
                const uint32_t a = base + j * (n + 1) + i;
                for (uint32_t v : {a, a + 1, a + n + 2, a, a + n + 2, a + n + 1}) {
                    mesh.indices.push_back(v);
                }
            }
        }
    };
    addQuadGrid(0.0f, 0.3f, 20, 0.002f);   // object surface (400 cells)
    addQuadGrid(0.2f, 0.3f, 20, 0.002f);   // desk patch far from the model
    addQuadGrid(0.0f, 0.36f, 1, 0.002f);   // speck within reach of the model
    pointcloud::PointCloud model;
    for (int j = 0; j <= 20; ++j) {
        for (int i = 0; i <= 20; ++i) {
            model.points.push_back({i * 0.002f, j * 0.002f, 0.3f});
        }
    }
    model.points.push_back({0.0f, 0.0f, 0.36f});
    const size_t total = mesh.triangleCount();
    const size_t removed = geometry::cleanMesh(mesh, model, 0.004f, 0.02f);
    REQUIRE(mesh.triangleCount() == 800);
    REQUIRE(removed == total - 800);
    for (const auto& v : mesh.vertices) {
        REQUIRE(v.x < 0.05f);
        REQUIRE(v.z < 0.31f);
    }
}

TEST_CASE("Taubin smoothing reduces noise without shrinking a sphere") {
    const double views[6][2] = {{0, 0}, {kPi / 2, 0}, {kPi, 0}, {3 * kPi / 2, 0},
                                {0, 1.3}, {0, -1.3}};
    std::vector<std::vector<uint16_t>> depths(6);
    std::vector<std::vector<uint8_t>> rgbs(6);
    std::vector<geometry::TsdfFrame> frames;
    for (int i = 0; i < 6; ++i) {
        frames.push_back(renderSphere(views[i][0], views[i][1], depths[i], rgbs[i]));
    }
    pointcloud::PointCloud bounds;
    bounds.points.push_back({-kRadius, -kRadius, -kRadius});
    bounds.points.push_back({kRadius, kRadius, kRadius});
    geometry::TsdfParams params;
    params.voxelM = 0.002f;
    auto mesh = geometry::reconstructTsdf(frames, bounds, 0.015f, params);
    // Add deterministic radial noise of ±1 mm.
    uint32_t state = 99;
    for (auto& v : mesh.vertices) {
        state = state * 1664525u + 1013904223u;
        const float n = (static_cast<float>(state >> 8) / static_cast<float>(1u << 24) - 0.5f) * 0.002f;
        const float r = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        v.x *= (r + n) / r;
        v.y *= (r + n) / r;
        v.z *= (r + n) / r;
    }
    auto radialStats = [&](double* meanR) {
        double sum = 0, sum2 = 0;
        for (const auto& v : mesh.vertices) {
            const double r = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            sum += r;
            sum2 += (r - kRadius) * (r - kRadius);
        }
        *meanR = sum / mesh.vertices.size();
        return std::sqrt(sum2 / mesh.vertices.size());
    };
    double meanBefore = 0, meanAfter = 0;
    const double errBefore = radialStats(&meanBefore);
    geometry::smoothMeshTaubin(mesh, 10);
    const double errAfter = radialStats(&meanAfter);
    REQUIRE(errAfter < errBefore * 0.6);
    REQUIRE(std::fabs(meanAfter - kRadius) < 0.0005); // no shrinkage
}
