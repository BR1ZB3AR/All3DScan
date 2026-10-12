#include "DepthMesh.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

namespace revoscan::geometry {
namespace {

struct VoxelKey {
    int x = 0;
    int y = 0;
    int z = 0;
    bool operator==(const VoxelKey& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        return (static_cast<size_t>(k.x) * 73856093u) ^
               (static_cast<size_t>(k.y) * 19349663u) ^
               (static_cast<size_t>(k.z) * 83492791u);
    }
};

inline bool depthOk(uint16_t d, float nearM, float farM) {
    if (d == 0) {
        return false;
    }
    const float z = static_cast<float>(d) * 0.001f;
    if (nearM > 0.0f && z < nearM) {
        return false;
    }
    if (farM > 0.0f && z > farM) {
        return false;
    }
    return true;
}

inline bool isValidSurfaceTriangle(const pointcloud::Point& p0,
                                   const pointcloud::Point& p1,
                                   const pointcloud::Point& p2,
                                   float maxEdgeM) {
    const float dx01 = p1.x - p0.x, dy01 = p1.y - p0.y, dz01 = p1.z - p0.z;
    const float dx12 = p2.x - p1.x, dy12 = p2.y - p1.y, dz12 = p2.z - p1.z;
    const float dx20 = p0.x - p2.x, dy20 = p0.y - p2.y, dz20 = p0.z - p2.z;

    const float l01_sq = dx01 * dx01 + dy01 * dy01 + dz01 * dz01;
    const float l12_sq = dx12 * dx12 + dy12 * dy12 + dz12 * dz12;
    const float l20_sq = dx20 * dx20 + dy20 * dy20 + dz20 * dz20;

    const float maxSq = maxEdgeM * maxEdgeM;
    if (l01_sq > maxSq || l12_sq > maxSq || l20_sq > maxSq) {
        return false;
    }

    const float lMinSq = std::min({l01_sq, l12_sq, l20_sq});
    const float lMaxSq = std::max({l01_sq, l12_sq, l20_sq});
    if (lMinSq < 1e-9f) {
        return false;
    }

    // Aspect ratio filter: reject stretched needles (max/min > 3.5)
    if (lMaxSq > 12.25f * lMinSq) {
        return false;
    }

    // Compute surface normal via cross product
    const float nx = dy01 * dz12 - dz01 * dy12;
    const float ny = dz01 * dx12 - dx01 * dz12;
    const float nz = dx01 * dy12 - dy01 * dx12;
    const float normSq = nx * nx + ny * ny + nz * nz;
    if (normSq < 1e-12f) {
        return false;
    }

    // Reject grazing-angle z-streaks (|nz| / ||N|| < 0.20)
    const float absNz = std::abs(nz);
    if (absNz * absNz < 0.04f * normSq) {
        return false;
    }

    return true;
}

} // namespace

TriangleMesh meshFromOrganizedDepth(const uint16_t* depthMm,
                                    uint32_t width,
                                    uint32_t height,
                                    const protocol::CameraIntrinsics& intrinsics,
                                    const uint8_t* rgb,
                                    size_t rgbBytes,
                                    const DepthMeshOptions& opts) {
    TriangleMesh mesh;
    mesh.metric = true;
    if (!depthMm || width < 2 || height < 2 || intrinsics.fx <= 0.0 ||
        intrinsics.fy <= 0.0) {
        return mesh;
    }
    const int stride = std::max(1, opts.stride);
    const size_t expected = static_cast<size_t>(width) * height;
    const bool hasColor = rgb && rgbBytes >= expected * 3;
    mesh.hasColors = hasColor;

    // Grid of vertex indices for the subsampled lattice (-1 = invalid).
    const uint32_t gw = (width + static_cast<uint32_t>(stride) - 1) /
                        static_cast<uint32_t>(stride);
    const uint32_t gh = (height + static_cast<uint32_t>(stride) - 1) /
                        static_cast<uint32_t>(stride);
    std::vector<int32_t> indexGrid(static_cast<size_t>(gw) * gh, -1);
    mesh.vertices.reserve(static_cast<size_t>(gw) * gh / 2);

    for (uint32_t gy = 0; gy < gh; ++gy) {
        const uint32_t y = gy * static_cast<uint32_t>(stride);
        if (y >= height) {
            break;
        }
        for (uint32_t gx = 0; gx < gw; ++gx) {
            const uint32_t x = gx * static_cast<uint32_t>(stride);
            if (x >= width) {
                break;
            }
            const size_t di = static_cast<size_t>(y) * width + x;
            const uint16_t d = depthMm[di];
            if (!depthOk(d, opts.nearM, opts.farM)) {
                continue;
            }
            const float z = static_cast<float>(d) * 0.001f;
            pointcloud::Point p;
            p.x = (static_cast<float>(x) - static_cast<float>(intrinsics.cx)) * z /
                  static_cast<float>(intrinsics.fx);
            p.y = (static_cast<float>(y) - static_cast<float>(intrinsics.cy)) * z /
                  static_cast<float>(intrinsics.fy);
            p.z = z;
            if (hasColor) {
                p.r = rgb[di * 3 + 0];
                p.g = rgb[di * 3 + 1];
                p.b = rgb[di * 3 + 2];
            }
            indexGrid[static_cast<size_t>(gy) * gw + gx] =
                static_cast<int32_t>(mesh.vertices.size());
            mesh.vertices.push_back(p);
        }
    }

    auto depthAt = [&](uint32_t gx, uint32_t gy) -> float {
        const uint32_t x = gx * static_cast<uint32_t>(stride);
        const uint32_t y = gy * static_cast<uint32_t>(stride);
        if (x >= width || y >= height) {
            return 0.0f;
        }
        return static_cast<float>(depthMm[static_cast<size_t>(y) * width + x]) *
               0.001f;
    };

    mesh.indices.reserve(static_cast<size_t>(gw) * gh * 6);
    for (uint32_t gy = 0; gy + 1 < gh; ++gy) {
        for (uint32_t gx = 0; gx + 1 < gw; ++gx) {
            const int32_t i00 = indexGrid[static_cast<size_t>(gy) * gw + gx];
            const int32_t i10 = indexGrid[static_cast<size_t>(gy) * gw + gx + 1];
            const int32_t i01 = indexGrid[static_cast<size_t>(gy + 1) * gw + gx];
            const int32_t i11 = indexGrid[static_cast<size_t>(gy + 1) * gw + gx + 1];

            const float maxEdge = opts.maxEdgeLengthM > 0.0f ? opts.maxEdgeLengthM : 0.035f;

            // Triangle 1: (0,0) -> (1,0) -> (0,1)
            if (i00 >= 0 && i10 >= 0 && i01 >= 0) {
                const auto& p0 = mesh.vertices[static_cast<size_t>(i00)];
                const auto& p1 = mesh.vertices[static_cast<size_t>(i10)];
                const auto& p2 = mesh.vertices[static_cast<size_t>(i01)];
                if (isValidSurfaceTriangle(p0, p1, p2, maxEdge)) {
                    mesh.indices.push_back(static_cast<uint32_t>(i00));
                    mesh.indices.push_back(static_cast<uint32_t>(i10));
                    mesh.indices.push_back(static_cast<uint32_t>(i01));
                }
            }

            // Triangle 2: (1,0) -> (1,1) -> (0,1)
            if (i10 >= 0 && i11 >= 0 && i01 >= 0) {
                const auto& p0 = mesh.vertices[static_cast<size_t>(i10)];
                const auto& p1 = mesh.vertices[static_cast<size_t>(i11)];
                const auto& p2 = mesh.vertices[static_cast<size_t>(i01)];
                if (isValidSurfaceTriangle(p0, p1, p2, maxEdge)) {
                    mesh.indices.push_back(static_cast<uint32_t>(i10));
                    mesh.indices.push_back(static_cast<uint32_t>(i11));
                    mesh.indices.push_back(static_cast<uint32_t>(i01));
                }
            }
        }
    }
    return mesh;
}

void mergeMeshVoxel(TriangleMesh& dst,
                    const TriangleMesh& src,
                    float voxelSizeM,
                    size_t maxVertices) {
    if (src.empty() || voxelSizeM <= 0.0f) {
        return;
    }
    dst.metric = dst.metric || src.metric;
    dst.hasColors = dst.hasColors || src.hasColors;

    std::unordered_map<VoxelKey, uint32_t, VoxelKeyHash> cells;
    cells.reserve(dst.vertices.size() + src.vertices.size());
    for (size_t i = 0; i < dst.vertices.size(); ++i) {
        const auto& p = dst.vertices[i];
        VoxelKey key{static_cast<int>(std::floor(p.x / voxelSizeM)),
                     static_cast<int>(std::floor(p.y / voxelSizeM)),
                     static_cast<int>(std::floor(p.z / voxelSizeM))};
        cells.emplace(key, static_cast<uint32_t>(i));
    }

    std::vector<int32_t> remap(src.vertices.size(), -1);
    for (size_t i = 0; i < src.vertices.size(); ++i) {
        const auto& p = src.vertices[i];
        VoxelKey key{static_cast<int>(std::floor(p.x / voxelSizeM)),
                     static_cast<int>(std::floor(p.y / voxelSizeM)),
                     static_cast<int>(std::floor(p.z / voxelSizeM))};
        auto it = cells.find(key);
        if (it != cells.end()) {
            // Newest sample wins (coverage / fresher IR).
            dst.vertices[it->second] = p;
            remap[i] = static_cast<int32_t>(it->second);
            continue;
        }
        if (dst.vertices.size() >= maxVertices) {
            continue;
        }
        const auto idx = static_cast<uint32_t>(dst.vertices.size());
        dst.vertices.push_back(p);
        cells.emplace(key, idx);
        remap[i] = static_cast<int32_t>(idx);
    }

    for (size_t t = 0; t + 2 < src.indices.size(); t += 3) {
        const int32_t a = remap[src.indices[t + 0]];
        const int32_t b = remap[src.indices[t + 1]];
        const int32_t c = remap[src.indices[t + 2]];
        if (a < 0 || b < 0 || c < 0 || a == b || b == c || a == c) {
            continue;
        }
        dst.indices.push_back(static_cast<uint32_t>(a));
        dst.indices.push_back(static_cast<uint32_t>(b));
        dst.indices.push_back(static_cast<uint32_t>(c));
    }

    // Bound triangle growth roughly with vertex budget (drop oldest half if
    // grossly oversized — rare with maxVertices).
    constexpr size_t kMaxTris = 400000;
    if (dst.indices.size() / 3 > kMaxTris) {
        dst.indices.resize(kMaxTris * 3);
    }
}

} // namespace revoscan::geometry
