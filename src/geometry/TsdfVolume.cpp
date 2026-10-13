#include "geometry/TsdfVolume.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_set>

namespace revoscan::geometry {
namespace {

constexpr float kMaxWeight = 64.0f; // keeps late frames able to correct the model

} // namespace

TsdfVolume::TsdfVolume(const std::array<float, 3>& boxMin,
                       const std::array<float, 3>& boxMax,
                       const TsdfParams& params)
    : params_(params) {
    const float ex = std::max(boxMax[0] - boxMin[0], 1e-3f);
    const float ey = std::max(boxMax[1] - boxMin[1], 1e-3f);
    const float ez = std::max(boxMax[2] - boxMin[2], 1e-3f);
    // Grow the voxel until the dense grid fits the memory cap.
    voxel_ = std::max(params.voxelM, 1e-4f);
    const double minVoxel =
        std::cbrt(static_cast<double>(ex) * ey * ez / static_cast<double>(params.maxVoxels));
    voxel_ = std::max(voxel_, static_cast<float>(minVoxel));
    nx_ = std::max(2, static_cast<int>(std::ceil(ex / voxel_)) + 1);
    ny_ = std::max(2, static_cast<int>(std::ceil(ey / voxel_)) + 1);
    nz_ = std::max(2, static_cast<int>(std::ceil(ez / voxel_)) + 1);
    origin_ = boxMin;
    trunc_ = params.truncVoxels * voxel_;
    voxels_.assign(static_cast<size_t>(nx_) * ny_ * nz_, Voxel{});
}

void TsdfVolume::integrate(const std::vector<uint16_t>& depthMm,
                           const std::vector<uint8_t>& rgb,
                           uint32_t width,
                           uint32_t height,
                           const protocol::CameraIntrinsics& k,
                           const std::array<double, 16>& m) {
    if (width == 0 || height == 0 ||
        depthMm.size() != static_cast<size_t>(width) * height || k.fx <= 0 || k.fy <= 0) {
        return;
    }
    const bool hasColor = rgb.size() == depthMm.size() * 3;
    // World→camera: R^T (p - t).
    const double r00 = m[0], r01 = m[1], r02 = m[2], tx = m[3];
    const double r10 = m[4], r11 = m[5], r12 = m[6], ty = m[7];
    const double r20 = m[8], r21 = m[9], r22 = m[10], tz = m[11];

#pragma omp parallel for schedule(dynamic, 1)
    for (int z = 0; z < nz_; ++z) {
        const double wz = origin_[2] + z * voxel_;
        for (int y = 0; y < ny_; ++y) {
            const double wy = origin_[1] + y * voxel_;
            for (int x = 0; x < nx_; ++x) {
                const double wx = origin_[0] + x * voxel_;
                const double dx = wx - tx, dy = wy - ty, dz = wz - tz;
                const double cz = r02 * dx + r12 * dy + r22 * dz;
                if (cz < params_.minDepthM) {
                    continue;
                }
                const double cx = r00 * dx + r10 * dy + r20 * dz;
                const double cy = r01 * dx + r11 * dy + r21 * dz;
                const long u = std::lround(k.fx * cx / cz + k.cx);
                const long v = std::lround(k.fy * cy / cz + k.cy);
                if (u < 0 || v < 0 || u >= static_cast<long>(width) ||
                    v >= static_cast<long>(height)) {
                    continue;
                }
                const size_t pix = static_cast<size_t>(v) * width + static_cast<size_t>(u);
                const uint16_t d = depthMm[pix];
                if (d == 0) {
                    continue;
                }
                const double depthM = d * 0.001;
                if (depthM < params_.minDepthM || depthM > params_.maxDepthM) {
                    continue;
                }
                const double sdf = depthM - cz;
                if (sdf < -trunc_) {
                    continue; // behind the surface: unobserved
                }
                const float t = static_cast<float>(std::min(1.0, sdf / trunc_));
                Voxel& vox = voxels_[index(x, y, z)];
                const float w = vox.weight;
                vox.tsdf = (vox.tsdf * w + t) / (w + 1.0f);
                if (hasColor && std::fabs(sdf) < trunc_) {
                    const float a = 1.0f / (w + 1.0f);
                    vox.r = static_cast<uint8_t>(vox.r + (rgb[pix * 3 + 0] - vox.r) * a);
                    vox.g = static_cast<uint8_t>(vox.g + (rgb[pix * 3 + 1] - vox.g) * a);
                    vox.b = static_cast<uint8_t>(vox.b + (rgb[pix * 3 + 2] - vox.b) * a);
                }
                vox.weight = std::min(w + 1.0f, kMaxWeight);
            }
        }
    }
}

TriangleMesh TsdfVolume::extractMesh() const {
    TriangleMesh mesh;
    mesh.metric = true;
    auto observed = [&](const Voxel& v) { return v.weight >= params_.minWeight; };

    // Only the observed sub-box can hold surface: during a live scan most of
    // the volume is still empty, so this keeps periodic extraction cheap.
    int lo[3] = {nx_, ny_, nz_};
    int hi[3] = {-1, -1, -1};
    for (int z = 0; z < nz_; ++z) {
        for (int y = 0; y < ny_; ++y) {
            for (int x = 0; x < nx_; ++x) {
                if (observed(voxels_[index(x, y, z)])) {
                    lo[0] = std::min(lo[0], x);
                    lo[1] = std::min(lo[1], y);
                    lo[2] = std::min(lo[2], z);
                    hi[0] = std::max(hi[0], x);
                    hi[1] = std::max(hi[1], y);
                    hi[2] = std::max(hi[2], z);
                }
            }
        }
    }
    // Cells span voxel pairs, so cell indices run lo..hi-1 on each axis.
    const int bx = hi[0] - lo[0], by = hi[1] - lo[1], bz = hi[2] - lo[2];
    if (bx < 1 || by < 1 || bz < 1) {
        return mesh;
    }
    auto cellIndex = [&](int x, int y, int z) {
        return (static_cast<size_t>(z - lo[2]) * by + (y - lo[1])) * bx + (x - lo[0]);
    };
    std::vector<int32_t> cellVertex(static_cast<size_t>(bx) * by * bz, -1);

    // 1. One vertex per cell whose 8 observed corners straddle the surface,
    //    at the mean of its edge zero-crossings.
    static constexpr int kCorner[8][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0},
                                          {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};
    static constexpr int kEdge[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                                         {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (int z = lo[2]; z < hi[2]; ++z) {
        for (int y = lo[1]; y < hi[1]; ++y) {
            for (int x = lo[0]; x < hi[0]; ++x) {
                float s[8];
                const Voxel* c[8];
                bool ok = true;
                int negatives = 0;
                for (int i = 0; i < 8 && ok; ++i) {
                    c[i] = &voxels_[index(x + kCorner[i][0], y + kCorner[i][1], z + kCorner[i][2])];
                    ok = observed(*c[i]);
                    s[i] = c[i]->tsdf;
                    negatives += s[i] < 0.0f;
                }
                if (!ok || negatives == 0 || negatives == 8) {
                    continue;
                }
                double px = 0, py = 0, pz = 0, cr = 0, cg = 0, cb = 0;
                int crossings = 0;
                for (const auto& e : kEdge) {
                    const float a = s[e[0]], b = s[e[1]];
                    if ((a < 0.0f) == (b < 0.0f)) {
                        continue;
                    }
                    const double t = a / (a - b);
                    for (int k = 0; k < 3; ++k) {
                        const double p0 = kCorner[e[0]][k], p1 = kCorner[e[1]][k];
                        (k == 0 ? px : k == 1 ? py : pz) += p0 + t * (p1 - p0);
                    }
                    const Voxel& near = std::fabs(a) < std::fabs(b) ? *c[e[0]] : *c[e[1]];
                    cr += near.r;
                    cg += near.g;
                    cb += near.b;
                    ++crossings;
                }
                pointcloud::Point p;
                p.x = static_cast<float>(origin_[0] + (x + px / crossings) * voxel_);
                p.y = static_cast<float>(origin_[1] + (y + py / crossings) * voxel_);
                p.z = static_cast<float>(origin_[2] + (z + pz / crossings) * voxel_);
                p.r = static_cast<uint8_t>(cr / crossings);
                p.g = static_cast<uint8_t>(cg / crossings);
                p.b = static_cast<uint8_t>(cb / crossings);
                cellVertex[cellIndex(x, y, z)] = static_cast<int32_t>(mesh.vertices.size());
                mesh.vertices.push_back(p);
            }
        }
    }

    // 2. One quad per sign-changing grid edge, joining the 4 cells around it.
    //    For an edge along axis a, the other axes (b, c) are taken cyclically so
    //    the quad's right-hand normal points along +a; flip when the start
    //    voxel is outside (normals point from inside to outside).
    for (int z = lo[2]; z <= hi[2]; ++z) {
        for (int y = lo[1]; y <= hi[1]; ++y) {
            for (int x = lo[0]; x <= hi[0]; ++x) {
                const Voxel& v0 = voxels_[index(x, y, z)];
                if (!observed(v0)) {
                    continue;
                }
                for (int axis = 0; axis < 3; ++axis) {
                    int e[3] = {x, y, z};
                    ++e[axis];
                    if (e[0] >= nx_ || e[1] >= ny_ || e[2] >= nz_) {
                        continue;
                    }
                    const Voxel& v1 = voxels_[index(e[0], e[1], e[2])];
                    if (!observed(v1) || (v0.tsdf < 0.0f) == (v1.tsdf < 0.0f)) {
                        continue;
                    }
                    const int b = (axis + 1) % 3;
                    const int c = (axis + 2) % 3;
                    int32_t q[4];
                    bool ok = true;
                    static constexpr int kOff[4][2] = {{-1, -1}, {0, -1}, {0, 0}, {-1, 0}};
                    for (int i = 0; i < 4 && ok; ++i) {
                        int cell[3] = {x, y, z};
                        cell[b] += kOff[i][0];
                        cell[c] += kOff[i][1];
                        if (cell[0] < lo[0] || cell[1] < lo[1] || cell[2] < lo[2] ||
                            cell[0] >= hi[0] || cell[1] >= hi[1] || cell[2] >= hi[2]) {
                            ok = false;
                            break;
                        }
                        q[i] = cellVertex[cellIndex(cell[0], cell[1], cell[2])];
                        ok = q[i] >= 0;
                    }
                    if (!ok) {
                        continue;
                    }
                    if (v0.tsdf >= 0.0f) {
                        std::swap(q[1], q[3]);
                    }
                    for (int t : {q[0], q[1], q[2], q[0], q[2], q[3]}) {
                        mesh.indices.push_back(static_cast<uint32_t>(t));
                    }
                }
            }
        }
    }
    mesh.hasColors = true;
    return mesh;
}

TriangleMesh reconstructTsdf(const std::vector<TsdfFrame>& frames,
                             const pointcloud::PointCloud& bounds,
                             float padM,
                             const TsdfParams& params) {
    if (frames.empty() || bounds.points.empty()) {
        return {};
    }
    std::array<float, 3> lo{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max()};
    std::array<float, 3> hi{std::numeric_limits<float>::lowest(),
                            std::numeric_limits<float>::lowest(),
                            std::numeric_limits<float>::lowest()};
    for (const auto& p : bounds.points) {
        lo = {std::min(lo[0], p.x), std::min(lo[1], p.y), std::min(lo[2], p.z)};
        hi = {std::max(hi[0], p.x), std::max(hi[1], p.y), std::max(hi[2], p.z)};
    }
    for (int i = 0; i < 3; ++i) {
        lo[i] -= padM;
        hi[i] += padM;
    }
    TsdfParams p = params;
    if (frames.size() < 3) {
        p.minWeight = 1.0f; // single-view previews: one observation is all there is
    }
    TsdfVolume volume(lo, hi, p);
    for (const auto& f : frames) {
        static const std::vector<uint8_t> kNoColor;
        volume.integrate(*f.depthMm, f.rgb ? *f.rgb : kNoColor, f.width, f.height, f.intrinsics,
                         f.cameraToWorld);
    }
    return volume.extractMesh();
}

size_t cleanMesh(TriangleMesh& mesh,
                 const pointcloud::PointCloud& keep,
                 float nearM,
                 float minComponentShare) {
    const size_t trisBefore = mesh.triangleCount();
    if (mesh.empty()) {
        return 0;
    }
    std::vector<uint8_t> alive(mesh.vertices.size(), 1);

    // 1. Base removal: vertices with no tracked-model point nearby.
    if (!keep.points.empty() && nearM > 0.0f) {
        auto key = [nearM](float x, float y, float z) {
            const auto ix = static_cast<int64_t>(std::floor(x / nearM));
            const auto iy = static_cast<int64_t>(std::floor(y / nearM));
            const auto iz = static_cast<int64_t>(std::floor(z / nearM));
            return (ix * 73856093) ^ (iy * 19349663) ^ (iz * 83492791);
        };
        std::unordered_set<int64_t> occupied;
        occupied.reserve(keep.points.size());
        for (const auto& p : keep.points) {
            occupied.insert(key(p.x, p.y, p.z));
        }
        for (size_t i = 0; i < mesh.vertices.size(); ++i) {
            const auto& v = mesh.vertices[i];
            bool found = false;
            for (int dz = -1; dz <= 1 && !found; ++dz) {
                for (int dy = -1; dy <= 1 && !found; ++dy) {
                    for (int dx = -1; dx <= 1 && !found; ++dx) {
                        found = occupied.count(key(v.x + dx * nearM, v.y + dy * nearM,
                                                   v.z + dz * nearM)) > 0;
                    }
                }
            }
            alive[i] = found;
        }
    }

    // 2. Connected components over surviving triangles (union-find on vertices).
    std::vector<uint32_t> parent(mesh.vertices.size());
    std::iota(parent.begin(), parent.end(), 0u);
    auto find = [&](uint32_t a) {
        while (parent[a] != a) {
            parent[a] = parent[parent[a]];
            a = parent[a];
        }
        return a;
    };
    auto triAlive = [&](size_t t) {
        return alive[mesh.indices[t]] && alive[mesh.indices[t + 1]] && alive[mesh.indices[t + 2]];
    };
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        if (!triAlive(t)) {
            continue;
        }
        const uint32_t a = find(mesh.indices[t]);
        parent[find(mesh.indices[t + 1])] = a;
        parent[find(mesh.indices[t + 2])] = a;
    }
    std::vector<size_t> compTris(mesh.vertices.size(), 0);
    size_t largest = 0;
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        if (triAlive(t)) {
            largest = std::max(largest, ++compTris[find(mesh.indices[t])]);
        }
    }
    const auto minTris = static_cast<size_t>(minComponentShare * static_cast<float>(largest));

    // 3. Rebuild with compacted vertices.
    std::vector<int64_t> remap(mesh.vertices.size(), -1);
    TriangleMesh out;
    out.metric = mesh.metric;
    out.hasColors = mesh.hasColors;
    out.hasNormals = mesh.hasNormals;
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        if (!triAlive(t) || compTris[find(mesh.indices[t])] < std::max<size_t>(minTris, 1)) {
            continue;
        }
        for (int k = 0; k < 3; ++k) {
            const uint32_t v = mesh.indices[t + k];
            if (remap[v] < 0) {
                remap[v] = static_cast<int64_t>(out.vertices.size());
                out.vertices.push_back(mesh.vertices[v]);
            }
            out.indices.push_back(static_cast<uint32_t>(remap[v]));
        }
    }
    mesh = std::move(out);
    return trisBefore - mesh.triangleCount();
}

void smoothMeshTaubin(TriangleMesh& mesh, int iterations, float lambda, float mu) {
    const size_t n = mesh.vertices.size();
    if (n == 0 || mesh.indices.size() < 3 || iterations <= 0) {
        return;
    }
    // Vertex adjacency (CSR) from triangle edges.
    std::vector<uint32_t> degree(n, 0);
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        for (int k = 0; k < 3; ++k) {
            degree[mesh.indices[t + k]] += 2;
        }
    }
    std::vector<size_t> start(n + 1, 0);
    for (size_t i = 0; i < n; ++i) {
        start[i + 1] = start[i] + degree[i];
    }
    std::vector<uint32_t> nbr(start[n]);
    std::vector<size_t> fill(start.begin(), start.end() - 1);
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const uint32_t a = mesh.indices[t], b = mesh.indices[t + 1], c = mesh.indices[t + 2];
        nbr[fill[a]++] = b;
        nbr[fill[a]++] = c;
        nbr[fill[b]++] = a;
        nbr[fill[b]++] = c;
        nbr[fill[c]++] = a;
        nbr[fill[c]++] = b;
    }
    std::vector<float> pos(n * 3);
    for (size_t i = 0; i < n; ++i) {
        pos[i * 3 + 0] = mesh.vertices[i].x;
        pos[i * 3 + 1] = mesh.vertices[i].y;
        pos[i * 3 + 2] = mesh.vertices[i].z;
    }
    std::vector<float> next(pos.size());
    auto step = [&](float factor) {
#pragma omp parallel for schedule(static)
        for (long i = 0; i < static_cast<long>(n); ++i) {
            const size_t b = start[static_cast<size_t>(i)], e = start[static_cast<size_t>(i) + 1];
            float* out = &next[static_cast<size_t>(i) * 3];
            const float* p = &pos[static_cast<size_t>(i) * 3];
            if (b == e) {
                out[0] = p[0];
                out[1] = p[1];
                out[2] = p[2];
                continue;
            }
            // Duplicated edges weight shared neighbours twice, which is fine for
            // a uniform umbrella operator on near-regular Surface Nets meshes.
            float sx = 0, sy = 0, sz = 0;
            for (size_t j = b; j < e; ++j) {
                const float* q = &pos[static_cast<size_t>(nbr[j]) * 3];
                sx += q[0];
                sy += q[1];
                sz += q[2];
            }
            const float inv = 1.0f / static_cast<float>(e - b);
            out[0] = p[0] + factor * (sx * inv - p[0]);
            out[1] = p[1] + factor * (sy * inv - p[1]);
            out[2] = p[2] + factor * (sz * inv - p[2]);
        }
        pos.swap(next);
    };
    for (int it = 0; it < iterations; ++it) {
        step(lambda);
        step(mu);
    }
    for (size_t i = 0; i < n; ++i) {
        mesh.vertices[i].x = pos[i * 3 + 0];
        mesh.vertices[i].y = pos[i * 3 + 1];
        mesh.vertices[i].z = pos[i * 3 + 2];
    }
}

} // namespace revoscan::geometry
