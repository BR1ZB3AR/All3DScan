#include "all3dscan/CloudSurface.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

// PoissonRecon wants this first; it decides what the rest compiles as.
#include "PreProcessor.h"

#include "Reconstructors.h"

namespace all3dscan::mesh {

namespace {

namespace pr = PoissonRecon;

using Real = float;
constexpr unsigned int kDim = 3;

// Too few points describe no surface, and the solver is unhappy with them.
constexpr uint64_t kFewestPoints = 64;

// The cloud, handed to the solver one oriented point at a time.
struct CloudStream : public pr::Reconstructor::InputOrientedSampleStream<Real, kDim> {
    CloudStream(const float* points, const float* normals, uint64_t count)
        : points_(points), normals_(normals), count_(count) {}

    void reset(void) override { at_ = 0; }

    bool read(pr::Point<Real, kDim>& p, pr::Point<Real, kDim>& n) override {
        // Points the scanner could not place, and normals that never settled
        // on a direction, would drag the surface towards nowhere.
        while (at_ < count_) {
            const float* point = points_ + at_ * 3;
            const float* normal = normals_ + at_ * 3;
            at_++;
            const float length = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] +
                                           normal[2] * normal[2]);
            if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
                !std::isfinite(point[2]) || !std::isfinite(length) || length < 1e-6f) {
                continue;
            }
            for (unsigned int d = 0; d < kDim; d++) {
                p[d] = point[d];
                n[d] = normal[d] / length;
            }
            return true;
        }
        return false;
    }

private:
    const float* points_;
    const float* normals_;
    uint64_t count_;
    uint64_t at_ = 0;
};

// The vertices the solver draws, with how much data stood behind each.
struct VertexSink : public pr::Reconstructor::OutputLevelSetVertexStream<Real, kDim> {
    VertexSink(std::vector<float>& into, std::vector<float>& density)
        : into_(into), density_(density) {}

    size_t size(void) const override { return density_.size(); }

    size_t write(const pr::Point<Real, kDim>& p, const pr::Point<Real, kDim>&,
                 const Real& weight) override {
        for (unsigned int d = 0; d < kDim; d++) {
            into_.push_back(p[d]);
        }
        density_.push_back(weight);
        return density_.size() - 1;
    }

private:
    std::vector<float>& into_;
    std::vector<float>& density_;
};

// Its faces. forceManifold is on, so these arrive as triangles; anything with
// more corners is fanned, as a precaution rather than an expectation.
struct FaceSink : public pr::Reconstructor::OutputFaceStream<2> {
    explicit FaceSink(std::vector<uint32_t>& into) : into_(into) {}

    size_t size(void) const override { return count_; }

    size_t write(const std::vector<pr::node_index_type>& polygon) override {
        for (size_t corner = 1; corner + 1 < polygon.size(); corner++) {
            into_.push_back(static_cast<uint32_t>(polygon[0]));
            into_.push_back(static_cast<uint32_t>(polygon[corner]));
            into_.push_back(static_cast<uint32_t>(polygon[corner + 1]));
        }
        return count_++;
    }

private:
    std::vector<uint32_t>& into_;
    size_t count_ = 0;
};

// Drops the vertices with the least data behind them, and every triangle that
// used one. The surface stops being closed, which is the point: what is cut
// is the part Poisson invented to close over a place the scan never reached.
void trimByDensity(CloudSurfaceMesh& mesh, float share) {
    if (share <= 0.0f || mesh.density.empty()) {
        return;
    }
    std::vector<float> sorted = mesh.density;
    std::sort(sorted.begin(), sorted.end());
    const size_t at = std::min(sorted.size() - 1,
                               static_cast<size_t>(sorted.size() * share));
    const float least = sorted[at];

    std::vector<uint32_t> renumbered(mesh.density.size(), UINT32_MAX);
    std::vector<float> vertices;
    std::vector<float> density;
    vertices.reserve(mesh.vertices.size());
    density.reserve(mesh.density.size());
    for (size_t v = 0; v < mesh.density.size(); v++) {
        if (mesh.density[v] < least) {
            continue;
        }
        renumbered[v] = static_cast<uint32_t>(density.size());
        for (int d = 0; d < 3; d++) {
            vertices.push_back(mesh.vertices[v * 3 + d]);
        }
        density.push_back(mesh.density[v]);
    }

    std::vector<uint32_t> indices;
    indices.reserve(mesh.indices.size());
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const uint32_t a = renumbered[mesh.indices[i]];
        const uint32_t b = renumbered[mesh.indices[i + 1]];
        const uint32_t c = renumbered[mesh.indices[i + 2]];
        if (a == UINT32_MAX || b == UINT32_MAX || c == UINT32_MAX) {
            continue;
        }
        indices.push_back(a);
        indices.push_back(b);
        indices.push_back(c);
    }

    mesh.trimmed_vertices = mesh.density.size() - density.size();
    mesh.trimmed_triangles = (mesh.indices.size() - indices.size()) / 3;
    mesh.vertices = std::move(vertices);
    mesh.density = std::move(density);
    mesh.indices = std::move(indices);
}

// A normal per vertex from the triangles sharing it, as SurfaceBuilder does:
// each cross product is twice its triangle's area, so adding them unscaled
// lets the larger faces weigh more.
void fillNormals(CloudSurfaceMesh& mesh) {
    mesh.normals.assign(mesh.vertices.size(), 0.0f);
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const float* a = &mesh.vertices[mesh.indices[i] * 3];
        const float* b = &mesh.vertices[mesh.indices[i + 1] * 3];
        const float* c = &mesh.vertices[mesh.indices[i + 2] * 3];
        const float u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        const float v[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        const float n[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                            u[0] * v[1] - u[1] * v[0]};
        for (int corner = 0; corner < 3; corner++) {
            const size_t at = mesh.indices[i + corner] * 3;
            for (int d = 0; d < 3; d++) {
                mesh.normals[at + d] += n[d];
            }
        }
    }
    for (size_t at = 0; at < mesh.normals.size(); at += 3) {
        const float length = std::sqrt(mesh.normals[at] * mesh.normals[at] +
                                       mesh.normals[at + 1] * mesh.normals[at + 1] +
                                       mesh.normals[at + 2] * mesh.normals[at + 2]);
        if (length > 1e-12f) {
            for (int d = 0; d < 3; d++) {
                mesh.normals[at + d] /= length;
            }
        } else {
            mesh.normals[at] = 0.0f;
            mesh.normals[at + 1] = 0.0f;
            mesh.normals[at + 2] = -1.0f; // a vertex no triangle reached
        }
    }
}

} // namespace

CloudSurfaceMesh reconstructFromCloud(const float* points, const float* normals, uint64_t count,
                               const CloudSurfaceSettings& settings) {
    if (points == nullptr || normals == nullptr) {
        throw std::invalid_argument("Poisson: no points to reconstruct from");
    }
    if (count < kFewestPoints) {
        throw std::invalid_argument("Poisson: " + std::to_string(count) +
                                    " points is too few to build a surface from, at least " +
                                    std::to_string(kFewestPoints) + " are needed");
    }
    if (settings.depth < 4 || settings.depth > 12) {
        throw std::invalid_argument("Poisson: the detail must be 4 to 12, got " +
                                    std::to_string(settings.depth));
    }
    if (!(settings.trim >= 0.0f && settings.trim < 0.9f)) {
        throw std::invalid_argument("Poisson: the trim must be 0 to 0.9, got " +
                                    std::to_string(settings.trim));
    }

    static const unsigned int kFEMSig =
        pr::FEMDegreeAndBType<pr::Reconstructor::Poisson::DefaultFEMDegree,
                              pr::Reconstructor::Poisson::DefaultFEMBoundary>::Signature;
    using FEMSigs = pr::IsotropicUIntPack<kDim, kFEMSig>;
    using Implicit = pr::Reconstructor::Implicit<Real, kDim, FEMSigs>;
    using Solver = pr::Reconstructor::Poisson::Solver<Real, kDim, FEMSigs>;

    pr::Reconstructor::Poisson::SolutionParameters<Real> solving;
    solving.verbose = false;
    solving.depth = static_cast<unsigned int>(settings.depth);
    solving.samplesPerNode = settings.samples_per_node;

    pr::Reconstructor::LevelSetExtractionParameters extracting;
    extracting.verbose = false;
    extracting.forceManifold = true;
    // Without this every vertex comes back with a density of 1 and there is
    // nothing to trim by.
    extracting.outputDensity = true;

    CloudSurfaceMesh mesh;
    CloudStream cloud(points, normals, count);
    std::unique_ptr<Implicit> implicit(Solver::Solve(cloud, solving));
    if (implicit == nullptr) {
        throw std::invalid_argument("Poisson: no surface could be fitted to these points");
    }
    VertexSink vertices(mesh.vertices, mesh.density);
    FaceSink faces(mesh.indices);
    implicit->extractLevelSet(vertices, faces, extracting);

    trimByDensity(mesh, settings.trim);
    fillNormals(mesh);
    return mesh;
}

bool CloudSurfaceBuilder::build(const float* xyz, const float* normals, uint64_t count, int depth,
                           float trim) {
    CloudSurfaceSettings settings;
    settings.depth = depth;
    settings.trim = trim;
    mesh_ = reconstructFromCloud(xyz, normals, count, settings);
    return mesh_.indices.size() >= 3;
}

std::unique_ptr<CloudSurfaceBuilder> makeCloudSurfaceBuilder() {
    return std::make_unique<CloudSurfaceBuilder>();
}

} // namespace all3dscan::mesh
