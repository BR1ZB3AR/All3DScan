#pragma once

#include <cstdint>
#include <memory>
#include <vector>

namespace all3dscan::mesh {

// Builds a surface from a point cloud that has no depth views behind it.
//
// SurfaceBuilder melts a scan's own depth views into a surface, which is the
// better answer when those views exist: it knows where the scanner looked and
// saw nothing, so it leaves holes where there is no evidence. A cloud opened
// from a file carries no such record, only points, and this is for those:
// screened Poisson reconstruction (third_party/PoissonRecon) fits one smooth
// surface through the points using their normals.
//
// Poisson closes over everything, including the places a scan never reached,
// so a partial scan comes back inflated: measured on a real POP 4 scan whose
// points spanned 270 x 152 x 131 mm, the untrimmed surface spanned
// 297 x 233 x 214. Each vertex therefore carries how much data stood behind
// it, and the thinnest are cut away; at 10% that same scan came back spanning
// 276 x 156 x 129, which is the data's own extent.
//
// Everything crossing this boundary is in millimetres.
struct CloudSurfaceSettings {
    // How finely the solver divides space: each step doubles the detail and
    // costs roughly twice the work. 8 suits a scan of an object.
    int depth = 8;
    // The share of vertices, least-supported first, to cut away. 0 keeps the
    // surface closed and inflated; much above 0.25 starts eating real surface.
    float trim = 0.10f;
    // Points per node for the solver's density estimate.
    float samples_per_node = 1.5f;
};

// What the reconstruction produced: x, y, z millimetres per vertex, a unit
// normal each, and three indices per triangle.
struct CloudSurfaceMesh {
    std::vector<float> vertices;
    std::vector<float> normals;
    std::vector<uint32_t> indices;
    // How much data stood behind each vertex, before any trimming.
    std::vector<float> density;
    // What the trim actually removed.
    uint64_t trimmed_vertices = 0;
    uint64_t trimmed_triangles = 0;
};

// Reconstructs from `points` (3 * count millimetres) and `normals` (3 * count,
// unit length). Throws std::invalid_argument if the sizes disagree, the count
// is too small to reconstruct from, or a setting is out of range.
CloudSurfaceMesh reconstructFromCloud(const float* points, const float* normals, uint64_t count,
                               const CloudSurfaceSettings& settings = {});

// The same, as something the Rust bridge can hold on to: it keeps the result
// alive so the arrays can be borrowed rather than copied across.
class CloudSurfaceBuilder {
public:
    // Throws std::invalid_argument on a cloud or a setting it cannot use.
    // `xyz` and `normals` each hold 3 * count floats.
    bool build(const float* xyz, const float* normals, uint64_t count, int depth, float trim);

    const std::vector<float>& vertices() const noexcept { return mesh_.vertices; }
    const std::vector<float>& normals() const noexcept { return mesh_.normals; }
    const std::vector<uint32_t>& indices() const noexcept { return mesh_.indices; }
    uint64_t triangles() const noexcept { return mesh_.indices.size() / 3; }
    uint64_t trimmed_vertices() const noexcept { return mesh_.trimmed_vertices; }
    uint64_t trimmed_triangles() const noexcept { return mesh_.trimmed_triangles; }

private:
    CloudSurfaceMesh mesh_;
};

std::unique_ptr<CloudSurfaceBuilder> makeCloudSurfaceBuilder();

} // namespace all3dscan::mesh
