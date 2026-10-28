#pragma once

#include <cstdint>
#include <memory>
#include <vector>

namespace all3dscan::mesh {

// Turns a scan's depth views into one closed surface.
//
// Each view is a depth image and the pose it was taken from. They are melted
// together into a truncated signed distance field and the surface drawn
// through it, which is the Qt app's own reconstruction
// (src/geometry/TsdfVolume): holes between views close over, and the noise
// that a point cloud shows as fur averages out.
//
// Everything crossing this boundary is in millimetres, as the engine holds
// it; the reconstruction underneath works in metres.
class SurfaceBuilder {
public:
    // Throws std::invalid_argument unless 0.2 <= voxel_mm <= 20. The voxel
    // is the detail the surface can hold, and halving it costs eight times
    // the memory, so the reconstruction grows it if the scan is large.
    explicit SurfaceBuilder(float voxel_mm);
    ~SurfaceBuilder();
    SurfaceBuilder(const SurfaceBuilder&) = delete;
    SurfaceBuilder& operator=(const SurfaceBuilder&) = delete;

    // Keeps one view: `depth` holds width * height millimetres, 0 where
    // there is none, and `pose` is a row-major 4x4 taking that camera's
    // points into the model's axes with its translation in millimetres.
    // Throws std::invalid_argument if the sizes do not agree.
    void addView(const uint16_t* depth, uint64_t depth_count, uint32_t width, uint32_t height,
                 double fx, double fy, double cx, double cy, const double* pose);

    // Builds the surface over the box `xyz` covers, which is the scan's own
    // cloud: 3 * points millimetres. False when there is nothing to build
    // from, or the views made no surface.
    bool build(const float* xyz, uint64_t points, int smoothing);

    // The surface: x, y, z millimetres per vertex, a unit normal each, and
    // three indices per triangle.
    const std::vector<float>& vertices() const noexcept { return vertices_; }
    const std::vector<float>& normals() const noexcept { return normals_; }
    const std::vector<uint32_t>& indices() const noexcept { return indices_; }
    uint64_t views() const noexcept;
    uint64_t triangles() const noexcept { return indices_.size() / 3; }
    void clear();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::vector<float> vertices_;
    std::vector<float> normals_;
    std::vector<uint32_t> indices_;
};

std::unique_ptr<SurfaceBuilder> makeSurfaceBuilder(float voxel_mm);

} // namespace all3dscan::mesh
