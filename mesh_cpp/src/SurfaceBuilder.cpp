#include "all3dscan/SurfaceBuilder.h"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#include "geometry/TsdfVolume.h"
#include "pointcloud/PointCloud.h"

namespace all3dscan::mesh {

namespace geometry = revoscan::geometry;
namespace pointcloud = revoscan::pointcloud;
namespace protocol = revoscan::protocol;

namespace {

// The reconstruction works in metres; the engine hands over millimetres.
constexpr float kMmPerM = 1000.0f;
// Room around the scan for the surface to close over its edges.
constexpr float kPadM = 0.01f;

// One kept view: its depth image and where it was taken from.
struct View {
    std::vector<uint16_t> depth;
    uint32_t width = 0;
    uint32_t height = 0;
    protocol::CameraIntrinsics intrinsics;
    std::array<double, 16> cameraToWorld{};
};

// A normal per vertex, from the triangles sharing it. Each triangle's
// cross product is already as long as twice its area, so adding them
// unscaled weights the larger faces more, which is what is wanted.
std::vector<float> vertexNormals(const geometry::TriangleMesh& surface) {
    std::vector<float> normals(surface.vertices.size() * 3, 0.0f);
    for (size_t i = 0; i + 2 < surface.indices.size(); i += 3) {
        const auto& a = surface.vertices[surface.indices[i]];
        const auto& b = surface.vertices[surface.indices[i + 1]];
        const auto& c = surface.vertices[surface.indices[i + 2]];
        const float ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
        const float vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
        const float nx = uy * vz - uz * vy;
        const float ny = uz * vx - ux * vz;
        const float nz = ux * vy - uy * vx;
        for (int corner = 0; corner < 3; ++corner) {
            const size_t at = surface.indices[i + corner] * 3;
            normals[at] += nx;
            normals[at + 1] += ny;
            normals[at + 2] += nz;
        }
    }
    for (size_t i = 0; i < normals.size(); i += 3) {
        const float length =
            std::sqrt(normals[i] * normals[i] + normals[i + 1] * normals[i + 1] +
                      normals[i + 2] * normals[i + 2]);
        if (length > 1e-12f) {
            normals[i] /= length;
            normals[i + 1] /= length;
            normals[i + 2] /= length;
        } else {
            normals[i] = 0.0f;
            normals[i + 1] = 0.0f;
            normals[i + 2] = -1.0f; // a vertex no triangle reached; face the sensor
        }
    }
    return normals;
}

} // namespace

struct SurfaceBuilder::Impl {
    float voxelM = 0.001f;
    std::vector<View> views;
};

SurfaceBuilder::SurfaceBuilder(float voxel_mm) : impl_(std::make_unique<Impl>()) {
    if (!(voxel_mm >= 0.2f && voxel_mm <= 20.0f)) {
        throw std::invalid_argument("SurfaceBuilder: voxel size must be 0.2 to 20 mm, got " +
                                    std::to_string(voxel_mm));
    }
    impl_->voxelM = voxel_mm / kMmPerM;
}

SurfaceBuilder::~SurfaceBuilder() = default;

uint64_t SurfaceBuilder::views() const noexcept {
    return impl_->views.size();
}

void SurfaceBuilder::clear() {
    impl_->views.clear();
    vertices_.clear();
    normals_.clear();
    indices_.clear();
}

void SurfaceBuilder::addView(const uint16_t* depth, uint64_t depth_count, uint32_t width,
                             uint32_t height, double fx, double fy, double cx, double cy,
                             const double* pose) {
    if (depth == nullptr || pose == nullptr ||
        depth_count != static_cast<uint64_t>(width) * height) {
        throw std::invalid_argument("SurfaceBuilder: the depth image is not width x height");
    }
    if (!(fx > 0.0 && fy > 0.0)) {
        throw std::invalid_argument("SurfaceBuilder: the camera has no focal length");
    }
    View view;
    view.depth.assign(depth, depth + depth_count);
    view.width = width;
    view.height = height;
    view.intrinsics = {.fx = fx, .fy = fy, .cx = cx, .cy = cy};
    std::copy(pose, pose + 16, view.cameraToWorld.begin());
    // The reconstruction places points in metres.
    view.cameraToWorld[3] /= kMmPerM;
    view.cameraToWorld[7] /= kMmPerM;
    view.cameraToWorld[11] /= kMmPerM;
    impl_->views.push_back(std::move(view));
}

bool SurfaceBuilder::build(const float* xyz, uint64_t points, int smoothing) {
    vertices_.clear();
    normals_.clear();
    indices_.clear();
    if (impl_->views.empty() || points == 0 || xyz == nullptr) {
        return false;
    }
    pointcloud::PointCloud bounds;
    bounds.points.resize(points);
    for (uint64_t i = 0; i < points; ++i) {
        bounds.points[i].x = xyz[3 * i] / kMmPerM;
        bounds.points[i].y = xyz[3 * i + 1] / kMmPerM;
        bounds.points[i].z = xyz[3 * i + 2] / kMmPerM;
    }

    std::vector<geometry::TsdfFrame> frames;
    frames.reserve(impl_->views.size());
    for (const auto& view : impl_->views) {
        geometry::TsdfFrame frame;
        frame.depthMm = &view.depth;
        frame.rgb = nullptr; // the surface is shaded, not textured
        frame.width = view.width;
        frame.height = view.height;
        frame.intrinsics = view.intrinsics;
        frame.cameraToWorld = view.cameraToWorld;
        frames.push_back(frame);
    }

    geometry::TsdfParams params;
    params.voxelM = impl_->voxelM;
    auto surface = geometry::reconstructTsdf(frames, bounds, kPadM, params);
    if (surface.empty()) {
        return false;
    }
    if (smoothing > 0) {
        geometry::smoothMeshTaubin(surface, smoothing);
    }

    normals_ = vertexNormals(surface);
    vertices_.resize(surface.vertices.size() * 3);
    for (size_t i = 0; i < surface.vertices.size(); ++i) {
        vertices_[3 * i] = surface.vertices[i].x * kMmPerM;
        vertices_[3 * i + 1] = surface.vertices[i].y * kMmPerM;
        vertices_[3 * i + 2] = surface.vertices[i].z * kMmPerM;
    }
    indices_ = std::move(surface.indices);
    return true;
}

std::unique_ptr<SurfaceBuilder> makeSurfaceBuilder(float voxel_mm) {
    return std::make_unique<SurfaceBuilder>(voxel_mm);
}

} // namespace all3dscan::mesh
