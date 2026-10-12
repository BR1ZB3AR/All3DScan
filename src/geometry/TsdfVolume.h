#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "geometry/TriangleMesh.h"
#include "protocol/ScannerProtocol.h"

namespace revoscan::geometry {

// Native volumetric fusion (no Python / Open3D). Depth keyframes with known
// camera→world poses are integrated into a dense truncated signed distance
// field over a bounded box; Surface Nets then extracts a closed, smooth,
// vertex-coloured triangle mesh. This replaces the Open3D TSDF worker, which
// cannot run where no Open3D build exists (e.g. Python 3.14).
struct TsdfParams {
    float voxelM = 0.0015f;   // requested voxel edge (grown if the box is large)
    float truncVoxels = 4.0f; // truncation band in voxels
    float minDepthM = 0.10f;
    float maxDepthM = 0.80f;
    size_t maxVoxels = 16u * 1000u * 1000u; // memory cap (12 B per voxel)
    float minWeight = 1.5f;   // voxels seen fewer times are treated as unknown
};

class TsdfVolume {
public:
    // Allocates a volume covering [boxMin, boxMax] (metres, world frame).
    TsdfVolume(const std::array<float, 3>& boxMin, const std::array<float, 3>& boxMax,
               const TsdfParams& params = {});

    // `depthMm` is width×height millimetres (0 = invalid); `rgb` optional
    // width×height×3. `cameraToWorld` is row-major 4×4.
    void integrate(const std::vector<uint16_t>& depthMm,
                   const std::vector<uint8_t>& rgb,
                   uint32_t width,
                   uint32_t height,
                   const protocol::CameraIntrinsics& intrinsics,
                   const std::array<double, 16>& cameraToWorld);

    // Surface Nets iso-surface at tsdf = 0 over well-observed voxels.
    TriangleMesh extractMesh() const;

    float voxelSize() const { return voxel_; }
    std::array<int, 3> dims() const { return {nx_, ny_, nz_}; }

private:
    struct Voxel {
        float tsdf = 1.0f;
        float weight = 0.0f;
        uint8_t r = 0, g = 0, b = 0, pad = 0;
    };
    size_t index(int x, int y, int z) const {
        return (static_cast<size_t>(z) * ny_ + y) * nx_ + x;
    }

    TsdfParams params_;
    std::array<float, 3> origin_{};
    float voxel_ = 0.0f;
    float trunc_ = 0.0f;
    int nx_ = 0, ny_ = 0, nz_ = 0;
    std::vector<Voxel> voxels_;
};

// Convenience: integrate keyframes into a volume fitted to `bounds` (cloud
// whose bounding box, padded by `padM`, defines the volume) and extract.
struct TsdfFrame {
    const std::vector<uint16_t>* depthMm = nullptr;
    const std::vector<uint8_t>* rgb = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    protocol::CameraIntrinsics intrinsics;
    std::array<double, 16> cameraToWorld{};
};

TriangleMesh reconstructTsdf(const std::vector<TsdfFrame>& frames,
                             const pointcloud::PointCloud& bounds,
                             float padM = 0.02f,
                             const TsdfParams& params = {});

} // namespace revoscan::geometry
