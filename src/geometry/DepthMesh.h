#pragma once

#include <cstdint>

#include "geometry/TriangleMesh.h"
#include "protocol/ScannerProtocol.h"

namespace revoscan::geometry {

struct DepthMeshOptions {
    // Subsample the depth grid (1 = full res, 2 = every other pixel).
    int stride = 2;
    // Reject a quad edge when neighboring depths differ by more than this (m).
    float maxEdgeLengthM = 0.018f;
    // Skip depths outside this band (metres). 0 disables a bound.
    float nearM = 0.10f;
    float farM = 0.80f;
};

// Build a triangle mesh from an organized metric depth map (row-major mm).
// Optional rgb is width*height*3; when present, vertices get those colors.
TriangleMesh meshFromOrganizedDepth(const uint16_t* depthMm,
                                    uint32_t width,
                                    uint32_t height,
                                    const protocol::CameraIntrinsics& intrinsics,
                                    const uint8_t* rgb,
                                    size_t rgbBytes,
                                    const DepthMeshOptions& opts = {});

// Voxel-weld `src` into `dst` (newest vertex wins). Caps growth at maxVertices.
void mergeMeshVoxel(TriangleMesh& dst,
                    const TriangleMesh& src,
                    float voxelSizeM,
                    size_t maxVertices = 180000);

} // namespace revoscan::geometry
