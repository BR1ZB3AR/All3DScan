#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "protocol/ScannerProtocol.h"

namespace revoscan::pointcloud {

struct Point {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
};

struct PointCloud {
    std::vector<Point> points;
};

// Preview depth mapping for vendor-raw Y16 (not metric / not for export).
// Defaults match RevoScan APK param_pop4.json model 0E1 high-accuracy ranges.
struct PreviewDepthScale {
    // Multiplies raw Y16 words before range mapping. Official SDK exposes
    // PROPERTY_EXT_DEPTH_SCALE; until that is read from the device, leave at
    // 1.0 or override via ALL3DSCAN_DEPTH_SCALE.
    double depthScale = 1.0;

    // Working volume in millimetres (param_pop4 zminRange..zmax / dmin..).
    float zMinMm = 210.0f;
    float zMaxMm = 500.0f;

    // Approximate pinhole for 640×512 POP4 depth (visualization only).
    float fx = 560.0f;
    float fy = 560.0f;

    // Robust raw→range mapping uses these percentiles of nonzero samples.
    float lowPercentile = 5.0f;
    float highPercentile = 95.0f;
};

PreviewDepthScale defaultPop4PreviewDepthScale();

// Real-time "scan only the object" isolation. Clips to a working-distance band
// around the nearest surface, then keeps the *nearest* connected cluster (not
// the largest — a bolt must win over a big background sheet).
struct ForegroundIsolation {
    float nearClipM = 0.12f;           // drop anything closer (sensor noise)
    float farClipM = 0.50f;            // drop anything further (far background)
    float nearBandM = 0.10f;           // keep nearest surface + this depth window
    float nearPercentile = 8.0f;       // robust nearest-depth estimate
    float clusterVoxelM = 0.015f;      // connectivity resolution (must bridge point spacing)
    float lateralPadM = 0.04f;         // XY pad around chosen cluster bbox
    float centerBiasM = 0.08f;         // prefer clusters near optical axis
    std::size_t minClusterPoints = 40; // ignore tiny detached specks
    float minClusterShareOfLargest = 0.05f; // ...and fragments < 5% of the largest
    bool keepOnlyBestCluster = true;   // Revo-style: one object at a time

    // Fronto-parallel dominant-plane removal (RANSAC). Written for the old
    // phase decoder, whose collapsed depth back-projected to a flat slab. With
    // the speckle decoder, flat surfaces are real — usually the object's own
    // front face — and removing them left only scraps (replay: ~150 of ~67k
    // points kept, tracking lost on a static scene). Off by default.
    bool removeDominantPlane = false;
    float planeThreshM = 0.006f;       // RANSAC inlier band (±6 mm)
    float planeMinFraction = 0.35f;    // only remove if plane ≥ this share of band
    float planeMaxFraction = 0.97f;    // above this it's a pure slab → keep (nothing else)
    float planeMinNormalZ = 0.80f;     // |nz| gate: only fronto-parallel planes
    int planeIterations = 80;          // RANSAC trials
};

PointCloud isolateForegroundObject(const PointCloud& in,
                                   const ForegroundIsolation& params = {});

// Back-projects a depth (+ optional RGB) frame into a camera-space point
// cloud using pinhole intrinsics. Points with zero depth are skipped.
// MetricMillimeters only — VendorRawY16 is rejected.
PointCloud fromDepthFrame(const protocol::RawFrame& frame,
                          const protocol::CameraIntrinsics& intrinsics);

// Visualization-only cloud from packed vendor Y16. Maps raw samples into the
// POP4 working distance using PreviewDepthScale (percentile + depthScale).
// Must not be treated as calibrated millimetres for export/meshing.
PointCloud previewFromVendorRawY16(const protocol::RawFrame& frame,
                                   const PreviewDepthScale& scale = {});

} // namespace revoscan::pointcloud
