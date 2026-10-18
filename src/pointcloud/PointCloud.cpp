#include "PointCloud.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <random>
#include <unordered_map>
#include <vector>

namespace revoscan::pointcloud {
namespace {

float percentileSorted(std::vector<float>& values, float pct) {
    if (values.empty()) {
        return 0.0f;
    }
    pct = std::clamp(pct, 0.0f, 100.0f);
    const size_t last = values.size() - 1;
    const size_t idx = static_cast<size_t>(
        std::lround((pct / 100.0) * static_cast<double>(last)));
    const size_t safeIdx = std::min(idx, last);
    std::nth_element(values.begin(),
                     values.begin() + static_cast<std::ptrdiff_t>(safeIdx),
                     values.end());
    return values[safeIdx];
}

void colorForDistanceMm(float depthMm, float zMinMm, float zMaxMm, Point& p) {
    // Match DistanceScaleWidget / param_pop4 histogram bands.
    const float h1 = 290.0f;
    const float h2 = 350.0f;
    const float h3 = 400.0f;
    const float h4 = 460.0f;
    (void)zMinMm;
    (void)zMaxMm;
    if (depthMm < h1) {
        p.r = 217;
        p.g = 92;
        p.b = 92; // too near
    } else if (depthMm < h2) {
        p.r = 22;
        p.g = 199;
        p.b = 193; // excellent
    } else if (depthMm < h3) {
        p.r = 91;
        p.g = 191;
        p.b = 91; // good
    } else if (depthMm < h4) {
        p.r = 224;
        p.g = 168;
        p.b = 62; // far
    } else {
        p.r = 217;
        p.g = 92;
        p.b = 92; // too far
    }
}

} // namespace

PreviewDepthScale defaultPop4PreviewDepthScale() {
    PreviewDepthScale scale;
    if (const char* env = std::getenv("ALL3DSCAN_DEPTH_SCALE")) {
        char* end = nullptr;
        const double parsed = std::strtod(env, &end);
        if (end != env && parsed > 0.0 && std::isfinite(parsed)) {
            scale.depthScale = parsed;
        }
    }
    return scale;
}

PointCloud fromDepthFrame(const protocol::RawFrame& frame,
                          const protocol::CameraIntrinsics& intrinsics) {
    PointCloud cloud;
    const size_t expected =
        static_cast<size_t>(frame.width) * static_cast<size_t>(frame.height);
    if (frame.encoding != protocol::DepthEncoding::MetricMillimeters ||
        frame.width == 0 || frame.height == 0 ||
        frame.depthMm.size() != expected ||
        intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0) {
        return cloud;
    }
    cloud.points.reserve(frame.depthMm.size());

    const bool hasColor = frame.rgb.size() == frame.depthMm.size() * 3;

    for (uint32_t y = 0; y < frame.height; ++y) {
        for (uint32_t x = 0; x < frame.width; ++x) {
            const size_t idx = static_cast<size_t>(y) * frame.width + x;
            const uint16_t depthMm = frame.depthMm[idx];
            if (depthMm == 0) {
                continue;
            }

            const float z = static_cast<float>(depthMm) / 1000.0f; // meters
            const float px = (static_cast<float>(x) - static_cast<float>(intrinsics.cx)) *
                             z / static_cast<float>(intrinsics.fx);
            const float py = (static_cast<float>(y) - static_cast<float>(intrinsics.cy)) *
                             z / static_cast<float>(intrinsics.fy);

            Point p;
            p.x = px;
            p.y = py;
            p.z = z;
            if (hasColor) {
                p.r = frame.rgb[idx * 3 + 0];
                p.g = frame.rgb[idx * 3 + 1];
                p.b = frame.rgb[idx * 3 + 2];
            }
            cloud.points.push_back(p);
        }
    }

    return cloud;
}

PointCloud previewFromVendorRawY16(const protocol::RawFrame& frame,
                                   const PreviewDepthScale& scaleIn) {
    PointCloud cloud;
    const size_t expected =
        static_cast<size_t>(frame.width) * static_cast<size_t>(frame.height);
    if (frame.encoding != protocol::DepthEncoding::VendorRawY16 ||
        frame.width == 0 || frame.height == 0 ||
        frame.depthMm.size() != expected) {
        return cloud;
    }

    PreviewDepthScale scale = scaleIn;
    if (scale.depthScale <= 0.0 || !std::isfinite(scale.depthScale)) {
        scale.depthScale = 1.0;
    }
    if (scale.zMaxMm <= scale.zMinMm) {
        scale.zMinMm = 210.0f;
        scale.zMaxMm = 500.0f;
    }
    if (scale.fx <= 1.0f) {
        scale.fx = static_cast<float>(frame.width) * 0.875f;
    }
    if (scale.fy <= 1.0f) {
        scale.fy = scale.fx;
    }

    std::vector<float> scaled;
    scaled.reserve(expected / 8);
    for (uint16_t sample : frame.depthMm) {
        if (sample == 0) {
            continue;
        }
        scaled.push_back(static_cast<float>(sample) * static_cast<float>(scale.depthScale));
    }
    if (scaled.empty()) {
        return cloud;
    }

    std::vector<float> forLow = scaled;
    std::vector<float> forHigh = scaled;
    float rawLow = percentileSorted(forLow, scale.lowPercentile);
    float rawHigh = percentileSorted(forHigh, scale.highPercentile);
    if (!(rawHigh > rawLow)) {
        // Degenerate frame: fall back to min/max of nonzero samples.
        rawLow = *std::min_element(scaled.begin(), scaled.end());
        rawHigh = *std::max_element(scaled.begin(), scaled.end());
    }
    if (!(rawHigh > rawLow)) {
        rawHigh = rawLow + 1.0f;
    }

    const float zMinM = scale.zMinMm / 1000.0f;
    const float zMaxM = scale.zMaxMm / 1000.0f;
    const float cx = static_cast<float>(frame.width) * 0.5f;
    const float cy = static_cast<float>(frame.height) * 0.5f;

    cloud.points.reserve(scaled.size());
    for (uint32_t y = 0; y < frame.height; ++y) {
        for (uint32_t x = 0; x < frame.width; ++x) {
            const size_t idx = static_cast<size_t>(y) * frame.width + x;
            const uint16_t raw = frame.depthMm[idx];
            if (raw == 0) {
                continue;
            }

            const float scaledRaw =
                static_cast<float>(raw) * static_cast<float>(scale.depthScale);
            float t = (scaledRaw - rawLow) / (rawHigh - rawLow);
            t = std::clamp(t, 0.0f, 1.0f);
            // Near samples map to zMin (closer to camera), far to zMax.
            const float z = zMinM + t * (zMaxM - zMinM);
            const float depthMm = z * 1000.0f;

            Point p;
            p.x = (static_cast<float>(x) - cx) * z / scale.fx;
            p.y = (static_cast<float>(y) - cy) * z / scale.fy;
            p.z = z;
            colorForDistanceMm(depthMm, scale.zMinMm, scale.zMaxMm, p);
            cloud.points.push_back(p);
        }
    }
    return cloud;
}

namespace {

// Remove the dominant fronto-parallel plane (background wall / collapsed slab)
// from `pts` via RANSAC. Returns the surviving points, or the input unchanged
// when no qualifying plane is found or removal would empty the cloud.
std::vector<Point> removeDominantFrontoPlane(const std::vector<Point>& pts,
                                             const ForegroundIsolation& params) {
    if (!params.removeDominantPlane || pts.size() < 50) {
        return pts;
    }
    std::mt19937 rng(1234567u); // deterministic for reproducible scans/tests
    std::uniform_int_distribution<size_t> pick(0, pts.size() - 1);
    const float thresh = params.planeThreshM;

    size_t bestInliers = 0;
    float bestA = 0.0f, bestB = 0.0f, bestC = 0.0f, bestD = 0.0f;
    for (int it = 0; it < params.planeIterations; ++it) {
        const Point& p0 = pts[pick(rng)];
        const Point& p1 = pts[pick(rng)];
        const Point& p2 = pts[pick(rng)];
        const float ux = p1.x - p0.x, uy = p1.y - p0.y, uz = p1.z - p0.z;
        const float vx = p2.x - p0.x, vy = p2.y - p0.y, vz = p2.z - p0.z;
        float nx = uy * vz - uz * vy;
        float ny = uz * vx - ux * vz;
        float nz = ux * vy - uy * vx;
        const float norm = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (norm < 1e-6f) {
            continue;
        }
        nx /= norm;
        ny /= norm;
        nz /= norm;
        // Only fronto-parallel planes (normal ~ optical axis) are background/slab.
        if (std::fabs(nz) < params.planeMinNormalZ) {
            continue;
        }
        const float d = -(nx * p0.x + ny * p0.y + nz * p0.z);
        size_t inliers = 0;
        for (const auto& p : pts) {
            if (std::fabs(nx * p.x + ny * p.y + nz * p.z + d) <= thresh) {
                ++inliers;
            }
        }
        if (inliers > bestInliers) {
            bestInliers = inliers;
            bestA = nx;
            bestB = ny;
            bestC = nz;
            bestD = d;
        }
    }

    const double frac =
        static_cast<double>(bestInliers) / static_cast<double>(pts.size());
    if (frac < params.planeMinFraction || frac > params.planeMaxFraction) {
        return pts;
    }
    std::vector<Point> kept;
    kept.reserve(pts.size() - bestInliers);
    for (const auto& p : pts) {
        if (std::fabs(bestA * p.x + bestB * p.y + bestC * p.z + bestD) > thresh) {
            kept.push_back(p);
        }
    }
    if (kept.size() < params.minClusterPoints) {
        return pts; // removal would gut the cloud — keep original
    }
    return kept;
}

struct IsoVoxel {
    int x = 0, y = 0, z = 0;
    bool operator==(const IsoVoxel& o) const { return x == o.x && y == o.y && z == o.z; }
};
struct IsoVoxelHash {
    size_t operator()(const IsoVoxel& k) const {
        return (static_cast<size_t>(k.x) * 73856093u) ^
               (static_cast<size_t>(k.y) * 19349663u) ^
               (static_cast<size_t>(k.z) * 83492791u);
    }
};

} // namespace

PointCloud isolateForegroundObject(const PointCloud& in,
                                   const ForegroundIsolation& params) {
    if (in.points.empty()) {
        return in;
    }

    // 1) Hard clip to the configured scanning distance, then estimate the
    //    nearest surface robustly (low percentile of depth).
    std::vector<float> zs;
    zs.reserve(in.points.size());
    for (const auto& p : in.points) {
        if (p.z >= params.nearClipM && p.z <= params.farClipM) {
            zs.push_back(p.z);
        }
    }
    if (zs.empty()) {
        return {};
    }
    const float zNear = percentileSorted(zs, params.nearPercentile);
    const float zFarBand = std::min(params.farClipM, zNear + params.nearBandM);
    // Also drop points clearly behind the object band (stricter than farClip).
    const float zLo = std::max(params.nearClipM, zNear - 0.02f);

    // 2) Keep only points inside the nearest-surface band.
    PointCloud band;
    band.points.reserve(in.points.size());
    for (const auto& p : in.points) {
        if (p.z >= zLo && p.z <= zFarBand) {
            band.points.push_back(p);
        }
    }

    // Strip the dominant fronto-parallel plane (back wall / collapsed depth
    // slab) so the object's relief survives clustering rather than being buried
    // in a filled sheet.
    const std::size_t beforePlane = band.points.size();
    band.points = removeDominantFrontoPlane(band.points, params);
    const std::size_t afterPlane = band.points.size();
    const bool trace = std::getenv("ALL3DSCAN_ISOLATE_TRACE") != nullptr;
    if (trace) {
        std::fprintf(stderr, "[isolate] in=%zu band=%zu afterPlane=%zu",
                     in.points.size(), beforePlane, afterPlane);
    }

    if (band.points.size() < params.minClusterPoints) {
        if (trace) {
            std::fprintf(stderr, " (too few to cluster)\n");
        }
        return band;
    }

    // 3) Connected-component labelling over an occupancy voxel grid (26-conn).
    const float voxel = params.clusterVoxelM > 0.0f ? params.clusterVoxelM : 0.015f;
    std::unordered_map<IsoVoxel, std::vector<int>, IsoVoxelHash> cells;
    cells.reserve(band.points.size());
    for (int i = 0; i < static_cast<int>(band.points.size()); ++i) {
        const auto& p = band.points[static_cast<size_t>(i)];
        IsoVoxel key{static_cast<int>(std::floor(p.x / voxel)),
                     static_cast<int>(std::floor(p.y / voxel)),
                     static_cast<int>(std::floor(p.z / voxel))};
        cells[key].push_back(i);
    }

    struct ClusterStats {
        size_t count = 0;
        double sumX = 0.0;
        double sumY = 0.0;
        double sumZ = 0.0;
        float minX = 0.0f, maxX = 0.0f, minY = 0.0f, maxY = 0.0f;
        float minZ = 0.0f, maxZ = 0.0f;
        bool initBBox = false;
    };

    std::unordered_map<IsoVoxel, int, IsoVoxelHash> label;
    label.reserve(cells.size());
    std::vector<ClusterStats> comps;
    std::vector<IsoVoxel> stack;
    for (const auto& entry : cells) {
        if (label.count(entry.first)) {
            continue;
        }
        const int comp = static_cast<int>(comps.size());
        ClusterStats stats;
        stack.clear();
        stack.push_back(entry.first);
        label[entry.first] = comp;
        while (!stack.empty()) {
            const IsoVoxel v = stack.back();
            stack.pop_back();
            for (int idx : cells.at(v)) {
                const auto& p = band.points[static_cast<size_t>(idx)];
                ++stats.count;
                stats.sumX += p.x;
                stats.sumY += p.y;
                stats.sumZ += p.z;
                if (!stats.initBBox) {
                    stats.minX = stats.maxX = p.x;
                    stats.minY = stats.maxY = p.y;
                    stats.minZ = stats.maxZ = p.z;
                    stats.initBBox = true;
                } else {
                    stats.minX = std::min(stats.minX, p.x);
                    stats.maxX = std::max(stats.maxX, p.x);
                    stats.minY = std::min(stats.minY, p.y);
                    stats.maxY = std::max(stats.maxY, p.y);
                    stats.minZ = std::min(stats.minZ, p.z);
                    stats.maxZ = std::max(stats.maxZ, p.z);
                }
            }
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        IsoVoxel n{v.x + dx, v.y + dy, v.z + dz};
                        if (cells.count(n) && !label.count(n)) {
                            label[n] = comp;
                            stack.push_back(n);
                        }
                    }
                }
            }
        }
        comps.push_back(stats);
    }

    // 4) Pick the *nearest* qualifying cluster (small objects beat big sheets).
    //    Secondary: prefer clusters near the optical axis (center of FOV).
    //    A cluster must also hold a minimum share of the largest one, so
    //    stray edge fragments a few mm nearer can't beat the real object.
    size_t largest = 0;
    for (const auto& c : comps) {
        largest = std::max(largest, c.count);
    }
    const size_t minCount = std::max<size_t>(
        params.minClusterPoints,
        static_cast<size_t>(params.minClusterShareOfLargest * static_cast<float>(largest)));
    int best = -1;
    double bestScore = 1e30;
    for (int i = 0; i < static_cast<int>(comps.size()); ++i) {
        const auto& c = comps[static_cast<size_t>(i)];
        if (c.count < minCount) {
            continue;
        }
        const double inv = 1.0 / static_cast<double>(c.count);
        const double cx = c.sumX * inv;
        const double cy = c.sumY * inv;
        const double cz = c.sumZ * inv;
        const double lateral = std::sqrt(cx * cx + cy * cy);
        // Lower is better: depth dominates, then how far off-center.
        const double score = cz + 0.35 * std::max(0.0, lateral - params.centerBiasM);
        if (score < bestScore) {
            bestScore = score;
            best = i;
        }
    }
    if (best < 0) {
        return band;
    }

    const ClusterStats& chosen = comps[static_cast<size_t>(best)];

    // One object usually arrives as several clusters, not one: patchy depth
    // breaks a surface into pieces, and a close object can be crossed by its
    // own gaps. Keeping strictly the largest discarded most of the object on a
    // real sweep (521 of 1614 points kept, and 12x less model than with
    // isolation off). Accept the other pieces too, provided each is a
    // meaningful size and sits at the chosen cluster's depth — background at
    // another depth is still rejected.
    // A fragment counts as part of this object when it lies within the chosen
    // cluster's own depth extent, plus a small margin for the gap that split
    // them. Deliberately not nearBandM: that is the whole working window, wide
    // enough to swallow a background surface a hand's width behind.
    constexpr float kFragmentMarginM = 0.02f;
    const float sameZLo = chosen.minZ - kFragmentMarginM;
    const float sameZHi = chosen.maxZ + kFragmentMarginM;
    std::vector<bool> accepted(comps.size(), false);
    accepted[static_cast<size_t>(best)] = true;
    float xMin = chosen.minX;
    float xMax = chosen.maxX;
    float yMin = chosen.minY;
    float yMax = chosen.maxY;
    size_t acceptedCount = chosen.count;
    if (params.keepOnlyBestCluster) {
        for (size_t i = 0; i < comps.size(); ++i) {
            if (accepted[i]) {
                continue;
            }
            const auto& c = comps[i];
            if (c.count < minCount) {
                continue;
            }
            const double cz = c.sumZ / static_cast<double>(c.count);
            if (cz < sameZLo || cz > sameZHi) {
                continue; // a different surface, not part of this object
            }
            accepted[i] = true;
            acceptedCount += c.count;
            xMin = std::min(xMin, c.minX);
            xMax = std::max(xMax, c.maxX);
            yMin = std::min(yMin, c.minY);
            yMax = std::max(yMax, c.maxY);
        }
    }
    const float pad = params.lateralPadM;
    xMin -= pad;
    xMax += pad;
    yMin -= pad;
    yMax += pad;

    PointCloud out;
    out.points.reserve(acceptedCount);
    for (const auto& entry : cells) {
        const int comp = label.at(entry.first);
        if (params.keepOnlyBestCluster && !accepted[static_cast<size_t>(comp)]) {
            continue;
        }
        if (!params.keepOnlyBestCluster &&
            comps[static_cast<size_t>(comp)].count < params.minClusterPoints) {
            continue;
        }
        for (int idx : entry.second) {
            const auto& p = band.points[static_cast<size_t>(idx)];
            // Soft lateral gate around the chosen object bbox (drops leftover
            // background that survived the depth band).
            if (p.x < xMin || p.x > xMax || p.y < yMin || p.y > yMax) {
                continue;
            }
            out.points.push_back(p);
        }
    }
    if (trace) {
        std::fprintf(stderr, " clusters=%zu out=%zu\n", comps.size(), out.points.size());
    }
    return out.points.empty() ? band : out;
}

} // namespace revoscan::pointcloud
