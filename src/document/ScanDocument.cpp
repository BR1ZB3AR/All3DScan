#include "ScanDocument.h"

#ifdef ALL3DSCAN_HAS_PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#endif

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace revoscan::document {
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

pointcloud::PointCloud voxelDownsample(const pointcloud::PointCloud& in, float voxel,
                                        bool preferNewest) {
    if (in.points.empty() || voxel <= 0.0f) {
        return in;
    }
    std::unordered_map<VoxelKey, pointcloud::Point, VoxelKeyHash> cells;
    cells.reserve(in.points.size());
    for (const auto& p : in.points) {
        VoxelKey key{
            static_cast<int>(std::floor(p.x / voxel)),
            static_cast<int>(std::floor(p.y / voxel)),
            static_cast<int>(std::floor(p.z / voxel)),
        };
        if (preferNewest) {
            // Live fusion: latest sample wins so coverage tint / fresh IR show.
            cells[key] = p;
        } else {
            cells.emplace(key, p);
        }
    }
    pointcloud::PointCloud out;
    out.points.reserve(cells.size());
    for (const auto& entry : cells) {
        out.points.push_back(entry.second);
    }
    return out;
}

} // namespace

void ScanDocument::clear() {
    std::unique_lock lock(mutex_);
    history_.clear();
    index_ = 0;
    nextId_ = 1;
}

uint64_t ScanDocument::revision() const {
    std::shared_lock lock(mutex_);
    return history_.empty() ? 0 : current().id;
}

bool ScanDocument::canUndoLocked() const { return index_ > 0 && !history_.empty(); }
bool ScanDocument::canRedoLocked() const {
    return !history_.empty() && index_ + 1 < history_.size();
}

bool ScanDocument::canUndo() const {
    std::shared_lock lock(mutex_);
    return canUndoLocked();
}

bool ScanDocument::canRedo() const {
    std::shared_lock lock(mutex_);
    return canRedoLocked();
}

bool ScanDocument::undo() {
    std::unique_lock lock(mutex_);
    if (!canUndoLocked()) {
        return false;
    }
    --index_;
    return true;
}

bool ScanDocument::redo() {
    std::unique_lock lock(mutex_);
    if (!canRedoLocked()) {
        return false;
    }
    ++index_;
    return true;
}

bool ScanDocument::isMetric() const {
    std::shared_lock lock(mutex_);
    return !history_.empty() && current().metric;
}

const pointcloud::PointCloud& ScanDocument::cloud() const {
    std::shared_lock lock(mutex_);
    return history_.empty() ? empty_.cloud : current().cloud;
}

const geometry::TriangleMesh& ScanDocument::mesh() const {
    std::shared_lock lock(mutex_);
    return history_.empty() ? empty_.mesh : current().mesh;
}

bool ScanDocument::cloudEmpty() const {
    std::shared_lock lock(mutex_);
    return history_.empty() || current().cloud.points.empty();
}

size_t ScanDocument::cloudSize() const {
    std::shared_lock lock(mutex_);
    return history_.empty() ? 0 : current().cloud.points.size();
}

bool ScanDocument::meshEmpty() const {
    std::shared_lock lock(mutex_);
    return history_.empty() || current().mesh.empty();
}

bool ScanDocument::meshHasColors() const {
    std::shared_lock lock(mutex_);
    return !history_.empty() && current().mesh.hasColors;
}

bool ScanDocument::meshIsMetric() const {
    std::shared_lock lock(mutex_);
    return !history_.empty() && current().mesh.metric;
}

pointcloud::PointCloud ScanDocument::cloudCopy() const {
    std::shared_lock lock(mutex_);
    return history_.empty() ? pointcloud::PointCloud{} : current().cloud;
}

geometry::TriangleMesh ScanDocument::meshCopy() const {
    std::shared_lock lock(mutex_);
    return history_.empty() ? geometry::TriangleMesh{} : current().mesh;
}

geometry::GeometrySnapshot ScanDocument::snapshot() const {
    std::shared_lock lock(mutex_);
    geometry::GeometrySnapshot snap;
    if (history_.empty()) {
        return snap;
    }
    const auto& rev = current();
    snap.revision = rev.id;
    snap.cloud = rev.cloud;
    snap.mesh = rev.mesh;
    snap.metric = rev.metric;
    snap.label = rev.label;
    return snap;
}

void ScanDocument::push(Revision rev) {
    if (!history_.empty() && index_ + 1 < history_.size()) {
        history_.erase(history_.begin() + static_cast<std::ptrdiff_t>(index_ + 1),
                       history_.end());
    }
    rev.id = nextId_++;
    history_.push_back(std::move(rev));
    index_ = history_.size() - 1;
}

const ScanDocument::Revision& ScanDocument::current() const {
    return history_[index_];
}

void ScanDocument::setCloud(pointcloud::PointCloud cloud, bool metric, const std::string& label) {
    std::unique_lock lock(mutex_);
    setCloudLocked(std::move(cloud), metric, label);
}

void ScanDocument::setCloudLocked(pointcloud::PointCloud cloud, bool metric,
                                  const std::string& label) {
    Revision rev;
    rev.cloud = std::move(cloud);
    rev.metric = metric;
    rev.label = label;
    if (!history_.empty()) {
        rev.mesh = current().mesh;
    }
    push(std::move(rev));
}

void ScanDocument::setMesh(geometry::TriangleMesh mesh, bool metric, const std::string& label) {
    std::unique_lock lock(mutex_);
    Revision rev;
    if (!history_.empty()) {
        rev.cloud = current().cloud;
    }
    rev.mesh = std::move(mesh);
    rev.metric = metric;
    rev.label = label;
    push(std::move(rev));
}

void ScanDocument::fuseMetricCloud(const pointcloud::PointCloud& cloud, float voxelSizeM) {
    fuseCloud(cloud, voxelSizeM, true, "fused");
}

void ScanDocument::fuseCloud(const pointcloud::PointCloud& cloud, float voxelSizeM,
                             bool metric, const std::string& label) {
    if (cloud.points.empty()) {
        return;
    }
    std::unique_lock lock(mutex_);
    // Only accumulate onto an existing base of the same metric-ness; switching
    // from preview to metric (or vice versa) starts a fresh model.
    if (!history_.empty() && current().metric == metric) {
        // Live path: mutate the current revision in place and drop undo history
        // so we do not copy a growing cloud into a new Revision every frame.
        if (history_.size() > 1 || index_ != 0) {
            Revision cur = std::move(history_[index_]);
            history_.clear();
            history_.push_back(std::move(cur));
            index_ = 0;
        }
        Revision& rev = history_[index_];
        rev.cloud.points.insert(rev.cloud.points.end(), cloud.points.begin(),
                                cloud.points.end());
        rev.cloud = voxelDownsample(rev.cloud, voxelSizeM, /*preferNewest=*/true);
        rev.label = label;
        rev.metric = metric;
        return;
    }
    pointcloud::PointCloud merged = cloud;
    merged = voxelDownsample(merged, voxelSizeM, /*preferNewest=*/true);
    setCloudLocked(std::move(merged), metric, label);
}

bool ScanDocument::cropBox(float minX, float minY, float minZ, float maxX, float maxY,
                           float maxZ) {
    std::unique_lock lock(mutex_);
    if (history_.empty()) {
        return false;
    }
    pointcloud::PointCloud cropped;
    for (const auto& p : current().cloud.points) {
        if (p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY && p.z >= minZ &&
            p.z <= maxZ) {
            cropped.points.push_back(p);
        }
    }
    setCloudLocked(std::move(cropped), current().metric, "crop");
    return true;
}

bool ScanDocument::removeStatisticalOutliers(size_t nbNeighbors, double stdRatio) {
    std::unique_lock lock(mutex_);
    if (history_.empty() || nbNeighbors == 0) {
        return false;
    }
    const auto& pts = current().cloud.points;
    if (pts.size() <= nbNeighbors) {
        return false;
    }

#ifdef ALL3DSCAN_HAS_PCL
    // Convert to PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_cloud->width = pts.size();
    pcl_cloud->height = 1;
    pcl_cloud->is_dense = true;
    pcl_cloud->points.resize(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        pcl_cloud->points[i].x = pts[i].x;
        pcl_cloud->points[i].y = pts[i].y;
        pcl_cloud->points[i].z = pts[i].z;
        pcl_cloud->points[i].r = pts[i].r;
        pcl_cloud->points[i].g = pts[i].g;
        pcl_cloud->points[i].b = pts[i].b;
    }

    // Apply StatisticalOutlierRemoval
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(pcl_cloud);
    sor.setMeanK(static_cast<int>(nbNeighbors));
    sor.setStddevMulThresh(stdRatio);
    pcl::PointCloud<pcl::PointXYZRGB> filtered_pcl;
    sor.filter(filtered_pcl);

    // Convert back to All3DScan point cloud
    pointcloud::PointCloud filtered;
    filtered.points.resize(filtered_pcl.points.size());
    for (size_t i = 0; i < filtered_pcl.points.size(); ++i) {
        filtered.points[i].x = filtered_pcl.points[i].x;
        filtered.points[i].y = filtered_pcl.points[i].y;
        filtered.points[i].z = filtered_pcl.points[i].z;
        filtered.points[i].r = filtered_pcl.points[i].r;
        filtered.points[i].g = filtered_pcl.points[i].g;
        filtered.points[i].b = filtered_pcl.points[i].b;
    }

    setCloudLocked(std::move(filtered), current().metric, "stat-outlier");
    return true;
#else
    std::vector<double> meanDists(pts.size(), 0.0);
    for (size_t i = 0; i < pts.size(); ++i) {
        std::vector<float> dists;
        dists.reserve(pts.size());
        for (size_t j = 0; j < pts.size(); ++j) {
            if (i == j) {
                continue;
            }
            const float dx = pts[i].x - pts[j].x;
            const float dy = pts[i].y - pts[j].y;
            const float dz = pts[i].z - pts[j].z;
            dists.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));
        }
        std::nth_element(dists.begin(),
                         dists.begin() + static_cast<std::ptrdiff_t>(nbNeighbors - 1),
                         dists.end());
        double sum = 0.0;
        for (size_t k = 0; k < nbNeighbors; ++k) {
            sum += dists[k];
        }
        meanDists[i] = sum / static_cast<double>(nbNeighbors);
    }

    double mean = 0.0;
    for (double d : meanDists) {
        mean += d;
    }
    mean /= static_cast<double>(meanDists.size());
    double var = 0.0;
    for (double d : meanDists) {
        const double t = d - mean;
        var += t * t;
    }
    const double stdev = std::sqrt(var / static_cast<double>(meanDists.size()));
    const double threshold = mean + stdRatio * stdev;

    pointcloud::PointCloud filtered;
    for (size_t i = 0; i < pts.size(); ++i) {
        if (meanDists[i] <= threshold) {
            filtered.points.push_back(pts[i]);
        }
    }
    setCloudLocked(std::move(filtered), current().metric, "stat-outlier");
    return true;
#endif
}

bool ScanDocument::deleteSelected(const std::vector<size_t>& indices) {
    std::unique_lock lock(mutex_);
    if (history_.empty() || indices.empty()) {
        return false;
    }
    std::unordered_set<size_t> kill(indices.begin(), indices.end());
    pointcloud::PointCloud kept;
    for (size_t i = 0; i < current().cloud.points.size(); ++i) {
        if (!kill.count(i)) {
            kept.points.push_back(current().cloud.points[i]);
        }
    }
    setCloudLocked(std::move(kept), current().metric, "delete");
    return true;
}

bool ScanDocument::basePlaneRemoval(double distanceThresholdM) {
    std::unique_lock lock(mutex_);
    if (history_.empty() || current().cloud.points.size() < 3) {
        return false;
    }
    // Simple horizontal plane near minimum Z (table / turntable base).
    float minZ = current().cloud.points.front().z;
    for (const auto& p : current().cloud.points) {
        minZ = std::min(minZ, p.z);
    }
    pointcloud::PointCloud kept;
    for (const auto& p : current().cloud.points) {
        if (p.z > minZ + static_cast<float>(distanceThresholdM)) {
            kept.points.push_back(p);
        }
    }
    setCloudLocked(std::move(kept), current().metric, "base-removal");
    return true;
}

bool ScanDocument::smoothMLS(double searchRadiusM) {
    std::unique_lock lock(mutex_);
    if (history_.empty() || current().cloud.points.size() < 10) {
        return false;
    }
#ifdef ALL3DSCAN_HAS_PCL
    const auto& pts = current().cloud.points;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_cloud->width = pts.size();
    pcl_cloud->height = 1;
    pcl_cloud->is_dense = true;
    pcl_cloud->points.resize(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        pcl_cloud->points[i].x = pts[i].x;
        pcl_cloud->points[i].y = pts[i].y;
        pcl_cloud->points[i].z = pts[i].z;
        pcl_cloud->points[i].r = pts[i].r;
        pcl_cloud->points[i].g = pts[i].g;
        pcl_cloud->points[i].b = pts[i].b;
    }

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(pcl_cloud);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(searchRadiusM);

    pcl::PointCloud<pcl::PointNormal> smoothed_pcl;
    mls.process(smoothed_pcl);

    pointcloud::PointCloud smoothed;
    smoothed.points.resize(smoothed_pcl.points.size());
    for (size_t i = 0; i < smoothed_pcl.points.size(); ++i) {
        smoothed.points[i].x = smoothed_pcl.points[i].x;
        smoothed.points[i].y = smoothed_pcl.points[i].y;
        smoothed.points[i].z = smoothed_pcl.points[i].z;
        if (i < pts.size()) {
            smoothed.points[i].r = pts[i].r;
            smoothed.points[i].g = pts[i].g;
            smoothed.points[i].b = pts[i].b;
        } else {
            smoothed.points[i].r = 200;
            smoothed.points[i].g = 200;
            smoothed.points[i].b = 200;
        }
    }
    setCloudLocked(std::move(smoothed), current().metric, "mls-smooth");
    return true;
#else
    return false;
#endif
}

} // namespace revoscan::document
