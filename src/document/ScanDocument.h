#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "geometry/TriangleMesh.h"
#include "pointcloud/PointCloud.h"

namespace revoscan::document {

// Immutable-revision scan document. Metric registration / edits push new
// revisions; undo/redo switches the current index. Vendor-raw previews must
// never be committed with metric=true.
class ScanDocument {
public:
    void clear();

    uint64_t revision() const;
    bool canUndo() const;
    bool canRedo() const;
    bool undo();
    bool redo();

    bool isMetric() const;
    const pointcloud::PointCloud& cloud() const;
    const geometry::TriangleMesh& mesh() const;
    geometry::GeometrySnapshot snapshot() const;

    // Appends a metric frame cloud into a voxel grid (simple native fusion).
    void fuseMetricCloud(const pointcloud::PointCloud& cloud, float voxelSizeM = 0.002f);

    // General incremental fusion. `metric=false` keeps the document non-metric
    // (blocks export) while still building a live registered preview model.
    // Live fusion mutates the current revision in place (no undo stack growth)
    // so per-frame cost stays bounded as the model grows.
    void fuseCloud(const pointcloud::PointCloud& cloud, float voxelSizeM, bool metric,
                   const std::string& label = "fused");

    void setCloud(pointcloud::PointCloud cloud, bool metric, const std::string& label = {});
    void setMesh(geometry::TriangleMesh mesh, bool metric, const std::string& label = {});

    // Editing operations (create a new revision).
    bool cropBox(float minX, float minY, float minZ, float maxX, float maxY, float maxZ);
    bool removeStatisticalOutliers(size_t nbNeighbors, double stdRatio);
    bool deleteSelected(const std::vector<size_t>& indices);
    bool basePlaneRemoval(double distanceThresholdM = 0.004);
    bool smoothMLS(double searchRadiusM = 0.005);

private:
    struct Revision {
        uint64_t id = 0;
        pointcloud::PointCloud cloud;
        geometry::TriangleMesh mesh;
        bool metric = false;
        std::string label;
    };

    void push(Revision rev);
    const Revision& current() const;

    std::vector<Revision> history_;
    size_t index_ = 0;
    uint64_t nextId_ = 1;
    Revision empty_;
};

} // namespace revoscan::document
