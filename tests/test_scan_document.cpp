#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "document/ScanDocument.h"

using namespace revoscan;

TEST_CASE("ScanDocument fuses metric clouds and supports undo/redo") {
    document::ScanDocument doc;
    pointcloud::PointCloud a;
    a.points.push_back({0.0f, 0.0f, 0.5f, 10, 20, 30});
    a.points.push_back({0.01f, 0.0f, 0.5f, 10, 20, 30});
    doc.fuseMetricCloud(a, 0.005f);
    REQUIRE(doc.isMetric());
    REQUIRE(doc.revision() == 1);
    REQUIRE(doc.cloud().points.size() >= 1);

    pointcloud::PointCloud b;
    b.points.push_back({0.2f, 0.0f, 0.5f, 1, 2, 3});
    doc.fuseMetricCloud(b, 0.005f);
    // Live fuse mutates in place (no undo stack growth per frame).
    REQUIRE(doc.revision() == 1);
    REQUIRE(doc.cloud().points.size() >= 2);

    REQUIRE(doc.cropBox(-1, -1, 0, 0.05f, 1, 1));
    REQUIRE(doc.canUndo());
    REQUIRE(doc.undo());
    REQUIRE(doc.revision() == 1);
    REQUIRE(doc.canRedo());
    REQUIRE(doc.redo());
}

TEST_CASE("ScanDocument base plane removal keeps elevated points") {
    document::ScanDocument doc;
    pointcloud::PointCloud cloud;
    cloud.points.push_back({0, 0, 0.01f, 0, 0, 0});
    cloud.points.push_back({0.01f, 0, 0.01f, 0, 0, 0});
    cloud.points.push_back({0, 0.01f, 0.20f, 0, 0, 0});
    doc.setCloud(cloud, true, "seed");
    REQUIRE(doc.basePlaneRemoval(0.05));
    REQUIRE(doc.cloud().points.size() == 1);
    REQUIRE(doc.cloud().points[0].z == Catch::Approx(0.20f));
}
