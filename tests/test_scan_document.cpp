#include <atomic>
#include <thread>

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

TEST_CASE("ScanDocument tolerates fusion on one thread while another reads") {
    // Mirrors the app: the scan pipeline fuses on its worker thread while the
    // UI copies the cloud for the 3D view. Before the document had a lock this
    // raced and crashed (PointCloudView::recomputeBounds).
    revoscan::document::ScanDocument doc;
    std::atomic<bool> stop{false};
    std::thread writer([&] {
        for (int frame = 0; frame < 300; ++frame) {
            revoscan::pointcloud::PointCloud c;
            for (int i = 0; i < 2000; ++i) {
                c.points.push_back({0.001f * static_cast<float>(i % 200),
                                    0.001f * static_cast<float>(frame), 0.3f});
            }
            doc.fuseCloud(c, 0.0005f, true, "fused");
        }
        stop = true;
    });
    size_t reads = 0;
    size_t lastSize = 0;
    while (!stop) {
        doc.withCloud([&](const revoscan::pointcloud::PointCloud& c) {
            double sum = 0.0;
            for (const auto& p : c.points) {
                sum += p.z;
            }
            lastSize = c.points.size();
            REQUIRE((lastSize == 0 || sum > 0.0));
        });
        const auto copy = doc.cloudCopy();
        REQUIRE(copy.points.size() >= lastSize);
        (void)doc.cloudEmpty();
        ++reads;
    }
    writer.join();
    REQUIRE(reads > 0);
    REQUIRE(doc.cloudSize() > 0);
}
