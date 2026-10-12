#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <fstream>
#include <string>

#include "io/GeometryExport.h"
#include "io/PlyImport.h"

using namespace revoscan;

namespace {

std::string tempPath(const char* name) {
    return std::string("/tmp/") + name;
}

} // namespace

TEST_CASE("GeometryExport rejects non-metric clouds") {
    pointcloud::PointCloud cloud;
    cloud.points.push_back({1, 2, 3, 4, 5, 6});
    auto result = io::writePlyCloud(tempPath("all3dscan_reject.ply"), cloud, false, {});
    REQUIRE_FALSE(result.ok);
    REQUIRE(result.error.find("non-metric") != std::string::npos);
}

TEST_CASE("GeometryExport writes metric PLY and binary STL") {
    pointcloud::PointCloud cloud;
    cloud.points.push_back({0.0f, 0.0f, 0.1f, 10, 20, 30});
    cloud.points.push_back({0.1f, 0.0f, 0.1f, 10, 20, 30});
    cloud.points.push_back({0.0f, 0.1f, 0.1f, 10, 20, 30});

    const auto ply = tempPath("all3dscan_metric.ply");
    auto plyResult = io::writePlyCloud(ply, cloud, true, {});
    REQUIRE(plyResult.ok);

    geometry::TriangleMesh mesh;
    mesh.metric = true;
    mesh.vertices = cloud.points;
    mesh.indices = {0, 1, 2};
    const auto stl = tempPath("all3dscan_metric.stl");
    io::ExportOptions opts;
    opts.unit = io::LengthUnit::Millimeters;
    auto stlResult = io::writeStlBinary(stl, mesh, opts);
    REQUIRE(stlResult.ok);

    std::ifstream in(stl, std::ios::binary);
    REQUIRE(in.good());
    in.seekg(80);
    uint32_t count = 0;
    in.read(reinterpret_cast<char*>(&count), 4);
    REQUIRE(count == 1);

    std::remove(ply.c_str());
    std::remove(stl.c_str());
}

TEST_CASE("PLY cloud round-trips through export and import") {
    pointcloud::PointCloud cloud;
    cloud.points.push_back({0.01f, -0.02f, 0.30f, 200, 100, 50});
    cloud.points.push_back({-0.05f, 0.04f, 0.25f, 10, 20, 30});

    const auto path = tempPath("all3dscan_roundtrip.ply");
    auto writeResult = io::writePlyCloud(path, cloud, true, {});
    REQUIRE(writeResult.ok);

    std::string error;
    auto loaded = io::readPlyCloud(path, &error);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->points.size() == cloud.points.size());
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        REQUIRE(loaded->points[i].x == Catch::Approx(cloud.points[i].x).margin(1e-5));
        REQUIRE(loaded->points[i].y == Catch::Approx(cloud.points[i].y).margin(1e-5));
        REQUIRE(loaded->points[i].z == Catch::Approx(cloud.points[i].z).margin(1e-5));
        REQUIRE(loaded->points[i].r == cloud.points[i].r);
        REQUIRE(loaded->points[i].g == cloud.points[i].g);
        REQUIRE(loaded->points[i].b == cloud.points[i].b);
    }
    std::remove(path.c_str());
}

TEST_CASE("PLY import rejects garbage") {
    const auto path = tempPath("all3dscan_garbage.ply");
    {
        std::ofstream out(path);
        out << "not a ply\n";
    }
    std::string error;
    REQUIRE_FALSE(io::readPlyCloud(path, &error).has_value());
    REQUIRE_FALSE(error.empty());
    std::remove(path.c_str());
}

TEST_CASE("STL export fails for empty mesh") {
    geometry::TriangleMesh mesh;
    mesh.metric = true;
    io::ExportOptions opts;
    opts.unit = io::LengthUnit::Millimeters;
    auto result = io::writeStlBinary(tempPath("empty.stl"), mesh, opts);
    REQUIRE_FALSE(result.ok);
}
