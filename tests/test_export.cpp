#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <fstream>
#include <sstream>

#include "io/PointCloudExport.h"

using namespace revoscan;

namespace {
pointcloud::PointCloud makeSampleCloud() {
    pointcloud::PointCloud cloud;
    cloud.points.push_back({.x = 1.0f, .y = 2.0f, .z = 3.0f, .r = 255, .g = 0, .b = 0});
    cloud.points.push_back({.x = -1.0f, .y = 0.0f, .z = 5.0f, .r = 0, .g = 255, .b = 0});
    return cloud;
}
}

TEST_CASE("writePly produces a valid-looking ASCII PLY file") {
    auto cloud = makeSampleCloud();
    const std::string path = "test_output.ply";
    REQUIRE(io::writePly(path, cloud));

    std::ifstream in(path);
    REQUIRE(in.is_open());
    std::stringstream buffer;
    buffer << in.rdbuf();
    const std::string contents = buffer.str();

    REQUIRE(contents.find("ply") == 0);
    REQUIRE(contents.find("element vertex 2") != std::string::npos);
    REQUIRE(contents.find("end_header") != std::string::npos);

    in.close();
    std::remove(path.c_str());
}

TEST_CASE("writeObj produces vertex lines for each point") {
    auto cloud = makeSampleCloud();
    const std::string path = "test_output.obj";
    REQUIRE(io::writeObj(path, cloud));

    std::ifstream in(path);
    REQUIRE(in.is_open());
    int vertexLines = 0;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line[0] == 'v') {
            ++vertexLines;
        }
    }
    REQUIRE(vertexLines == 2);

    in.close();
    std::remove(path.c_str());
}
