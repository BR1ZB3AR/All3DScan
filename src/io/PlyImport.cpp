#include "PlyImport.h"

#include <fstream>
#include <sstream>
#include <vector>

namespace revoscan::io {

std::optional<pointcloud::PointCloud> readPlyCloud(const std::string& path,
                                                   std::string* errorOut) {
    auto fail = [&](const std::string& msg) -> std::optional<pointcloud::PointCloud> {
        if (errorOut) {
            *errorOut = msg;
        }
        return std::nullopt;
    };

    std::ifstream in(path);
    if (!in.is_open()) {
        return fail("cannot open " + path);
    }

    std::string line;
    if (!std::getline(in, line) || line.rfind("ply", 0) != 0) {
        return fail("not a PLY file");
    }

    size_t vertexCount = 0;
    bool ascii = false;
    std::vector<std::string> properties;
    bool inVertexElement = false;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        std::string token;
        ss >> token;
        if (token == "format") {
            std::string fmt;
            ss >> fmt;
            ascii = fmt == "ascii";
        } else if (token == "element") {
            std::string name;
            size_t count = 0;
            ss >> name >> count;
            inVertexElement = name == "vertex";
            if (inVertexElement) {
                vertexCount = count;
            }
        } else if (token == "property" && inVertexElement) {
            std::string type, name;
            ss >> type;
            if (type == "list") {
                std::string a, b;
                ss >> a >> b >> name;
            } else {
                ss >> name;
            }
            properties.push_back(name);
        } else if (token == "end_header") {
            break;
        }
    }

    if (!ascii) {
        return fail("only ASCII PLY import is supported");
    }
    if (vertexCount == 0) {
        return fail("PLY contains no vertices");
    }

    int xIdx = -1, yIdx = -1, zIdx = -1, rIdx = -1, gIdx = -1, bIdx = -1;
    for (size_t i = 0; i < properties.size(); ++i) {
        const auto& name = properties[i];
        const int idx = static_cast<int>(i);
        if (name == "x") xIdx = idx;
        else if (name == "y") yIdx = idx;
        else if (name == "z") zIdx = idx;
        else if (name == "red") rIdx = idx;
        else if (name == "green") gIdx = idx;
        else if (name == "blue") bIdx = idx;
    }
    if (xIdx < 0 || yIdx < 0 || zIdx < 0) {
        return fail("PLY vertex element lacks x/y/z properties");
    }

    pointcloud::PointCloud cloud;
    cloud.points.reserve(vertexCount);
    std::vector<double> fields(properties.size(), 0.0);
    for (size_t v = 0; v < vertexCount; ++v) {
        if (!std::getline(in, line)) {
            return fail("PLY truncated before all vertices were read");
        }
        std::istringstream ss(line);
        for (auto& value : fields) {
            if (!(ss >> value)) {
                return fail("malformed PLY vertex line");
            }
        }
        pointcloud::Point p;
        p.x = static_cast<float>(fields[static_cast<size_t>(xIdx)]);
        p.y = static_cast<float>(fields[static_cast<size_t>(yIdx)]);
        p.z = static_cast<float>(fields[static_cast<size_t>(zIdx)]);
        if (rIdx >= 0 && gIdx >= 0 && bIdx >= 0) {
            p.r = static_cast<uint8_t>(fields[static_cast<size_t>(rIdx)]);
            p.g = static_cast<uint8_t>(fields[static_cast<size_t>(gIdx)]);
            p.b = static_cast<uint8_t>(fields[static_cast<size_t>(bIdx)]);
        }
        cloud.points.push_back(p);
    }
    return cloud;
}

} // namespace revoscan::io
