#include "ResourcePaths.h"

#include <cstdlib>
#include <filesystem>
#include <unistd.h>

namespace revoscan::util {
namespace {

namespace fs = std::filesystem;

std::optional<std::string> executableDir() {
    char buffer[4096];
    const ssize_t n = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (n <= 0) {
        return std::nullopt;
    }
    buffer[n] = '\0';
    return fs::path(buffer).parent_path().string();
}

void appendIfExists(std::vector<std::string>& out, const fs::path& candidate) {
    std::error_code ec;
    if (fs::is_regular_file(candidate, ec)) {
        out.push_back(candidate.string());
    }
}

std::vector<std::string> dataRoots() {
    std::vector<std::string> roots;
    if (const char* overrideDir = std::getenv("ALL3DSCAN_DATA_DIR")) {
        roots.emplace_back(overrideDir);
    }
    if (const char* appDir = std::getenv("APPDIR")) {
        roots.emplace_back(fs::path(appDir) / "usr" / "share" / "all3dscan");
        roots.emplace_back(fs::path(appDir) / "usr" / "share");
    }
    if (auto exeDir = executableDir()) {
        const fs::path binDir(*exeDir);
        roots.emplace_back(binDir / ".." / "share" / "all3dscan");
        roots.emplace_back(binDir / ".." / "share");
        // Source-tree / build-dir fallback: <root>/resources and <root>/tools
        roots.emplace_back(binDir / ".." / "resources");
        roots.emplace_back(binDir / ".." / ".." / "resources");
        roots.emplace_back(binDir / ".." / "tools");
        roots.emplace_back(binDir / ".." / ".." / "tools");
    }
    roots.emplace_back("/usr/local/share/all3dscan");
    roots.emplace_back("/usr/share/all3dscan");
    roots.emplace_back("/usr/local/share");
    roots.emplace_back("/usr/share");
    return roots;
}

std::vector<std::string> libRoots() {
    std::vector<std::string> roots;
    if (const char* overrideDir = std::getenv("ALL3DSCAN_LIB_DIR")) {
        roots.emplace_back(overrideDir);
    }
    if (const char* appDir = std::getenv("APPDIR")) {
        roots.emplace_back(fs::path(appDir) / "usr" / "lib" / "all3dscan");
        roots.emplace_back(fs::path(appDir) / "usr" / "lib" / "x86_64-linux-gnu" / "all3dscan");
        roots.emplace_back(fs::path(appDir) / "usr" / "lib" / "aarch64-linux-gnu" / "all3dscan");
        roots.emplace_back(fs::path(appDir) / "usr" / "lib");
    }
    if (auto exeDir = executableDir()) {
        const fs::path binDir(*exeDir);
        roots.emplace_back(binDir / ".." / "lib" / "all3dscan");
        roots.emplace_back(binDir / ".." / "lib" / "x86_64-linux-gnu" / "all3dscan");
        roots.emplace_back(binDir / ".." / "lib" / "aarch64-linux-gnu" / "all3dscan");
        roots.emplace_back(binDir / ".." / "lib");
        roots.emplace_back(binDir / ".." / "tools" / "pipeline");
        roots.emplace_back(binDir / ".." / ".." / "tools" / "pipeline");
    }
    roots.emplace_back("/usr/local/lib/all3dscan");
    roots.emplace_back("/usr/lib/x86_64-linux-gnu/all3dscan");
    roots.emplace_back("/usr/lib/aarch64-linux-gnu/all3dscan");
    roots.emplace_back("/usr/lib/all3dscan");
    return roots;
}

} // namespace

std::optional<std::string> findDataFile(const std::string& relativePath,
                                        const std::vector<std::string>& extraCandidates) {
    if (!relativePath.empty() && relativePath.front() == '/') {
        std::error_code ec;
        if (fs::is_regular_file(relativePath, ec)) {
            return relativePath;
        }
    }

    std::vector<std::string> candidates = extraCandidates;
    for (const auto& root : dataRoots()) {
        appendIfExists(candidates, fs::path(root) / relativePath);
    }
    for (const auto& candidate : candidates) {
        std::error_code ec;
        if (fs::is_regular_file(candidate, ec)) {
            return candidate;
        }
    }
    return std::nullopt;
}

std::optional<std::string> findDevicesJson() {
    return findDataFile("devices.json", {
        "resources/devices.json",
        "../resources/devices.json",
    });
}

std::optional<std::string> findMeshWorkerScript() {
    std::vector<std::string> candidates = {
        "tools/pipeline/mesh_worker.py",
        "../tools/pipeline/mesh_worker.py",
    };
    for (const auto& root : libRoots()) {
        appendIfExists(candidates, fs::path(root) / "mesh_worker.py");
        appendIfExists(candidates, fs::path(root) / "pipeline" / "mesh_worker.py");
    }
    for (const auto& candidate : candidates) {
        std::error_code ec;
        if (fs::is_regular_file(candidate, ec)) {
            return candidate;
        }
    }
    return std::nullopt;
}

std::optional<std::string> findAppIcon() {
    return findDataFile(
        "icons/hicolor/scalable/apps/io.github.all3dscan.All3DScan.svg",
        {
            "resources/io.github.all3dscan.All3DScan.svg",
            "../resources/io.github.all3dscan.All3DScan.svg",
            "/usr/share/icons/hicolor/scalable/apps/io.github.all3dscan.All3DScan.svg",
        });
}

} // namespace revoscan::util
