#pragma once

#include <optional>
#include <string>

namespace revoscan::bridge {

// Thin launcher for tools/pipeline/mesh_worker.py.
// C++ owns USB/V4L2/UI; Python owns Open3D meshing until ported.
class PythonWorker {
public:
    // Empty workerScript resolves via util::findMeshWorkerScript()
    // (.deb / AppImage / source-tree layouts).
    explicit PythonWorker(std::string workerScript = {},
                          std::string pythonExecutable = "python3");
    ~PythonWorker();

    PythonWorker(const PythonWorker&) = delete;
    PythonWorker& operator=(const PythonWorker&) = delete;

    bool start();
    void stop();
    bool running() const { return pid_ > 0; }

    // Sends one JSON command and waits for one JSON reply.
    std::optional<std::string> request(const std::string& jsonLine, int timeoutMs = 60000);

    // Convenience: run Poisson meshing on an exported PLY.
    std::optional<std::string> meshPly(const std::string& inputPly,
                                       const std::string& outputMesh,
                                       int poissonDepth = 9,
                                       int timeoutMs = 120000);

    // Convenience: volumetric TSDF reconstruction from a keyframe manifest.
    std::optional<std::string> reconstructTsdf(const std::string& manifestPath,
                                               int timeoutMs = 180000);

    const std::string& lastError() const { return lastError_; }

private:
    bool writeLine(const std::string& line);
    std::optional<std::string> readLine(int timeoutMs);

    std::string workerScript_;
    std::string pythonExecutable_;
    std::string lastError_;
    int pid_ = -1;
    int stdinFd_ = -1;
    int stdoutFd_ = -1;
};

} // namespace revoscan::bridge
