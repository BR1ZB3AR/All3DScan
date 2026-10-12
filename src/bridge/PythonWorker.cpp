#include "PythonWorker.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sstream>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <nlohmann/json.hpp>

#include "util/ResourcePaths.h"

namespace revoscan::bridge {
namespace {

bool setCloexec(int fd) {
    const int flags = fcntl(fd, F_GETFD);
    if (flags < 0) {
        return false;
    }
    return fcntl(fd, F_SETFD, flags | FD_CLOEXEC) == 0;
}

} // namespace

PythonWorker::PythonWorker(std::string workerScript, std::string pythonExecutable)
    : workerScript_(std::move(workerScript)), pythonExecutable_(std::move(pythonExecutable)) {}

PythonWorker::~PythonWorker() {
    stop();
}

bool PythonWorker::start() {
    if (running()) {
        return true;
    }
    if (workerScript_.empty()) {
        if (auto resolved = util::findMeshWorkerScript()) {
            workerScript_ = *resolved;
        } else {
            lastError_ = "could not locate mesh_worker.py "
                         "(set ALL3DSCAN_LIB_DIR or pass an absolute path)";
            return false;
        }
    }

    int inPipe[2] = {-1, -1};
    int outPipe[2] = {-1, -1};
    if (pipe(inPipe) != 0 || pipe(outPipe) != 0) {
        lastError_ = "pipe() failed";
        return false;
    }

    const pid_t pid = fork();
    if (pid < 0) {
        lastError_ = "fork() failed";
        close(inPipe[0]);
        close(inPipe[1]);
        close(outPipe[0]);
        close(outPipe[1]);
        return false;
    }

    if (pid == 0) {
        // Child: stdin <- parent write, stdout -> parent read.
        dup2(inPipe[0], STDIN_FILENO);
        dup2(outPipe[1], STDOUT_FILENO);
        close(inPipe[0]);
        close(inPipe[1]);
        close(outPipe[0]);
        close(outPipe[1]);
        execlp(pythonExecutable_.c_str(), pythonExecutable_.c_str(),
               workerScript_.c_str(), static_cast<char*>(nullptr));
        _exit(127);
    }

    // Parent keeps write end of inPipe and read end of outPipe.
    close(inPipe[0]);
    close(outPipe[1]);
    stdinFd_ = inPipe[1];
    stdoutFd_ = outPipe[0];
    setCloexec(stdinFd_);
    setCloexec(stdoutFd_);
    pid_ = static_cast<int>(pid);

    auto pong = request(R"({"cmd":"ping"})", 5000);
    if (!pong || pong->find("\"ok\":true") == std::string::npos) {
        lastError_ = "Python worker failed to respond to ping "
                     "(is python3 and the worker script available?)";
        stop();
        return false;
    }
    lastError_.clear();
    return true;
}

void PythonWorker::stop() {
    if (stdinFd_ >= 0) {
        // Best-effort quit; ignore write errors during teardown.
        const std::string quit = R"({"cmd":"quit"})" "\n";
        (void)!write(stdinFd_, quit.data(), quit.size());
        close(stdinFd_);
        stdinFd_ = -1;
    }
    if (stdoutFd_ >= 0) {
        close(stdoutFd_);
        stdoutFd_ = -1;
    }
    if (pid_ > 0) {
        int status = 0;
        if (waitpid(pid_, &status, WNOHANG) == 0) {
            kill(pid_, SIGTERM);
            waitpid(pid_, &status, 0);
        }
        pid_ = -1;
    }
}

bool PythonWorker::writeLine(const std::string& line) {
    if (stdinFd_ < 0) {
        lastError_ = "worker not started";
        return false;
    }
    std::string payload = line;
    if (payload.empty() || payload.back() != '\n') {
        payload.push_back('\n');
    }
    size_t written = 0;
    while (written < payload.size()) {
        const ssize_t n = write(stdinFd_, payload.data() + written, payload.size() - written);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            lastError_ = std::string("write to worker failed: ") + std::strerror(errno);
            return false;
        }
        written += static_cast<size_t>(n);
    }
    return true;
}

std::optional<std::string> PythonWorker::readLine(int timeoutMs) {
    if (stdoutFd_ < 0) {
        lastError_ = "worker not started";
        return std::nullopt;
    }

    std::string buffer;
    const auto deadlineMs = timeoutMs;
    while (true) {
        pollfd pfd{};
        pfd.fd = stdoutFd_;
        pfd.events = POLLIN;
        const int ready = poll(&pfd, 1, deadlineMs);
        if (ready == 0) {
            lastError_ = "timed out waiting for worker reply";
            return std::nullopt;
        }
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            lastError_ = std::string("poll failed: ") + std::strerror(errno);
            return std::nullopt;
        }

        char chunk[4096];
        const ssize_t n = read(stdoutFd_, chunk, sizeof(chunk));
        if (n == 0) {
            lastError_ = "worker closed stdout";
            return std::nullopt;
        }
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            lastError_ = std::string("read failed: ") + std::strerror(errno);
            return std::nullopt;
        }
        buffer.append(chunk, static_cast<size_t>(n));
        const auto newline = buffer.find('\n');
        if (newline != std::string::npos) {
            std::string line = buffer.substr(0, newline);
            return line;
        }
    }
}

std::optional<std::string> PythonWorker::request(const std::string& jsonLine, int timeoutMs) {
    if (!running() && !start()) {
        return std::nullopt;
    }
    if (!writeLine(jsonLine)) {
        return std::nullopt;
    }
    return readLine(timeoutMs);
}

std::optional<std::string> PythonWorker::meshPly(const std::string& inputPly,
                                                 const std::string& outputMesh,
                                                 int poissonDepth,
                                                 int timeoutMs) {
    const nlohmann::json command = {
        {"cmd", "mesh"},
        {"ply", inputPly},
        {"out", outputMesh},
        {"method", "poisson"},
        {"depth", poissonDepth},
    };
    auto response = request(command.dump(), timeoutMs);
    if (!response) {
        return std::nullopt;
    }
    if (response->find("\"ok\":true") == std::string::npos) {
        lastError_ = *response;
        return std::nullopt;
    }
    return response;
}

std::optional<std::string> PythonWorker::reconstructTsdf(const std::string& manifestPath,
                                                         int timeoutMs) {
    const nlohmann::json command = {
        {"cmd", "tsdf"},
        {"manifest", manifestPath},
    };
    auto response = request(command.dump(), timeoutMs);
    if (!response) {
        return std::nullopt;
    }
    if (response->find("\"ok\":true") == std::string::npos) {
        lastError_ = *response;
        return std::nullopt;
    }
    return response;
}

} // namespace revoscan::bridge
