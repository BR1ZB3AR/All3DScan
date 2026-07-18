#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace revoscan::protocol {

// One captured frame from the scanner. Depth is in millimeters, row-major,
// width*height entries. rgb is optional (interleaved 8-bit RGB), also
// row-major. Real POP4 sensor resolutions are unknown until Phase B capture
// work fills them in; MockProtocol uses placeholder sizes.
struct RawFrame {
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<uint16_t> depthMm;
    std::vector<uint8_t> rgb; // empty if not available
    uint64_t timestampUs = 0;
};

struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

struct ScannerParams {
    // Placeholder tunables (exposure, gain, projector pattern, etc.) — the
    // real parameter set is unknown until the protocol is captured. Kept as
    // a free-form key/value map so callers don't need code changes when
    // Phase B adds real parameters.
    std::vector<std::pair<std::string, double>> values;
};

enum class ConnectionState {
    Disconnected,
    Connected,
    Streaming,
};

// Abstraction over "however we talk to the scanner". MockProtocol
// implements this today with synthetic data; Phase B adds a
// RevopointProtocol implementation once the real USB command/frame format
// is known from captured traffic. The GUI and pointcloud pipeline are built
// entirely against this interface so swapping implementations requires no
// changes outside src/protocol/.
class IScannerProtocol {
public:
    virtual ~IScannerProtocol() = default;

    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual ConnectionState state() const = 0;

    virtual bool startStream() = 0;
    virtual void stopStream() = 0;

    // Blocks until a frame is available or timeout elapses; nullopt on
    // timeout or disconnect.
    virtual std::optional<RawFrame> readFrame(int timeoutMs) = 0;

    virtual void setParams(const ScannerParams& params) = 0;
    virtual CameraIntrinsics intrinsics() const = 0;
};

std::unique_ptr<IScannerProtocol> makeMockProtocol();

} // namespace revoscan::protocol
