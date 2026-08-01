#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace revoscan::usb {

struct V4l2Frame {
    std::vector<uint8_t> bytes;
    uint64_t timestampUs = 0;
};

// Finds a V4L2 capture node belonging to the requested USB device and
// advertising the exact pixel format and frame size. Node numbers are never
// assumed because they vary with probe order.
std::optional<std::string> findV4l2CaptureNode(uint16_t vendorId,
                                               uint16_t productId,
                                               uint32_t pixelFormat,
                                               uint32_t width,
                                               uint32_t height);

class V4l2Capture {
public:
    V4l2Capture() = default;
    ~V4l2Capture();

    V4l2Capture(const V4l2Capture&) = delete;
    V4l2Capture& operator=(const V4l2Capture&) = delete;

    bool open(const std::string& path,
              uint32_t pixelFormat,
              uint32_t width,
              uint32_t height,
              uint32_t bufferCount = 4);
    void close();

    bool start();
    void stop();
    bool isOpen() const { return fd_ >= 0; }
    bool isStreaming() const { return streaming_; }

    std::optional<V4l2Frame> readFrame(int timeoutMs);
    const std::string& lastError() const { return lastError_; }

private:
    struct Buffer {
        void* address = nullptr;
        size_t length = 0;
    };

    void setError(const std::string& operation);

    int fd_ = -1;
    bool streaming_ = false;
    std::vector<Buffer> buffers_;
    std::string lastError_;
};

} // namespace revoscan::usb
