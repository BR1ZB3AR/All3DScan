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

// Like findV4l2CaptureNode, but only considers nodes whose sysfs `name`
// contains nameSubstring (case-sensitive), e.g. "RGB" vs "Depth".
std::optional<std::string> findV4l2CaptureNodeNamed(uint16_t vendorId,
                                                    uint16_t productId,
                                                    uint32_t pixelFormat,
                                                    uint32_t width,
                                                    uint32_t height,
                                                    const std::string& nameSubstring);

// When uvcvideo is bound to the POP4 depth interfaces but no /dev/video*
// nodes exist (common after XU detach/reattach), force an unbind+bind cycle
// so the streaming interface re-registers. Returns true if sysfs writes were
// attempted successfully.
bool rebindUvcvideoForUsbDevice(uint16_t vendorId, uint16_t productId);

// UVC extension-unit SET_CUR through uvcvideo (UVCIOC_CTRL_QUERY) on an
// open video node. Unlike a raw libusb control transfer it needs no driver
// detach, so it works while the node is streaming.
bool uvcExtensionSetCur(int fd, uint8_t unit, uint8_t selector, const uint8_t* data,
                        uint16_t size, std::string* error = nullptr);

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
    // True once the node reported the device gone (poll hang-up or error, or
    // an ioctl with ENODEV): the scanner unplugged or re-enumerated, and this
    // descriptor will never deliver another frame. Cleared by close().
    bool deviceGone() const { return deviceGone_; }
    int fd() const { return fd_; }

    // Waits up to timeoutMs for a frame. With newest, takes the newest one the

    // driver has ready and gives the older ones back unread: a consumer slower

    // than the camera otherwise always gets the oldest of the queued buffers,

    // which runs as many frames behind as there are buffers.

    std::optional<V4l2Frame> readFrame(int timeoutMs, bool newest = false);
    const std::string& lastError() const { return lastError_; }

private:
    struct Buffer {
        void* address = nullptr;
        size_t length = 0;
    };

    void setError(const std::string& operation);

    int fd_ = -1;
    bool streaming_ = false;
    bool deviceGone_ = false;
    std::vector<Buffer> buffers_;
    std::string lastError_;
};

} // namespace revoscan::usb
